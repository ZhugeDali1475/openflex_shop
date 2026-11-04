"""Explicitly synthetic evaluator for interface tests; never valid thesis evidence."""

from copy import deepcopy
import math

import rclpy
from openflex_thesis_msgs.action import EvaluateCandidates
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .geometry import clamp, quaternion_to_yaw, relative_planar, PlanarPose


class MockEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_evaluator")
        self.declare_parameter("action_name", "/thesis/evaluate_candidates")
        self.declare_parameter("allow_mock", False)
        self.declare_parameter("ideal_distance_m", 0.75)
        self.declare_parameter("distance_half_width_m", 0.35)
        self.declare_parameter("maximum_lateral_m", 0.45)
        self.callback_group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self,
            EvaluateCandidates,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        if not bool(self.get_parameter("allow_mock").value):
            self.get_logger().warn(
                "Mock evaluator is disabled. Set allow_mock:=true only for interface tests."
            )

    def goal_callback(self, _goal) -> GoalResponse:
        return (
            GoalResponse.ACCEPT
            if bool(self.get_parameter("allow_mock").value)
            else GoalResponse.REJECT
        )

    def execute(self, goal_handle):
        goal = goal_handle.request
        output = deepcopy(goal.candidates)
        output.evaluator_id = "synthetic_mock_v1_DO_NOT_USE_FOR_RESULTS"
        output.physical_evaluation = False
        object_pose = goal.target.pose.pose
        object_yaw = quaternion_to_yaw(
            object_pose.orientation.x,
            object_pose.orientation.y,
            object_pose.orientation.z,
            object_pose.orientation.w,
        )
        target = PlanarPose(object_pose.position.x, object_pose.position.y, object_yaw)
        ideal = float(self.get_parameter("ideal_distance_m").value)
        half_width = float(self.get_parameter("distance_half_width_m").value)
        max_lateral = float(self.get_parameter("maximum_lateral_m").value)
        grasp_by_id = {int(item.grasp_id): item for item in goal.grasps.grasps}
        total = len(output.candidates)
        for index, candidate in enumerate(output.candidates):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = EvaluateCandidates.Result()
                result.success = False
                result.message = "canceled"
                return result
            orientation = candidate.base_pose.orientation
            base = PlanarPose(
                candidate.base_pose.position.x,
                candidate.base_pose.position.y,
                quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
            )
            relative = relative_planar(target, base)
            distance = math.hypot(relative.x, relative.y)
            reach = clamp(1.0 - abs(distance - ideal) / max(half_width, 1.0e-6), 0.0, 1.0)
            lateral_margin = clamp(1.0 - abs(relative.y) / max(max_lateral, 1.0e-6), 0.0, 1.0)
            face_error = abs(math.atan2(math.sin(base.yaw - math.atan2(
                target.y - base.y, target.x - base.x
            )), math.cos(base.yaw - math.atan2(target.y - base.y, target.x - base.x))))
            orientation_margin = clamp(1.0 - face_error / 0.7, 0.0, 1.0)
            candidate.evaluated = True
            candidate.ik_valid = reach > 0.05
            candidate.collision_free = distance > 0.48
            grasp = grasp_by_id.get(int(candidate.grasp_id))
            lift_match = clamp(1.0 - abs(candidate.lift_seed + 0.15) / 0.6, 0.0, 1.0)
            candidate.reachability = reach * (0.7 + 0.3 * lift_match)
            candidate.joint_margin = min(lateral_margin, orientation_margin)
            candidate.robustness = min(reach, lateral_margin) * 0.95
            candidate.lift_position = candidate.lift_seed
            if grasp is not None:
                candidate.grasp_quality = grasp.quality
                candidate.visibility_score = grasp.visibility_score
            prefix = "openarmx_left" if candidate.active_arm == candidate.ARM_LEFT else "openarmx_right"
            candidate.joint_names = ["lift_joint"] + [
                f"{prefix}_joint{joint}" for joint in range(1, 8)
            ]
            synthetic = [candidate.lift_position] + [0.0] * 7
            candidate.pregrasp_joint_positions = synthetic
            candidate.grasp_joint_positions = synthetic
            if candidate.move_score <= 0.0:
                candidate.move_score = clamp(1.0 - abs(distance - ideal), 0.0, 1.0)
            feedback = EvaluateCandidates.Feedback()
            feedback.phase = "synthetic_evaluation"
            feedback.evaluated_candidates = index + 1
            feedback.total_candidates = total
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result = EvaluateCandidates.Result()
        result.success = True
        result.evaluated_candidates = output
        result.message = "synthetic interface test completed; not physical evidence"
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockEvaluatorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
