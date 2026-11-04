"""Synthetic AnyGrasp server for offline interface and mission tests only."""

from copy import deepcopy
import math
import uuid

from openflex_thesis_msgs.action import GenerateGrasps
from openflex_thesis_msgs.msg import GraspCandidate, GraspCandidateArray
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MockAnyGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_anygrasp")
        self.declare_parameter("action_name", "/thesis/generate_grasps")
        self.declare_parameter("allow_mock", False)
        self.server = ActionServer(
            self,
            GenerateGrasps,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().warn("SYNTHETIC AnyGrasp active; never use it as thesis evidence")

    def goal_callback(self, _request) -> GoalResponse:
        return GoalResponse.ACCEPT if bool(self.get_parameter("allow_mock").value) else GoalResponse.REJECT

    def execute(self, goal_handle):
        goal = goal_handle.request
        output = GraspCandidateArray()
        output.header = deepcopy(goal.target.header)
        output.header.stamp = self.get_clock().now().to_msg()
        output.request_id = uuid.uuid4().hex
        output.object_id = goal.target.object_id
        output.map_version = goal.target.map_version
        output.observation_sequence = goal.target.observation_sequence
        output.stage = goal.stage
        output.physical_perception = False
        count = max(1, int(goal.top_k))
        object_pose = goal.target.pose.pose
        for index in range(count):
            grasp = GraspCandidate()
            grasp.grasp_id = 1000 * int(goal.stage) + index
            grasp.stage = goal.stage
            grasp.grasp_pose.header = deepcopy(goal.target.header)
            grasp.pregrasp_pose.header = deepcopy(goal.target.header)
            lateral = (index - (count - 1) / 2.0) * 0.025
            grasp.grasp_pose.pose.position.x = object_pose.position.x
            grasp.grasp_pose.pose.position.y = object_pose.position.y + lateral
            grasp.grasp_pose.pose.position.z = object_pose.position.z
            grasp.pregrasp_pose.pose.position.x = object_pose.position.x + 0.14
            grasp.pregrasp_pose.pose.position.y = object_pose.position.y + lateral
            grasp.pregrasp_pose.pose.position.z = object_pose.position.z
            # A normalized nominal shelf-facing TCP orientation.
            grasp.grasp_pose.pose.orientation.y = math.sqrt(0.5)
            grasp.grasp_pose.pose.orientation.w = math.sqrt(0.5)
            grasp.pregrasp_pose.pose.orientation = deepcopy(grasp.grasp_pose.pose.orientation)
            grasp.approach.x = -1.0
            grasp.width = 0.05
            grasp.quality = max(0.1, 0.95 - 0.05 * index)
            grasp.visibility_score = (
                0.95 if goal.stage == GenerateGrasps.Goal.REFINED else 0.80
            )
            grasp.collision_score = 0.90
            grasp.observation_sequence = goal.target.observation_sequence
            grasp.physical_perception = False
            grasp.source = "synthetic_mock_DO_NOT_USE_FOR_RESULTS"
            output.grasps.append(grasp)
        goal_handle.succeed()
        result = GenerateGrasps.Result()
        result.success = True
        result.grasps = output
        result.message = "synthetic AnyGrasp candidates generated"
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockAnyGraspNode()
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
