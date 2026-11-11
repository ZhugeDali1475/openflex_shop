"""Read-only MoveIt evaluator for AnyGrasp-conditioned (base, lift, grasp) tuples."""

from copy import deepcopy
import math
import os
import time
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK, GetStateValidity
from openflex_thesis_msgs.action import EvaluateCandidates
from openflex_thesis_msgs.msg import GraspCandidate
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import yaml

from .geometry import (
    Quaternion,
    Vector3,
    clamp,
    quaternion_conjugate,
    quaternion_multiply,
    rotate_vector,
    subtract_vector,
    yaw_to_quaternion,
)


class MoveItEvaluatorNode(Node):
    """Query IK and state validity without ever executing a trajectory."""

    def __init__(self) -> None:
        super().__init__("moveit_evaluator")
        default_groups = os.path.join(
            get_package_share_directory("openflex_tc_wam"), "config", "planning_groups.yaml"
        )
        default_scene = os.path.join(
            get_package_share_directory("openflex_tc_wam"), "config", "shelf_scene.yaml"
        )
        self.declare_parameter("action_name", "/thesis/evaluate_candidates")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("base_frame", "base_link")
        # Height of base_link above the floor plane the map frame uses. The
        # URDF states it directly: base_footprint_joint is base_link ->
        # base_footprint at xyz="0 0 -0.17". TC-WAM candidate base poses are
        # planar (z=0) poses of that floor plane, so every conversion into
        # base_link must remove this height; leaving it in placed all Cartesian
        # targets 0.17 m too high.
        self.declare_parameter("base_mount_height_m", 0.17)
        self.declare_parameter("planning_groups_file", default_groups)
        self.declare_parameter("shelf_scene_file", default_scene)
        self.declare_parameter("require_calibrated_scene", True)
        self.declare_parameter("joint_state_timeout_sec", 0.5)
        self.declare_parameter("ik_timeout_sec", 0.08)
        self.declare_parameter("service_wait_timeout_sec", 5.0)
        self.declare_parameter("lift_constraint_tolerance_m", 0.025)
        self.declare_parameter("robust_translation_floor_m", 0.01)
        self.declare_parameter("robust_yaw_floor_rad", 0.03)
        self.declare_parameter("robustness_samples", 2)
        self.callback_group = ReentrantCallbackGroup()
        self.ik_client = self.create_client(
            GetPositionIK,
            str(self.get_parameter("ik_service").value),
            callback_group=self.callback_group,
        )
        self.validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("state_validity_service").value),
            callback_group=self.callback_group,
        )
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            str(self.get_parameter("apply_planning_scene_service").value),
            callback_group=self.callback_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.on_joint_state,
            20,
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            EvaluateCandidates,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        with open(
            str(self.get_parameter("planning_groups_file").value), "r", encoding="utf-8"
        ) as stream:
            self.group_contracts: Dict[str, dict] = (
                yaml.safe_load(stream) or {}
            ).get("planning_groups", {})
        with open(
            str(self.get_parameter("shelf_scene_file").value), "r", encoding="utf-8"
        ) as stream:
            self.shelf_scene = yaml.safe_load(stream) or {}
        self.joint_state: Optional[JointState] = None
        self.joint_state_receipt_ns = 0
        self.busy = False

    def on_joint_state(self, msg: JointState) -> None:
        self.joint_state = deepcopy(msg)
        self.joint_state_receipt_ns = self.get_clock().now().nanoseconds

    def goal_callback(self, request: EvaluateCandidates.Goal) -> GoalResponse:
        groups = {candidate.planning_group for candidate in request.candidates.candidates}
        valid = bool(
            request.grasps.grasps
            and request.grasps.object_id == request.target.object_id
            and groups
            and all(group in self.group_contracts for group in groups)
        )
        return GoalResponse.ACCEPT if not self.busy and valid else GoalResponse.REJECT

    def fresh_joint_state(self) -> bool:
        if self.joint_state is None:
            return False
        age = (self.get_clock().now().nanoseconds - self.joint_state_receipt_ns) * 1.0e-9
        return age <= float(self.get_parameter("joint_state_timeout_sec").value)

    @staticmethod
    def pose_components(pose) -> Tuple[Vector3, Quaternion]:
        return (
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        )

    def pose_in_candidate_base(
        self,
        candidate_pose,
        pose_stamped: PoseStamped,
        expected_frame: str,
        perturbation: Tuple[float, float, float],
    ) -> PoseStamped:
        if pose_stamped.header.frame_id != expected_frame:
            raise ValueError(
                f"grasp frame '{pose_stamped.header.frame_id}' must be '{expected_frame}'"
            )
        base_position, base_rotation = self.base_link_position(candidate_pose)
        world_position, world_rotation = self.pose_components(pose_stamped.pose)
        world_position = (
            world_position[0] + perturbation[0],
            world_position[1] + perturbation[1],
            world_position[2],
        )
        world_rotation = quaternion_multiply(
            yaw_to_quaternion(perturbation[2]), world_rotation
        )
        base_inverse = quaternion_conjugate(base_rotation)
        relative_position = rotate_vector(
            base_inverse, subtract_vector(world_position, base_position)
        )
        relative_rotation = quaternion_multiply(base_inverse, world_rotation)
        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = str(self.get_parameter("base_frame").value)
        output.pose.position.x, output.pose.position.y, output.pose.position.z = relative_position
        output.pose.orientation.x, output.pose.orientation.y, \
            output.pose.orientation.z, output.pose.orientation.w = relative_rotation
        return output

    def seed_state(self, seed: Optional[RobotState], lift_seed: float) -> RobotState:
        output = deepcopy(seed) if seed is not None else RobotState()
        if seed is None and self.joint_state is not None:
            output.joint_state = deepcopy(self.joint_state)
            output.is_diff = True
        names = list(output.joint_state.name)
        positions = list(output.joint_state.position)
        if "lift_joint" in names:
            positions[names.index("lift_joint")] = lift_seed
        else:
            names.append("lift_joint")
            positions.append(lift_seed)
        output.joint_state.name = names
        output.joint_state.position = positions
        return output

    async def solve_ik(
        self,
        group_name: str,
        ik_link: str,
        pose: PoseStamped,
        seed: Optional[RobotState],
        lift_seed: float,
    ) -> Tuple[bool, Optional[RobotState]]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = ik_link
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = False
        request.ik_request.robot_state = self.seed_state(seed, lift_seed)
        lift_constraint = JointConstraint()
        lift_constraint.joint_name = "lift_joint"
        lift_constraint.position = lift_seed
        tolerance = float(self.get_parameter("lift_constraint_tolerance_m").value)
        lift_constraint.tolerance_above = tolerance
        lift_constraint.tolerance_below = tolerance
        lift_constraint.weight = 1.0
        request.ik_request.constraints.joint_constraints.append(lift_constraint)
        timeout = float(self.get_parameter("ik_timeout_sec").value)
        request.ik_request.timeout.sec = int(timeout)
        request.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1.0e9)
        response = await self.ik_client.call_async(request)
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, None
        return True, response.solution

    async def collision_free(self, group_name: str, state: RobotState) -> bool:
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = ""
        response = await self.validity_client.call_async(request)
        return bool(response is not None and response.valid)

    def base_link_position(self, candidate_pose):
        """Map-frame position of base_link for a candidate base pose.

        A candidate base pose is a planar pose on the floor plane the map frame
        uses, while the conversions below produce poses expressed in
        base_link, which sits base_mount_height_m above that plane.
        """
        position, rotation = self.pose_components(candidate_pose)
        height = float(self.get_parameter("base_mount_height_m").value)
        return (position[0], position[1], position[2] + height), rotation

    def shelf_objects_in_candidate_base(self, candidate_pose, target_frame: str):
        scene_frame = str(self.shelf_scene.get("frame_id", ""))
        if not scene_frame or scene_frame != target_frame:
            raise ValueError(
                f"shelf scene frame '{scene_frame}' must match target frame '{target_frame}'"
            )
        base_position, base_rotation = self.base_link_position(candidate_pose)
        base_inverse = quaternion_conjugate(base_rotation)
        objects = []
        for specification in self.shelf_scene.get("collision_objects", []):
            if specification.get("shape") != "box":
                raise ValueError("only box shelf collision objects are supported")
            dimensions = [float(value) for value in specification.get("dimensions", [])]
            position = tuple(float(value) for value in specification.get("position", []))
            orientation = tuple(float(value) for value in specification.get("orientation_xyzw", []))
            if len(dimensions) != 3 or len(position) != 3 or len(orientation) != 4:
                raise ValueError(f"invalid shelf object: {specification.get('id', '')}")
            relative_position = rotate_vector(
                base_inverse, subtract_vector(position, base_position)
            )
            relative_orientation = quaternion_multiply(base_inverse, orientation)
            collision = CollisionObject()
            collision.header.stamp = self.get_clock().now().to_msg()
            collision.header.frame_id = str(self.get_parameter("base_frame").value)
            collision.id = str(specification["id"])
            collision.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = dimensions
            collision.primitives.append(primitive)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = relative_position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = relative_orientation
            collision.primitive_poses.append(pose)
            objects.append(collision)
        if not objects:
            raise ValueError("shelf scene contains no collision objects")
        return objects

    async def apply_shelf_scene(self, candidate_pose, target_frame: str) -> bool:
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = self.shelf_objects_in_candidate_base(
            candidate_pose, target_frame
        )
        response = await self.scene_client.call_async(request)
        return bool(response is not None and response.success)

    async def clear_shelf_scene(self) -> None:
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        for specification in self.shelf_scene.get("collision_objects", []):
            collision = CollisionObject()
            collision.header.frame_id = str(self.get_parameter("base_frame").value)
            collision.id = str(specification.get("id", ""))
            collision.operation = CollisionObject.REMOVE
            request.scene.world.collision_objects.append(collision)
        await self.scene_client.call_async(request)

    @staticmethod
    def state_positions(state: RobotState, names: List[str]) -> List[float]:
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        if any(name not in positions for name in names):
            return []
        return [float(positions[name]) for name in names]

    @staticmethod
    def joint_margin(state: RobotState, contract: dict) -> float:
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        margins: List[float] = []
        for name, lower, upper in zip(
            contract["joint_names"], contract["joint_lower"], contract["joint_upper"]
        ):
            if name not in positions or float(upper) <= float(lower):
                continue
            normalized = (positions[name] - float(lower)) / (float(upper) - float(lower))
            margins.append(clamp(2.0 * min(normalized, 1.0 - normalized), 0.0, 1.0))
        return min(margins) if margins else 0.0

    async def evaluate_grasp(
        self,
        candidate,
        grasp: GraspCandidate,
        expected_frame: str,
        perturbation: Tuple[float, float, float],
    ) -> Tuple[bool, bool, float, Optional[RobotState], Optional[RobotState]]:
        group = candidate.planning_group
        pregrasp = self.pose_in_candidate_base(
            candidate.base_pose, grasp.pregrasp_pose, expected_frame, perturbation
        )
        pre_ok, pre_state = await self.solve_ik(
            group, candidate.ik_link, pregrasp, None, candidate.lift_seed
        )
        if not pre_ok or pre_state is None:
            return False, False, 0.0, None, None
        pre_valid = await self.collision_free(group, pre_state)
        final_pose = self.pose_in_candidate_base(
            candidate.base_pose, grasp.grasp_pose, expected_frame, perturbation
        )
        grasp_ok, grasp_state = await self.solve_ik(
            group, candidate.ik_link, final_pose, pre_state, candidate.lift_seed
        )
        if not grasp_ok or grasp_state is None:
            return False, False, 0.0, pre_state, None
        grasp_valid = await self.collision_free(group, grasp_state)
        contract = self.group_contracts[group]
        margin = min(
            self.joint_margin(pre_state, contract), self.joint_margin(grasp_state, contract)
        )
        return True, bool(pre_valid and grasp_valid), margin, pre_state, grasp_state

    def perturbations(self, target, requested: bool) -> List[Tuple[float, float, float]]:
        if not requested:
            return []
        covariance = target.pose.covariance
        dx = max(
            math.sqrt(max(0.0, float(covariance[0]))),
            float(self.get_parameter("robust_translation_floor_m").value),
        )
        dy = max(
            math.sqrt(max(0.0, float(covariance[7]))),
            float(self.get_parameter("robust_translation_floor_m").value),
        )
        dyaw = max(
            math.sqrt(max(0.0, float(covariance[35]))),
            float(self.get_parameter("robust_yaw_floor_rad").value),
        )
        samples = [
            (dx, 0.0, 0.0), (-dx, 0.0, 0.0),
            (0.0, dy, 0.0), (0.0, -dy, 0.0),
            (0.0, 0.0, dyaw), (0.0, 0.0, -dyaw),
        ]
        return samples[: max(0, int(self.get_parameter("robustness_samples").value))]

    async def execute(self, goal_handle):
        self.busy = True
        scene_applied = False
        try:
            request = goal_handle.request
            wait = float(self.get_parameter("service_wait_timeout_sec").value)
            if not self.ik_client.wait_for_service(timeout_sec=wait):
                return self.abort(goal_handle, 20, "MoveIt /compute_ik service unavailable")
            if not self.validity_client.wait_for_service(timeout_sec=wait):
                return self.abort(goal_handle, 21, "MoveIt state-validity service unavailable")
            if not self.scene_client.wait_for_service(timeout_sec=wait):
                return self.abort(goal_handle, 26, "MoveIt planning-scene service unavailable")
            if not self.fresh_joint_state():
                return self.abort(goal_handle, 22, "fresh /joint_states required")
            scene_calibrated = bool(self.shelf_scene.get("calibrated", False))
            if bool(self.get_parameter("require_calibrated_scene").value) and not scene_calibrated:
                return self.abort(goal_handle, 27, "shelf_scene.yaml is not calibrated")
            grasp_by_id = {int(item.grasp_id): item for item in request.grasps.grasps}
            for candidate in request.candidates.candidates:
                contract = self.group_contracts[candidate.planning_group]
                if (
                    int(contract.get("active_arm", 0)) != int(request.active_arm)
                    or str(contract.get("ik_link", "")) != candidate.ik_link
                ):
                    return self.abort(
                        goal_handle, 25, "active arm, planning group and IK link mismatch"
                    )
            output = deepcopy(request.candidates)
            output.evaluator_id = "moveit_anygrasp_lift_arm_ik_validity_v3"
            output.physical_evaluation = bool(
                scene_calibrated and request.grasps.physical_perception
            )
            perturbations = self.perturbations(request.target, request.evaluate_robustness)
            started = time.monotonic()
            last_base = None
            total = len(output.candidates)
            for index, candidate in enumerate(output.candidates):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = EvaluateCandidates.Result()
                    result.message = "canceled"
                    return result
                if request.timeout_sec > 0.0 and time.monotonic() - started > request.timeout_sec:
                    return self.abort(goal_handle, 24, "candidate evaluation timeout")
                grasp = grasp_by_id.get(int(candidate.grasp_id))
                if grasp is None:
                    candidate.evaluated = True
                    candidate.rejection_reason = "grasp_id_missing"
                    continue
                base_signature = (
                    candidate.base_pose.position.x,
                    candidate.base_pose.position.y,
                    candidate.base_pose.orientation.z,
                    candidate.base_pose.orientation.w,
                )
                if base_signature != last_base:
                    if not await self.apply_shelf_scene(
                        candidate.base_pose, request.target.header.frame_id
                    ):
                        return self.abort(goal_handle, 28, "failed to apply candidate shelf scene")
                    scene_applied = True
                    last_base = base_signature
                ik_valid, state_valid, margin, pre_state, grasp_state = await self.evaluate_grasp(
                    candidate, grasp, request.target.header.frame_id, (0.0, 0.0, 0.0)
                )
                robust_success = 0
                if state_valid:
                    for perturbation in perturbations:
                        _, valid, _, _, _ = await self.evaluate_grasp(
                            candidate, grasp, request.target.header.frame_id, perturbation
                        )
                        robust_success += int(valid)
                candidate.evaluated = True
                candidate.ik_valid = ik_valid
                candidate.collision_free = state_valid
                candidate.reachability = 1.0 if ik_valid else 0.0
                candidate.joint_margin = margin if state_valid else 0.0
                candidate.robustness = (
                    robust_success / float(len(perturbations))
                    if perturbations else (1.0 if state_valid else 0.0)
                )
                candidate.grasp_quality = grasp.quality
                candidate.visibility_score = grasp.visibility_score
                contract = self.group_contracts[candidate.planning_group]
                names = [str(name) for name in contract["joint_names"]]
                if pre_state is not None and grasp_state is not None:
                    pre_positions = self.state_positions(pre_state, names)
                    grasp_positions = self.state_positions(grasp_state, names)
                    if pre_positions and grasp_positions:
                        candidate.joint_names = names
                        candidate.pregrasp_joint_positions = pre_positions
                        candidate.grasp_joint_positions = grasp_positions
                        candidate.lift_position = grasp_positions[names.index("lift_joint")]
                feedback = EvaluateCandidates.Feedback()
                feedback.phase = "moveit_lift_arm_ik_and_state_validity"
                feedback.evaluated_candidates = index + 1
                feedback.total_candidates = total
                goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result = EvaluateCandidates.Result()
            result.success = True
            result.evaluated_candidates = output
            result.message = "MoveIt lift+arm feasibility evaluation completed"
            return result
        except Exception as error:
            self.get_logger().error(f"MoveIt evaluation raised: {error}")
            return self.abort(goal_handle, 99, str(error))
        finally:
            if scene_applied and self.scene_client.service_is_ready():
                try:
                    await self.clear_shelf_scene()
                except Exception as error:
                    self.get_logger().error(f"failed to clear temporary shelf scene: {error}")
            self.busy = False

    @staticmethod
    def abort(goal_handle, code: int, message: str) -> EvaluateCandidates.Result:
        goal_handle.abort()
        result = EvaluateCandidates.Result()
        result.error_code = code
        result.message = message
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItEvaluatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
