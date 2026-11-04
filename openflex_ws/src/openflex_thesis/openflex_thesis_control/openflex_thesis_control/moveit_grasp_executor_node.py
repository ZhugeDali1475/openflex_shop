"""MoveIt lift+arm grasp executor guarded by a locked, measured-stopped base."""

from copy import deepcopy
import math
import time
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK, GetStateValidity
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.action import ExecuteMoveItGrasp
from openflex_thesis_msgs.msg import ControlMode, GraspCandidate, GraspStatus, ObjectMemory
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from .async_tools import ros_sleep


class MoveItGraspExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_grasp_executor")
        self.declare_parameter("action_name", "/thesis/execute_moveit_grasp")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
        self.declare_parameter("left_gripper_action", "/left_gripper_controller/gripper_cmd")
        self.declare_parameter("right_gripper_action", "/right_gripper_controller/gripper_cmd")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("grasp_status_topic", "/thesis/grasp_status")
        self.declare_parameter("memory_topic", "/thesis/object_memory")
        self.declare_parameter("control_mode_topic", "/thesis/control_mode")
        self.declare_parameter("measured_velocity_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("joint_state_timeout_sec", 0.5)
        self.declare_parameter("measured_timeout_sec", 0.25)
        self.declare_parameter("memory_timeout_sec", 0.50)
        self.declare_parameter("stopped_linear_mps", 0.02)
        self.declare_parameter("stopped_angular_rps", 0.03)
        self.declare_parameter("server_wait_timeout_sec", 5.0)
        self.declare_parameter("ik_timeout_sec", 0.20)
        self.declare_parameter("planning_time_sec", 5.0)
        self.declare_parameter("planning_attempts", 3)
        self.declare_parameter("velocity_scaling", 0.10)
        self.declare_parameter("acceleration_scaling", 0.10)
        self.declare_parameter("gripper_open_position", 0.044)
        self.declare_parameter("gripper_closed_position", 0.0)
        self.declare_parameter("gripper_max_effort", 20.0)
        self.declare_parameter("verification_timeout_sec", 8.0)
        self.declare_parameter("require_calibrated_object_geometry", True)
        self.declare_parameter("require_physical_plan", True)
        self.declare_parameter("object_geometry_calibrated", False)
        self.declare_parameter("object_box_dimensions_m", [0.06, 0.06, 0.12])
        self.declare_parameter(
            "left_touch_links",
            ["openarmx_left_hand", "openarmx_left_left_finger", "openarmx_left_right_finger"],
        )
        self.declare_parameter(
            "right_touch_links",
            ["openarmx_right_hand", "openarmx_right_left_finger", "openarmx_right_right_finger"],
        )
        self.declare_parameter(
            "left_transport_positions", [-0.10, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            "right_transport_positions", [-0.10, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0]
        )
        self.callback_group = ReentrantCallbackGroup()
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            str(self.get_parameter("move_group_action").value),
            callback_group=self.callback_group,
        )
        self.left_gripper_client = ActionClient(
            self,
            GripperCommand,
            str(self.get_parameter("left_gripper_action").value),
            callback_group=self.callback_group,
        )
        self.right_gripper_client = ActionClient(
            self,
            GripperCommand,
            str(self.get_parameter("right_gripper_action").value),
            callback_group=self.callback_group,
        )
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
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ControlMode,
            str(self.get_parameter("control_mode_topic").value),
            self.on_mode,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.on_joint_state,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("measured_velocity_topic").value),
            self.on_odometry,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            ObjectMemory,
            str(self.get_parameter("memory_topic").value),
            self.on_memory,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            GraspStatus,
            str(self.get_parameter("grasp_status_topic").value),
            self.on_grasp_status,
            20,
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            ExecuteMoveItGrasp,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_mode = ControlMode.HOLD
        self.joint_state: Optional[JointState] = None
        self.joint_state_receipt = 0.0
        self.odometry: Optional[Odometry] = None
        self.odometry_receipt = 0.0
        self.grasp_status: Optional[GraspStatus] = None
        self.grasp_status_receipt = 0.0
        self.object_memory: Optional[ObjectMemory] = None
        self.object_memory_receipt = 0.0
        self.active_object_id = ""
        self.active_map_version = 0
        self.object_attached = False
        self.active_move_group_handle = None
        self.busy = False

    def on_mode(self, msg: ControlMode) -> None:
        self.control_mode = int(msg.mode)

    def on_joint_state(self, msg: JointState) -> None:
        self.joint_state = deepcopy(msg)
        self.joint_state_receipt = time.monotonic()

    def on_odometry(self, msg: Odometry) -> None:
        self.odometry = deepcopy(msg)
        self.odometry_receipt = time.monotonic()

    def on_grasp_status(self, msg: GraspStatus) -> None:
        self.grasp_status = deepcopy(msg)
        self.grasp_status_receipt = time.monotonic()

    def on_memory(self, msg: ObjectMemory) -> None:
        self.object_memory = deepcopy(msg)
        self.object_memory_receipt = time.monotonic()
        if (
            self.active_move_group_handle is not None
            and not self.object_attached
            and not self.memory_contract_valid()
        ):
            self.active_move_group_handle.cancel_goal_async()

    def goal_callback(self, request: ExecuteMoveItGrasp.Goal) -> GoalResponse:
        valid = bool(
            request.target.object_id
            and request.refined_grasps.grasps
            and request.refined_grasps.object_id == request.target.object_id
            and request.refined_grasps.map_version == request.target.map_version
            and request.coarse_plan.object_id == request.target.object_id
            and request.coarse_plan.map_version == request.target.map_version
            and request.coarse_plan.active_arm == request.active_arm
            and request.refined_grasps.stage == GraspCandidate.STAGE_REFINED
            and request.coarse_plan.planning_group.endswith("_arm_with_lift")
            and request.coarse_plan.ready_region.region_id
        )
        return GoalResponse.ACCEPT if not self.busy and valid else GoalResponse.REJECT

    def cancel_callback(self, _request) -> CancelResponse:
        if self.active_move_group_handle is not None:
            self.active_move_group_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def hardware_guard_valid(self) -> bool:
        now = time.monotonic()
        if self.control_mode != ControlMode.MOVEIT:
            return False
        if (
            self.joint_state is None
            or now - self.joint_state_receipt
            > float(self.get_parameter("joint_state_timeout_sec").value)
            or self.odometry is None
            or now - self.odometry_receipt
            > float(self.get_parameter("measured_timeout_sec").value)
        ):
            return False
        twist = self.odometry.twist.twist
        base_guard = bool(
            abs(twist.linear.x) <= float(self.get_parameter("stopped_linear_mps").value)
            and abs(twist.linear.y) <= float(self.get_parameter("stopped_linear_mps").value)
            and abs(twist.angular.z) <= float(self.get_parameter("stopped_angular_rps").value)
        )
        return bool(base_guard and (self.object_attached or self.memory_contract_valid()))

    def memory_contract_valid(self) -> bool:
        memory = self.object_memory
        return bool(
            memory is not None
            and time.monotonic() - self.object_memory_receipt
            <= float(self.get_parameter("memory_timeout_sec").value)
            and memory.object_id == self.active_object_id
            and memory.map_version == self.active_map_version
            and memory.visible
            and memory.grasp_authorized
        )

    @staticmethod
    def quaternion_multiply(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def rotate(q, vector):
        x, y, z = vector
        rotated = MoveItGraspExecutorNode.quaternion_multiply(
            MoveItGraspExecutorNode.quaternion_multiply(q, (x, y, z, 0.0)),
            (-q[0], -q[1], -q[2], q[3]),
        )
        return rotated[:3]

    def pose_to_base(self, source: PoseStamped) -> Optional[PoseStamped]:
        base_frame = str(self.get_parameter("base_frame").value)
        if source.header.frame_id == base_frame:
            return deepcopy(source)
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, source.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().error(f"cannot transform refined grasp to {base_frame}: {error}")
            return None
        tq = transform.transform.rotation
        q_transform = (tq.x, tq.y, tq.z, tq.w)
        p = source.pose.position
        rotated = self.rotate(q_transform, (p.x, p.y, p.z))
        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = base_frame
        output.pose.position.x = rotated[0] + transform.transform.translation.x
        output.pose.position.y = rotated[1] + transform.transform.translation.y
        output.pose.position.z = rotated[2] + transform.transform.translation.z
        q = source.pose.orientation
        result_q = self.quaternion_multiply(q_transform, (q.x, q.y, q.z, q.w))
        output.pose.orientation.x, output.pose.orientation.y, \
            output.pose.orientation.z, output.pose.orientation.w = result_q
        return output

    async def solve_ik(
        self, group: str, link: str, pose: PoseStamped, seed: Optional[RobotState]
    ) -> Tuple[bool, Optional[RobotState]]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = group
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state = deepcopy(seed) if seed is not None else RobotState()
        if seed is None and self.joint_state is not None:
            request.ik_request.robot_state.joint_state = deepcopy(self.joint_state)
            request.ik_request.robot_state.is_diff = True
        timeout = float(self.get_parameter("ik_timeout_sec").value)
        request.ik_request.timeout.sec = int(timeout)
        request.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1.0e9)
        response = await self.ik_client.call_async(request)
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, None
        validity = GetStateValidity.Request()
        validity.robot_state = response.solution
        validity.group_name = group
        checked = await self.validity_client.call_async(validity)
        if checked is None or not checked.valid:
            return False, None
        return True, response.solution

    async def feasible_pair(self, group, link, grasp):
        pregrasp = self.pose_to_base(grasp.pregrasp_pose)
        final = self.pose_to_base(grasp.grasp_pose)
        if pregrasp is None or final is None:
            return None
        pre_ok, pre_state = await self.solve_ik(group, link, pregrasp, None)
        if not pre_ok or pre_state is None:
            return None
        grasp_ok, grasp_state = await self.solve_ik(group, link, final, pre_state)
        if not grasp_ok or grasp_state is None:
            return None
        return pre_state, grasp_state

    async def move_to_state(self, group: str, state: RobotState) -> bool:
        if not self.hardware_guard_valid():
            return False
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = int(self.get_parameter("planning_attempts").value)
        goal.request.allowed_planning_time = float(self.get_parameter("planning_time_sec").value)
        goal.request.max_velocity_scaling_factor = float(self.get_parameter("velocity_scaling").value)
        goal.request.max_acceleration_scaling_factor = float(
            self.get_parameter("acceleration_scaling").value
        )
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        active_prefix = (
            "openarmx_left_joint" if group.startswith("left_") else "openarmx_right_joint"
        )
        constraints = Constraints()
        constraints.name = "validated_lift_arm_ik_target"
        for name, position in positions.items():
            if name != "lift_joint" and not name.startswith(active_prefix):
                continue
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints.append(constraints)
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = await self.move_group_client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return False
        self.active_move_group_handle = handle
        try:
            wrapper = await handle.get_result_async()
            return bool(
                wrapper.status == GoalStatus.STATUS_SUCCEEDED
                and wrapper.result.error_code.val == MoveItErrorCodes.SUCCESS
                and self.hardware_guard_valid()
            )
        finally:
            self.active_move_group_handle = None

    async def command_gripper(self, arm: int, position: float) -> bool:
        client = self.left_gripper_client if arm == 1 else self.right_gripper_client
        if not client.server_is_ready():
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = float(self.get_parameter("gripper_max_effort").value)
        handle = await client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return False
        wrapper = await handle.get_result_async()
        return bool(
            wrapper.status == GoalStatus.STATUS_SUCCEEDED
            and (wrapper.result.reached_goal or wrapper.result.stalled)
        )

    async def attach_object(self, object_id: str, ik_link: str, arm: int) -> bool:
        dimensions = [float(value) for value in self.get_parameter("object_box_dimensions_m").value]
        if len(dimensions) != 3 or any(value <= 0.0 for value in dimensions):
            return False
        attached = AttachedCollisionObject()
        attached.link_name = ik_link
        attached.touch_links = list(
            self.get_parameter("left_touch_links" if arm == 1 else "right_touch_links").value
        )
        attached.object.header.frame_id = ik_link
        attached.object.id = f"thesis_grasped_{object_id}"
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        attached.object.primitives.append(primitive)
        pose = Pose()
        pose.orientation.w = 1.0
        attached.object.primitive_poses.append(pose)
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects.append(attached)
        response = await self.scene_client.call_async(request)
        return bool(response is not None and response.success)

    async def wait_for_verification(self, object_id: str, map_version: int) -> bool:
        started = time.monotonic()
        timeout = float(self.get_parameter("verification_timeout_sec").value)
        last_diagnostic = 0.0
        while time.monotonic() - started <= timeout:
            status = self.grasp_status
            fresh = time.monotonic() - self.grasp_status_receipt <= 0.5
            if (
                fresh
                and status is not None
                and status.object_id == object_id
                and status.map_version == map_version
                and status.verified
                and status.object_retained
            ):
                return True
            now = time.monotonic()
            if now - last_diagnostic >= 1.0:
                last_diagnostic = now
                if status is None:
                    detail = "no GraspStatus received"
                else:
                    detail = (
                        f"fresh={fresh} object={status.object_id} "
                        f"map_version={status.map_version} verified={status.verified} "
                        f"retained={status.object_retained} "
                        f"contact={status.contact_detected} "
                        f"gripper_closed={status.gripper_closed}"
                    )
                self.get_logger().warn(
                    f"grasp verification waiting for object={object_id} "
                    f"map_version={map_version}: {detail}"
                )
            await ros_sleep(self, 0.05)
        return False

    def transport_state(self, arm: int, group: str) -> RobotState:
        prefix = "openarmx_left" if arm == 1 else "openarmx_right"
        positions = list(
            self.get_parameter(
                "left_transport_positions" if arm == 1 else "right_transport_positions"
            ).value
        )
        names = ["lift_joint"] + [f"{prefix}_joint{index}" for index in range(1, 8)]
        if len(positions) != len(names):
            raise ValueError(f"transport target for {group} must contain 8 positions")
        state = RobotState()
        state.joint_state.name = names
        state.joint_state.position = positions
        state.is_diff = True
        return state

    async def execute(self, goal_handle):
        self.busy = True
        started = time.monotonic()
        goal = goal_handle.request
        self.grasp_status = None
        self.active_object_id = goal.target.object_id
        self.active_map_version = goal.target.map_version
        self.object_attached = False
        try:
            if not self.hardware_guard_valid():
                return self.abort(goal_handle, 70, "MoveIt ownership/base-stop guard invalid")
            if goal.require_physical_perception and not goal.refined_grasps.physical_perception:
                return self.abort(goal_handle, 71, "physical refined AnyGrasp result required")
            if goal.require_physical_perception and any(
                not grasp.physical_perception for grasp in goal.refined_grasps.grasps
            ):
                return self.abort(goal_handle, 71, "all refined grasps must be physical")
            if (
                bool(self.get_parameter("require_physical_plan").value)
                and not goal.coarse_plan.physical_evaluation
            ):
                return self.abort(goal_handle, 82, "physical TC-WAM plan required")
            if (
                bool(self.get_parameter("require_calibrated_object_geometry").value)
                and not bool(self.get_parameter("object_geometry_calibrated").value)
            ):
                return self.abort(goal_handle, 72, "attached-object geometry is not calibrated")
            wait = float(self.get_parameter("server_wait_timeout_sec").value)
            dependencies = (
                self.move_group_client.wait_for_server(timeout_sec=wait)
                and self.ik_client.wait_for_service(timeout_sec=wait)
                and self.validity_client.wait_for_service(timeout_sec=wait)
                and self.scene_client.wait_for_service(timeout_sec=wait)
            )
            gripper = self.left_gripper_client if goal.active_arm == 1 else self.right_gripper_client
            dependencies = dependencies and gripper.wait_for_server(timeout_sec=wait)
            if not dependencies:
                return self.abort(goal_handle, 73, "MoveIt or controller dependency unavailable")
            group = goal.coarse_plan.planning_group
            link = goal.coarse_plan.ik_link
            ordered = sorted(
                goal.refined_grasps.grasps,
                key=lambda item: (-item.quality, item.grasp_id),
            )
            selected = None
            selected_states = None
            for index, grasp in enumerate(ordered):
                feedback = ExecuteMoveItGrasp.Feedback()
                feedback.phase = "current_base_ik_collision_validation"
                feedback.tested_grasps = index + 1
                feedback.active_grasp_id = grasp.grasp_id
                feedback.elapsed_sec = float(time.monotonic() - started)
                goal_handle.publish_feedback(feedback)
                pair = await self.feasible_pair(group, link, grasp)
                if pair is not None:
                    selected = grasp
                    selected_states = pair
                    break
            if selected is None or selected_states is None:
                goal_handle.succeed()
                result = ExecuteMoveItGrasp.Result()
                result.reposition_required = True
                result.error_code = 74
                result.message = "no refined grasp is reachable at the current base pose"
                return result
            if not await self.command_gripper(
                goal.active_arm, float(self.get_parameter("gripper_open_position").value)
            ):
                return self.abort(goal_handle, 75, "failed to open gripper")
            pre_state, grasp_state = selected_states
            if not await self.move_to_state(group, pre_state):
                self.get_logger().error("MoveIt pregrasp planning or execution failed")
                return self.abort(goal_handle, 76, "pregrasp planning/execution failed", True)
            if not await self.move_to_state(group, grasp_state):
                self.get_logger().error("MoveIt final grasp planning or execution failed")
                return self.abort(goal_handle, 77, "grasp planning/execution failed", True)
            if not await self.command_gripper(
                goal.active_arm, float(self.get_parameter("gripper_closed_position").value)
            ):
                self.get_logger().error("Right gripper close action failed")
                return self.abort(goal_handle, 78, "failed to close gripper")
            if not await self.wait_for_verification(goal.target.object_id, goal.target.map_version):
                self.get_logger().error("Independent grasp verification timed out")
                return self.abort(goal_handle, 79, "independent grasp verification failed")
            if not await self.attach_object(goal.target.object_id, link, goal.active_arm):
                return self.abort(goal_handle, 80, "failed to attach object in planning scene")
            self.object_attached = True
            if not await self.move_to_state(group, self.transport_state(goal.active_arm, group)):
                return self.abort(goal_handle, 81, "transport-pose execution failed")
            goal_handle.succeed()
            result = ExecuteMoveItGrasp.Result()
            result.success = True
            result.grasp_verified = True
            result.transport_safe = True
            result.executed_grasp_id = selected.grasp_id
            positions = dict(zip(grasp_state.joint_state.name, grasp_state.joint_state.position))
            result.executed_lift_position = float(positions.get("lift_joint", 0.0))
            result.message = "MoveIt jointly executed lift+arm grasp and transport pose"
            return result
        except Exception as error:
            self.get_logger().error(f"MoveIt grasp execution raised: {error}")
            return self.abort(goal_handle, 99, str(error))
        finally:
            self.active_move_group_handle = None
            self.active_object_id = ""
            self.active_map_version = 0
            self.object_attached = False
            self.busy = False

    @staticmethod
    def abort(goal_handle, code: int, message: str, reposition_required: bool = False):
        goal_handle.abort()
        result = ExecuteMoveItGrasp.Result()
        result.reposition_required = reposition_required
        result.error_code = code
        result.message = message
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItGraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=6)
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
