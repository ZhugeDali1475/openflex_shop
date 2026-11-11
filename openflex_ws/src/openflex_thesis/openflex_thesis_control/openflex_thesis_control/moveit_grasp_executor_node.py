"""MoveIt lift+arm grasp executor guarded by a locked, measured-stopped base."""

from copy import deepcopy
import json
import math
import os
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
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionIK, GetStateValidity
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.action import ExecuteMoveItGrasp
from openflex_thesis_msgs.msg import ControlMode, GraspCandidate, GraspStatus, ObjectMemory, PlaceStatus
import rclpy
import yaml
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .async_tools import ros_sleep
from .attached_object import relative_pose, summarize_box
from .scene_contract import scene_readback_matches, shelf_collision_objects
from .validity_diagnostics import failure_category, ros_message, sample_pairing, stamp_ns


class MoveItGraspExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_grasp_executor")
        self.declare_parameter("action_name", "/thesis/execute_moveit_grasp")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
        self.declare_parameter("shelf_scene_file", "")
        self.declare_parameter("require_shelf_scene", True)
        self.declare_parameter("require_calibrated_shelf_scene", True)
        self.declare_parameter("get_planning_scene_service", "/get_planning_scene")
        self.declare_parameter("left_gripper_action", "/left_gripper_controller/gripper_cmd")
        self.declare_parameter("right_gripper_action", "/right_gripper_controller/gripper_cmd")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("simulation_diagnostic_topic", "/thesis_sim/diagnostic_state")
        self.declare_parameter("grasp_status_topic", "/thesis/grasp_status")
        self.declare_parameter("place_status_topic", "/thesis/place/status")
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
        self.declare_parameter("tcp_lookup_timeout_sec", 1.0)
        self.declare_parameter("detach_after_place", True)
        self.declare_parameter("require_transport_start_valid", True)
        self.declare_parameter("object_box_dimensions_m", [0.06, 0.06, 0.12])
        self.declare_parameter(
            "left_touch_links",
            [
                "openarmx_left_link7",
                "openarmx_left_hand",
                "openarmx_left_left_finger",
                "openarmx_left_right_finger",
            ],
        )
        self.declare_parameter(
            "right_touch_links",
            [
                "openarmx_right_link7",
                "openarmx_right_hand",
                "openarmx_right_left_finger",
                "openarmx_right_right_finger",
            ],
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
        self.scene_readback_client = self.create_client(
            GetPlanningScene,
            str(self.get_parameter("get_planning_scene_service").value),
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
            String,
            str(self.get_parameter("simulation_diagnostic_topic").value),
            self.on_simulation_diagnostic,
            qos,
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
        self.create_subscription(
            PlaceStatus,
            str(self.get_parameter("place_status_topic").value),
            self.on_place_status,
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
        self.joint_diagnostic_sample = (None, 0.0)
        self.simulation_diagnostic_sample = (None, 0.0)
        self.attachment_diagnostic = None
        self.goal_target_diagnostic = None
        self.odometry: Optional[Odometry] = None
        self.odometry_receipt = 0.0
        self.grasp_status: Optional[GraspStatus] = None
        self.grasp_status_receipt = 0.0
        self.object_memory: Optional[ObjectMemory] = None
        self.object_memory_receipt = 0.0
        self.active_object_id = ""
        self.active_map_version = 0
        self.object_attached = False
        # The planning scene entry must outlive the grasp action, because the
        # release happens in a different action later in the mission.
        self.scene_attachment = None
        self.pending_detach = None
        self.create_timer(0.2, self.drain_pending_detach, callback_group=self.callback_group)
        self.active_move_group_handle = None
        self.busy = False
        self.shelf_scene = self.load_shelf_scene()

    def on_mode(self, msg: ControlMode) -> None:
        self.control_mode = int(msg.mode)
        if self.control_mode != ControlMode.MOVEIT and self.active_move_group_handle is not None:
            self.active_move_group_handle.cancel_goal_async()

    def on_joint_state(self, msg: JointState) -> None:
        received_at = time.monotonic()
        sample = deepcopy(msg)
        self.joint_state = sample
        self.joint_state_receipt = received_at
        self.joint_diagnostic_sample = (sample, received_at)

    def on_simulation_diagnostic(self, msg: String) -> None:
        try:
            snapshot = json.loads(msg.data)
            if (snapshot.get("schema_version") != 1
                    or snapshot.get("source") != "mujoco_diagnostic_only_NOT_PHYSICAL"
                    or not isinstance(snapshot.get("stamp"), dict)):
                raise ValueError("unknown simulation diagnostic schema")
            stamp_ns(snapshot["stamp"])
            self.simulation_diagnostic_sample = (snapshot, time.monotonic())
        except (ValueError, TypeError, AttributeError, KeyError, OverflowError) as error:
            self.get_logger().warning(f"ignoring simulation diagnostic: {error}")

    def on_odometry(self, msg: Odometry) -> None:
        self.odometry = deepcopy(msg)
        self.odometry_receipt = time.monotonic()

    def on_grasp_status(self, msg: GraspStatus) -> None:
        self.grasp_status = deepcopy(msg)
        self.grasp_status_receipt = time.monotonic()

    def on_place_status(self, msg: PlaceStatus) -> None:
        """Drop the attached box once MuJoCo confirms the release.

        Without this the planning scene keeps the previous trial's grasped
        object forever and later trials collide against stale geometry.
        """
        if not bool(self.get_parameter("detach_after_place").value):
            return
        if not (msg.target_reached and msg.released and msg.verified):
            return
        attachment = self.scene_attachment
        # Critical guard: only detach if we currently have an attached object
        # AND it matches the released object_id AND we're not busy executing
        # a grasp action (which would mean this is a stale message from a
        # previous trial).
        if attachment is None or attachment[0] != msg.object_id:
            return
        # Additional guard: ignore stale messages from previous runs.
        # The bridge publishes place_status for 2 seconds after release,
        # so we need to ensure this matches the current active object.
        if self.active_object_id != msg.object_id:
            return
        # Final guard: only detach if we're not in the middle of a grasp action
        # (busy flag is True during grasp execution).
        if self.busy:
            return
        self.pending_detach = attachment
        self.scene_attachment = None
        self.object_attached = False

    def drain_pending_detach(self) -> None:
        attachment = self.pending_detach
        if attachment is None:
            return
        self.pending_detach = None
        object_id, link, arm = attachment
        request = self.detach_object_request(object_id, link, arm)
        future = self.scene_client.call_async(request)
        future.add_done_callback(
            lambda done: self.on_detached(object_id, done)
        )

    def on_detached(self, object_id: str, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - report, never crash the executor
            self.get_logger().error(f"detach of {object_id} failed: {error}")
            return
        if response is not None and response.success:
            self.get_logger().info(f"removed attached object for {object_id} from the planning scene")
        else:
            self.get_logger().error(f"planning scene refused detach of {object_id}")

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

    def load_shelf_scene(self):
        path = str(self.get_parameter("shelf_scene_file").value)
        if not path:
            return {}
        if not os.path.isfile(path):
            raise ValueError(f"shelf scene file does not exist: {path}")
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}

    def shelf_objects_in_current_base(self):
        if not self.shelf_scene:
            return []
        scene_frame = str(self.shelf_scene.get("frame_id", ""))
        base_frame = str(self.get_parameter("base_frame").value)
        transform = self.tf_buffer.lookup_transform(base_frame, scene_frame, Time())
        return shelf_collision_objects(self.shelf_scene, transform, base_frame)

    async def apply_shelf_scene(self):
        if (self.shelf_scene and bool(self.get_parameter("require_calibrated_shelf_scene").value)
                and not self.shelf_scene.get("calibrated", False)):
            return False
        objects = self.shelf_objects_in_current_base()
        if not objects:
            return not bool(self.get_parameter("require_shelf_scene").value)
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = objects
        response = await self.scene_client.call_async(request)
        if response is None or not response.success:
            return False
        readback = GetPlanningScene.Request()
        readback.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = await self.scene_readback_client.call_async(readback)
        verified = bool(response is not None and scene_readback_matches(
            objects, response.scene.world.collision_objects, self.pose_to_base))
        self.get_logger().info(
            "EXECUTION_SCENE " + json.dumps({"verified": verified,
                "object_ids": [item.id for item in objects], "pose_source": "measured_tf"})
        )
        return verified

    async def clear_shelf_scene(self):
        if not self.shelf_scene:
            return
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

    def pose_to_base(self, source: PoseStamped) -> Optional[PoseStamped]:
        base_frame = str(self.get_parameter("base_frame").value)
        if source.header.frame_id == base_frame:
            self.get_logger().info(
                "G5DBG pose_to_base early-return frame=%s in=(%.4f, %.4f, %.4f)"
                % (
                    source.header.frame_id,
                    source.pose.position.x,
                    source.pose.position.y,
                    source.pose.position.z,
                ),
                throttle_duration_sec=2.0,
            )
            return deepcopy(source)
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, source.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().error(f"cannot transform refined grasp to {base_frame}: {error}")
            return None
        self.get_logger().info(
            "G5DBG pose_to_base frame=%s->%s in=(%.4f, %.4f, %.4f) "
            "T=(%.4f, %.4f, %.4f) q=(%.4f, %.4f, %.4f, %.4f)"
            % (
                source.header.frame_id,
                base_frame,
                source.pose.position.x,
                source.pose.position.y,
                source.pose.position.z,
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
            throttle_duration_sec=2.0,
        )
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
            self.last_validation_failure = "ik_timeout" if response and response.error_code.val == MoveItErrorCodes.TIMED_OUT else "ik_collision"
            return False, None
        validity = GetStateValidity.Request()
        validity.robot_state = response.solution
        validity.group_name = ""
        checked = await self.validity_client.call_async(validity)
        if checked is None or not checked.valid:
            self.last_validation_failure = "collision" if checked else "infrastructure"
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

    async def move_to_state(
        self, group: str, state: RobotState, label: str = "move"
    ) -> bool:
        if not self.hardware_guard_valid():
            return False
        if not await self.apply_shelf_scene():
            self.last_move_failure = "scene_unverified"
            return False
        valid, detail = await self.transportation_start_valid(group, state)
        if not valid:
            self.get_logger().error(f"{label} full-robot validity failed: {detail}")
            self.last_move_failure = "collision"
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
        self.get_logger().info(
            f"{label} target joints: "
            + str(
                {
                    item.joint_name: round(float(item.position), 6)
                    for item in constraints.joint_constraints
                }
            )
        )
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
            success = bool(
                wrapper.status == GoalStatus.STATUS_SUCCEEDED
                and wrapper.result.error_code.val == MoveItErrorCodes.SUCCESS
                and self.hardware_guard_valid()
            )
            self.last_move_failure = (
                "planning_timeout" if wrapper.result.error_code.val == MoveItErrorCodes.TIMED_OUT
                else "planning_or_execution"
            )
            if self.joint_state is not None:
                actual = dict(
                    zip(self.joint_state.name, self.joint_state.position)
                )
                self.get_logger().info(
                    f"{label} result success={success}: "
                    + str(
                        {
                            item.joint_name: round(
                                float(actual.get(item.joint_name, float("nan"))),
                                6,
                            )
                            for item in constraints.joint_constraints
                        }
                    )
                )
            return success
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

    async def attach_object(
        self,
        object_id: str,
        ik_link: str,
        arm: int,
        object_pose: PoseStamped,
        tcp_pose: PoseStamped,
    ) -> bool:
        """Attach the grasp box with the object's real pose relative to the TCP.

        The previous implementation used the identity pose, which put the box's
        long axis on the approach axis and made its back face overlap
        `openarmx_<arm>_link7` by 10.4 mm. MoveIt then rejected the transport
        start state. See `attached_object.py` for the derivation.
        """
        dimensions = [float(value) for value in self.get_parameter("object_box_dimensions_m").value]
        if len(dimensions) != 3 or any(value <= 0.0 for value in dimensions):
            return False
        tcp = tcp_pose.pose
        offset, orientation = relative_pose(
            (tcp.position.x, tcp.position.y, tcp.position.z),
            (tcp.orientation.x, tcp.orientation.y, tcp.orientation.z, tcp.orientation.w),
            (
                object_pose.pose.position.x,
                object_pose.pose.position.y,
                object_pose.pose.position.z,
            ),
            (
                object_pose.pose.orientation.x,
                object_pose.pose.orientation.y,
                object_pose.pose.orientation.z,
                object_pose.pose.orientation.w,
            ),
        )
        attached = AttachedCollisionObject()
        attached.link_name = ik_link
        attached.touch_links = list(
            self.get_parameter("left_touch_links" if arm == 1 else "right_touch_links").value
        )
        attached.object.header.frame_id = ik_link
        attached.object.id = self.attached_object_id(object_id)
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        attached.object.primitives.append(primitive)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = offset
        pose.orientation.x, pose.orientation.y = orientation[0], orientation[1]
        pose.orientation.z, pose.orientation.w = orientation[2], orientation[3]
        attached.object.primitive_poses.append(pose)
        half_size = [0.5 * value for value in dimensions]
        self.get_logger().info(
            "attach {} to {}: {}".format(
                object_id,
                ik_link,
                json.dumps(
                    {
                        "offset_in_link_m": [round(value, 6) for value in offset],
                        "orientation_in_link_xyzw": [round(value, 6) for value in orientation],
                        "box": summarize_box(half_size, orientation),
                    },
                    sort_keys=True,
                ),
            )
        )
        self.attachment_diagnostic = {
            "object_pose_in_base": ros_message(object_pose),
            "tcp_pose_in_base": ros_message(tcp_pose),
            "requested_attachment": ros_message(attached),
        }
        return await self.apply_scene_object(attached, CollisionObject.ADD)

    def attached_object_id(self, object_id: str) -> str:
        return f"thesis_grasped_{object_id}"

    def detach_object_request(self, object_id: str, ik_link: str, arm: int) -> ApplyPlanningScene.Request:
        attached = AttachedCollisionObject()
        attached.link_name = ik_link
        attached.touch_links = list(
            self.get_parameter("left_touch_links" if arm == 1 else "right_touch_links").value
        )
        attached.object.header.frame_id = ik_link
        attached.object.id = self.attached_object_id(object_id)
        attached.object.operation = CollisionObject.REMOVE
        return self.scene_request(attached, CollisionObject.REMOVE)

    async def detach_object(self, object_id: str, ik_link: str, arm: int) -> bool:
        """Remove the attached box so the next trial starts from a clean scene."""
        response = await self.scene_client.call_async(
            self.detach_object_request(object_id, ik_link, arm)
        )
        return bool(response is not None and response.success)

    @staticmethod
    def scene_request(attached: AttachedCollisionObject, operation: int) -> ApplyPlanningScene.Request:
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        attached.object.operation = operation
        request.scene.robot_state.attached_collision_objects.append(attached)
        return request

    async def apply_scene_object(self, attached: AttachedCollisionObject, operation: int) -> bool:
        response = await self.scene_client.call_async(self.scene_request(attached, operation))
        return bool(response is not None and response.success)

    async def transportation_start_valid(self, group: str, target: RobotState) -> Tuple[bool, str]:
        """Check the state the planner will start from, and the transport goal.

        `Motion planning start tree could not be initialized` means the *start*
        state was already invalid, so the current state is checked first. Both
        checks report the colliding body pairs; the planner message does not.
        """
        joint_sample, receipt = self.joint_diagnostic_sample
        simulation, simulation_receipt = self.simulation_diagnostic_sample
        sampled_at = time.monotonic()
        current = RobotState()
        current.is_diff = True
        if joint_sample is not None:
            current.joint_state = deepcopy(joint_sample)
        states = [("current", current), ("transport", deepcopy(target))]
        context = {
            "schema_version": 1,
            "sampled_ros_stamp": ros_message(self.get_clock().now().to_msg()),
            "sampled_monotonic_sec": sampled_at,
            "planning_group": group,
            "object_id": self.active_object_id,
            "map_version": self.active_map_version,
            "failure_stage": self.failure_stage,
            "goal_target": ros_message(self.goal_target_diagnostic),
            "latest_memory": ros_message(self.object_memory),
            "memory_received_age_sec": (
                sampled_at - self.object_memory_receipt if self.object_memory is not None else None),
            "joint_received_age_sec": sampled_at - receipt if joint_sample is not None else None,
            "attachment_input": deepcopy(self.attachment_diagnostic),
            "simulation": deepcopy(simulation),
            "simulation_pairing": sample_pairing(
                joint_sample, simulation,
                sampled_at - simulation_receipt if simulation is not None else None,
                self.active_object_id),
            "sampling_note": "Local samples precede sequential validity queries; scene readback is later. Not an atomic MoveIt snapshot.",
        }
        failures = []
        checks = []
        for label, state in states:
            validity = GetStateValidity.Request()
            validity.group_name = ""
            validity.robot_state = state
            check = {"label": label, "request": ros_message(validity),
                     "started_monotonic_sec": time.monotonic(), "response": None}
            try:
                checked = await self.validity_client.call_async(validity)
                check["response"] = ros_message(checked)
            except Exception as error:
                checked = None
                check["error"] = str(error)
            check["finished_monotonic_sec"] = time.monotonic()
            checks.append(check)
            if checked is None:
                failures.append(f"{label}: validity service returned nothing")
                continue
            if checked.valid:
                continue
            pairs = sorted(
                {
                    f"{contact.contact_body_1}<->{contact.contact_body_2}"
                    for contact in checked.contacts
                }
            )
            failures.append(f"{label}: " + ("; ".join(pairs) if pairs else "invalid without contacts"))
        if failures:
            context["checks"] = checks
            context["failure_category"] = failure_category(checks)
            await self.record_validity_failure(context)
        return (not failures), " | ".join(failures)

    async def record_validity_failure(self, snapshot) -> None:
        try:
            request = GetPlanningScene.Request()
            request.components.components = (
                PlanningSceneComponents.ROBOT_STATE
                | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.TRANSFORMS
                | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                | PlanningSceneComponents.LINK_PADDING_AND_SCALING
            )
            snapshot["scene_readback_started_monotonic_sec"] = time.monotonic()
            future = self.scene_readback_client.call_async(request)
            deadline = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < deadline:
                await ros_sleep(self, 0.02)
            if not future.done():
                future.cancel()
                raise TimeoutError("diagnostic scene readback exceeded 1 second")
            response = future.result()
            snapshot["scene_readback"] = ros_message(response)
            if response is None:
                snapshot["scene_readback_error"] = "service returned nothing"
        except Exception as error:
            snapshot["scene_readback_error"] = str(error)
        snapshot["scene_readback_finished_monotonic_sec"] = time.monotonic()
        self.get_logger().error("VALIDITY_FAILURE_SNAPSHOT " + json.dumps(snapshot))

    async def lookup_tcp_pose(self, ik_link: str) -> Optional[PoseStamped]:
        timeout = float(self.get_parameter("tcp_lookup_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while time.monotonic() <= deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    str(self.get_parameter("base_frame").value), ik_link, Time()
                )
            except TransformException:
                await ros_sleep(self, 0.02)
                continue
            output = PoseStamped()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = str(self.get_parameter("base_frame").value)
            output.pose.position.x = transform.transform.translation.x
            output.pose.position.y = transform.transform.translation.y
            output.pose.position.z = transform.transform.translation.z
            rotation = transform.transform.rotation
            output.pose.orientation.x = rotation.x
            output.pose.orientation.y = rotation.y
            output.pose.orientation.z = rotation.z
            output.pose.orientation.w = rotation.w
            return output
        self.get_logger().error(f"cannot resolve {ik_link} within {timeout:.2f} s")
        return None

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
                # Attaching and moving the target legitimately increments the
                # object-memory map version during this same skill call.
                and status.map_version >= map_version
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
        shelf_scene_applied = False
        retain_shelf_scene = False
        goal = goal_handle.request
        self.grasp_status = None
        self.active_object_id = goal.target.object_id
        self.active_map_version = goal.target.map_version
        self.goal_target_diagnostic = deepcopy(goal.target)
        self.attachment_diagnostic = None
        self.object_attached = False
        self.tested_grasp_ids = []
        self.grasp_failure_reasons = []
        self.last_validation_failure = "ik_collision"
        self.last_move_failure = "planning_or_execution"
        self.failure_stage = "precondition"
        self.selected_grasp_id = 0
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
                and self.scene_readback_client.wait_for_service(timeout_sec=wait)
            )
            gripper = self.left_gripper_client if goal.active_arm == 1 else self.right_gripper_client
            dependencies = dependencies and gripper.wait_for_server(timeout_sec=wait)
            if not dependencies:
                return self.abort(goal_handle, 73, "MoveIt or controller dependency unavailable")
            shelf_scene_applied = await self.apply_shelf_scene()
            if not shelf_scene_applied:
                return self.abort(
                    goal_handle,
                    86,
                    "failed to apply shelf collision scene before grasp execution",
                )
            group = goal.coarse_plan.planning_group
            link = goal.coarse_plan.ik_link
            ordered = sorted(
                goal.refined_grasps.grasps,
                key=lambda item: (-item.quality, item.grasp_id),
            )
            selected = None
            selected_states = None
            for index, grasp in enumerate(ordered):
                self.failure_stage = "validation"
                feedback = ExecuteMoveItGrasp.Feedback()
                feedback.phase = "current_base_ik_collision_validation"
                feedback.tested_grasps = index + 1
                feedback.active_grasp_id = grasp.grasp_id
                feedback.elapsed_sec = float(time.monotonic() - started)
                goal_handle.publish_feedback(feedback)
                pair = await self.feasible_pair(group, link, grasp)
                self.tested_grasp_ids.append(grasp.grasp_id)
                self.grasp_failure_reasons.append("feasible_not_executed" if pair else self.last_validation_failure)
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
                result.recovery_safe = True
                result.failure_stage = "validation"
                result.failure_reason = "ik_collision"
                result.tested_grasp_ids = list(self.tested_grasp_ids)
                result.grasp_failure_reasons = list(self.grasp_failure_reasons)
                return result
            self.selected_grasp_id = selected.grasp_id
            self.get_logger().info(
                f"selected refined grasp={selected.grasp_id} "
                f"quality={selected.quality:.6f}"
            )
            if not await self.command_gripper(
                goal.active_arm, float(self.get_parameter("gripper_open_position").value)
            ):
                return self.abort(goal_handle, 75, "failed to open gripper")
            pre_state, grasp_state = selected_states
            self.failure_stage = "pregrasp"
            if not await self.move_to_state(group, pre_state, "pregrasp"):
                self.get_logger().error("MoveIt pregrasp planning or execution failed")
                return await self.recover_abort(goal_handle, 76, group, goal.active_arm)
            self.failure_stage = "grasp"
            if not await self.move_to_state(group, grasp_state, "grasp"):
                self.get_logger().error("MoveIt final grasp planning or execution failed")
                return await self.recover_abort(goal_handle, 77, group, goal.active_arm)
            self.failure_stage = "verification"
            if not await self.command_gripper(
                goal.active_arm, float(self.get_parameter("gripper_closed_position").value)
            ):
                self.get_logger().error("Right gripper close action failed")
                return self.abort(goal_handle, 78, "failed to close gripper")
            if not await self.wait_for_verification(goal.target.object_id, goal.target.map_version):
                self.get_logger().error("Independent grasp verification timed out")
                status = self.grasp_status
                empty = bool(status is not None and not status.object_retained
                             and not status.contact_detected
                             and time.monotonic() - self.grasp_status_receipt <= 0.5)
                if empty:
                    self.last_move_failure = "grasp_verification"
                    return await self.recover_abort(goal_handle, 79, group, goal.active_arm)
                return self.abort(goal_handle, 79, "independent grasp verification failed")
            # ObjectMemory carries a PoseWithCovariance plus a separate
            # header; pose_to_base() speaks PoseStamped (and needs the frame).
            memory_pose = PoseStamped()
            memory_pose.header = deepcopy(goal.target.header)
            memory_pose.pose = deepcopy(goal.target.pose.pose)
            object_pose = self.pose_to_base(memory_pose)
            if object_pose is None:
                return self.abort(goal_handle, 84, "cannot express the target pose in the base frame")
            tcp_pose = await self.lookup_tcp_pose(link)
            if tcp_pose is None:
                return self.abort(goal_handle, 85, f"cannot resolve attached frame {link}")
            if not await self.attach_object(
                goal.target.object_id, link, goal.active_arm, object_pose, tcp_pose
            ):
                return self.abort(goal_handle, 80, "failed to attach object in planning scene")
            self.object_attached = True
            self.scene_attachment = (goal.target.object_id, link, goal.active_arm)
            self.failure_stage = "transport"
            transport_target = self.transport_state(goal.active_arm, group)
            if bool(self.get_parameter("require_transport_start_valid").value):
                valid, detail = await self.transportation_start_valid(group, transport_target)
                if not valid:
                    await self.detach_object(goal.target.object_id, link, goal.active_arm)
                    self.object_attached = False
                    return self.abort(
                        goal_handle,
                        83,
                        f"transport posture invalid with attached object: {detail}",
                    )
            if not await self.move_to_state(
                group,
                transport_target,
                "transport",
            ):
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
            retain_shelf_scene = True
            return result
        except Exception as error:
            self.get_logger().error(f"MoveIt grasp execution raised: {error}")
            return self.abort(goal_handle, 99, str(error))
        finally:
            if shelf_scene_applied and not retain_shelf_scene and self.scene_client.service_is_ready():
                try:
                    await self.clear_shelf_scene()
                except Exception as error:
                    self.get_logger().error(f"failed to clear grasp shelf scene: {error}")
            self.active_move_group_handle = None
            self.active_object_id = ""
            self.active_map_version = 0
            self.object_attached = False
            self.busy = False

    async def recover_abort(self, goal_handle, code, group, arm):
        reason = self.last_move_failure
        safe = self.hardware_guard_valid() and await self.command_gripper(
            arm, float(self.get_parameter("gripper_open_position").value)
        )
        if safe:
            safe = await self.move_to_state(group, self.transport_state(arm, group), "recovery_safe_park")
        result = self.abort(goal_handle, code, "planning/execution failed; safety park attempted", True)
        result.failure_reason = reason
        result.recovery_safe = bool(safe)
        return result

    def abort(self, goal_handle, code: int, message: str, reposition_required: bool = False):
        goal_handle.abort()
        result = ExecuteMoveItGrasp.Result()
        result.reposition_required = reposition_required
        result.error_code = code
        result.message = message
        result.executed_grasp_id = self.selected_grasp_id
        result.failure_stage = self.failure_stage
        result.failure_reason = "unsafe_or_unverified"
        result.tested_grasp_ids = list(self.tested_grasp_ids)
        result.grasp_failure_reasons = list(self.grasp_failure_reasons)
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
