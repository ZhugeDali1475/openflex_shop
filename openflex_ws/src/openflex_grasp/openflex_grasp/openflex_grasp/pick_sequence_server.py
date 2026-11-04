"""Plan and execute a reviewed, single-use real-robot pick sequence."""

import copy
import json
import math
import threading
import time
from uuid import uuid4

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from openarmx_integrated_moveit_msgs.action import CartesianApproach, PlanPregrasp
from openarmx_task_msgs.action import ExecutePick, PlanPick
from openarmx_task_msgs.msg import TaskError
from openflex_grasp_interfaces.action import FitGrasp
from openflex_grasp_interfaces.srv import GetDetectionSnapshot, ReverifyDetection

from .arm_context import arm_context
from .backend_owner import BackendOwner
from .pick_plan_store import PickPlan, PickPlanStore
from .pointcloud_safety import (
    pointcloud_xyz,
    require_pose_above_minimum,
    transformed_minimum_z,
)
from .product_moveit_bridge import transform_pose_via_planar_base


MAX_HEIGHT_OFFSET_M = 0.20
MAX_APPROACH_DISTANCE_M = 0.40
GRIPPER_CLOSED_POSITION_M = 0.0
GRIPPER_OPEN_POSITION_M = 0.044
GRIPPER_STATE_VALID_TOLERANCE_M = 0.005
GRIPPER_RESULT_TOLERANCE_M = 0.002
# A contact result at the fully-open stop is not evidence of a grasp. Require
# motion beyond normal controller/feedback tolerance before accepting a stall.
GRIPPER_MIN_GRASP_CLOSURE_M = 0.003
GRIPPER_ATTACHED_MAX_POSITION_M = round(
    GRIPPER_OPEN_POSITION_M - GRIPPER_MIN_GRASP_CLOSURE_M, 3
)


def validate_height_offset(value, name="height_offset_m"):
    offset = float(value)
    if not math.isfinite(offset):
        raise ValueError(f"{name} must be finite")
    if offset < 0.0 or offset > MAX_HEIGHT_OFFSET_M:
        raise ValueError(
            f"{name} must be between 0 and {MAX_HEIGHT_OFFSET_M:.2f} m"
        )
    return offset


def offset_pose_height(pose, offset_m, name="height_offset_m"):
    result = copy.deepcopy(pose)
    result.pose.position.z += validate_height_offset(offset_m, name)
    return result


def resolve_height_offsets(goal):
    staged = (
        float(goal.initial_raise_m),
        float(goal.pregrasp_height_offset_m),
        float(goal.grasp_height_offset_m),
        float(goal.retreat_height_offset_m),
    )
    if not any(abs(value) > 1e-9 for value in staged):
        legacy = validate_height_offset(goal.height_offset_m)
        staged = (legacy, legacy, legacy, legacy)
    names = (
        "initial_raise_m",
        "pregrasp_height_offset_m",
        "grasp_height_offset_m",
        "retreat_height_offset_m",
    )
    return tuple(
        validate_height_offset(value, name)
        for value, name in zip(staged, names)
    )


def validate_approach_distance(value):
    distance = float(value)
    if not math.isfinite(distance):
        raise ValueError("approach_distance_m must be finite")
    if distance < 0.0 or distance > MAX_APPROACH_DISTANCE_M:
        raise ValueError(
            "approach_distance_m must be zero or between 0.005 and "
            f"{MAX_APPROACH_DISTANCE_M:.2f} m"
        )
    if 0.0 < distance < 0.005:
        raise ValueError("approach_distance_m must be at least 0.005 m")
    return distance


def apply_approach_distance(pregrasp, grasp, requested_distance_m):
    """Keep the grasp fixed and move pregrasp along the current approach line."""
    requested = validate_approach_distance(requested_distance_m)
    delta = (
        float(grasp.pose.position.x - pregrasp.pose.position.x),
        float(grasp.pose.position.y - pregrasp.pose.position.y),
        float(grasp.pose.position.z - pregrasp.pose.position.z),
    )
    current = math.sqrt(sum(value * value for value in delta))
    if current < 1e-6:
        raise ValueError("pregrasp and grasp poses are coincident")
    if requested <= 0.0:
        return copy.deepcopy(pregrasp), current
    direction = tuple(value / current for value in delta)
    adjusted = copy.deepcopy(pregrasp)
    adjusted.pose.position.x = grasp.pose.position.x - direction[0] * requested
    adjusted.pose.position.y = grasp.pose.position.y - direction[1] * requested
    adjusted.pose.position.z = grasp.pose.position.z - direction[2] * requested
    return adjusted, requested


def reverse_arm_trajectory(trajectory, expected_joint_names):
    """Return the time-reversed copy of a collision-checked arm/lift trajectory."""
    result = copy.deepcopy(trajectory)
    joint_trajectory = result.joint_trajectory
    expected = tuple(expected_joint_names)
    if tuple(joint_trajectory.joint_names) != expected or len(expected) not in (7, 8):
        raise RuntimeError(
            "trajectory must contain the selected arm joints, optionally with lift_joint"
        )
    if len(expected) == 8 and expected[0] != "lift_joint":
        raise RuntimeError("8-DOF trajectory must start with lift_joint")
    if result.multi_dof_joint_trajectory.joint_names:
        raise RuntimeError("trajectory unexpectedly contains base motion")
    points = joint_trajectory.points
    if len(points) < 2:
        raise RuntimeError("trajectory has too few points to reverse")
    total_ns = (
        int(points[-1].time_from_start.sec) * 1_000_000_000
        + int(points[-1].time_from_start.nanosec)
    )
    if total_ns <= 0:
        raise RuntimeError("trajectory has no positive duration")
    reversed_points = []
    for source in reversed(points):
        point = copy.deepcopy(source)
        source_ns = (
            int(source.time_from_start.sec) * 1_000_000_000
            + int(source.time_from_start.nanosec)
        )
        target_ns = total_ns - source_ns
        point.time_from_start.sec = target_ns // 1_000_000_000
        point.time_from_start.nanosec = target_ns % 1_000_000_000
        point.velocities = [-float(value) for value in source.velocities]
        # Acceleration is unchanged under time reversal; controller effort is
        # not reused because it is direction/load dependent.
        point.effort = []
        reversed_points.append(point)
    joint_trajectory.points = reversed_points
    return result


class PickSequenceServer(Node):
    def __init__(self):
        super().__init__("openflex_pick_sequence")
        self._owner = BackendOwner("pick_sequence")
        defaults = {
            "plan_ttl_s": 120.0,
            "target_frame": "odom",
            "max_arm_start_error_deg": 1.5,
            "max_lift_start_error_m": 0.003,
            "reverify_center_shift_m": 0.015,
            "reverify_size_change_ratio": 0.15,
            "reverify_mask_iou": 0.65,
            "reverify_valid_depth_ratio": 0.70,
            "cartesian_start_position_tolerance_m": 0.04,
            "cartesian_start_orientation_tolerance_deg": 20.0,
            "object_bottom_clearance_m": 0.01,
            "gripper_open_m": GRIPPER_OPEN_POSITION_M,
            "gripper_attached_min_position_m": 0.002,
            "gripper_attached_max_position_m": GRIPPER_ATTACHED_MAX_POSITION_M,
            "gripper_max_effort": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._plans = PickPlanStore()
        self._busy = threading.Lock()
        self._active_child_lock = threading.Lock()
        self._active_child_goal = None
        self._joints = {}
        # GraspNet can take longer than tf2's 10 s default cache. Keep the
        # frozen RGB-D frame's transform until planning consumes the result.
        self._tf = Buffer(cache_time=Duration(seconds=240.0))
        self._tf_listener = TransformListener(self._tf, self)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self._snapshot = self.create_client(
            GetDetectionSnapshot, "/openflex_grasp/get_detection_snapshot"
        )
        self._reverify = self.create_client(
            ReverifyDetection, "/openflex_grasp/reverify_detection"
        )
        self._fit = ActionClient(self, FitGrasp, "/openflex_grasp/fit_grasp")
        self._pregrasp = ActionClient(self, PlanPregrasp, "/openflex/plan_pregrasp")
        self._cartesian = ActionClient(
            self, CartesianApproach, "/openflex/cartesian_approach"
        )
        self._trajectory = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._grippers = {
            side: ActionClient(self, GripperCommand, arm_context(side).gripper_action)
            for side in ("left", "right")
        }
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._debug_grasp = self.create_publisher(
            String, "/openflex_grasp/debug/grasp_result", debug_qos
        )
        self._debug_execution = self.create_publisher(
            String, "/openflex_grasp/debug/execution_result", debug_qos
        )
        self._debug_path = self.create_publisher(
            NavPath, "/openflex_grasp/debug/grasp_path", debug_qos
        )
        self._plan_action = ActionServer(
            self,
            PlanPick,
            "/plan_pick",
            execute_callback=self._plan,
            goal_callback=self._plan_goal,
            cancel_callback=self._cancel,
        )
        self._execute_action = ActionServer(
            self,
            ExecutePick,
            "/execute_pick",
            execute_callback=self._execute,
            goal_callback=self._execute_goal,
            cancel_callback=self._cancel,
        )
        self.get_logger().info(
            "Pick sequence ready: arm/lift only, cached plan execution"
        )

    def _on_joints(self, message):
        self._joints = dict(zip(message.name, message.position))

    def _plan_goal(self, goal):
        try:
            arm_context(goal.arm_name)
            if int(goal.max_dof) != PlanPick.Goal.ARM_WITH_LIFT:
                return GoalResponse.REJECT
            if not goal.object_id or not goal.detection_id:
                return GoalResponse.REJECT
            resolve_height_offsets(goal)
            validate_approach_distance(goal.approach_distance_m)
        except Exception:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_goal(self, goal):
        try:
            arm_context(goal.arm_name)
            PickPlanStore.validate_id(goal.plan_id)
        except Exception:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, _handle):
        with self._active_child_lock:
            child = self._active_child_goal
        if child is not None:
            child.cancel_goal_async()
        return CancelResponse.ACCEPT

    @staticmethod
    def _feedback(handle, action_type, phase, progress):
        message = action_type.Feedback()
        message.phase, message.progress = phase, float(progress)
        handle.publish_feedback(message)

    async def _plan(self, handle):
        result = PlanPick.Result()
        acquired = False
        try:
            if not self._busy.acquire(blocking=False):
                raise RuntimeError("pick planner/executor is busy")
            acquired = True
            goal = handle.request
            context = arm_context(goal.arm_name)
            self._feedback(handle, PlanPick, "LOAD_DETECTION", 0.10)
            snapshot = await self._snapshot_call(goal.detection_id, False)
            if handle.is_cancel_requested:
                raise RuntimeError("planning canceled after loading detection")
            if snapshot.arm_name != context.arm_name or snapshot.object_id != goal.object_id:
                raise RuntimeError("detection object or arm does not match plan request")
            self._feedback(handle, PlanPick, "GRASPNET", 0.25)
            fit = await self._fit_call(goal, snapshot, handle)
            (
                initial_raise_m,
                pregrasp_height_offset_m,
                grasp_height_offset_m,
                retreat_height_offset_m,
            ) = resolve_height_offsets(goal)
            raw_pregrasp = self._transform_at_detection(fit.pregrasp_pose)
            raw_grasp = self._transform_at_detection(fit.grasp_pose)
            pregrasp = offset_pose_height(
                raw_pregrasp,
                pregrasp_height_offset_m,
                "pregrasp_height_offset_m",
            )
            grasp = offset_pose_height(
                raw_grasp,
                grasp_height_offset_m,
                "grasp_height_offset_m",
            )
            pregrasp, approach_distance_m = apply_approach_distance(
                pregrasp, grasp, goal.approach_distance_m
            )
            retreat_pregrasp = copy.deepcopy(pregrasp)
            retreat_pregrasp.pose.position.z += (
                retreat_height_offset_m - pregrasp_height_offset_m
            )
            minimum_tcp_z = self._minimum_product_tcp_z(snapshot.product_points)
            require_pose_above_minimum(pregrasp, minimum_tcp_z, "pregrasp")
            require_pose_above_minimum(grasp, minimum_tcp_z, "grasp")
            require_pose_above_minimum(
                retreat_pregrasp, minimum_tcp_z, "retreat"
            )
            self._publish_fit_debug(
                goal,
                snapshot.object_pose,
                pregrasp,
                grasp,
                fit,
                "GraspNet 位姿已就绪，正在验证手臂可达性",
            )
            velocity = float(goal.velocity_scale or 0.08)
            acceleration = float(goal.acceleration_scale or 0.08)
            raise_trajectory = None
            lower_trajectory = None
            photo_pose = None
            raised_photo_pose = None
            current_arm_state = None
            raised_state = None
            retreat_matches_pregrasp = math.isclose(
                retreat_height_offset_m,
                pregrasp_height_offset_m,
                abs_tol=1e-9,
            )
            photo_pose = self._lookup_current_tcp_pose(context)
            current_arm_state = self._current_arm_state(context)
            safe_photo_z = max(
                float(photo_pose.pose.position.z) + initial_raise_m,
                minimum_tcp_z,
            )
            effective_raise_m = safe_photo_z - float(photo_pose.pose.position.z)
            if effective_raise_m > 1e-6:
                raised_photo_pose = copy.deepcopy(photo_pose)
                raised_photo_pose.pose.position.z = safe_photo_z
                self._feedback(handle, PlanPick, "PLAN_RAISE", 0.38)
                raise_plan = await self._plan_cartesian(
                    photo_pose,
                    raised_photo_pose,
                    context.arm_name,
                    current_arm_state,
                    velocity,
                    acceleration,
                    handle,
                    label="pregrasp initial raise",
                )
                raised_state = self._trajectory_end_state(raise_plan.trajectory)
                self._feedback(handle, PlanPick, "PLAN_PREGRASP", 0.50)
                pregrasp_plan = await self._plan_cartesian_prefer_straight(
                    raised_photo_pose,
                    pregrasp,
                    context.arm_name,
                    raised_state,
                    velocity,
                    acceleration,
                    handle,
                    label="raised-photo-to-pregrasp transit",
                    minimum_tcp_z=minimum_tcp_z,
                )
                pregrasp_trajectory = pregrasp_plan.trajectory
                pre_state = self._trajectory_end_state(pregrasp_trajectory)
                raise_trajectory = raise_plan.trajectory
                if photo_pose.pose.position.z + 1e-6 >= minimum_tcp_z:
                    lower_trajectory = reverse_arm_trajectory(
                        raise_trajectory, context.planning_joint_names
                    )
            else:
                self._feedback(handle, PlanPick, "PLAN_PREGRASP", 0.45)
                pregrasp_trajectory = await self._plan_pregrasp_prefer_straight(
                    photo_pose,
                    pregrasp,
                    context.arm_name,
                    current_arm_state,
                    goal.max_dof,
                    velocity,
                    acceleration,
                    handle,
                    minimum_tcp_z=minimum_tcp_z,
                )
                pre_state = self._trajectory_end_state(pregrasp_trajectory)
            self._feedback(handle, PlanPick, "PLAN_APPROACH", 0.65)
            approach = await self._plan_cartesian(
                pregrasp,
                grasp,
                context.arm_name,
                pre_state,
                velocity,
                acceleration,
                handle,
                label="pregrasp-to-grasp feed",
                minimum_tcp_z=minimum_tcp_z,
            )
            self._feedback(handle, PlanPick, "PLAN_RETREAT", 0.82)
            if retreat_matches_pregrasp:
                retreat_trajectory = reverse_arm_trajectory(
                    approach.trajectory, context.planning_joint_names
                )
            else:
                grasp_state = self._trajectory_end_state(approach.trajectory)
                retreat_plan = await self._plan_cartesian(
                    grasp,
                    retreat_pregrasp,
                    context.arm_name,
                    grasp_state,
                    velocity,
                    acceleration,
                    handle,
                    label="grasp-to-retreat feed",
                    minimum_tcp_z=minimum_tcp_z,
                )
                retreat_trajectory = retreat_plan.trajectory
            self._feedback(handle, PlanPick, "PLAN_RETURN_PHOTO", 0.90)
            if retreat_matches_pregrasp:
                pregrasp_names = tuple(
                    pregrasp_trajectory.joint_trajectory.joint_names
                )
                if pregrasp_names not in (
                    context.joint_names,
                    context.planning_joint_names,
                ):
                    raise RuntimeError(
                        "pregrasp trajectory contains joints outside the selected arm/lift group"
                    )
                photo_return = reverse_arm_trajectory(
                    pregrasp_trajectory, pregrasp_names
                )
            else:
                retreat_state = self._trajectory_end_state(retreat_trajectory)
                return_pose = raised_photo_pose or photo_pose
                photo_return_plan = await self._plan_cartesian_prefer_straight(
                    retreat_pregrasp,
                    return_pose,
                    context.arm_name,
                    retreat_state,
                    velocity,
                    acceleration,
                    handle,
                    label="retreat-to-photo transit",
                    minimum_tcp_z=minimum_tcp_z,
                    end_joint_positions=(
                        raised_state.position
                        if raised_state is not None
                        else current_arm_state.position
                    ),
                )
                photo_return = photo_return_plan.trajectory
            if not self._joints:
                raise RuntimeError("current joint state is unavailable")
            now = time.monotonic()
            plan_id = str(uuid4())
            self._plans.put(
                PickPlan(
                    plan_id=plan_id,
                    detection_id=goal.detection_id,
                    object_id=goal.object_id,
                    arm_name=context.arm_name,
                    expires_at=now + float(self.get_parameter("plan_ttl_s").value),
                    start_joints=dict(self._joints),
                    pregrasp_pose=copy.deepcopy(pregrasp),
                    grasp_pose=copy.deepcopy(grasp),
                    raise_trajectory=copy.deepcopy(raise_trajectory),
                    pregrasp_trajectory=copy.deepcopy(pregrasp_trajectory),
                    approach_trajectory=copy.deepcopy(approach.trajectory),
                    retreat_trajectory=copy.deepcopy(retreat_trajectory),
                    photo_return_trajectory=copy.deepcopy(photo_return),
                    lower_trajectory=copy.deepcopy(lower_trajectory),
                    required_width_m=float(fit.required_width_m),
                    score=float(fit.score),
                    candidate_count=int(fit.candidate_count),
                )
            )
            result.success, result.plan_id = True, plan_id
            result.pregrasp_pose, result.grasp_pose = pregrasp, grasp
            result.required_width_m, result.score = fit.required_width_m, fit.score
            result.candidate_count = fit.candidate_count
            result.error_code = TaskError.OK
            result.message = (
                f"cached odom plan ready; arm={context.arm_name} max_dof={goal.max_dof} "
                f"raise={effective_raise_m:.3f}m "
                f"pregrasp={pregrasp_height_offset_m:.3f}m "
                f"grasp={grasp_height_offset_m:.3f}m "
                f"retreat={retreat_height_offset_m:.3f}m "
                f"approach_distance={approach_distance_m:.3f}m "
                f"minimum_tcp_z={minimum_tcp_z:.3f}m "
                f"ttl={self.get_parameter('plan_ttl_s').value:.0f}s"
            )
            try:
                self._publish_plan_debug(
                    goal,
                    snapshot.object_pose,
                    pregrasp,
                    grasp,
                    fit,
                    plan_id,
                )
            except Exception as debug_error:
                self.get_logger().warning(
                    f"Pick plan succeeded but debug publish failed: {debug_error}"
                )
            self._publish_execution_debug(
                True, "已规划，等待人工确认执行", "plan_ready", False
            )
            self._feedback(handle, PlanPick, "PLAN_READY", 1.0)
            handle.succeed()
        except Exception as error:
            result.success, result.error_code, result.message = (
                False,
                TaskError.CANCELLED if handle.is_cancel_requested else TaskError.PLANNING_FAILED,
                str(error),
            )
            self.get_logger().error(f"Pick planning aborted: {error}")
            self._publish_execution_debug(False, result.message, "plan_failed", False)
            handle.canceled() if handle.is_cancel_requested else handle.abort()
        finally:
            if acquired:
                self._busy.release()
        return result

    async def _execute(self, handle):
        result = ExecutePick.Result()
        acquired = False
        try:
            if not self._busy.acquire(blocking=False):
                raise RuntimeError("pick planner/executor is busy")
            acquired = True
            goal = handle.request
            plan = self._plans.take(goal.plan_id)
            if plan.object_id != goal.object_id or plan.arm_name != goal.arm_name:
                raise RuntimeError("plan object or arm does not match execute request")
            self._publish_execution_debug(
                True, "正在复核商品与机器人起始状态", "reverify", True
            )
            self._feedback(handle, ExecutePick, "REVERIFY_OBJECT", 0.08)
            verification = await self._reverify_call(plan.detection_id)
            if handle.is_cancel_requested:
                raise RuntimeError("execution canceled during visual reverification")
            if not verification.success:
                raise RuntimeError(f"visual reverification failed: {verification.message}")
            self._validate_robot_start(plan)
            if handle.is_cancel_requested:
                raise RuntimeError("execution canceled before motion")
            at_raised_photo = False
            if plan.raise_trajectory is not None:
                self._feedback(handle, ExecutePick, "EXECUTE_RAISE", 0.14)
                await self._execute_motion(
                    plan.raise_trajectory, handle, "预抓取前上抬"
                )
                at_raised_photo = True
            self._feedback(handle, ExecutePick, "EXECUTE_PREGRASP", 0.25)
            at_raised_photo = False
            await self._execute_motion(
                plan.pregrasp_trajectory, handle, "预抓取"
            )
            gripper = self._grippers[plan.arm_name]
            approached = False
            at_pregrasp = True
            try:
                await self._gripper(
                    gripper,
                    float(self.get_parameter("gripper_open_m").value),
                    handle,
                )
                self._feedback(handle, ExecutePick, "EXECUTE_APPROACH", 0.45)
                at_pregrasp = False
                await self._execute_motion(
                    plan.approach_trajectory, handle, "直线进给"
                )
                approached = True
                self._feedback(handle, ExecutePick, "CLOSE_GRIPPER", 0.65)
                closed = await self._gripper(
                    gripper, GRIPPER_CLOSED_POSITION_M, handle
                )
                closed_position = float(closed.position)
                attached = (
                    bool(closed.stalled)
                    and float(
                        self.get_parameter("gripper_attached_min_position_m").value
                    )
                    <= closed_position
                    <= float(
                        self.get_parameter("gripper_attached_max_position_m").value
                    )
                )
                if not attached:
                    raise RuntimeError("gripper close did not provide object-attached evidence")
            finally:
                if approached:
                    self._feedback(handle, ExecutePick, "SAFE_RETREAT", 0.78)
                    await self._execute_motion(
                        plan.retreat_trajectory, handle, "安全回撤"
                    )
                    at_pregrasp = True
                if at_pregrasp:
                    self._feedback(handle, ExecutePick, "RETURN_PHOTO", 0.90)
                    await self._execute_motion(
                        plan.photo_return_trajectory,
                        handle,
                        "返回抬高拍照位"
                        if plan.lower_trajectory is not None
                        else "返回拍照位",
                    )
                    at_raised_photo = plan.lower_trajectory is not None
                if at_raised_photo:
                    self._feedback(handle, ExecutePick, "LOWER_TO_PHOTO", 0.95)
                    await self._execute_motion(
                        plan.lower_trajectory, handle, "下降回拍照位"
                    )
            await self._snapshot_call(plan.detection_id, True)
            result.success, result.object_attached = True, True
            result.error_code = TaskError.OK
            result.message = (
                f"cached plan {plan.plan_id} executed; object attached and arm "
                "returned to the safe photo-side pose"
            )
            self._publish_execution_debug(True, result.message, "complete", True)
            self._feedback(handle, ExecutePick, "COMPLETE", 1.0)
            handle.succeed()
        except Exception as error:
            result.success, result.object_attached = False, False
            result.error_code = TaskError.CANCELLED if handle.is_cancel_requested else TaskError.PICK_FAILED
            result.message = str(error)
            self._publish_execution_debug(
                False,
                str(error),
                "cancelled" if handle.is_cancel_requested else "failed",
                True,
            )
            handle.canceled() if handle.is_cancel_requested else handle.abort()
        finally:
            if acquired:
                self._busy.release()
        return result

    @staticmethod
    def _pose_payload(pose):
        return {
            "frame_id": pose.header.frame_id,
            "position": [
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            ],
            "orientation": [
                float(pose.pose.orientation.x),
                float(pose.pose.orientation.y),
                float(pose.pose.orientation.z),
                float(pose.pose.orientation.w),
            ],
        }

    def _publish_fit_debug(self, goal, object_pose, pregrasp, grasp, fit, message, plan_id=""):
        payload = {
            "valid": True,
            "message": str(message),
            "detection_id": goal.detection_id,
            "plan_id": plan_id,
            "arm_name": goal.arm_name,
            "object_pose": self._pose_payload(object_pose),
            "pregrasp_pose": self._pose_payload(pregrasp),
            "grasp_pose": self._pose_payload(grasp),
            "required_width_m": float(fit.required_width_m),
            "score": float(fit.score),
            "candidate_count": int(fit.candidate_count),
            "constrained_candidate_count": int(fit.constrained_candidate_count),
        }
        self._debug_grasp.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        path = NavPath()
        path.header = copy.deepcopy(pregrasp.header)
        path.poses = [copy.deepcopy(pregrasp), copy.deepcopy(grasp)]
        self._debug_path.publish(path)

    def _publish_plan_debug(self, goal, object_pose, pregrasp, grasp, fit, plan_id):
        self._publish_fit_debug(
            goal,
            object_pose,
            pregrasp,
            grasp,
            fit,
            "GraspNet 与四段轨迹规划已就绪（含返回拍照位）",
            plan_id,
        )

    def _publish_execution_debug(self, accepted, message, stage, execute):
        try:
            payload = {
                "accepted": bool(accepted),
                "message": str(message),
                "stage": str(stage),
                "execute": bool(execute),
            }
            self._debug_execution.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
        except Exception as debug_error:
            self.get_logger().warning(
                f"Pick sequence debug publish failed: {debug_error}"
            )

    async def _snapshot_call(self, detection_id, consume=False):
        if not self._snapshot.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("detection snapshot service unavailable")
        request = GetDetectionSnapshot.Request()
        request.detection_id, request.consume = detection_id, bool(consume)
        response = await self._snapshot.call_async(request)
        if response is None or not response.success:
            raise RuntimeError(f"detection snapshot failed: {'' if response is None else response.message}")
        return response

    async def _fit_call(self, goal, snapshot, parent_handle):
        if not self._fit.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("resident GraspNet worker unavailable")
        request = FitGrasp.Goal()
        request.detection_id, request.arm_name = goal.detection_id, goal.arm_name
        request.grasp_strategy = goal.grasp_strategy or "graspnet_shelf"
        request.product_points, request.scene_points = snapshot.product_points, snapshot.scene_points
        request.timeout_sec = goal.timeout_sec
        sent = await self._fit.send_goal_async(request)
        if not sent.accepted:
            raise RuntimeError("GraspNet request rejected")
        result = await self._child_result(sent, parent_handle, "GraspNet")
        if not result.success:
            raise RuntimeError(f"GraspNet failed: {result.message}")
        return result

    def _transform_at_detection(self, source):
        target = str(self.get_parameter("target_frame").value)
        try:
            return transform_pose_via_planar_base(
                self._tf,
                source,
                target_frame=target,
                lookup_time=Time.from_msg(source.header.stamp),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise RuntimeError(
                f"detection-time TF {source.header.frame_id}->{target} unavailable: {error}"
            ) from error

    def _lookup_current_tcp_pose(self, context):
        target = str(self.get_parameter("target_frame").value)
        source = PoseStamped()
        source.header.stamp = self.get_clock().now().to_msg()
        source.header.frame_id = context.tcp_frame
        source.pose.orientation.w = 1.0
        try:
            return transform_pose_via_planar_base(
                self._tf,
                source,
                target_frame=target,
                lookup_time=Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise RuntimeError(
                f"current TCP pose {context.tcp_frame}->{target} unavailable: {error}"
            ) from error

    def _minimum_product_tcp_z(self, cloud):
        source_frame = str(cloud.header.frame_id).strip()
        if not source_frame:
            raise RuntimeError("product point cloud has no frame_id")
        target_frame = str(self.get_parameter("target_frame").value).strip()
        transform_target = "base_link" if target_frame == "odom" else target_frame
        points = pointcloud_xyz(cloud)
        if source_frame == transform_target:
            translation = (0.0, 0.0, 0.0)
            rotation = (0.0, 0.0, 0.0, 1.0)
        else:
            try:
                transform = self._tf.lookup_transform(
                    transform_target,
                    source_frame,
                    Time.from_msg(cloud.header.stamp),
                    timeout=Duration(seconds=1.0),
                )
            except TransformException as error:
                raise RuntimeError(
                    f"product-cloud TF {source_frame}->{transform_target} "
                    f"unavailable: {error}"
                ) from error
            t = transform.transform.translation
            q = transform.transform.rotation
            translation = (t.x, t.y, t.z)
            rotation = (q.x, q.y, q.z, q.w)
        clearance = float(self.get_parameter("object_bottom_clearance_m").value)
        if not math.isfinite(clearance) or clearance < 0.0 or clearance > 0.10:
            raise RuntimeError("object_bottom_clearance_m must be in [0.0, 0.10] m")
        return transformed_minimum_z(points, translation, rotation) + clearance

    def _current_arm_state(self, context):
        missing = [
            name for name in context.planning_joint_names if name not in self._joints
        ]
        if missing:
            raise RuntimeError(
                "current arm/lift state is incomplete: " + ", ".join(missing)
            )
        state = JointState()
        state.name = list(context.planning_joint_names)
        state.position = [
            float(self._joints[name]) for name in context.planning_joint_names
        ]
        return state

    async def _plan_pregrasp(
        self, pose, arm, max_dof, velocity, acceleration, parent_handle
    ):
        if not self._pregrasp.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("PlanPregrasp Action unavailable")
        goal = PlanPregrasp.Goal()
        goal.arm = goal.LEFT if arm == "left" else goal.RIGHT
        if int(max_dof) != PlanPregrasp.Goal.ARM_WITH_LIFT:
            raise RuntimeError(
                "grasp pregrasp must use ARM_WITH_LIFT=8; base motion is forbidden"
            )
        goal.max_dof = PlanPregrasp.Goal.ARM_WITH_LIFT
        goal.tcp_pregrasp_pose = pose
        goal.base_yaw_mode = goal.CURRENT_STATE
        goal.velocity_scale, goal.acceleration_scale = velocity, acceleration
        goal.planning_timeout_sec, goal.execute = 8.0, False
        sent = await self._pregrasp.send_goal_async(goal)
        if not sent.accepted:
            raise RuntimeError("PlanPregrasp request rejected")
        result = await self._child_result(sent, parent_handle, "PlanPregrasp")
        if result.error_code.val != 1:
            raise RuntimeError(f"pregrasp planning failed: {result.message}")
        if result.selected_stage not in (result.STAGE_A, result.STAGE_B):
            raise RuntimeError(
                "grasp pregrasp selected whole-body Stage C; base motion is forbidden"
            )
        names = tuple(result.arm_trajectory.joint_trajectory.joint_names)
        context = arm_context(arm)
        expected = (
            context.joint_names
            if result.selected_stage == result.STAGE_A
            else context.planning_joint_names
        )
        if names != expected:
            raise RuntimeError(
                "pregrasp trajectory joints do not match the selected arm/lift stage"
            )
        return result

    async def _plan_pregrasp_prefer_straight(
        self,
        start,
        end,
        arm,
        start_state,
        max_dof,
        velocity,
        acceleration,
        parent_handle,
        *,
        minimum_tcp_z=None,
    ):
        try:
            straight = await self._plan_cartesian(
                start,
                end,
                arm,
                start_state,
                velocity,
                acceleration,
                parent_handle,
                label="photo-to-pregrasp straight transit",
                minimum_tcp_z=minimum_tcp_z,
            )
            return straight.trajectory
        except RuntimeError as straight_error:
            if parent_handle.is_cancel_requested:
                raise
            self.get_logger().warning(
                "Straight photo-to-pregrasp planning failed; falling back to "
                f"height-validated MoveIt: {straight_error}"
            )
            fallback = await self._plan_cartesian(
                start,
                end,
                arm,
                start_state,
                velocity,
                acceleration,
                parent_handle,
                label="photo-to-pregrasp MoveIt fallback",
                path_mode=CartesianApproach.Goal.UNCONSTRAINED_MOVEIT,
                minimum_tcp_z=minimum_tcp_z,
            )
            return fallback.trajectory

    async def _plan_cartesian_prefer_straight(
        self,
        start,
        end,
        arm,
        start_state,
        velocity,
        acceleration,
        parent_handle,
        *,
        label,
        minimum_tcp_z=None,
        end_joint_positions=None,
    ):
        try:
            straight = await self._plan_cartesian(
                start,
                end,
                arm,
                start_state,
                velocity,
                acceleration,
                parent_handle,
                label=f"{label} straight",
                minimum_tcp_z=minimum_tcp_z,
            )
            if end_joint_positions is not None:
                points = straight.trajectory.joint_trajectory.points
                actual = points[-1].positions if points else ()
                expected = tuple(float(value) for value in end_joint_positions)
                if len(actual) != len(expected) or any(
                    abs(float(value) - target) > 1e-3
                    for value, target in zip(actual, expected)
                ):
                    raise RuntimeError(
                        "straight path reaches the TCP pose with a different "
                        "joint posture"
                    )
            return straight
        except RuntimeError as straight_error:
            if parent_handle.is_cancel_requested:
                raise
            self.get_logger().warning(
                f"Straight {label} failed; falling back to standard MoveIt: "
                f"{straight_error}"
            )
            return await self._plan_cartesian(
                start,
                end,
                arm,
                start_state,
                velocity,
                acceleration,
                parent_handle,
                label=f"{label} MoveIt fallback",
                path_mode=CartesianApproach.Goal.UNCONSTRAINED_MOVEIT,
                minimum_tcp_z=minimum_tcp_z,
                end_joint_positions=end_joint_positions,
            )

    async def _plan_cartesian(
        self,
        start,
        end,
        arm,
        start_state,
        velocity,
        acceleration,
        parent_handle,
        label="Cartesian planning",
        path_mode=CartesianApproach.Goal.STRICT_CARTESIAN,
        minimum_tcp_z=None,
        end_joint_positions=None,
    ):
        if not self._cartesian.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("CartesianApproach Action unavailable")
        goal = CartesianApproach.Goal()
        goal.arm = goal.LEFT if arm == "left" else goal.RIGHT
        goal.start_pose, goal.end_pose = start, end
        goal.use_start_joint_state, goal.start_joint_state = True, start_state
        goal.eef_step, goal.min_fraction = 0.005, 0.99
        position_tolerance = float(
            self.get_parameter("cartesian_start_position_tolerance_m").value
        )
        orientation_tolerance = float(
            self.get_parameter("cartesian_start_orientation_tolerance_deg").value
        )
        if not 0.0 < position_tolerance <= 0.10:
            raise RuntimeError(
                "cartesian_start_position_tolerance_m must be in (0.0, 0.10]"
            )
        if not 0.0 < orientation_tolerance <= 45.0:
            raise RuntimeError(
                "cartesian_start_orientation_tolerance_deg must be in (0.0, 45.0]"
            )
        goal.start_position_tolerance = position_tolerance
        goal.start_orientation_tolerance_deg = orientation_tolerance
        goal.velocity_scale, goal.acceleration_scale = velocity, acceleration
        goal.orientation_mode, goal.path_mode = goal.INTERPOLATE, int(path_mode)
        if minimum_tcp_z is not None:
            minimum = float(minimum_tcp_z)
            if not math.isfinite(minimum):
                raise RuntimeError("minimum_tcp_z must be finite")
            goal.use_minimum_tcp_z = True
            goal.minimum_tcp_z = minimum
        if goal.path_mode == goal.UNCONSTRAINED_MOVEIT:
            goal.corridor_planning_timeout = 8.0
            if end_joint_positions is not None:
                goal.end_joint_positions = [
                    float(value) for value in end_joint_positions
                ]
        goal.execute = False
        sent = await self._cartesian.send_goal_async(goal)
        if not sent.accepted:
            raise RuntimeError("Cartesian planning request rejected")
        result = await self._child_result(sent, parent_handle, label)
        if result.error_code.val != 1 or result.path_fraction < 0.99:
            raise RuntimeError(f"{label} failed: {result.message}")
        return result

    async def _child_result(self, sent, parent_handle, label):
        with self._active_child_lock:
            self._active_child_goal = sent
        try:
            wrapped = await sent.get_result_async()
        finally:
            with self._active_child_lock:
                self._active_child_goal = None
        if parent_handle.is_cancel_requested:
            raise RuntimeError(f"{label} canceled")
        return wrapped.result

    @staticmethod
    def _trajectory_end_state(trajectory):
        names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if not names or not points or len(names) != len(points[-1].positions):
            raise RuntimeError("planned trajectory has no valid endpoint state")
        state = JointState()
        state.name, state.position = list(names), list(points[-1].positions)
        return state

    async def _reverify_call(self, detection_id):
        if not self._reverify.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("visual reverification service unavailable")
        request = ReverifyDetection.Request()
        request.detection_id = detection_id
        request.max_center_shift_m = float(self.get_parameter("reverify_center_shift_m").value)
        request.max_size_change_ratio = float(self.get_parameter("reverify_size_change_ratio").value)
        request.min_mask_iou = float(self.get_parameter("reverify_mask_iou").value)
        request.min_valid_depth_ratio = float(self.get_parameter("reverify_valid_depth_ratio").value)
        return await self._reverify.call_async(request)

    def _validate_robot_start(self, plan):
        arm_limit = math.radians(float(self.get_parameter("max_arm_start_error_deg").value))
        lift_limit = float(self.get_parameter("max_lift_start_error_m").value)
        relevant = set(arm_context(plan.arm_name).planning_joint_names)
        for name in relevant:
            if name not in plan.start_joints or name not in self._joints:
                raise RuntimeError(f"required start joint {name} is unavailable")
            limit = lift_limit if name == "lift_joint" else arm_limit
            if abs(float(self._joints[name]) - float(plan.start_joints[name])) > limit:
                raise RuntimeError(f"joint {name} moved since planning; plan invalidated")

    async def _execute_motion(self, trajectory, parent_handle, stage):
        if not self._trajectory.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"{stage}执行失败：MoveIt ExecuteTrajectory Action 不可用")
        names = list(trajectory.joint_trajectory.joint_names)
        points = list(trajectory.joint_trajectory.points)
        if not names or not points or len(names) != len(points[0].positions):
            raise RuntimeError(f"{stage}执行失败：轨迹起点无效")
        deviations = []
        for index, name in enumerate(names):
            if name in self._joints:
                deviations.append(
                    (abs(float(self._joints[name]) - float(points[0].positions[index])), name)
                )
        start_detail = ""
        if deviations:
            error, joint = max(deviations)
            start_detail = f"；发送前最大起点偏差 {joint}={error:.6f} rad"
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        sent = await self._trajectory.send_goal_async(goal)
        if not sent.accepted:
            raise RuntimeError(f"{stage}执行被拒绝{start_detail}")
        with self._active_child_lock:
            self._active_child_goal = sent
        try:
            result = (await sent.get_result_async()).result
        finally:
            with self._active_child_lock:
                self._active_child_goal = None
        if parent_handle.is_cancel_requested:
            raise RuntimeError(f"{stage}执行已取消")
        if result.error_code.val != 1:
            raise RuntimeError(
                f"{stage}轨迹执行失败：MoveIt code {result.error_code.val}{start_detail}"
            )

    async def _gripper(self, client, position, parent_handle):
        if not client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("gripper Action unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter("gripper_max_effort").value)
        sent = await client.send_goal_async(goal)
        if not sent.accepted:
            raise RuntimeError("gripper command rejected")
        with self._active_child_lock:
            self._active_child_goal = sent
        try:
            wrapped = await sent.get_result_async()
        finally:
            with self._active_child_lock:
                self._active_child_goal = None
        if parent_handle.is_cancel_requested:
            raise RuntimeError("execution canceled during gripper command")
        status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"gripper command failed with terminal status {status}")
        result = getattr(wrapped, "result", None)
        if result is None:
            raise RuntimeError("gripper command completed without a result")

        target = float(position)
        actual = float(result.position)
        if (
            not math.isfinite(actual)
            or actual < GRIPPER_CLOSED_POSITION_M - GRIPPER_STATE_VALID_TOLERANCE_M
            or actual > GRIPPER_OPEN_POSITION_M + GRIPPER_STATE_VALID_TOLERANCE_M
        ):
            raise RuntimeError(
                f"gripper command returned invalid position {actual:.4f} m"
            )

        reached = bool(result.reached_goal)
        stalled = bool(result.stalled)
        if target >= GRIPPER_OPEN_POSITION_M - GRIPPER_STATE_VALID_TOLERANCE_M:
            open_error = abs(actual - target)
            if not (
                (reached or stalled)
                and open_error <= GRIPPER_RESULT_TOLERANCE_M
            ):
                raise RuntimeError(
                    "gripper failed to open: "
                    f"target={target:.4f} m, actual={actual:.4f} m, "
                    f"reached={reached}, stalled={stalled}"
                )
        elif target <= GRIPPER_CLOSED_POSITION_M + GRIPPER_STATE_VALID_TOLERANCE_M:
            if stalled and actual > float(
                self.get_parameter("gripper_attached_max_position_m").value
            ):
                raise RuntimeError(
                    "gripper stalled before meaningful closure: "
                    f"actual={actual:.4f} m"
                )
            close_reached = (
                reached and abs(actual - target) <= GRIPPER_RESULT_TOLERANCE_M
            )
            if not close_reached and not stalled:
                raise RuntimeError(
                    "gripper failed to close: "
                    f"target={target:.4f} m, actual={actual:.4f} m, "
                    f"reached={reached}, stalled={stalled}"
                )
        elif not reached or abs(actual - target) > GRIPPER_RESULT_TOLERANCE_M:
            raise RuntimeError(
                "gripper failed to reach intermediate target: "
                f"target={target:.4f} m, actual={actual:.4f} m"
            )
        return result

    def destroy_node(self):
        self._owner.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickSequenceServer()
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


if __name__ == "__main__":
    main()
