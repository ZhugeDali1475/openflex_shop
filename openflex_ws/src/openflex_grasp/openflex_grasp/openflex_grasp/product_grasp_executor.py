from __future__ import annotations

import copy
from collections import Counter
import json
import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from openarmx_integrated_moveit_msgs.action import (
    CartesianApproach,
    ExecuteJointPosture,
    PlanPregrasp,
)

from .product_moveit_bridge import (
    parse_arm,
    parse_bool,
    parse_max_dof,
    transform_pose_via_planar_base,
)


MOVEIT_SUCCESS = 1
MOVEIT_CONTROL_FAILED = -4
MOVEIT_GOAL_TOLERANCE_VIOLATED = -5
MOVEIT_NO_IK_SOLUTION = -31
CARTESIAN_INTERPOLATE = 0
CARTESIAN_ROTATE_BEFORE_TRANSLATION = 1
CARTESIAN_TRANSLATE_BEFORE_ROTATION = 2
CARTESIAN_STRICT = 0
CARTESIAN_BOUNDED_CORRIDOR = 1
CARTESIAN_UNCONSTRAINED_MOVEIT = 2
RIGHT_ARM_JOINTS = tuple(f"openarmx_right_joint{index}" for index in range(1, 8))
LEFT_ARM_JOINTS = tuple(f"openarmx_left_joint{index}" for index in range(1, 8))


def joint_posture_spec(
    arm: str,
    arm_positions,
    *,
    lift_position: float | None = None,
) -> tuple[str, tuple[str, ...], tuple[float, ...]]:
    normalized_arm = str(arm).strip().lower()
    if normalized_arm not in {"left", "right"}:
        raise ValueError("arm must be left or right")
    names = LEFT_ARM_JOINTS if normalized_arm == "left" else RIGHT_ARM_JOINTS
    positions = tuple(float(position) for position in arm_positions)
    if len(positions) != len(names) or not all(
        math.isfinite(position) for position in positions
    ):
        raise ValueError("joint posture requires seven finite arm positions")
    if lift_position is None:
        return f"{normalized_arm}_arm", names, positions
    lift = float(lift_position)
    if not math.isfinite(lift):
        raise ValueError("lift posture position must be finite")
    return (
        f"{normalized_arm}_arm_with_lift",
        ("lift_joint", *names),
        (lift, *positions),
    )


class MotionStateUncertainError(RuntimeError):
    pass


def single_finger_position(
    total_width: float,
    *,
    joint_max_position: float = 0.044,
    opening_scale: float = 2.0,
) -> float:
    width = float(total_width)
    joint_max = float(joint_max_position)
    scale = float(opening_scale)
    max_total_width = joint_max * scale
    if (
        not math.isfinite(width)
        or not math.isfinite(joint_max)
        or not math.isfinite(scale)
        or joint_max <= 0.0
        or scale <= 0.0
        or width < 0.0
        or width > max_total_width
    ):
        raise ValueError(
            f"gripper width must be finite and within [0, {max_total_width}] m; got {width}"
        )
    return width / scale


def robot_gripper_commands(
    required_width: float,
    *,
    joint_max_position: float = 0.044,
    opening_scale: float = 2.0,
    preopen_margin: float = 0.004,
    compression: float = 0.003,
    configured_close_position: float = 0.0,
) -> tuple[float, float, float, float]:
    width = float(required_width)
    joint_max = float(joint_max_position)
    scale = float(opening_scale)
    max_total_width = joint_max * scale
    if (
        not math.isfinite(width)
        or not math.isfinite(joint_max)
        or not math.isfinite(scale)
        or width < 0.0
        or joint_max <= 0.0
        or scale <= 0.0
    ):
        raise ValueError("gripper width and calibration must be finite and non-negative")
    candidate_position = single_finger_position(
        min(width, max_total_width),
        joint_max_position=joint_max,
        opening_scale=scale,
    )
    margin = float(preopen_margin)
    grip_compression = float(compression)
    configured_close = float(configured_close_position)
    if (
        not math.isfinite(margin)
        or not math.isfinite(grip_compression)
        or margin < 0.0
        or grip_compression < 0.0
    ):
        raise ValueError("gripper width margins must be finite and non-negative")

    grasp_width = max(0.0, width - margin)
    if not math.isfinite(configured_close):
        raise ValueError("configured gripper close position must be finite")
    if configured_close >= 0.0:
        if configured_close > float(joint_max_position):
            raise ValueError(
                "configured gripper close position must be within "
                f"[0, {joint_max_position}] m"
            )
        close_position = configured_close
    else:
        close_position = single_finger_position(
            min(max_total_width, max(0.0, grasp_width - grip_compression)),
            joint_max_position=joint_max,
            opening_scale=scale,
        )

    return float(joint_max_position), close_position, candidate_position, grasp_width


def gripper_goal_accepted(
    target: float,
    actual: float,
    *,
    reached_goal: bool,
    stalled: bool,
    accept_stalled: bool = False,
    stalled_tolerance: float | None = None,
) -> bool:
    if reached_goal:
        return True
    if not accept_stalled or not stalled:
        return False
    if stalled_tolerance is None:
        return True
    tolerance = float(stalled_tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("gripper stalled tolerance must be finite and non-negative")
    return abs(float(target) - float(actual)) <= tolerance


def pose_distance(first: PoseStamped, second: PoseStamped) -> float:
    return math.sqrt(
        (first.pose.position.x - second.pose.position.x) ** 2
        + (first.pose.position.y - second.pose.position.y) ** 2
        + (first.pose.position.z - second.pose.position.z) ** 2
    )


def translated_pose(
    source: PoseStamped,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> PoseStamped:
    offsets = (float(x), float(y), float(z))
    if not all(math.isfinite(value) for value in offsets):
        raise ValueError("pose translation offsets must be finite")
    translated = copy.deepcopy(source)
    translated.pose.position.x += offsets[0]
    translated.pose.position.y += offsets[1]
    translated.pose.position.z += offsets[2]
    return translated


def ordered_joint_positions(
    names, positions, expected_names=RIGHT_ARM_JOINTS
) -> tuple[float, ...]:
    if len(names) != len(positions):
        raise ValueError("joint state names and positions have different lengths")
    by_name = {name: float(position) for name, position in zip(names, positions)}
    missing = [name for name in expected_names if name not in by_name]
    if missing:
        raise ValueError(f"joint state is missing: {', '.join(missing)}")
    result = tuple(by_name[name] for name in expected_names)
    if not all(math.isfinite(position) for position in result):
        raise ValueError("joint state contains non-finite right-arm positions")
    return result


def project_pose_onto_line(
    current: PoseStamped,
    line_start: PoseStamped,
    line_end: PoseStamped,
) -> tuple[PoseStamped, float]:
    direction = (
        line_end.pose.position.x - line_start.pose.position.x,
        line_end.pose.position.y - line_start.pose.position.y,
        line_end.pose.position.z - line_start.pose.position.z,
    )
    length_squared = sum(component * component for component in direction)
    if length_squared <= 1e-12:
        raise ValueError("cannot project onto a zero-length approach line")
    offset = (
        current.pose.position.x - line_start.pose.position.x,
        current.pose.position.y - line_start.pose.position.y,
        current.pose.position.z - line_start.pose.position.z,
    )
    progress = sum(offset[index] * direction[index] for index in range(3)) / length_squared
    projected = PoseStamped()
    projected.header = current.header
    projected.pose.position.x = line_start.pose.position.x + progress * direction[0]
    projected.pose.position.y = line_start.pose.position.y + progress * direction[1]
    projected.pose.position.z = line_start.pose.position.z + progress * direction[2]
    projected.pose.orientation = line_start.pose.orientation
    lateral = math.sqrt(
        (current.pose.position.x - projected.pose.position.x) ** 2
        + (current.pose.position.y - projected.pose.position.y) ** 2
        + (current.pose.position.z - projected.pose.position.z) ** 2
    )
    return projected, lateral


def require_path_fraction(path_fraction: float, minimum: float, label: str) -> float:
    fraction = float(path_fraction)
    threshold = float(minimum)
    if not math.isfinite(threshold) or threshold < 0.0 or threshold > 1.0:
        raise ValueError(f"Cartesian minimum fraction must be within [0, 1]; got {minimum}")
    if not math.isfinite(fraction) or fraction + 1e-9 < threshold:
        raise RuntimeError(
            f"{label} returned path_fraction={fraction:.4f}, below required {threshold:.4f}"
        )
    return fraction


def resolve_pregrasp_max_dof(value, grasp_mode: str) -> int:
    max_dof = parse_max_dof(value)
    mode = str(grasp_mode).strip().lower()
    if mode not in {"shelf", "basket"}:
        raise ValueError("grasp_mode must be shelf or basket")
    if max_dof == PlanPregrasp.Goal.AUTO:
        return (
            PlanPregrasp.Goal.ARM_WITH_LIFT
            if mode == "basket"
            else PlanPregrasp.Goal.ARM_ONLY
        )
    if max_dof not in {
        PlanPregrasp.Goal.ARM_ONLY,
        PlanPregrasp.Goal.ARM_WITH_LIFT,
    }:
        raise ValueError("pregrasp_max_dof must be auto, 7, or 8; base motion is forbidden")
    return max_dof


def require_pregrasp_stage(selected_stage: int, max_dof: int, label: str) -> None:
    stage = int(selected_stage)
    allowed = {int(PlanPregrasp.Result.STAGE_A)}
    if int(max_dof) >= int(PlanPregrasp.Goal.ARM_WITH_LIFT):
        allowed.add(int(PlanPregrasp.Result.STAGE_B))
    if stage not in allowed:
        raise RuntimeError(
            f"{label} selected forbidden MoveIt stage {selected_stage} for max_dof={max_dof}"
        )


def require_arm_only_stage(selected_stage: int, label: str) -> None:
    if int(selected_stage) != int(PlanPregrasp.Result.STAGE_A):
        raise RuntimeError(
            f"{label} selected MoveIt stage {selected_stage}; only right-arm STAGE_A is allowed"
        )


def bounded_corridor_max_path_length(
    direct_distance: float,
    *,
    path_scale: float = 2.5,
    extra_path: float = 0.10,
) -> float:
    distance = float(direct_distance)
    scale = float(path_scale)
    extra = float(extra_path)
    if (
        not math.isfinite(distance)
        or not math.isfinite(scale)
        or not math.isfinite(extra)
        or distance <= 0.0
        or scale < 1.0
        or extra < 0.0
    ):
        raise ValueError("bounded-corridor path limits are invalid")
    return max(distance * scale, distance + extra)


def vertical_lift_feed_target(
    current_lift: float,
    tcp_delta_z: float,
    *,
    speed_m_s: float = 0.04,
    min_position_m: float = -0.65,
    max_position_m: float = 0.30,
) -> tuple[float, float]:
    current = float(current_lift)
    delta = float(tcp_delta_z)
    speed = float(speed_m_s)
    minimum = float(min_position_m)
    maximum = float(max_position_m)
    if not all(math.isfinite(value) for value in (current, delta, speed, minimum, maximum)):
        raise ValueError("lift feed values must be finite")
    if speed <= 0.0 or minimum >= maximum:
        raise ValueError("lift feed speed and limits are invalid")
    target = current + delta
    if target < minimum or target > maximum:
        raise ValueError(
            f"lift feed target {target:.4f} m is outside [{minimum:.4f}, {maximum:.4f}] m"
        )
    duration = max(0.25, abs(delta) / speed)
    return target, duration


def reduced_basket_descent_delta(
    nominal_delta_z: float,
    reduction_m: float,
) -> float:
    delta = float(nominal_delta_z)
    reduction = float(reduction_m)
    if not math.isfinite(delta) or not math.isfinite(reduction):
        raise ValueError("basket descent values must be finite")
    if reduction < 0.0:
        raise ValueError("basket descent reduction must be non-negative")
    if delta > 1e-6:
        raise ValueError(
            f"basket grasp feed must descend vertically; nominal delta is {delta:.4f} m"
        )
    nominal_descent = max(0.0, -delta)
    return -max(0.0, nominal_descent - reduction)


def duplicate_grasp_pipeline_nodes(nodes_and_namespaces) -> tuple[str, ...]:
    monitored = {
        "/product_pointcloud",
        "/straight_grasp",
        "/right_product_grasp_executor",
    }
    names = []
    for name, namespace in nodes_and_namespaces:
        prefix = str(namespace).rstrip("/")
        full_name = f"{prefix}/{name}" if prefix else f"/{name}"
        names.append(full_name)
    counts = Counter(names)
    return tuple(sorted(name for name in monitored if counts[name] > 1))


class ProductGraspExecutorNode(Node):
    def __init__(self):
        super().__init__("product_grasp_executor")
        self.declare_parameter("enabled", False)
        self.declare_parameter("execute", False)
        self.declare_parameter("arm", "right")
        self.declare_parameter("grasp_mode", "shelf")
        self.declare_parameter("product", "")
        self.declare_parameter("target_topic", "/product_perception/target")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("tcp_frame", "openarmx_right_hand_tcp")
        self.declare_parameter("cartesian_action", "/openflex/cartesian_approach")
        self.declare_parameter("plan_action", "/openflex/plan_pregrasp")
        self.declare_parameter(
            "joint_posture_action", "/openflex/execute_joint_posture"
        )
        self.declare_parameter(
            "lift_trajectory_action", "/lift_joint_controller/follow_joint_trajectory"
        )
        self.declare_parameter("gripper_action", "/right_gripper_controller/gripper_cmd")
        self.declare_parameter("grasp_result_topic", "/product_perception/grasp_result")
        self.declare_parameter("pregrasp_topic", "/product_perception/pregrasp_pose")
        self.declare_parameter("grasp_topic", "/product_perception/grasp_pose")
        self.declare_parameter("gripper_width_topic", "/product_perception/gripper_width")
        self.declare_parameter("send_once", True)
        self.declare_parameter("use_latest_transform", True)
        self.declare_parameter("transform_timeout_s", 1.0)
        self.declare_parameter("action_wait_timeout_s", 30.0)
        self.declare_parameter("action_result_timeout_s", 90.0)
        self.declare_parameter("pregrasp_planning_attempts", 2)
        self.declare_parameter("photo_pose_timeout_s", 15.0)
        self.declare_parameter("perception_wait_timeout_s", 300.0)
        self.declare_parameter("camera_settle_s", 1.0)
        self.declare_parameter("return_to_photo", True)
        self.declare_parameter("postgrasp_lift_m", 0.01)
        self.declare_parameter("basket_lift_feed_speed_m_s", 0.04)
        self.declare_parameter("basket_final_descent_reduction_m", 0.0)
        self.declare_parameter("lift_min_position_m", -0.65)
        self.declare_parameter("lift_max_position_m", 0.30)
        self.declare_parameter("pregrasp_max_dof", "auto")
        self.declare_parameter("unconstrained_moveit", True)
        self.declare_parameter("prefer_straight_grasp_feed", True)
        self.declare_parameter("cartesian_eef_step_m", 0.005)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_min_fraction", 0.99)
        self.declare_parameter("cartesian_start_position_tolerance_m", 0.015)
        self.declare_parameter("basket_pregrasp_position_tolerance_m", 0.015)
        self.declare_parameter("cartesian_start_orientation_tolerance_deg", 10.0)
        self.declare_parameter("cartesian_velocity_scale", 0.08)
        self.declare_parameter("cartesian_acceleration_scale", 0.08)
        self.declare_parameter("photo_pregrasp_corridor_radius_m", 0.06)
        self.declare_parameter("corridor_max_path_scale", 2.5)
        self.declare_parameter("corridor_max_extra_path_m", 0.10)
        self.declare_parameter("corridor_max_backtracking_m", 0.03)
        self.declare_parameter("corridor_planning_timeout_s", 8.0)
        self.declare_parameter("straight_line_tolerance_m", 0.005)
        self.declare_parameter("gripper_joint_max_position_m", 0.043)
        self.declare_parameter("gripper_open_stall_tolerance_m", 0.002)
        self.declare_parameter("gripper_opening_scale", 2.0)
        self.declare_parameter("gripper_preopen_margin_m", 0.004)
        self.declare_parameter("gripper_grip_compression_m", 0.003)
        self.declare_parameter("gripper_close_position_m", 0.0)
        self.declare_parameter("gripper_max_effort", 10.0)
        self.declare_parameter("payload_compensation_enabled", False)
        self.declare_parameter("gravity_comp_node", "/gravity_comp_node")
        self.declare_parameter("payload_mass_kg", 0.55)
        self.declare_parameter("payload_com_x_m", 0.0)
        self.declare_parameter("payload_com_y_m", 0.0)
        self.declare_parameter("payload_com_z_m", 0.05)
        self.declare_parameter("payload_parameter_timeout_s", 5.0)

        self._lock = threading.Lock()
        self._pregrasp: PoseStamped | None = None
        self._grasp: PoseStamped | None = None
        self._width: float | None = None
        self._worker_active = False
        self._completed = False
        self._valid_grasp_ready = False
        self._startup_ready = False
        self._photo_pose: PoseStamped | None = None
        self._photo_joint_positions: tuple[float, ...] | None = None
        self._photo_lift_position: float | None = None
        self._right_joint_positions: tuple[float, ...] | None = None
        self._lift_position: float | None = None

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cartesian_client = ActionClient(
            self, CartesianApproach, str(self.get_parameter("cartesian_action").value)
        )
        self._pregrasp_client = ActionClient(
            self, PlanPregrasp, str(self.get_parameter("plan_action").value)
        )
        self._joint_posture_client = ActionClient(
            self,
            ExecuteJointPosture,
            str(self.get_parameter("joint_posture_action").value),
        )
        self._lift_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("lift_trajectory_action").value),
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, str(self.get_parameter("gripper_action").value)
        )
        gravity_node = str(self.get_parameter("gravity_comp_node").value).rstrip("/")
        self._gravity_parameter_client = self.create_client(
            SetParameters, f"{gravity_node}/set_parameters"
        )
        self._result_pub = self.create_publisher(
            String, "/product_perception/grasp_execution_result", 10
        )
        self._target_pub = self.create_publisher(
            String, str(self.get_parameter("target_topic").value), 10
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pregrasp_topic").value),
            self._on_pregrasp,
            1,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("grasp_topic").value),
            self._on_grasp,
            1,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("gripper_width_topic").value),
            self._on_width,
            1,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("grasp_result_topic").value),
            self._on_result,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 30)
        self.get_logger().info(
            f"Product grasp executor ready: enabled={self._bool_param('enabled')}, "
            f"execute={self._bool_param('execute')}"
        )
        if self._bool_param("enabled"):
            threading.Thread(
                target=self._prepare_capture,
                name="product-grasp-prepare",
                daemon=True,
            ).start()

    def _bool_param(self, name: str) -> bool:
        return parse_bool(self.get_parameter(name).value)

    def _on_pregrasp(self, message: PoseStamped):
        with self._lock:
            self._pregrasp = message
        self._try_start_worker()

    def _on_joint_state(self, message: JointState):
        if "lift_joint" in message.name:
            index = message.name.index("lift_joint")
            if index < len(message.position):
                with self._lock:
                    self._lift_position = float(message.position[index])
        try:
            positions = ordered_joint_positions(message.name, message.position)
        except ValueError:
            return
        with self._lock:
            self._right_joint_positions = positions

    def _on_grasp(self, message: PoseStamped):
        with self._lock:
            self._grasp = message
        self._try_start_worker()

    def _on_width(self, message: Float32):
        with self._lock:
            self._width = float(message.data)
        self._try_start_worker()

    def _on_result(self, message: String):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as error:
            self._publish("rejected", False, f"invalid grasp_result JSON: {error}")
            return
        if not payload.get("valid", False):
            self._publish("rejected", False, f"invalid grasp: {payload.get('reason', 'unknown')}")
            return
        if payload.get("backend") != "graspnet":
            self._publish("rejected", False, "grasp backend is not graspnet")
            return
        expected_mode = str(self.get_parameter("grasp_mode").value).strip().lower()
        if payload.get("grasp_mode") != expected_mode:
            self._publish(
                "rejected",
                False,
                f"grasp mode mismatch: expected {expected_mode}, got {payload.get('grasp_mode')}",
            )
            return
        if not self._bool_param("enabled"):
            self._publish("disarmed", True, "executor is disabled")
            return

        with self._lock:
            self._valid_grasp_ready = True
        self._try_start_worker()

    def _try_start_worker(self):
        with self._lock:
            if (
                not self._startup_ready
                or not self._valid_grasp_ready
                or self._worker_active
                or (self._completed and self._bool_param("send_once"))
            ):
                return
            if (
                self._photo_pose is None
                or self._photo_joint_positions is None
                or self._pregrasp is None
                or self._grasp is None
                or self._width is None
                or (
                    str(self.get_parameter("grasp_mode").value).strip().lower()
                    == "basket"
                    and self._photo_lift_position is None
                )
            ):
                return
            pregrasp = self._copy_pose(self._pregrasp)
            grasp = self._copy_pose(self._grasp)
            photo_pose = self._copy_pose(self._photo_pose)
            photo_joint_positions = tuple(self._photo_joint_positions)
            photo_lift_position = self._photo_lift_position
            width = self._width
            self._valid_grasp_ready = False
            self._worker_active = True

        thread = threading.Thread(
            target=self._run,
            args=(
                pregrasp,
                grasp,
                width,
                photo_pose,
                photo_joint_positions,
                photo_lift_position,
            ),
            name="product-grasp-executor",
            daemon=True,
        )
        thread.start()

    def _prepare_capture(self):
        try:
            if parse_arm(self.get_parameter("arm").value) != PlanPregrasp.Goal.RIGHT:
                raise RuntimeError("product grasp execution is restricted to the right arm")
            pregrasp_max_dof = resolve_pregrasp_max_dof(
                self.get_parameter("pregrasp_max_dof").value,
                str(self.get_parameter("grasp_mode").value),
            )

            product = str(self.get_parameter("product").value).strip()
            if not product:
                raise RuntimeError("product is empty; pass product:='<商品名称>'")
            if self._bool_param("execute") and any(
                name == "openarmx_freedrive"
                for name, _namespace in self.get_node_names_and_namespaces()
            ):
                raise RuntimeError(
                    "free-drive is active; stop /openarmx_freedrive with Ctrl-C "
                    "before executing a grasp"
                )

            photo_joint_positions = self._lookup_current_right_joint_positions()
            basket_mode = (
                str(self.get_parameter("grasp_mode").value).strip().lower()
                == "basket"
            )
            photo_lift_position = (
                self._lookup_current_lift_position()
                if pregrasp_max_dof >= PlanPregrasp.Goal.ARM_WITH_LIFT
                else None
            )
            photo_pose = self._lookup_current_tcp_pose()
            with self._lock:
                self._photo_pose = photo_pose
                self._photo_joint_positions = photo_joint_positions
                self._photo_lift_position = photo_lift_position
            self._publish(
                "photo_pose_registered",
                True,
                "current right-arm TCP pose registered as the photo pose",
                frame_id=photo_pose.header.frame_id,
                tcp_frame=str(self.get_parameter("tcp_frame").value),
                right_arm_joint_positions=list(photo_joint_positions),
                lift_joint_position=photo_lift_position,
            )

            wait_timeout = float(self.get_parameter("action_wait_timeout_s").value)
            if not self._cartesian_client.wait_for_server(timeout_sec=wait_timeout):
                raise RuntimeError("right-arm Cartesian action server is unavailable")
            if (
                pregrasp_max_dof >= PlanPregrasp.Goal.ARM_WITH_LIFT
                and not self._pregrasp_client.wait_for_server(timeout_sec=wait_timeout)
            ):
                raise RuntimeError("arm-and-lift pregrasp action server is unavailable")
            if (
                self._bool_param("execute")
                and self._bool_param("return_to_photo")
                and not self._joint_posture_client.wait_for_server(
                    timeout_sec=wait_timeout
                )
            ):
                raise RuntimeError("joint posture action server is unavailable")
            if (
                str(self.get_parameter("grasp_mode").value).strip().lower() == "basket"
                and self._bool_param("execute")
                and not self._lift_client.wait_for_server(timeout_sec=wait_timeout)
            ):
                raise RuntimeError("lift trajectory action server is unavailable")
            if self._bool_param("execute"):
                if not self._gripper_client.wait_for_server(timeout_sec=wait_timeout):
                    raise RuntimeError("right gripper action server is unavailable")
                joint_max = float(self.get_parameter("gripper_joint_max_position_m").value)
                self._publish(
                    "gripper_open",
                    True,
                    "fully opening right gripper before taking the photo",
                    gripper_target_position_m=joint_max,
                )
                self._send_gripper(
                    joint_max,
                    accept_stalled=True,
                    stalled_tolerance=float(
                        self.get_parameter("gripper_open_stall_tolerance_m").value
                    ),
                )
                self._set_payload_compensation(0.0)

            self._wait_for_perception_pipeline()
            settle = max(0.0, float(self.get_parameter("camera_settle_s").value))
            if settle > 0.0:
                time.sleep(settle)

            with self._lock:
                self._pregrasp = None
                self._grasp = None
                self._width = None
                self._valid_grasp_ready = False
                self._startup_ready = True
            message = String()
            message.data = product
            self._target_pub.publish(message)
            self._publish(
                "perception_requested",
                True,
                f"photo pose confirmed; requested GraspNet grasp for {product!r}",
                product=product,
            )
        except Exception as error:
            self._publish("startup_aborted", False, str(error))
            self.get_logger().error(f"Product grasp startup aborted: {error}")

    def _run(
        self,
        pregrasp: PoseStamped,
        grasp: PoseStamped,
        width: float,
        photo_pose: PoseStamped,
        photo_joint_positions: tuple[float, ...],
        photo_lift_position: float | None,
    ):
        success = False
        execute = self._bool_param("execute")
        photo_odom = None
        pregrasp_odom = None
        grasp_odom = None
        known_pose = None
        object_grasped = False
        pregrasp_max_dof = PlanPregrasp.Goal.ARM_ONLY
        basket_mode = False
        try:
            if pregrasp.header.frame_id != grasp.header.frame_id:
                raise RuntimeError("pregrasp and grasp poses use different frames")
            if self._stamp(pregrasp) != self._stamp(grasp):
                raise RuntimeError("pregrasp and grasp poses are from different sensor frames")
            joint_max = float(self.get_parameter("gripper_joint_max_position_m").value)
            opening_scale = float(self.get_parameter("gripper_opening_scale").value)
            open_position, close_position, candidate_position, grasp_width = (
                robot_gripper_commands(
                    width,
                    joint_max_position=joint_max,
                    opening_scale=opening_scale,
                    preopen_margin=float(
                        self.get_parameter("gripper_preopen_margin_m").value
                    ),
                    compression=float(
                        self.get_parameter("gripper_grip_compression_m").value
                    ),
                    configured_close_position=float(
                        self.get_parameter("gripper_close_position_m").value
                    ),
                )
            )
            photo_odom = self._transform_pose(photo_pose)
            pregrasp_odom = self._transform_pose(pregrasp)
            grasp_odom = self._transform_pose(grasp)
            known_pose = photo_odom
            approach_distance = pose_distance(pregrasp_odom, grasp_odom)
            pregrasp_max_dof = resolve_pregrasp_max_dof(
                self.get_parameter("pregrasp_max_dof").value,
                str(self.get_parameter("grasp_mode").value),
            )
            basket_mode = (
                str(self.get_parameter("grasp_mode").value).strip().lower()
                == "basket"
            )
            basket_feed_delta_z = None
            if basket_mode:
                basket_feed_delta_z = reduced_basket_descent_delta(
                    grasp_odom.pose.position.z - pregrasp_odom.pose.position.z,
                    float(
                        self.get_parameter(
                            "basket_final_descent_reduction_m"
                        ).value
                    ),
                )
                approach_distance = abs(basket_feed_delta_z)
            use_lift_planning = (
                pregrasp_max_dof >= PlanPregrasp.Goal.ARM_WITH_LIFT
            )

            unconstrained_moveit = self._bool_param("unconstrained_moveit")
            transit_path_mode = (
                CARTESIAN_UNCONSTRAINED_MOVEIT
                if unconstrained_moveit
                else CARTESIAN_BOUNDED_CORRIDOR
            )
            feed_path_mode = (
                CARTESIAN_STRICT
                if self._bool_param("prefer_straight_grasp_feed")
                else CARTESIAN_UNCONSTRAINED_MOVEIT
                if unconstrained_moveit
                else CARTESIAN_STRICT
            )
            allow_feed_fallback = (
                self._bool_param("prefer_straight_grasp_feed")
                and unconstrained_moveit
            )
            line_tolerance = None
            if not unconstrained_moveit:
                line_tolerance = float(
                    self.get_parameter("straight_line_tolerance_m").value
                )
                if not math.isfinite(line_tolerance) or line_tolerance <= 0.0:
                    raise RuntimeError(
                        "straight_line_tolerance_m must be finite and positive"
                    )

            photo_pregrasp_plan_fraction = None
            photo_pregrasp_stage = None
            lift_target = None
            lift_duration = None
            if not execute:
                self._publish(
                    "photo_to_pregrasp_preflight",
                    True,
                    "planning the arm-and-lift MoveIt path"
                    if use_lift_planning
                    else "planning the standard unconstrained right-arm MoveIt path"
                    if unconstrained_moveit
                    else "checking the bounded right-arm TCP corridor from photo pose to pregrasp",
                )
                if use_lift_planning:
                    pregrasp_result = self._checked_pregrasp(
                        pregrasp_odom,
                        max_dof=pregrasp_max_dof,
                        execute=False,
                        label="photo-to-pregrasp preflight",
                    )
                    photo_pregrasp_stage = int(pregrasp_result.selected_stage)
                    if basket_mode:
                        planned_basket_grasp = translated_pose(
                            pregrasp_odom,
                            z=basket_feed_delta_z,
                        )
                        planned_lift = (
                            float(pregrasp_result.lift_goal)
                            if photo_pregrasp_stage
                            == int(PlanPregrasp.Result.STAGE_B)
                            else self._lookup_current_lift_position()
                        )
                        lift_target, lift_duration = self._checked_basket_lift_feed(
                            pregrasp_odom,
                            planned_basket_grasp,
                            execute=False,
                            label="basket pregrasp-to-grasp preflight",
                            start_lift_position=planned_lift,
                        )
                    else:
                        lift_target = None
                        lift_duration = None
                else:
                    photo_pregrasp_plan_fraction = self._checked_cartesian(
                        photo_odom,
                        pregrasp_odom,
                        execute=False,
                        label="photo-to-pregrasp preflight",
                        path_mode=transit_path_mode,
                    )
                self._publish(
                    "plan_only_complete",
                    True,
                    "arm-and-lift MoveIt path is reachable"
                    if use_lift_planning
                    else "standard unconstrained right-arm MoveIt path is reachable"
                    if unconstrained_moveit
                    else "bounded photo-to-pregrasp corridor is reachable; final straight feed is checked after execution reaches pregrasp",
                    approach_distance_m=approach_distance,
                    photo_to_pregrasp_path_fraction=photo_pregrasp_plan_fraction,
                    photo_to_pregrasp_stage=photo_pregrasp_stage,
                    pregrasp_max_dof=pregrasp_max_dof,
                    basket_lift_target_position_m=lift_target if basket_mode else None,
                    basket_lift_feed_duration_s=lift_duration if basket_mode else None,
                    final_feed_deferred=True,
                )
                success = True
                return

            self._publish(
                "gripper_preopen",
                True,
                "fully opening the calibrated right gripper before approach",
                gripper_target_position_m=open_position,
                graspnet_candidate_position_m=candidate_position,
            )
            self._send_gripper(
                open_position,
                accept_stalled=True,
                stalled_tolerance=float(
                    self.get_parameter("gripper_open_stall_tolerance_m").value
                ),
            )
            self._publish(
                "photo_to_pregrasp_execute",
                True,
                "planning and executing the arm-and-lift MoveIt path"
                if use_lift_planning
                else "planning and executing the standard unconstrained right-arm MoveIt path"
                if unconstrained_moveit
                else "planning, validating, and executing the bounded photo-to-pregrasp corridor",
            )
            photo_pregrasp_fraction = None
            if use_lift_planning:
                pregrasp_result = self._checked_pregrasp(
                    pregrasp_odom,
                    max_dof=pregrasp_max_dof,
                    execute=True,
                    label="photo-to-pregrasp execution",
                )
                photo_pregrasp_stage = int(pregrasp_result.selected_stage)
            else:
                photo_pregrasp_fraction = self._checked_cartesian(
                    photo_odom,
                    pregrasp_odom,
                    execute=True,
                    label="photo-to-pregrasp execution",
                    path_mode=transit_path_mode,
                )
            photo_pregrasp_plan_fraction = photo_pregrasp_fraction
            known_pose = pregrasp_odom

            current_pregrasp = self._lookup_current_tcp_pose()
            pregrasp_position_error = pose_distance(current_pregrasp, pregrasp_odom)
            known_pose = current_pregrasp
            pregrasp_position_tolerance = float(
                self.get_parameter(
                    "basket_pregrasp_position_tolerance_m"
                    if basket_mode
                    else "cartesian_start_position_tolerance_m"
                ).value
            )
            if pregrasp_position_error > pregrasp_position_tolerance:
                raise RuntimeError(
                    f"arm-and-lift execution ended {pregrasp_position_error:.4f} m from "
                    f"the pregrasp pose; limit is {pregrasp_position_tolerance:.4f} m"
                )
            line_residual = None
            if not unconstrained_moveit:
                _, line_residual = project_pose_onto_line(
                    current_pregrasp,
                    pregrasp_odom,
                    grasp_odom,
                )
                if line_residual > line_tolerance:
                    raise RuntimeError(
                        f"pregrasp tracking is {line_residual:.4f} m lateral to the grasp approach line; "
                        f"limit is {line_tolerance:.4f} m"
                    )
            self._publish(
                "pregrasp_verified",
                True,
                "MoveIt reached the requested pregrasp endpoint"
                if unconstrained_moveit
                else "measured TCP is on the grasp approach line",
                lateral_error_m=line_residual,
                position_error_m=pregrasp_position_error,
            )

            approach_plan_fraction = None
            approach_fallback_reason = ""
            if basket_mode:
                basket_grasp_pose = translated_pose(
                    current_pregrasp,
                    z=basket_feed_delta_z,
                )
                lift_target, lift_duration = self._checked_basket_lift_feed(
                    current_pregrasp,
                    basket_grasp_pose,
                    execute=False,
                    label="basket pregrasp-to-grasp preflight",
                )
                approach_plan_fraction = 1.0
                self._publish(
                    "grasp_feed_execute",
                    True,
                    "lowering the aligned gripper vertically with the lift column",
                    lift_target_position_m=lift_target,
                    lift_duration_s=lift_duration,
                    nominal_descent_m=max(
                        0.0,
                        pregrasp_odom.pose.position.z - grasp_odom.pose.position.z,
                    ),
                    descent_reduction_m=float(
                        self.get_parameter(
                            "basket_final_descent_reduction_m"
                        ).value
                    ),
                    effective_descent_m=abs(float(basket_feed_delta_z)),
                )
                self._checked_basket_lift_feed(
                    current_pregrasp,
                    basket_grasp_pose,
                    execute=True,
                    label="basket pregrasp-to-grasp execution",
                )
                approach_fraction = 1.0
                known_pose = basket_grasp_pose
            elif feed_path_mode == CARTESIAN_STRICT:
                self._publish(
                    "grasp_feed_preflight",
                    True,
                    "checking the straight grasp feed from pregrasp to grasp",
                )
                try:
                    approach_plan_fraction = self._checked_cartesian(
                        pregrasp_odom,
                        grasp_odom,
                        execute=False,
                        label="pregrasp-to-grasp preflight",
                        path_mode=CARTESIAN_STRICT,
                    )
                except RuntimeError as error:
                    if not allow_feed_fallback:
                        raise
                    approach_fallback_reason = str(error)
                    feed_path_mode = CARTESIAN_UNCONSTRAINED_MOVEIT
                    self._publish(
                        "grasp_feed_fallback",
                        True,
                        "complete straight feed is unavailable; using standard right-arm MoveIt",
                        straight_preflight_error=approach_fallback_reason,
                    )
            if not basket_mode:
                self._publish(
                    "grasp_feed_execute",
                    True,
                    "executing the preflighted straight grasp feed"
                    if feed_path_mode == CARTESIAN_STRICT
                    else "planning and executing a standard right-arm MoveIt fallback",
                )
                approach_fraction = self._checked_cartesian(
                    pregrasp_odom,
                    grasp_odom,
                    execute=True,
                    label="pregrasp-to-grasp execution",
                    path_mode=feed_path_mode,
                )
                known_pose = grasp_odom

            self._publish(
                "gripper_close",
                True,
                "closing calibrated right gripper to the physical zero target",
                gripper_target_position_m=close_position,
            )
            close_result = self._send_gripper(close_position, accept_stalled=True)
            object_grasped = True
            self._set_payload_compensation(
                float(self.get_parameter("payload_mass_kg").value)
            )

            lift_plan_fraction = None
            lift_fraction = None
            lifted_to_pregrasp_plan_fraction = None
            lifted_to_pregrasp_fraction = None
            pregrasp_to_photo_fraction = None
            if self._bool_param("return_to_photo"):
                postgrasp_lift = float(self.get_parameter("postgrasp_lift_m").value)
                if not math.isfinite(postgrasp_lift) or postgrasp_lift <= 0.0:
                    raise RuntimeError("postgrasp_lift_m must be finite and positive")
                if basket_mode:
                    (
                        lift_plan_fraction,
                        lift_fraction,
                        known_pose,
                    ) = self._return_basket_to_photo_pose(
                        photo_odom,
                        pregrasp_odom,
                        photo_joint_positions,
                        photo_lift_position,
                        pregrasp_max_dof=pregrasp_max_dof,
                        label_prefix="postgrasp return",
                    )
                else:
                    (
                        lift_plan_fraction,
                        lift_fraction,
                        lifted_to_pregrasp_plan_fraction,
                        lifted_to_pregrasp_fraction,
                        pregrasp_to_photo_fraction,
                        known_pose,
                    ) = self._return_held_object_to_photo_pose(
                        photo_odom,
                        pregrasp_odom,
                        photo_joint_positions,
                        postgrasp_lift,
                        photo_lift_position=photo_lift_position,
                        label_prefix="postgrasp return",
                        pregrasp_max_dof=pregrasp_max_dof,
                    )

            self._publish(
                "complete",
                True,
                "right-arm grasp completed; held product lifted, returned to pregrasp, then returned to the photo pose",
                approach_distance_m=approach_distance,
                gripper_open_position_m=open_position,
                gripper_close_position_m=close_position,
                gripper_graspnet_candidate_position_m=candidate_position,
                gripper_opening_width_m=width,
                gripper_grasp_width_m=grasp_width,
                gripper_stalled=bool(close_result.stalled),
                pregrasp_line_residual_m=line_residual,
                photo_to_pregrasp_plan_fraction=photo_pregrasp_plan_fraction,
                photo_to_pregrasp_path_fraction=photo_pregrasp_fraction,
                photo_to_pregrasp_stage=photo_pregrasp_stage,
                pregrasp_max_dof=pregrasp_max_dof,
                approach_plan_fraction=approach_plan_fraction,
                approach_path_fraction=approach_fraction,
                approach_straight_fallback_reason=approach_fallback_reason,
                postgrasp_lift_m=float(self.get_parameter("postgrasp_lift_m").value),
                lift_plan_fraction=lift_plan_fraction,
                lift_path_fraction=lift_fraction,
                lifted_to_pregrasp_plan_fraction=lifted_to_pregrasp_plan_fraction,
                lifted_to_pregrasp_path_fraction=lifted_to_pregrasp_fraction,
                pregrasp_to_photo_path_fraction=pregrasp_to_photo_fraction,
            )
            success = True
        except Exception as error:
            recovered = False
            recovery_error = ""
            if (
                execute
                and not isinstance(error, MotionStateUncertainError)
                and self._bool_param("return_to_photo")
                and known_pose is not None
                and pregrasp_odom is not None
                and photo_odom is not None
                and pose_distance(known_pose, photo_odom) > 0.001
            ):
                try:
                    self._publish(
                        "abort_recovery",
                        True,
                        "motion state is known; reversing Cartesian segments to the photo pose",
                    )
                    if object_grasped:
                        if basket_mode:
                            self._return_basket_to_photo_pose(
                                photo_odom,
                                pregrasp_odom,
                                photo_joint_positions,
                                photo_lift_position,
                                pregrasp_max_dof=pregrasp_max_dof,
                                label_prefix="held-object abort recovery",
                            )
                        else:
                            self._return_held_object_to_photo_pose(
                                photo_odom,
                                pregrasp_odom,
                                photo_joint_positions,
                                float(self.get_parameter("postgrasp_lift_m").value),
                                photo_lift_position=photo_lift_position,
                                label_prefix="held-object abort recovery",
                                recovery=True,
                                pregrasp_max_dof=pregrasp_max_dof,
                            )
                    else:
                        if basket_mode and pose_distance(known_pose, pregrasp_odom) > 0.01:
                            self._return_basket_to_photo_pose(
                                photo_odom,
                                pregrasp_odom,
                                photo_joint_positions,
                                photo_lift_position,
                                pregrasp_max_dof=pregrasp_max_dof,
                                label_prefix="abort recovery",
                            )
                        else:
                            self._return_to_photo_pose(
                                known_pose,
                                pregrasp_odom,
                                photo_odom,
                                photo_joint_positions,
                                photo_lift_position=photo_lift_position,
                                label_prefix="abort recovery",
                                pregrasp_max_dof=pregrasp_max_dof,
                            )
                    recovered = True
                except Exception as recovery_exception:
                    recovery_error = str(recovery_exception)
                    self.get_logger().error(f"Abort recovery failed: {recovery_exception}")
            self._publish(
                "aborted",
                False,
                str(error),
                returned_to_photo=recovered,
                recovery_error=recovery_error,
            )
            self.get_logger().error(f"Product grasp execution aborted: {error}")
        finally:
            with self._lock:
                self._worker_active = False
                if success:
                    self._completed = True

    def _checked_cartesian(
        self,
        start: PoseStamped,
        end: PoseStamped,
        *,
        execute: bool,
        label: str,
        orientation_mode: int = CARTESIAN_INTERPOLATE,
        path_mode: int = CARTESIAN_STRICT,
        end_joint_positions=None,
    ) -> float:
        result = self._send_cartesian(
            start,
            end,
            execute=execute,
            label=label,
            orientation_mode=orientation_mode,
            path_mode=path_mode,
            end_joint_positions=end_joint_positions,
        )
        self._require_moveit_success(result, label)
        if path_mode == CARTESIAN_UNCONSTRAINED_MOVEIT:
            return float(result.path_fraction)
        return require_path_fraction(
            result.path_fraction,
            float(self.get_parameter("cartesian_min_fraction").value),
            label,
        )

    def _return_to_photo_pose(
        self,
        current_pose: PoseStamped,
        pregrasp_pose: PoseStamped,
        photo_pose: PoseStamped,
        photo_joint_positions: tuple[float, ...],
        *,
        photo_lift_position: float | None,
        label_prefix: str,
        pregrasp_max_dof: int = PlanPregrasp.Goal.ARM_ONLY,
    ):
        unconstrained_moveit = self._bool_param("unconstrained_moveit")
        feed_path_mode = (
            CARTESIAN_UNCONSTRAINED_MOVEIT
            if unconstrained_moveit
            else CARTESIAN_STRICT
        )
        if pose_distance(current_pose, pregrasp_pose) > 0.001:
            if not unconstrained_moveit:
                self._checked_cartesian(
                    current_pose,
                    pregrasp_pose,
                    execute=False,
                    label=f"{label_prefix} grasp-to-pregrasp preflight",
                    path_mode=feed_path_mode,
                )
            self._checked_cartesian(
                current_pose,
                pregrasp_pose,
                execute=True,
                label=f"{label_prefix} grasp-to-pregrasp execution",
                path_mode=feed_path_mode,
            )
        self._checked_joint_posture(
            photo_joint_positions,
            lift_position=(
                photo_lift_position
                if pregrasp_max_dof >= PlanPregrasp.Goal.ARM_WITH_LIFT
                else None
            ),
            execute=True,
            label=f"{label_prefix} pregrasp-to-photo execution",
        )

    def _return_held_object_to_photo_pose(
        self,
        photo_pose: PoseStamped,
        pregrasp_pose: PoseStamped,
        photo_joint_positions: tuple[float, ...],
        lift_m: float,
        *,
        photo_lift_position: float | None,
        label_prefix: str,
        recovery: bool = False,
        pregrasp_max_dof: int = PlanPregrasp.Goal.ARM_ONLY,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
        PoseStamped,
    ]:
        lift = float(lift_m)
        if not math.isfinite(lift) or lift <= 0.0:
            raise RuntimeError("postgrasp_lift_m must be finite and positive")

        current = self._lookup_current_tcp_pose()
        if recovery:
            safe_height = max(current.pose.position.z, photo_pose.pose.position.z + lift)
            lift_target = translated_pose(
                current,
                z=safe_height - current.pose.position.z,
            )
        else:
            lift_target = translated_pose(current, z=lift)
        lift_plan = None
        lift_path = None
        if pose_distance(current, lift_target) > 0.001:
            self._publish(
                "lift_preflight", True, "lifting the held product vertically"
            )
            lift_plan = self._checked_cartesian(
                current,
                lift_target,
                execute=False,
                label=f"{label_prefix} lift preflight",
                path_mode=CARTESIAN_STRICT,
            )
            current = self._lookup_current_tcp_pose()
            self._publish("lift_execute", True, "lifting the held product vertically")
            lift_path = self._checked_cartesian(
                current,
                lift_target,
                execute=True,
                label=f"{label_prefix} lift execution",
                path_mode=CARTESIAN_STRICT,
            )

        feed_plan = None
        feed_path = None
        feed_mode = CARTESIAN_STRICT
        allow_feed_fallback = self._bool_param("unconstrained_moveit")
        current = self._lookup_current_tcp_pose()
        if pose_distance(current, pregrasp_pose) > 0.001:
            self._publish(
                "lifted_to_pregrasp_preflight",
                True,
                "checking the straight lifted-to-pregrasp feed",
            )
            try:
                feed_plan = self._checked_cartesian(
                    current,
                    pregrasp_pose,
                    execute=False,
                    label=f"{label_prefix} lifted-to-pregrasp preflight",
                    path_mode=CARTESIAN_STRICT,
                )
            except RuntimeError as error:
                if not allow_feed_fallback:
                    raise
                feed_mode = CARTESIAN_UNCONSTRAINED_MOVEIT
                self._publish(
                    "lifted_to_pregrasp_fallback",
                    True,
                    "straight lifted-to-pregrasp feed unavailable; using standard right-arm MoveIt",
                    straight_preflight_error=str(error),
                )
            current = self._lookup_current_tcp_pose()
            self._publish(
                "lifted_to_pregrasp_execute",
                True,
                "executing straight lifted-to-pregrasp feed"
                if feed_mode == CARTESIAN_STRICT
                else "executing standard right-arm MoveIt to pregrasp",
            )
            feed_path = self._checked_cartesian(
                current,
                pregrasp_pose,
                execute=True,
                label=f"{label_prefix} lifted-to-pregrasp execution",
                path_mode=feed_mode,
            )

        self._publish(
            "pregrasp_to_photo_execute",
            True,
            "executing standard right-arm MoveIt to the recorded photo pose",
        )
        self._checked_joint_posture(
            photo_joint_positions,
            lift_position=(
                photo_lift_position
                if pregrasp_max_dof >= PlanPregrasp.Goal.ARM_WITH_LIFT
                else None
            ),
            execute=True,
            label=f"{label_prefix} pregrasp-to-photo execution",
        )
        photo_path = None
        return lift_plan, lift_path, feed_plan, feed_path, photo_path, photo_pose

    def _checked_pregrasp(
        self,
        target: PoseStamped,
        *,
        max_dof: int,
        execute: bool,
        label: str,
    ):
        goal = PlanPregrasp.Goal()
        goal.arm = parse_arm(self.get_parameter("arm").value)
        goal.max_dof = int(max_dof)
        goal.tcp_pregrasp_pose = target
        goal.base_yaw_mode = PlanPregrasp.Goal.CURRENT_STATE
        goal.desired_base_yaw_odom = 0.0
        goal.base_yaw_tolerance = 0.0
        goal.base_yaw_samples = 0
        goal.front_distance = 0.0
        goal.velocity_scale = float(
            self.get_parameter("cartesian_velocity_scale").value
        )
        goal.acceleration_scale = float(
            self.get_parameter("cartesian_acceleration_scale").value
        )
        goal.planning_timeout_sec = float(
            self.get_parameter("corridor_planning_timeout_s").value
        )
        goal.execute = bool(execute)
        attempts = int(self.get_parameter("pregrasp_planning_attempts").value)
        if attempts < 1 or attempts > 5:
            raise ValueError("pregrasp_planning_attempts must be within [1, 5]")
        result = None
        for attempt in range(1, attempts + 1):
            result = self._send_action(self._pregrasp_client, goal, label)
            if int(result.error_code.val) != MOVEIT_NO_IK_SOLUTION:
                break
            if attempt < attempts:
                self.get_logger().warning(
                    f"{label} found no IK solution on attempt {attempt}/{attempts}; "
                    "retrying after RobotState/TF refresh"
                )
                time.sleep(0.25)
        assert result is not None
        self._require_moveit_success(result, label)
        require_pregrasp_stage(result.selected_stage, max_dof, label)
        return result

    def _checked_joint_posture(
        self,
        arm_positions: tuple[float, ...],
        *,
        lift_position: float | None,
        execute: bool,
        label: str,
    ):
        planning_group, names, positions = joint_posture_spec(
            str(self.get_parameter("arm").value),
            arm_positions,
            lift_position=lift_position,
        )
        goal = ExecuteJointPosture.Goal()
        goal.planning_group = planning_group
        goal.joint_goal.name = list(names)
        goal.joint_goal.position = list(positions)
        goal.velocity_scale = float(
            self.get_parameter("cartesian_velocity_scale").value
        )
        goal.acceleration_scale = float(
            self.get_parameter("cartesian_acceleration_scale").value
        )
        goal.planning_timeout_sec = float(
            self.get_parameter("corridor_planning_timeout_s").value
        )
        goal.execute = bool(execute)
        result = self._send_action(self._joint_posture_client, goal, label)
        self._require_moveit_success(result, label)
        return result

    def _checked_basket_lift_feed(
        self,
        start: PoseStamped,
        end: PoseStamped,
        *,
        execute: bool,
        label: str,
        start_lift_position: float | None = None,
    ) -> tuple[float, float]:
        horizontal_error = math.hypot(
            end.pose.position.x - start.pose.position.x,
            end.pose.position.y - start.pose.position.y,
        )
        if horizontal_error > 0.005:
            raise RuntimeError(
                f"{label} is not a vertical lift feed; horizontal offset={horizontal_error:.4f} m"
            )
        current_lift = (
            self._lookup_current_lift_position()
            if start_lift_position is None
            else float(start_lift_position)
        )
        target, duration = vertical_lift_feed_target(
            current_lift,
            end.pose.position.z - start.pose.position.z,
            speed_m_s=float(
                self.get_parameter("basket_lift_feed_speed_m_s").value
            ),
            min_position_m=float(self.get_parameter("lift_min_position_m").value),
            max_position_m=float(self.get_parameter("lift_max_position_m").value),
        )
        if not execute:
            return target, duration

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["lift_joint"]
        point = JointTrajectoryPoint()
        point.positions = [target]
        seconds = int(duration)
        point.time_from_start = DurationMsg(
            sec=seconds,
            nanosec=int((duration - seconds) * 1e9),
        )
        goal.trajectory.points = [point]
        result = self._send_action(self._lift_client, goal, label)
        if int(result.error_code) != int(FollowJointTrajectory.Result.SUCCESSFUL):
            raise MotionStateUncertainError(
                f"{label} failed with trajectory code {result.error_code}: "
                f"{result.error_string}"
            )
        return target, duration

    def _return_basket_to_photo_pose(
        self,
        photo_pose: PoseStamped,
        pregrasp_pose: PoseStamped,
        photo_joint_positions: tuple[float, ...],
        photo_lift_position: float | None,
        *,
        pregrasp_max_dof: int,
        label_prefix: str,
    ) -> tuple[float, float, PoseStamped]:
        if photo_lift_position is None:
            raise RuntimeError("recorded basket photo lift position is unavailable")
        current = self._lookup_current_tcp_pose()
        retract_pose = translated_pose(
            current,
            z=pregrasp_pose.pose.position.z - current.pose.position.z,
        )
        target, duration = self._checked_basket_lift_feed(
            current,
            retract_pose,
            execute=False,
            label=f"{label_prefix} lift retract preflight",
        )
        self._publish(
            "basket_lift_retract_execute",
            True,
            "raising the basket grasp vertically with the lift column",
            lift_target_position_m=target,
            lift_duration_s=duration,
        )
        self._checked_basket_lift_feed(
            current,
            retract_pose,
            execute=True,
            label=f"{label_prefix} lift retract execution",
        )
        self._publish(
            "basket_exact_photo_restore",
            True,
            "restoring the recorded right-arm and lift joint configuration",
            right_arm_joint_positions=list(photo_joint_positions),
            lift_joint_position=photo_lift_position,
        )
        self._checked_joint_posture(
            photo_joint_positions,
            lift_position=photo_lift_position,
            execute=True,
            label=f"{label_prefix} pregrasp-to-photo execution",
        )
        return 1.0, 1.0, photo_pose

    def _send_cartesian(
        self,
        start: PoseStamped,
        end: PoseStamped,
        *,
        execute: bool,
        label: str,
        orientation_mode: int = CARTESIAN_INTERPOLATE,
        path_mode: int = CARTESIAN_STRICT,
        end_joint_positions=None,
    ):
        goal = CartesianApproach.Goal()
        goal.arm = parse_arm(self.get_parameter("arm").value)
        goal.start_pose = start
        goal.end_pose = end
        goal.eef_step = float(self.get_parameter("cartesian_eef_step_m").value)
        goal.jump_threshold = float(self.get_parameter("cartesian_jump_threshold").value)
        goal.min_fraction = float(self.get_parameter("cartesian_min_fraction").value)
        goal.start_position_tolerance = float(
            self.get_parameter("cartesian_start_position_tolerance_m").value
        )
        goal.start_orientation_tolerance_deg = float(
            self.get_parameter("cartesian_start_orientation_tolerance_deg").value
        )
        goal.velocity_scale = float(self.get_parameter("cartesian_velocity_scale").value)
        goal.acceleration_scale = float(self.get_parameter("cartesian_acceleration_scale").value)
        goal.orientation_mode = int(orientation_mode)
        goal.path_mode = int(path_mode)
        goal.end_joint_positions = (
            [] if end_joint_positions is None else list(end_joint_positions)
        )
        if path_mode == CARTESIAN_BOUNDED_CORRIDOR:
            direct_distance = pose_distance(start, end)
            goal.corridor_radius = float(
                self.get_parameter("photo_pregrasp_corridor_radius_m").value
            )
            goal.corridor_max_path_length = bounded_corridor_max_path_length(
                direct_distance,
                path_scale=float(self.get_parameter("corridor_max_path_scale").value),
                extra_path=float(self.get_parameter("corridor_max_extra_path_m").value),
            )
            goal.corridor_max_backtracking = float(
                self.get_parameter("corridor_max_backtracking_m").value
            )
            goal.corridor_planning_timeout = float(
                self.get_parameter("corridor_planning_timeout_s").value
            )
        elif path_mode == CARTESIAN_UNCONSTRAINED_MOVEIT:
            goal.corridor_planning_timeout = float(
                self.get_parameter("corridor_planning_timeout_s").value
            )
        goal.execute = execute
        return self._send_action(self._cartesian_client, goal, label)

    def _send_gripper(
        self,
        position: float,
        *,
        accept_stalled: bool = False,
        stalled_tolerance: float | None = None,
    ):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter("gripper_max_effort").value)
        result = self._send_action(self._gripper_client, goal, "gripper")
        self.get_logger().info(
            f"Right gripper result: target={position:.4f} m, "
            f"actual={result.position:.4f} m, reached={result.reached_goal}, "
            f"stalled={result.stalled}"
        )
        if not gripper_goal_accepted(
            position,
            result.position,
            reached_goal=bool(result.reached_goal),
            stalled=bool(result.stalled),
            accept_stalled=accept_stalled,
            stalled_tolerance=stalled_tolerance,
        ):
            raise RuntimeError(
                f"gripper did not reach {position:.4f} m; position={result.position:.4f}, "
                f"stalled={result.stalled}"
            )
        return result

    def _set_payload_compensation(self, mass_kg: float):
        if not self._bool_param("payload_compensation_enabled"):
            return
        mass = float(mass_kg)
        com = (
            float(self.get_parameter("payload_com_x_m").value),
            float(self.get_parameter("payload_com_y_m").value),
            float(self.get_parameter("payload_com_z_m").value),
        )
        if not math.isfinite(mass) or mass < 0.0 or not all(
            math.isfinite(value) for value in com
        ):
            raise RuntimeError("payload mass and COM must be finite; mass must be non-negative")

        timeout = max(
            0.1, float(self.get_parameter("payload_parameter_timeout_s").value)
        )
        if not self._gravity_parameter_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "gravity compensation parameter service is unavailable; restart the integrated robot stack"
            )
        request = SetParameters.Request()
        parameters = [
                Parameter("right_payload_mass_kg", Parameter.Type.DOUBLE, mass),
                Parameter("right_payload_com_x_m", Parameter.Type.DOUBLE, com[0]),
                Parameter("right_payload_com_y_m", Parameter.Type.DOUBLE, com[1]),
                Parameter("right_payload_com_z_m", Parameter.Type.DOUBLE, com[2]),
        ]
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        response = self._wait_future(
            self._gravity_parameter_client.call_async(request),
            timeout,
            "right payload gravity parameters",
        )
        rejected = [result.reason for result in response.results if not result.successful]
        if rejected:
            raise RuntimeError(
                "gravity compensation rejected right payload parameters: "
                + "; ".join(rejected)
            )
        self.get_logger().info(
            f"Right payload gravity compensation set to {mass:.3f} kg "
            f"at TCP COM offset ({com[0]:.3f}, {com[1]:.3f}, {com[2]:.3f}) m"
        )

    def _send_action(self, client: ActionClient, goal, label: str):
        wait_timeout = float(self.get_parameter("action_wait_timeout_s").value)
        if not client.wait_for_server(timeout_sec=wait_timeout):
            raise RuntimeError(f"{label} action server is unavailable")
        goal_handle = self._wait_future(
            client.send_goal_async(goal), wait_timeout, f"{label} goal response"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label} goal was rejected")
        try:
            wrapped = self._wait_future(
                goal_handle.get_result_async(),
                float(self.get_parameter("action_result_timeout_s").value),
                f"{label} result",
            )
        except RuntimeError as error:
            goal_handle.cancel_goal_async()
            raise MotionStateUncertainError(
                f"{label} was accepted but its final motion state is unknown: {error}"
            ) from error
        return wrapped.result

    @staticmethod
    def _wait_future(future, timeout: float, label: str):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=max(0.1, timeout)):
            raise RuntimeError(f"timed out waiting for {label}")
        error = future.exception()
        if error is not None:
            raise RuntimeError(f"{label} failed: {error}") from error
        return future.result()

    @staticmethod
    def _require_moveit_success(result, label: str):
        code = int(result.error_code.val)
        if code != MOVEIT_SUCCESS:
            error_type = (
                MotionStateUncertainError
                if code in {MOVEIT_CONTROL_FAILED, MOVEIT_GOAL_TOLERANCE_VIOLATED}
                else RuntimeError
            )
            raise error_type(
                f"{label} failed with MoveIt code {result.error_code.val}: {result.message}"
            )

    def _transform_pose(self, message: PoseStamped) -> PoseStamped:
        lookup_time = Time() if self._bool_param("use_latest_transform") else Time.from_msg(message.header.stamp)
        try:
            return transform_pose_via_planar_base(
                self._tf_buffer,
                message,
                target_frame=self._target_frame,
                lookup_time=lookup_time,
                timeout=Duration(seconds=float(self.get_parameter("transform_timeout_s").value)),
            )
        except TransformException as error:
            raise RuntimeError(
                f"failed to transform {message.header.frame_id} to {self._target_frame}: {error}"
            ) from error

    def _lookup_current_tcp_pose(self) -> PoseStamped:
        source = PoseStamped()
        source.header.stamp = self.get_clock().now().to_msg()
        source.header.frame_id = str(self.get_parameter("tcp_frame").value)
        source.pose.orientation.w = 1.0
        try:
            return transform_pose_via_planar_base(
                self._tf_buffer,
                source,
                target_frame=self._target_frame,
                lookup_time=Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter("photo_pose_timeout_s").value)
                ),
            )
        except TransformException as error:
            raise RuntimeError(
                f"failed to register current right-arm photo pose: {error}"
            ) from error

    def _lookup_current_right_joint_positions(self) -> tuple[float, ...]:
        deadline = time.monotonic() + float(
            self.get_parameter("photo_pose_timeout_s").value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                if self._right_joint_positions is not None:
                    return tuple(self._right_joint_positions)
            time.sleep(0.02)
        raise RuntimeError("timed out waiting for complete right-arm joint positions")

    def _lookup_current_lift_position(self) -> float:
        deadline = time.monotonic() + float(
            self.get_parameter("photo_pose_timeout_s").value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                if self._lift_position is not None:
                    return float(self._lift_position)
            time.sleep(0.02)
        raise RuntimeError("timed out waiting for lift_joint position")

    def _wait_for_perception_pipeline(self):
        timeout = float(self.get_parameter("perception_wait_timeout_s").value)
        deadline = time.monotonic() + max(0.1, timeout)
        grasp_result_topic = str(self.get_parameter("grasp_result_topic").value)
        while rclpy.ok():
            duplicates = duplicate_grasp_pipeline_nodes(
                self.get_node_names_and_namespaces()
            )
            if duplicates:
                raise RuntimeError(
                    "duplicate grasp pipeline nodes detected "
                    f"({', '.join(duplicates)}); stop the older grab_product launch "
                    "before starting a new one"
                )
            target_subscribers = self._target_pub.get_subscription_count()
            grasp_publishers = self.count_publishers(grasp_result_topic)
            if target_subscribers > 1 or grasp_publishers > 1:
                raise RuntimeError(
                    "multiple product perception/grasp pipelines are publishing the shared topics; "
                    "stop the older grab_product launch before starting a new one"
                )
            target_ready = target_subscribers == 1
            graspnet_ready = grasp_publishers == 1
            if target_ready and graspnet_ready:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "timed out waiting for product segmentation and GraspNet nodes"
                )
            time.sleep(0.1)

    def _publish(self, stage: str, accepted: bool, message: str, **fields):
        payload = {
            "stage": stage,
            "accepted": bool(accepted),
            "message": message,
            "enabled": self._bool_param("enabled"),
            "execute": self._bool_param("execute"),
            **fields,
        }
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False)
        self._result_pub.publish(output)
        if accepted:
            self.get_logger().info(output.data)
        else:
            self.get_logger().warning(output.data)

    @staticmethod
    def _stamp(message: PoseStamped):
        return message.header.stamp.sec, message.header.stamp.nanosec

    @staticmethod
    def _copy_pose(message: PoseStamped) -> PoseStamped:
        copied = PoseStamped()
        copied.header = message.header
        copied.pose = message.pose
        return copied


def main(args=None):
    rclpy.init(args=args)
    node = ProductGraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
