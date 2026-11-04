"""Low-speed scan/TF shelf fine-alignment action server."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from openarmx_task_msgs.action import ShelfFineAlign, ShelfTraverse
from openarmx_task_msgs.msg import TaskError
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt8
from tf2_ros import Buffer, TransformException, TransformListener

from .shelf_fine_align_core import (
    ShelfCornerFilter,
    ShelfPlaneFilter,
    compute_shelf_corner_command,
    compute_shelf_traverse_command,
    measure_scan_shelf_corners,
    measure_scan_shelf_plane,
    normalize_angle,
)


def quaternion_to_yaw(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def localization_state_allows_alignment(state_seen: bool, state_ok: bool) -> bool:
    """Interpret /localization_ok as a latched state, not a heartbeat.

    The ICP publisher uses reliable + transient-local QoS and publishes only
    when its health state changes.  Once ``true`` has been received it remains
    authoritative until a later ``false`` message arrives.  Live scan, odom,
    and TF checks independently guard the actual fine-alignment motion.
    """
    return bool(state_seen and state_ok)


def alignment_control_yaw_error(
    map_yaw_error: float, _measured_shelf_yaw_error: float
) -> float:
    """Use the configured station heading as the stable motion reference.

    Sparse shelf returns can switch abruptly between uprights and lips even
    though the chassis barely moved. They remain authoritative for left/right
    clearance, while the saved map heading closes the angular control loop.
    """
    return normalize_angle(map_yaw_error)


def alignment_target_clearance(
    requested_clearance: float,
    measured_clearance: float,
    yaw_only: bool,
) -> float:
    """Freeze longitudinal control at the current clearance for yaw-only work."""
    return float(measured_clearance if yaw_only else requested_clearance)


def first_measurement_timed_out(
    now: float,
    action_started_at: float,
    measurement_time: float,
    acquisition_timeout: float,
) -> bool:
    """Keep initial acquisition independent from raw scan freshness."""
    return (
        measurement_time <= 0.0
        and now - action_started_at > acquisition_timeout
    )


def filtered_measurement_is_stale(
    now: float, measurement_time: float, measurement_timeout: float
) -> bool:
    """Allow brief sparse-return gaps without accepting stale geometry."""
    return (
        measurement_time > 0.0
        and now - measurement_time > measurement_timeout
    )


LOCALIZATION_GUARD_INACTIVE = 0
LOCALIZATION_GUARD_ACTIVE = 1
LOCALIZATION_GUARD_COMPLETED = 2


def fine_alignment_guard_terminal_state(
    guard_requested: bool, alignment_succeeded: bool
) -> int | None:
    if not guard_requested:
        return None
    return (
        LOCALIZATION_GUARD_COMPLETED
        if alignment_succeeded
        else LOCALIZATION_GUARD_INACTIVE
    )


class ShelfFineAlignNode(Node):
    """Approach sparse shelf front edges with both body corners protected."""

    def __init__(self) -> None:
        super().__init__("shelf_fine_align")
        self.declare_parameter("action_name", "/shelf_fine_align")
        self.declare_parameter("traverse_action_name", "/return_to_shelf_anchor")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom_safe")
        self.declare_parameter("localization_topic", "/localization_ok")
        self.declare_parameter(
            "localization_guard_state_topic",
            "/shelf_fine_align/localization_guard_state",
        )
        # This raw command still passes through collision_monitor and the
        # localization safety gate before reaching the base.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("half_length", 0.46)
        self.declare_parameter("half_width", 0.45)
        self.declare_parameter("roi_forward_min", 0.46)
        self.declare_parameter("roi_forward_max", 1.20)
        self.declare_parameter("roi_half_width", 0.55)
        self.declare_parameter("maximum_measured_clearance", 0.45)
        self.declare_parameter("filter_window", 7)
        self.declare_parameter("filter_minimum_samples", 5)
        self.declare_parameter("filter_quantile", 0.25)
        self.declare_parameter("corner_minimum_lateral_offset", 0.20)
        self.declare_parameter("corner_minimum_points_per_side", 2)
        # Delivery counters contain sparse legs and a projected table edge.
        # Fit one front-spanning surface for these stations so an isolated leg
        # cannot masquerade as the complete table front.
        self.declare_parameter("plane_fit_station_ids", ["jiaofu2"])
        self.declare_parameter("plane_minimum_points", 5)
        self.declare_parameter("plane_minimum_lateral_span", 0.30)
        self.declare_parameter("plane_minimum_points_per_side", 2)
        # Delivery fronts are reconstructed from a sparse Livox projection.
        # Keep the per-frame two-sided/span checks, then require the existing
        # 5-of-7 temporal filter before any motion is allowed.
        self.declare_parameter("plane_inlier_tolerance", 0.040)
        self.declare_parameter("plane_maximum_normal_yaw", math.radians(12.0))
        self.declare_parameter("plane_filter_quantile", 0.50)
        self.declare_parameter("plane_acquisition_timeout", 3.00)
        self.declare_parameter("plane_measurement_timeout", 1.50)
        self.declare_parameter("target_clearance_hysteresis", 0.003)
        self.declare_parameter("scan_timeout", 0.50)
        self.declare_parameter("acquisition_timeout", 1.50)
        self.declare_parameter("measurement_timeout", 0.80)
        self.declare_parameter("odom_timeout", 0.50)
        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("settle_samples", 10)
        self.declare_parameter("yaw_align_gate", math.radians(3.0))
        self.declare_parameter("rotate_kp", 1.2)
        self.declare_parameter("approach_kp", 0.55)
        self.declare_parameter("maximum_angular_speed", 0.10)
        self.declare_parameter("minimum_angular_speed", 0.070)
        # These limits apply only to the guarded post-Nav2 shelf approach.
        # The deployed swerve controller first zeros chassis commands below
        # 0.015 m/s and then zeros individual wheel commands below 0.020 m/s.
        # A 0.018 m/s chassis request therefore reached the safety chain but
        # often produced no wheel motion. Keep the effective command safely
        # above both thresholds; scan, clearance, and collision guards still
        # decide when it must stop.
        self.declare_parameter("maximum_linear_speed", 0.040)
        self.declare_parameter("minimum_linear_speed", 0.030)
        self.declare_parameter("minimum_safe_clearance", 0.035)
        self.declare_parameter("minimum_start_clearance", 0.040)
        self.declare_parameter("maximum_start_yaw_error", math.radians(8.0))
        self.declare_parameter("maximum_start_linear_speed", 0.02)
        self.declare_parameter("maximum_start_angular_speed", 0.03)
        self.declare_parameter("traverse_lateral_kp", 0.60)
        self.declare_parameter("traverse_maximum_lateral_speed", 0.040)
        self.declare_parameter("traverse_minimum_lateral_speed", 0.030)
        self.declare_parameter("traverse_maximum_distance", 1.20)

        def parameter(name):
            return self.get_parameter(name).value
        self.map_frame = str(parameter("map_frame"))
        self.base_frame = str(parameter("base_frame"))
        self.half_length = float(parameter("half_length"))
        self.half_width = float(parameter("half_width"))
        self.roi_forward_min = float(parameter("roi_forward_min"))
        self.roi_forward_max = float(parameter("roi_forward_max"))
        self.roi_half_width = float(parameter("roi_half_width"))
        self.maximum_measured_clearance = float(
            parameter("maximum_measured_clearance")
        )
        self.corner_minimum_lateral_offset = float(
            parameter("corner_minimum_lateral_offset")
        )
        self.corner_minimum_points_per_side = int(
            parameter("corner_minimum_points_per_side")
        )
        self.plane_fit_station_ids = frozenset(
            str(value).strip()
            for value in parameter("plane_fit_station_ids")
            if str(value).strip()
        )
        self.plane_minimum_points = int(parameter("plane_minimum_points"))
        self.plane_minimum_lateral_span = float(
            parameter("plane_minimum_lateral_span")
        )
        self.plane_minimum_points_per_side = int(
            parameter("plane_minimum_points_per_side")
        )
        self.plane_inlier_tolerance = float(
            parameter("plane_inlier_tolerance")
        )
        self.plane_maximum_normal_yaw = float(
            parameter("plane_maximum_normal_yaw")
        )
        self.plane_acquisition_timeout = float(
            parameter("plane_acquisition_timeout")
        )
        self.plane_measurement_timeout = float(
            parameter("plane_measurement_timeout")
        )
        self.target_clearance_hysteresis = float(
            parameter("target_clearance_hysteresis")
        )
        self.scan_timeout = float(parameter("scan_timeout"))
        self.acquisition_timeout = float(parameter("acquisition_timeout"))
        self.measurement_timeout = float(parameter("measurement_timeout"))
        self.odom_timeout = float(parameter("odom_timeout"))
        self.control_period = float(parameter("control_period"))
        self.settle_samples = int(parameter("settle_samples"))
        self.yaw_align_gate = float(parameter("yaw_align_gate"))
        self.rotate_kp = float(parameter("rotate_kp"))
        self.approach_kp = float(parameter("approach_kp"))
        self.maximum_angular_speed = float(parameter("maximum_angular_speed"))
        self.minimum_angular_speed = float(parameter("minimum_angular_speed"))
        self.maximum_linear_speed = float(parameter("maximum_linear_speed"))
        self.minimum_linear_speed = float(parameter("minimum_linear_speed"))
        self.minimum_safe_clearance = float(parameter("minimum_safe_clearance"))
        self.minimum_start_clearance = float(parameter("minimum_start_clearance"))
        self.maximum_start_yaw_error = float(parameter("maximum_start_yaw_error"))
        self.maximum_start_linear_speed = float(
            parameter("maximum_start_linear_speed")
        )
        self.maximum_start_angular_speed = float(
            parameter("maximum_start_angular_speed")
        )
        self.traverse_lateral_kp = float(parameter("traverse_lateral_kp"))
        self.traverse_maximum_lateral_speed = float(
            parameter("traverse_maximum_lateral_speed")
        )
        self.traverse_minimum_lateral_speed = float(
            parameter("traverse_minimum_lateral_speed")
        )
        self.traverse_maximum_distance = float(
            parameter("traverse_maximum_distance")
        )

        self._corner_filter = ShelfCornerFilter(
            int(parameter("filter_window")),
            int(parameter("filter_minimum_samples")),
            half_width=self.half_width,
            quantile=float(parameter("filter_quantile")),
        )
        self._plane_filter = ShelfPlaneFilter(
            int(parameter("filter_window")),
            int(parameter("filter_minimum_samples")),
            half_length=self.half_length,
            quantile=float(parameter("plane_filter_quantile")),
        )
        self._lock = threading.Lock()
        self._latest_scan_time = 0.0
        self._latest_measurement_time = 0.0
        self._latest_plane_measurement_time = 0.0
        self._latest_odom_time = 0.0
        self._localization_state_seen = False
        self._localization_ok = False
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._active_goal = False
        self._active_use_plane_fit = False

        callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(parameter("cmd_vel_topic")), 10
        )
        guard_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.localization_guard_pub = self.create_publisher(
            UInt8,
            str(parameter("localization_guard_state_topic")),
            guard_qos,
        )
        self._publish_localization_guard_state(LOCALIZATION_GUARD_INACTIVE)
        self.create_subscription(
            LaserScan,
            str(parameter("scan_topic")),
            self._scan_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            Odometry,
            str(parameter("odom_topic")),
            self._odom_callback,
            20,
            callback_group=callback_group,
        )
        self.create_subscription(
            Bool,
            str(parameter("localization_topic")),
            self._localization_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=callback_group,
        )
        self._server = ActionServer(
            self,
            ShelfFineAlign,
            str(parameter("action_name")),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )
        self._traverse_server = ActionServer(
            self,
            ShelfTraverse,
            str(parameter("traverse_action_name")),
            execute_callback=self.traverse_execute_callback,
            goal_callback=self.traverse_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            "Fine alignment ready: shelf front-corner or delivery front-plane "
            "ranging with 1-degree capable low-speed control"
        )
        self.get_logger().info(
            "Delivery front-plane validation: "
            f">={self.plane_minimum_points} points, "
            f"residual<={self.plane_inlier_tolerance:.3f}m, "
            f"acquire<={self.plane_acquisition_timeout:.1f}s, "
            f"gap<={self.plane_measurement_timeout:.1f}s"
        )

    def _scan_callback(self, message: LaserScan) -> None:
        with self._lock:
            use_plane_fit = self._active_use_plane_fit
        scan_frame = (message.header.frame_id or "").strip() or self.base_frame
        point_transform = None
        measurement = None
        if scan_frame != self.base_frame:
            try:
                stamp = Time.from_msg(message.header.stamp)
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan_frame,
                    stamp,
                    timeout=Duration(seconds=0.10),
                )
                point_transform = self._planar_transform(transform)
            except TransformException as exc:
                # Keep the scan timestamp fresh, but discard measurements that
                # cannot be expressed in base_link. The action then fails
                # closed after the normal no-return timeout.
                self.get_logger().warn(
                    f"Cannot transform /scan from {scan_frame} to "
                    f"{self.base_frame}: {exc}"
                )
                measurement = None
        else:
            point_transform = None
        if scan_frame == self.base_frame or point_transform is not None:
            if use_plane_fit:
                measurement = measure_scan_shelf_plane(
                    message.ranges,
                    message.angle_min,
                    message.angle_increment,
                    half_length=self.half_length,
                    half_width=self.half_width,
                    roi_forward_min=self.roi_forward_min,
                    roi_forward_max=self.roi_forward_max,
                    roi_half_width=self.roi_half_width,
                    maximum_clearance=self.maximum_measured_clearance,
                    minimum_points=self.plane_minimum_points,
                    minimum_lateral_span=self.plane_minimum_lateral_span,
                    minimum_points_per_side=self.plane_minimum_points_per_side,
                    inlier_tolerance=self.plane_inlier_tolerance,
                    maximum_normal_yaw=self.plane_maximum_normal_yaw,
                    point_transform=point_transform,
                )
            else:
                measurement = measure_scan_shelf_corners(
                    message.ranges,
                    message.angle_min,
                    message.angle_increment,
                    half_length=self.half_length,
                    half_width=self.half_width,
                    roi_forward_min=self.roi_forward_min,
                    roi_forward_max=self.roi_forward_max,
                    roi_half_width=self.roi_half_width,
                    maximum_clearance=self.maximum_measured_clearance,
                    minimum_lateral_offset=self.corner_minimum_lateral_offset,
                    minimum_points_per_side=self.corner_minimum_points_per_side,
                    point_transform=point_transform,
                )
        now = time.monotonic()
        with self._lock:
            self._latest_scan_time = now
            if measurement is not None:
                if use_plane_fit:
                    self._plane_filter.add(measurement)
                    self._latest_plane_measurement_time = now
                else:
                    self._corner_filter.add(measurement)
                    self._latest_measurement_time = now

    @staticmethod
    def _planar_transform(transform):
        """Return a 2-D point transform from a TF TransformStamped.

        Applying the quaternion rotation keeps the footprint distance correct
        for a physically offset or rotated lidar instead of assuming that the
        scan frame is coincident with base_link.
        """
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w

        def apply(point_x: float, point_y: float) -> tuple[float, float]:
            # Quaternion-vector multiplication, with z=0 for a planar scan.
            tx = 2.0 * (qy * 0.0 - qz * point_y)
            ty = 2.0 * (qz * point_x - qx * 0.0)
            tz = 2.0 * (qx * point_y - qy * point_x)
            return (
                point_x + qw * tx + (qy * tz - qz * ty) + translation.x,
                point_y + qw * ty + (qz * tx - qx * tz) + translation.y,
            )

        return apply

    def _odom_callback(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        with self._lock:
            self._latest_odom_time = time.monotonic()
            self._linear_speed = math.hypot(linear.x, linear.y)
            self._angular_speed = abs(message.twist.twist.angular.z)

    def _localization_callback(self, message: Bool) -> None:
        with self._lock:
            self._localization_state_seen = True
            self._localization_ok = bool(message.data)

    def goal_callback(self, request: ShelfFineAlign.Goal) -> GoalResponse:
        if request.target_clearance < self.minimum_safe_clearance:
            self.get_logger().error(
                f"Rejecting unsafe shelf clearance {request.target_clearance:.3f} m"
            )
            return GoalResponse.REJECT
        if request.yaw_tolerance < math.radians(0.5):
            return GoalResponse.REJECT
        with self._lock:
            if self._active_goal:
                return GoalResponse.REJECT
            self._active_goal = True
            self._active_use_plane_fit = (
                request.station_id in self.plane_fit_station_ids
            )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_command(
        self, linear_x: float, angular_z: float, linear_y: float = 0.0
    ) -> None:
        message = Twist()
        message.linear.x = float(linear_x)
        message.linear.y = float(linear_y)
        message.angular.z = float(angular_z)
        self.cmd_pub.publish(message)

    def _publish_localization_guard_state(self, state: int) -> None:
        message = UInt8()
        message.data = int(state)
        self.localization_guard_pub.publish(message)

    async def _sleep(self) -> None:
        future = Future()
        holder = []

        def wake() -> None:
            if not future.done():
                future.set_result(None)
            if holder:
                holder[0].cancel()

        holder.append(self.create_timer(self.control_period, wake))
        await future

    def _yaw(self) -> float:
        return self._pose()[2]

    def _pose(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.15),
        )
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            quaternion_to_yaw(transform.transform.rotation),
        )

    def traverse_goal_callback(self, request: ShelfTraverse.Goal) -> GoalResponse:
        values = (
            request.target_map_x,
            request.target_map_y,
            request.target_map_yaw,
            request.target_clearance,
            request.clearance_tolerance,
            request.lateral_tolerance,
            request.yaw_tolerance,
            request.max_lateral_speed,
            request.timeout_sec,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return GoalResponse.REJECT
        if (
            not request.station_id
            or request.target_clearance < self.minimum_safe_clearance
            or request.clearance_tolerance <= 0.0
            or request.lateral_tolerance <= 0.0
            or request.yaw_tolerance < math.radians(0.5)
            or not 0.0
            < request.max_lateral_speed
            <= self.traverse_maximum_lateral_speed
            or request.timeout_sec <= 0.0
        ):
            return GoalResponse.REJECT
        try:
            current_x, current_y, _ = self._pose()
        except TransformException:
            return GoalResponse.REJECT
        if math.hypot(
            request.target_map_x - current_x,
            request.target_map_y - current_y,
        ) > self.traverse_maximum_distance:
            return GoalResponse.REJECT
        target_forward_axis = (
            math.cos(request.target_map_yaw),
            math.sin(request.target_map_yaw),
        )
        normal_error = (
            (request.target_map_x - current_x) * target_forward_axis[0]
            + (request.target_map_y - current_y) * target_forward_axis[1]
        )
        if abs(normal_error) > 0.08:
            return GoalResponse.REJECT
        with self._lock:
            if self._active_goal:
                return GoalResponse.REJECT
            self._active_goal = True
            self._active_use_plane_fit = False
        return GoalResponse.ACCEPT

    @staticmethod
    def _traverse_result(
        success: bool,
        error_code: int,
        message: str,
        lateral_error: float = 0.0,
        clearance: float = 0.0,
        yaw_error: float = 0.0,
    ) -> ShelfTraverse.Result:
        result = ShelfTraverse.Result()
        result.success = bool(success)
        result.error_code = int(error_code)
        result.final_lateral_error = float(abs(lateral_error))
        result.final_clearance = float(clearance)
        result.final_yaw_error = float(abs(yaw_error))
        result.message = str(message)
        return result

    async def traverse_execute_callback(self, goal_handle):
        request = goal_handle.request
        action_started_at = time.monotonic()
        deadline = action_started_at + float(request.timeout_sec)
        stable_samples = 0
        started = False
        succeeded = False
        last_lateral_error = 0.0
        last_clearance = 0.0
        last_yaw_error = 0.0
        with self._lock:
            self._corner_filter.clear()
            self._latest_measurement_time = 0.0
        self._publish_localization_guard_state(LOCALIZATION_GUARD_ACTIVE)
        try:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._traverse_result(
                        False, TaskError.CANCELLED, "shelf traverse cancelled"
                    )
                now = time.monotonic()
                (
                    corners,
                    filter_samples,
                    scan_time,
                    measurement_time,
                    odom_time,
                    state_seen,
                    localization_ok,
                    linear_speed,
                    angular_speed,
                ) = self._snapshot()
                if not localization_state_allows_alignment(
                    state_seen, localization_ok
                ):
                    goal_handle.abort()
                    return self._traverse_result(
                        False, TaskError.SAFETY_STOP, "localization is not safe"
                    )
                if (
                    now - odom_time > self.odom_timeout
                    or now - scan_time > self.scan_timeout
                ):
                    goal_handle.abort()
                    return self._traverse_result(
                        False,
                        TaskError.SAFETY_STOP,
                        "shelf traverse sensors are stale",
                    )
                if first_measurement_timed_out(
                    now,
                    action_started_at,
                    measurement_time,
                    self.acquisition_timeout,
                ):
                    goal_handle.abort()
                    return self._traverse_result(
                        False,
                        TaskError.SAFETY_STOP,
                        "no shelf return in the expected scan ROI",
                    )
                if filtered_measurement_is_stale(
                    now, measurement_time, self.measurement_timeout
                ):
                    goal_handle.abort()
                    return self._traverse_result(
                        False,
                        TaskError.SAFETY_STOP,
                        "shelf return disappeared during traverse",
                    )
                if corners is None:
                    feedback = ShelfTraverse.Feedback()
                    feedback.phase = "ACQUIRE_SHELF_CORNERS"
                    feedback.stable_samples = int(filter_samples)
                    goal_handle.publish_feedback(feedback)
                    await self._sleep()
                    continue
                if not started and (
                    linear_speed > self.maximum_start_linear_speed
                    or angular_speed > self.maximum_start_angular_speed
                ):
                    self._publish_command(0.0, 0.0)
                    await self._sleep()
                    continue
                try:
                    current_x, current_y, current_yaw = self._pose()
                except TransformException:
                    await self._sleep()
                    continue
                target_y_axis = (
                    -math.sin(request.target_map_yaw),
                    math.cos(request.target_map_yaw),
                )
                last_lateral_error = (
                    (request.target_map_x - current_x) * target_y_axis[0]
                    + (request.target_map_y - current_y) * target_y_axis[1]
                )
                last_clearance = float(corners.minimum_corner_clearance)
                last_yaw_error = normalize_angle(
                    request.target_map_yaw - current_yaw
                )
                if last_clearance < self.minimum_safe_clearance:
                    goal_handle.abort()
                    return self._traverse_result(
                        False,
                        TaskError.SAFETY_STOP,
                        "filtered shelf clearance violated during traverse "
                        f"(left={corners.left_clearance:.3f} m, "
                        f"right={corners.right_clearance:.3f} m, "
                        f"limit={self.minimum_safe_clearance:.3f} m)",
                        last_lateral_error,
                        last_clearance,
                        last_yaw_error,
                    )
                started = True
                command = compute_shelf_traverse_command(
                    last_lateral_error,
                    request.lateral_tolerance,
                    last_clearance,
                    request.target_clearance,
                    request.clearance_tolerance,
                    last_yaw_error,
                    request.yaw_tolerance,
                    clearance_hysteresis=self.target_clearance_hysteresis,
                    yaw_align_gate=self.yaw_align_gate,
                    rotate_kp=self.rotate_kp,
                    approach_kp=self.approach_kp,
                    lateral_kp=self.traverse_lateral_kp,
                    maximum_angular_speed=self.maximum_angular_speed,
                    maximum_forward_speed=self.maximum_linear_speed,
                    minimum_forward_speed=self.minimum_linear_speed,
                    maximum_lateral_speed=request.max_lateral_speed,
                    minimum_lateral_speed=min(
                        self.traverse_minimum_lateral_speed,
                        request.max_lateral_speed,
                    ),
                    lateral_only=bool(getattr(request, "lateral_only", False)),
                    minimum_angular_speed=self.minimum_angular_speed,
                )
                if command.phase == "TOO_CLOSE":
                    goal_handle.abort()
                    return self._traverse_result(
                        False,
                        TaskError.SAFETY_STOP,
                        "shelf is too close for forward-only traverse correction",
                        last_lateral_error,
                        last_clearance,
                        last_yaw_error,
                    )
                stable_samples = (
                    stable_samples + 1 if command.within_tolerance else 0
                )
                self._publish_command(
                    command.linear_x, command.angular_z, command.linear_y
                )
                feedback = ShelfTraverse.Feedback()
                feedback.phase = command.phase
                feedback.remaining_lateral_distance = float(
                    abs(last_lateral_error)
                )
                feedback.measured_clearance = last_clearance
                feedback.yaw_error = float(abs(last_yaw_error))
                feedback.stable_samples = int(stable_samples)
                goal_handle.publish_feedback(feedback)
                if stable_samples >= self.settle_samples:
                    succeeded = True
                    goal_handle.succeed()
                    return self._traverse_result(
                        True,
                        TaskError.OK,
                        "shelf traverse target reached",
                        last_lateral_error,
                        last_clearance,
                        last_yaw_error,
                    )
                await self._sleep()
            goal_handle.abort()
            return self._traverse_result(
                False,
                TaskError.TIMEOUT,
                "shelf traverse timed out",
                last_lateral_error,
                last_clearance,
                last_yaw_error,
            )
        finally:
            self._publish_command(0.0, 0.0)
            self._publish_localization_guard_state(
                LOCALIZATION_GUARD_COMPLETED
                if succeeded
                else LOCALIZATION_GUARD_INACTIVE
            )
            with self._lock:
                self._active_goal = False
                self._active_use_plane_fit = False

    def _snapshot(self):
        with self._lock:
            if self._active_use_plane_fit:
                measurement = self._plane_filter.value
                filter_samples = self._plane_filter.sample_count
                measurement_time = self._latest_plane_measurement_time
            else:
                measurement = self._corner_filter.value
                filter_samples = self._corner_filter.sample_count
                measurement_time = self._latest_measurement_time
            return (
                measurement,
                filter_samples,
                self._latest_scan_time,
                measurement_time,
                self._latest_odom_time,
                self._localization_state_seen,
                self._localization_ok,
                self._linear_speed,
                self._angular_speed,
            )

    @staticmethod
    def _result(
        success: bool,
        error_code: int,
        message: str,
        clearance: float = 0.0,
        yaw_error: float = 0.0,
    ) -> ShelfFineAlign.Result:
        result = ShelfFineAlign.Result()
        result.success = success
        result.error_code = int(error_code)
        result.final_clearance = float(clearance)
        result.final_yaw_error = float(abs(yaw_error))
        result.message = message
        return result

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        guard_requested = not bool(getattr(request, "measure_only", False))
        alignment_succeeded = False
        timeout = request.timeout_sec if request.timeout_sec > 0.0 else 20.0
        action_started_at = time.monotonic()
        deadline = action_started_at + timeout
        stable_samples = 0
        started = False
        with self._lock:
            use_plane_fit = self._active_use_plane_fit
            if use_plane_fit:
                self._plane_filter.clear()
                self._latest_plane_measurement_time = 0.0
            else:
                self._corner_filter.clear()
                self._latest_measurement_time = 0.0
        acquisition_timeout = (
            self.plane_acquisition_timeout
            if use_plane_fit
            else self.acquisition_timeout
        )
        measurement_timeout = (
            self.plane_measurement_timeout
            if use_plane_fit
            else self.measurement_timeout
        )
        self.get_logger().info(
            f"Fine alignment {request.station_id}: observation="
            f"{'front_plane' if use_plane_fit else 'front_corners'}, "
            f"target={request.target_clearance:.3f}m, "
            f"yaw_only={bool(getattr(request, 'yaw_only', False))}"
        )
        if guard_requested:
            self._publish_localization_guard_state(LOCALIZATION_GUARD_ACTIVE)
        try:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    self._publish_command(0.0, 0.0)
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, "shelf fine alignment cancelled"
                    )
                now = time.monotonic()
                (
                    corners,
                    filter_samples,
                    scan_time,
                    measurement_time,
                    odom_time,
                    localization_state_seen,
                    localization_ok,
                    linear_speed,
                    angular_speed,
                ) = self._snapshot()
                if not localization_state_allows_alignment(
                    localization_state_seen, localization_ok
                ):
                    self._publish_command(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(
                        False, TaskError.SAFETY_STOP, "localization is not safe"
                    )
                if now - odom_time > self.odom_timeout:
                    self._publish_command(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(False, TaskError.SAFETY_STOP, "safe odometry is stale")
                if now - scan_time > self.scan_timeout:
                    self._publish_command(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(False, TaskError.SAFETY_STOP, "shelf scan is stale")
                if first_measurement_timed_out(
                    now,
                    action_started_at,
                    measurement_time,
                    acquisition_timeout,
                ):
                    self._publish_command(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        (
                            "delivery front plane was not stable in the expected scan ROI"
                            if use_plane_fit
                            else "no shelf return in the expected scan ROI"
                        ),
                    )
                if filtered_measurement_is_stale(
                    now, measurement_time, measurement_timeout
                ):
                    self._publish_command(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        (
                            "delivery front plane disappeared from the expected scan ROI"
                            if use_plane_fit
                            else "shelf return disappeared from scan"
                        ),
                    )
                if corners is None:
                    feedback = ShelfFineAlign.Feedback()
                    feedback.phase = (
                        "ACQUIRE_DELIVERY_FRONT_PLANE"
                        if use_plane_fit
                        else "ACQUIRE_SHELF_CORNERS"
                    )
                    feedback.stable_samples = int(filter_samples)
                    goal_handle.publish_feedback(feedback)
                    self._publish_command(0.0, 0.0)
                    await self._sleep()
                    continue
                try:
                    current_yaw = self._yaw()
                except TransformException:
                    self._publish_command(0.0, 0.0)
                    await self._sleep()
                    continue
                map_yaw_error = normalize_angle(request.target_map_yaw - current_yaw)

                if getattr(request, "measure_only", False):
                    self._publish_command(0.0, 0.0)
                    feedback = ShelfFineAlign.Feedback()
                    feedback.phase = "MEASUREMENT_READY"
                    feedback.measured_clearance = float(
                        corners.minimum_corner_clearance
                    )
                    feedback.yaw_error = float(abs(corners.shelf_yaw_error))
                    feedback.stable_samples = int(filter_samples)
                    goal_handle.publish_feedback(feedback)
                    goal_handle.succeed()
                    return self._result(
                        True,
                        TaskError.OK,
                        "front-surface measurement acquired; no motion commanded",
                        corners.minimum_corner_clearance,
                        corners.shelf_yaw_error,
                    )

                if not started:
                    if (
                        linear_speed > self.maximum_start_linear_speed
                        or angular_speed > self.maximum_start_angular_speed
                    ):
                        self._publish_command(0.0, 0.0)
                        await self._sleep()
                        continue
                    if corners.minimum_corner_clearance < self.minimum_start_clearance:
                        goal_handle.abort()
                        return self._result(
                            False,
                            TaskError.SAFETY_STOP,
                            "shelf is already inside the safe alignment start distance",
                            corners.minimum_corner_clearance,
                            corners.shelf_yaw_error,
                        )
                    if abs(map_yaw_error) > self.maximum_start_yaw_error:
                        goal_handle.abort()
                        return self._result(
                            False,
                            TaskError.INVALID_REQUEST,
                            "Nav2 stopped too far from the configured shelf heading",
                            corners.minimum_corner_clearance,
                            map_yaw_error,
                        )
                    started = True

                control_yaw_error = alignment_control_yaw_error(
                    map_yaw_error, corners.shelf_yaw_error
                )

                if corners.minimum_corner_clearance < self.minimum_safe_clearance:
                    self._publish_command(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        "filtered front-corner shelf clearance violated",
                        corners.minimum_corner_clearance,
                        control_yaw_error,
                    )

                yaw_only = bool(getattr(request, "yaw_only", False))
                command = compute_shelf_corner_command(
                    corners.left_clearance,
                    corners.right_clearance,
                    alignment_target_clearance(
                        request.target_clearance,
                        corners.minimum_corner_clearance,
                        yaw_only,
                    ),
                    request.clearance_tolerance,
                    control_yaw_error,
                    request.yaw_tolerance,
                    target_clearance_hysteresis=self.target_clearance_hysteresis,
                    yaw_align_gate=self.yaw_align_gate,
                    rotate_kp=self.rotate_kp,
                    approach_kp=self.approach_kp,
                    maximum_angular_speed=self.maximum_angular_speed,
                    maximum_linear_speed=self.maximum_linear_speed,
                    minimum_linear_speed=self.minimum_linear_speed,
                    minimum_angular_speed=self.minimum_angular_speed,
                )
                if command.phase == "TOO_CLOSE":
                    self._publish_command(0.0, 0.0)
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        "a front corner is too close for forward-only fine alignment",
                        corners.minimum_corner_clearance,
                        control_yaw_error,
                    )
                stable_samples = stable_samples + 1 if command.within_tolerance else 0
                self._publish_command(command.linear_x, command.angular_z)
                feedback = ShelfFineAlign.Feedback()
                feedback.phase = command.phase
                feedback.measured_clearance = float(corners.minimum_corner_clearance)
                feedback.yaw_error = float(abs(control_yaw_error))
                feedback.stable_samples = int(stable_samples)
                goal_handle.publish_feedback(feedback)
                if stable_samples >= self.settle_samples:
                    self._publish_command(0.0, 0.0)
                    alignment_succeeded = True
                    goal_handle.succeed()
                    return self._result(
                        True,
                        TaskError.OK,
                        (
                            "delivery front-plane fine alignment complete"
                            if use_plane_fit
                            else "shelf fine alignment complete"
                        ),
                        corners.minimum_corner_clearance,
                        control_yaw_error,
                    )
                await self._sleep()

            self._publish_command(0.0, 0.0)
            goal_handle.abort()
            with self._lock:
                filtered_corners = (
                    self._plane_filter.value
                    if self._active_use_plane_fit
                    else self._corner_filter.value
                )
            clearance = (
                filtered_corners.minimum_corner_clearance
                if filtered_corners is not None
                else 0.0
            )
            return self._result(
                False,
                TaskError.TIMEOUT,
                (
                    "delivery front-plane fine alignment timed out"
                    if use_plane_fit
                    else "shelf fine alignment timed out"
                ),
                clearance,
            )
        finally:
            self._publish_command(0.0, 0.0)
            terminal_guard_state = fine_alignment_guard_terminal_state(
                guard_requested, alignment_succeeded
            )
            if terminal_guard_state is not None:
                self._publish_localization_guard_state(terminal_guard_state)
            with self._lock:
                self._active_goal = False
                self._active_use_plane_fit = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShelfFineAlignNode()
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
