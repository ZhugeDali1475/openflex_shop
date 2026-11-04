"""Competition task executor for stop, dock, perceive, pick, and place flows."""

import math
import os
import time
from typing import Any, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from tf2_ros import Buffer, TransformException, TransformListener

from openarmx_task_msgs.action import (
    DetectObject,
    DockToStation,
    ExecuteTransport,
    MoveToShelfView,
    PickObject,
    PlaceObject,
    RetreatFromStation,
    ShelfFineAlign,
)
from openarmx_task_msgs.msg import TaskError

from .station_db import StationDatabase
from .error_contract import child_action_failure, stable_error_code
from .manipulation_contract import (
    make_nominal_detection_id,
    normalize_arm_name,
    require_detection_id,
)


class TaskExecutorNode(Node):
    """Owns the public task action and composes independent subsystem actions."""

    def __init__(self) -> None:
        super().__init__("openarmx_task_executor")

        self.declare_parameter("station_config", "")
        self.declare_parameter("station_mission", "")
        self.declare_parameter("action_name", "/execute_transport")
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("dock_action_name", "/dock_to_station")
        self.declare_parameter("shelf_align_action_name", "/shelf_fine_align")
        self.declare_parameter("retreat_action_name", "/retreat_from_station")
        self.declare_parameter("detect_action_name", "/detect_object")
        self.declare_parameter("pick_action_name", "/pick_object")
        self.declare_parameter("place_action_name", "/place_object")
        self.declare_parameter(
            "shelf_view_action_name", "/competition/move_to_shelf_view"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom_safe")
        self.declare_parameter("base_stop_linear_speed", 0.02)
        self.declare_parameter("base_stop_angular_speed", 0.03)
        self.declare_parameter("base_stop_hold_sec", 0.5)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("server_wait_sec", 2.0)
        self.declare_parameter("default_timeout_sec", 180.0)
        self.declare_parameter("enable_docking", True)
        self.declare_parameter("enable_detection", True)
        self.declare_parameter("enable_shelf_view", False)
        self.declare_parameter("allow_nominal_detection_fallback", False)
        self.declare_parameter("publish_stop_cmd", False)
        self.declare_parameter("stop_cmd_topic", "/cmd_vel")
        self.declare_parameter("default_arm_name", "left")
        self.declare_parameter("max_manipulation_dof", 8)
        self.declare_parameter("lift_joint_min_position", -0.650)
        self.declare_parameter("lift_joint_max_position", 0.300)
        self.declare_parameter("enable_post_pick_retreat", False)
        self.declare_parameter("post_pick_retreat_speed", 0.04)
        self.declare_parameter("post_pick_retreat_timeout_sec", 5.0)

        station_config = str(self.get_parameter("station_config").value)
        if not station_config:
            station_config = os.path.join(
                get_package_share_directory("openarmx_task_executor"),
                "config",
                "stations.yaml",
            )
        self.database = StationDatabase.from_yaml(
            station_config,
            str(self.get_parameter("station_mission").value),
        )

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.base_stop_linear_speed = float(
            self.get_parameter("base_stop_linear_speed").value
        )
        self.base_stop_angular_speed = float(
            self.get_parameter("base_stop_angular_speed").value
        )
        self.base_stop_hold_sec = float(self.get_parameter("base_stop_hold_sec").value)
        self.server_wait_sec = float(self.get_parameter("server_wait_sec").value)
        self.default_timeout_sec = float(self.get_parameter("default_timeout_sec").value)
        self.enable_docking = bool(self.get_parameter("enable_docking").value)
        self.enable_detection = bool(self.get_parameter("enable_detection").value)
        self.enable_shelf_view = bool(
            self.get_parameter("enable_shelf_view").value
        )
        self.allow_nominal_detection_fallback = bool(
            self.get_parameter("allow_nominal_detection_fallback").value
        )
        self.publish_stop_cmd = bool(self.get_parameter("publish_stop_cmd").value)
        self.default_arm_name = str(self.get_parameter("default_arm_name").value)
        self.default_arm_name = normalize_arm_name(self.default_arm_name)
        self.max_manipulation_dof = int(
            self.get_parameter("max_manipulation_dof").value
        )
        if self.max_manipulation_dof not in (7, 8):
            raise ValueError("max_manipulation_dof must be 7 or 8; base motion is not allowed here")
        self.lift_joint_min_position = float(
            self.get_parameter("lift_joint_min_position").value
        )
        self.lift_joint_max_position = float(
            self.get_parameter("lift_joint_max_position").value
        )
        if self.lift_joint_min_position >= self.lift_joint_max_position:
            raise ValueError("lift joint position limits are invalid")
        self.enable_post_pick_retreat = bool(
            self.get_parameter("enable_post_pick_retreat").value
        )
        self.post_pick_retreat_speed = float(
            self.get_parameter("post_pick_retreat_speed").value
        )
        self.post_pick_retreat_timeout_sec = float(
            self.get_parameter("post_pick_retreat_timeout_sec").value
        )
        if self.enable_post_pick_retreat and (
            self.post_pick_retreat_speed <= 0.0
            or self.post_pick_retreat_timeout_sec <= 0.0
        ):
            raise ValueError("post-pick retreat parameters must be positive and complete")

        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("nav_action_name").value)
        )
        self.dock_client = ActionClient(
            self, DockToStation, str(self.get_parameter("dock_action_name").value)
        )
        self.shelf_align_client = ActionClient(
            self,
            ShelfFineAlign,
            str(self.get_parameter("shelf_align_action_name").value),
        )
        self.retreat_client = ActionClient(
            self,
            RetreatFromStation,
            str(self.get_parameter("retreat_action_name").value),
        )
        self.detect_client = ActionClient(
            self, DetectObject, str(self.get_parameter("detect_action_name").value)
        )
        self.pick_client = ActionClient(
            self, PickObject, str(self.get_parameter("pick_action_name").value)
        )
        self.place_client = ActionClient(
            self, PlaceObject, str(self.get_parameter("place_action_name").value)
        )
        self.shelf_view_client = ActionClient(
            self,
            MoveToShelfView,
            str(self.get_parameter("shelf_view_action_name").value),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_odom_time = 0.0
        self.last_linear_speed = math.inf
        self.last_angular_speed = math.inf
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
        )
        self.stop_pub = self.create_publisher(
            Twist, str(self.get_parameter("stop_cmd_topic").value), 10
        )

        self._active_goal = False
        self._last_failure_message = ""
        self._server = ActionServer(
            self,
            ExecuteTransport,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(f"Loaded station database: {station_config}")
        self.get_logger().info(
            "Task executor ready; MoveIt will connect through PickObject/PlaceObject actions"
        )
        if self.enable_post_pick_retreat:
            self.get_logger().info(
                "Per-station retreat enabled: "
                f"speed={self.post_pick_retreat_speed:.3f}m/s"
            )
        if self.enable_shelf_view:
            self.get_logger().info(
                "Direct shelf-level observation posture is enabled before detection; "
                "MoveIt planning remains reserved for the grasp trajectories"
            )

    def _odom_callback(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.last_linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        self.last_angular_speed = abs(angular.z)
        self.last_odom_time = time.monotonic()

    def goal_callback(self, _goal_request: ExecuteTransport.Goal) -> GoalResponse:
        if self._active_goal:
            self.get_logger().warn("Rejecting a transport task because another task is active")
            return GoalResponse.REJECT
        self._active_goal = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _feedback(self, goal_handle, phase: str, progress: float, retry_count: int = 0) -> None:
        feedback = ExecuteTransport.Feedback()
        feedback.phase = phase
        feedback.progress = max(0.0, min(1.0, float(progress)))
        feedback.retry_count = max(0, min(255, int(retry_count)))
        goal_handle.publish_feedback(feedback)

    def _result(self, success: bool, error_code: int, message: str) -> ExecuteTransport.Result:
        result = ExecuteTransport.Result()
        result.success = success
        result.error_code = int(error_code)
        result.message = message
        return result

    def _fail(self, error_code: int, message: str, fallback: int = None) -> int:
        fallback_code = int(error_code if fallback is None else fallback)
        code = stable_error_code(error_code, fallback_code)
        self._last_failure_message = str(message).strip()
        return code

    def _child_fail(self, payload, fallback: int, default_message: str) -> int:
        failure = child_action_failure(payload, fallback, default_message)
        self._last_failure_message = failure.message
        return failure.code

    def _cancelled(self, goal_handle) -> bool:
        return bool(goal_handle.is_cancel_requested)

    def _deadline(self, request: ExecuteTransport.Goal) -> float:
        timeout = float(request.timeout_sec)
        if timeout <= 0.0:
            timeout = self.default_timeout_sec
        return time.monotonic() + timeout

    def _expired(self, deadline: float) -> bool:
        return time.monotonic() >= deadline

    async def _sleep(self, seconds: float) -> None:
        """Yield an async action callback using an rclpy timer, not asyncio."""
        future = Future()
        timer_holder = []

        def wake() -> None:
            if timer_holder:
                timer_holder[0].cancel()
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(max(0.001, seconds), wake)
        timer_holder.append(timer)
        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    async def _wait_for_server(self, client: ActionClient, goal_handle, deadline: float) -> bool:
        server_deadline = min(deadline, time.monotonic() + self.server_wait_sec)
        while not client.server_is_ready():
            if self._cancelled(goal_handle) or time.monotonic() >= server_deadline:
                return False
            client.wait_for_server(
                timeout_sec=min(0.2, max(0.01, server_deadline - time.monotonic()))
            )
            await self._sleep(0.02)
        return True

    async def _call_action(
        self, client: ActionClient, goal, goal_handle, deadline: float
    ) -> Optional[Tuple[int, Any]]:
        if not await self._wait_for_server(client, goal_handle, deadline):
            return None
        send_future = client.send_goal_async(goal)
        child_handle = await send_future
        if not child_handle.accepted:
            return None
        result_future = child_handle.get_result_async()
        while not result_future.done():
            if self._cancelled(goal_handle) or self._expired(deadline):
                cancel_future = child_handle.cancel_goal_async()
                await cancel_future
                return None
            await self._sleep(0.05)
        child_result = result_future.result()
        return child_result.status, child_result.result

    async def _preflight(self, source, destination, goal_handle, deadline: float) -> int:
        """Verify every required subsystem before allowing the base to move."""
        checks = [
            (self.nav_client, TaskError.NAVIGATION_UNAVAILABLE, "Nav2"),
            (self.pick_client, TaskError.MOVEIT_UNAVAILABLE, "pick MoveIt server"),
            (self.place_client, TaskError.MOVEIT_UNAVAILABLE, "place MoveIt server"),
        ]
        if self.enable_docking:
            alignment_clients = set()
            for station in (source, destination):
                if not station.docking_enabled:
                    continue
                if station.alignment_mode == "shelf_scan":
                    alignment_clients.add((self.shelf_align_client, "shelf alignment server"))
                else:
                    alignment_clients.add((self.dock_client, "marker docking server"))
            for client, label in alignment_clients:
                checks.append((client, TaskError.DOCKING_UNAVAILABLE, label))
        if self.enable_post_pick_retreat and (
            source.retreat_after_pick or destination.retreat_after_place
        ):
            checks.append(
                (self.retreat_client, TaskError.SAFETY_STOP, "station retreat server")
            )
        if self.enable_detection:
            checks.append(
                (
                    self.detect_client,
                    TaskError.DETECTION_UNAVAILABLE,
                    "object detector",
                )
            )
        if self.enable_shelf_view:
            checks.append(
                (
                    self.shelf_view_client,
                    TaskError.MOVEIT_UNAVAILABLE,
                    "shelf-view trajectory bridge",
                )
            )

        for client, error_code, label in checks:
            available = await self._wait_for_server(client, goal_handle, deadline)
            if available:
                continue
            if (
                client is self.detect_client
                and self.allow_nominal_detection_fallback
            ):
                self.get_logger().warn(
                    "Object detector unavailable during preflight; nominal fallback is enabled"
                )
                continue
            if self._cancelled(goal_handle):
                return self._fail(TaskError.CANCELLED, "preflight was cancelled")
            if self._expired(deadline):
                return self._fail(TaskError.TIMEOUT, "preflight timed out")
            self.get_logger().error(f"Preflight failed: {label} is unavailable")
            return self._fail(error_code, f"preflight failed: {label} is unavailable")
        return TaskError.OK

    async def _wait_for_base_stop(self, goal_handle, deadline: float) -> bool:
        stable_since: Optional[float] = None
        while True:
            if self._cancelled(goal_handle) or self._expired(deadline):
                return False
            now = time.monotonic()
            fresh = now - self.last_odom_time <= self.odom_timeout_sec
            stopped = (
                self.last_linear_speed <= self.base_stop_linear_speed
                and self.last_angular_speed <= self.base_stop_angular_speed
            )
            if fresh and stopped:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.base_stop_hold_sec:
                    return True
            else:
                stable_since = None
            if self.publish_stop_cmd:
                self.stop_pub.publish(Twist())
            await self._sleep(0.05)

    async def _retreat_from_station(
        self,
        station,
        goal_handle,
        deadline: float,
        feedback_phase: str,
        feedback_progress: float,
    ) -> int:
        self._feedback(goal_handle, feedback_phase, feedback_progress)
        goal = RetreatFromStation.Goal()
        goal.station_id = station.station_id
        goal.distance = float(station.retreat_distance)
        goal.max_speed = min(
            float(station.retreat_speed), self.post_pick_retreat_speed
        )
        goal.timeout_sec = min(
            self.post_pick_retreat_timeout_sec,
            max(0.0, deadline - time.monotonic()),
        )
        result = await self._call_action(
            self.retreat_client, goal, goal_handle, deadline
        )
        if result is None:
            return self._fail(
                TaskError.CANCELLED
                if self._cancelled(goal_handle)
                else TaskError.SAFETY_STOP,
                "station retreat was cancelled"
                if self._cancelled(goal_handle)
                else "station retreat server unavailable or timed out",
            )
        status, payload = result
        if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
            return self._child_fail(
                payload, TaskError.SAFETY_STOP, "station retreat failed"
            )
        self.get_logger().info(
            f"Station retreat complete at {station.station_id}: "
            f"{payload.traveled_distance:.3f}m"
        )
        return TaskError.OK

    async def _move_to_shelf_view(
        self,
        source,
        slot,
        arm_name: str,
        source_level: int,
        goal_handle,
        deadline: float,
    ) -> int:
        goal = MoveToShelfView.Goal()
        goal.station_id = source.station_id
        goal.shelf_level = int(source_level)
        goal.arm = (
            MoveToShelfView.Goal.RIGHT
            if arm_name == "right"
            else MoveToShelfView.Goal.LEFT
        )
        goal.use_explicit_lift_position = slot.nominal_lift_position is not None
        goal.lift_position = (
            float(slot.nominal_lift_position)
            if slot.nominal_lift_position is not None
            else 0.0
        )
        goal.velocity_scale = 0.20
        goal.acceleration_scale = 0.20
        goal.planning_timeout_sec = min(
            5.0, max(0.1, deadline - time.monotonic())
        )
        goal.timeout_sec = min(30.0, max(0.1, deadline - time.monotonic()))
        goal.execute = True
        result = await self._call_action(
            self.shelf_view_client, goal, goal_handle, deadline
        )
        if result is None:
            if self._cancelled(goal_handle):
                return self._fail(TaskError.CANCELLED, "shelf-view request cancelled")
            return self._fail(
                TaskError.TIMEOUT
                if self._expired(deadline)
                else TaskError.MOVEIT_UNAVAILABLE,
                "shelf-view request timed out"
                if self._expired(deadline)
                else "shelf-view trajectory bridge unavailable",
            )
        status, payload = result
        if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
            return self._child_fail(
                payload, TaskError.PLANNING_FAILED, "shelf-view direct posture failed"
            )
        self.get_logger().info(
            f"Direct shelf-view posture reached via trajectory bridge at "
            f"{source.station_id}, level={source_level}, "
            f"lift={payload.lift_position:.4f} (no planning in this phase)"
        )
        return TaskError.OK

    def _pose_in_base_frame(self, pose):
        if pose.header.frame_id == self.base_frame:
            return pose
        try:
            return self.tf_buffer.transform(
                pose, self.base_frame, timeout=Duration(seconds=0.5)
            )
        except TransformException as exc:
            self.get_logger().error(
                f"Cannot transform object pose {pose.header.frame_id}->{self.base_frame}: {exc}"
            )
            return None

    async def _navigate(
        self, station, goal_handle, deadline: float, feedback_progress: float
    ) -> int:
        goal = NavigateToPose.Goal()
        goal.pose = station.nav_pose.to_msg(self.get_clock().now().to_msg())
        result = await self._call_action(self.nav_client, goal, goal_handle, deadline)
        if result is None:
            if self._cancelled(goal_handle):
                return self._fail(TaskError.CANCELLED, "navigation request cancelled")
            return self._fail(
                TaskError.TIMEOUT
                if self._expired(deadline)
                else TaskError.NAVIGATION_UNAVAILABLE,
                "navigation request timed out"
                if self._expired(deadline)
                else "Nav2 action server unavailable",
            )
        status, _ = result
        if status != GoalStatus.STATUS_SUCCEEDED:
            return self._fail(TaskError.NAVIGATION_FAILED, "Nav2 goal failed")
        self._feedback(goal_handle, "NAVIGATION_COMPLETE", feedback_progress)
        return TaskError.OK

    async def _dock(
        self, station, goal_handle, deadline: float, feedback_progress: float
    ) -> int:
        if not self.enable_docking or not station.docking_enabled:
            self._feedback(goal_handle, "DOCKING_SKIPPED", feedback_progress)
            return TaskError.OK
        if station.alignment_mode == "shelf_scan":
            goal = ShelfFineAlign.Goal()
            goal.station_id = station.station_id
            goal.target_clearance = station.shelf_target_clearance
            goal.clearance_tolerance = station.shelf_clearance_tolerance
            goal.target_map_yaw = station.nav_pose.yaw
            goal.yaw_tolerance = station.yaw_tolerance
            goal.measure_only = False
            goal.timeout_sec = min(
                station.shelf_alignment_timeout,
                max(0.0, deadline - time.monotonic()),
            )
            result = await self._call_action(
                self.shelf_align_client, goal, goal_handle, deadline
            )
            if result is None:
                return self._fail(
                    TaskError.CANCELLED
                    if self._cancelled(goal_handle)
                    else TaskError.DOCKING_UNAVAILABLE,
                    "shelf alignment cancelled"
                    if self._cancelled(goal_handle)
                    else "shelf alignment server unavailable or timed out",
                )
            status, payload = result
            if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
                return self._child_fail(
                    payload, TaskError.DOCKING_FAILED, "shelf alignment failed"
                )
            self._feedback(goal_handle, "SHELF_ALIGNMENT_COMPLETE", feedback_progress)
            return TaskError.OK
        goal = DockToStation.Goal()
        goal.station_id = station.station_id
        goal.marker_id = station.marker_id
        goal.position_tolerance = station.position_tolerance
        goal.yaw_tolerance = station.yaw_tolerance
        goal.timeout_sec = max(0.0, deadline - time.monotonic())
        if station.target_marker_pose is not None:
            goal.use_target_marker_pose = True
            goal.target_marker_pose = station.target_marker_pose.to_msg(
                self.get_clock().now().to_msg()
            ).pose
        result = await self._call_action(self.dock_client, goal, goal_handle, deadline)
        if result is None:
            return self._fail(
                TaskError.CANCELLED
                if self._cancelled(goal_handle)
                else TaskError.DOCKING_UNAVAILABLE,
                "station docking cancelled"
                if self._cancelled(goal_handle)
                else "station docking server unavailable or timed out",
            )
        status, payload = result
        if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
            return self._child_fail(
                payload, TaskError.DOCKING_FAILED, "station docking failed"
            )
        self._feedback(goal_handle, "DOCKING_COMPLETE", feedback_progress)
        return TaskError.OK

    async def _detect(
        self, request, station, slot, arm_name: str, goal_handle, deadline: float
    ):
        nominal_pose = request.nominal_object_pose
        has_nominal_pose = bool(
            request.use_nominal_object_pose and nominal_pose.header.frame_id
        )
        if not self.enable_detection:
            return (
                nominal_pose
                if has_nominal_pose
                else slot.pose.to_msg(self.get_clock().now().to_msg()),
                make_nominal_detection_id(arm_name),
                TaskError.OK,
            )
        goal = DetectObject.Goal()
        goal.object_id = request.object_id
        goal.station_id = station.station_id
        goal.slot_id = request.source_slot
        goal.arm_name = arm_name
        goal.use_shelf_level = bool(request.use_source_level)
        goal.shelf_level = int(request.source_level)
        goal.use_nominal_object_pose = has_nominal_pose
        if has_nominal_pose:
            goal.nominal_object_pose = nominal_pose
        goal.min_confidence = 0.5
        goal.timeout_sec = max(0.0, deadline - time.monotonic())
        result = await self._call_action(self.detect_client, goal, goal_handle, deadline)
        if result is None:
            if self.allow_nominal_detection_fallback:
                self.get_logger().warn("Detection server unavailable; using nominal slot pose")
                return (
                    nominal_pose
                    if has_nominal_pose
                    else slot.pose.to_msg(self.get_clock().now().to_msg()),
                    make_nominal_detection_id(arm_name),
                    TaskError.OK,
                )
            return None, "", self._fail(
                TaskError.DETECTION_UNAVAILABLE,
                "object detector unavailable or timed out",
            )
        status, payload = result
        if status != GoalStatus.STATUS_SUCCEEDED or not payload.found:
            return None, "", self._child_fail(
                payload, TaskError.OBJECT_NOT_FOUND, "object was not detected"
            )
        try:
            detection_id = require_detection_id(payload.detection_id)
        except ValueError as exc:
            self.get_logger().error(f"Detector returned an invalid detection_id: {exc}")
            return None, "", self._fail(TaskError.INVALID_REQUEST, str(exc))
        return payload.object_pose, detection_id, TaskError.OK

    async def _pick(
        self, request, object_pose, detection_id: str, arm_name: str,
        station, slot, source_level,
        goal_handle, deadline: float
    ):
        goal = PickObject.Goal()
        goal.object_id = request.object_id
        goal.detection_id = detection_id
        goal.object_pose = object_pose
        goal.slot_id = request.source_slot
        goal.use_shelf_level = source_level >= 0
        goal.shelf_level = int(source_level)
        nominal_lift = self.database.level_lift_position(
            station.station_id, slot, source_level
        )
        if nominal_lift is not None and not (
            self.lift_joint_min_position
            <= nominal_lift
            <= self.lift_joint_max_position
        ):
            self.get_logger().error(
                f"Calibrated lift position {nominal_lift:.3f} for "
                f"{station.station_id} level {source_level} is outside "
                f"[{self.lift_joint_min_position:.3f}, "
                f"{self.lift_joint_max_position:.3f}]"
            )
            return self._fail(
                TaskError.INVALID_REQUEST,
                "calibrated lift position is outside configured limits",
            )
        goal.use_nominal_lift_position = nominal_lift is not None
        if nominal_lift is not None:
            goal.nominal_lift_position = float(nominal_lift)
        goal.max_dof = int(self.max_manipulation_dof)
        goal.arm_name = arm_name
        goal.grasp_strategy = slot.grasp_strategy
        goal.retreat_to_transport = True
        goal.timeout_sec = max(0.0, deadline - time.monotonic())
        result = await self._call_action(self.pick_client, goal, goal_handle, deadline)
        if result is None:
            return self._fail(
                TaskError.CANCELLED
                if self._cancelled(goal_handle)
                else TaskError.MOVEIT_UNAVAILABLE,
                "pick request cancelled"
                if self._cancelled(goal_handle)
                else "pick action server unavailable or timed out",
            )
        status, payload = result
        if (
            status != GoalStatus.STATUS_SUCCEEDED
            or not payload.success
            or not payload.object_attached
        ):
            return self._child_fail(
                payload, TaskError.PICK_FAILED, "pick did not attach the object"
            )
        return TaskError.OK

    async def _place(self, request, station, slot, goal_handle, deadline: float):
        destination_slot = self.database.exact_slot(
            station.station_id, str(request.destination_slot)
        )
        place_spec = destination_slot.pose if destination_slot is not None else station.place_pose
        if place_spec is None:
            return self._fail(TaskError.INVALID_REQUEST, "place pose is not configured")
        place_pose = place_spec.to_msg(self.get_clock().now().to_msg())
        place_pose = self._pose_in_base_frame(place_pose)
        if place_pose is None:
            return self._fail(TaskError.TF_UNAVAILABLE, "place pose TF transform failed")
        goal = PlaceObject.Goal()
        goal.object_id = request.object_id
        goal.place_pose = place_pose
        goal.slot_id = request.destination_slot
        place_level = (
            destination_slot.level if destination_slot is not None else -1
        )
        goal.use_shelf_level = place_level >= 0
        goal.shelf_level = int(place_level)
        nominal_lift = self.database.level_lift_position(
            station.station_id, destination_slot, place_level
        )
        if nominal_lift is not None and not (
            self.lift_joint_min_position
            <= nominal_lift
            <= self.lift_joint_max_position
        ):
            return self._fail(
                TaskError.INVALID_REQUEST,
                "place lift position is outside configured limits",
            )
        goal.use_nominal_lift_position = nominal_lift is not None
        if nominal_lift is not None:
            goal.nominal_lift_position = float(nominal_lift)
        goal.max_dof = int(self.max_manipulation_dof)
        goal.arm_name = slot.arm_name or self.default_arm_name
        goal.place_strategy = station.place_strategy
        goal.release_object = True
        goal.timeout_sec = max(0.0, deadline - time.monotonic())
        result = await self._call_action(self.place_client, goal, goal_handle, deadline)
        if result is None:
            return self._fail(
                TaskError.CANCELLED
                if self._cancelled(goal_handle)
                else TaskError.MOVEIT_UNAVAILABLE,
                "place request cancelled"
                if self._cancelled(goal_handle)
                else "place action server unavailable or timed out",
            )
        status, payload = result
        if (
            status != GoalStatus.STATUS_SUCCEEDED
            or not payload.success
            or not payload.object_released
        ):
            return self._child_fail(
                payload, TaskError.PLACE_FAILED, "place did not release the object"
            )
        return TaskError.OK

    async def _phase_with_retries(
        self,
        operation,
        goal_handle,
        deadline: float,
        phase: str,
        start: float,
        retries: int,
    ):
        attempts = max(0, retries) + 1
        for attempt in range(attempts):
            self._feedback(goal_handle, phase, start, attempt)
            code = await operation()
            if code == TaskError.OK:
                return code
            if code == TaskError.CANCELLED or self._cancelled(goal_handle):
                return TaskError.CANCELLED
            if attempt + 1 < attempts:
                self.get_logger().warn(f"{phase} failed with code {code}; retrying")
        return code

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        try:
            self._last_failure_message = ""
            deadline = self._deadline(request)
            retries = int(request.max_retries)
            source = self.database.get(request.source_station)
            destination = self.database.get(request.destination_station)
            slot_key = str(request.source_slot).strip() or request.object_id
            slot = self.database.slot(request.source_station, slot_key)
            if source is None or destination is None:
                result = self._result(
                    False,
                    TaskError.STATION_NOT_FOUND,
                    "source or destination station not found",
                )
                goal_handle.abort()
                return result
            if slot is None:
                result = self._result(
                    False,
                    TaskError.INVALID_REQUEST,
                    "object slot is not configured at source station",
                )
                goal_handle.abort()
                return result
            try:
                arm_name = normalize_arm_name(slot.arm_name)
            except ValueError as exc:
                result = self._result(False, TaskError.INVALID_REQUEST, str(exc))
                goal_handle.abort()
                return result
            source_level = (
                int(request.source_level)
                if request.use_source_level
                else int(slot.level)
            )
            if self.enable_shelf_view and source_level < 0:
                result = self._result(
                    False,
                    TaskError.INVALID_REQUEST,
                    "shelf-view is enabled but the source shelf level is unknown",
                )
                goal_handle.abort()
                return result
            if destination.place_pose is None:
                result = self._result(
                    False,
                    TaskError.INVALID_REQUEST,
                    "destination station has no place_pose",
                )
                goal_handle.abort()
                return result

            self._feedback(goal_handle, "PREFLIGHT", 0.02)
            code = await self._preflight(
                source, destination, goal_handle, deadline
            )
            if code == TaskError.OK:
                code = await self._phase_with_retries(
                    lambda: self._navigate(source, goal_handle, deadline, 0.12),
                    goal_handle,
                    deadline,
                    "NAVIGATE_TO_PICK",
                    0.05,
                    retries,
                )
            if code == TaskError.OK:
                self._feedback(goal_handle, "WAIT_FOR_BASE_STOP", 0.18)
                if not await self._wait_for_base_stop(goal_handle, deadline):
                    code = self._fail(
                        TaskError.CANCELLED
                        if self._cancelled(goal_handle)
                        else TaskError.BASE_NOT_STOPPED,
                        "task cancelled while waiting for the base to stop"
                        if self._cancelled(goal_handle)
                        else "base did not remain stopped before shelf alignment",
                    )
            if code == TaskError.OK:
                code = await self._phase_with_retries(
                    lambda: self._dock(source, goal_handle, deadline, 0.27),
                    goal_handle, deadline, "FINE_DOCK_PICK", 0.20, retries,
                )
            if code == TaskError.OK:
                self._feedback(goal_handle, "WAIT_AFTER_DOCK", 0.30)
                if not await self._wait_for_base_stop(goal_handle, deadline):
                    code = self._fail(
                        TaskError.CANCELLED
                        if self._cancelled(goal_handle)
                        else TaskError.BASE_NOT_STOPPED,
                        "task cancelled while waiting after shelf alignment"
                        if self._cancelled(goal_handle)
                        else "base did not remain stopped after shelf alignment",
                    )
            if code == TaskError.OK and self.enable_shelf_view:
                self._feedback(goal_handle, "MOVE_TO_SHELF_VIEW", 0.33)
                code = await self._move_to_shelf_view(
                    source,
                    slot,
                    arm_name,
                    source_level,
                    goal_handle,
                    deadline,
                )
            object_pose = None
            detection_id = ""
            if code == TaskError.OK:
                self._feedback(goal_handle, "DETECT_OBJECT", 0.35)
                object_pose, detection_id, code = await self._detect(
                    request, source, slot, arm_name, goal_handle, deadline
                )
            if code == TaskError.OK:
                object_pose = self._pose_in_base_frame(object_pose)
                if object_pose is None:
                    code = self._fail(
                        TaskError.TF_UNAVAILABLE,
                        "detected object pose TF transform failed",
                    )
            if code == TaskError.OK:
                self._feedback(goal_handle, "PICK_OBJECT", 0.48)
                code = await self._pick(
                    request, object_pose, detection_id, arm_name,
                    source, slot, source_level,
                    goal_handle, deadline
                )
            retreat_after_pick = (
                self.enable_post_pick_retreat and source.retreat_after_pick
            )
            if code == TaskError.OK and retreat_after_pick:
                code = await self._retreat_from_station(
                    source,
                    goal_handle,
                    deadline,
                    "RETREAT_FROM_PICK",
                    0.52,
                )
            if code == TaskError.OK and retreat_after_pick:
                self._feedback(goal_handle, "WAIT_AFTER_PICK_RETREAT", 0.54)
                if not await self._wait_for_base_stop(goal_handle, deadline):
                    code = self._fail(
                        TaskError.CANCELLED
                        if self._cancelled(goal_handle)
                        else TaskError.BASE_NOT_STOPPED,
                        "task cancelled while waiting after pick retreat"
                        if self._cancelled(goal_handle)
                        else "base did not remain stopped after pick retreat",
                    )
            if code == TaskError.OK:
                code = await self._phase_with_retries(
                    lambda: self._navigate(destination, goal_handle, deadline, 0.67),
                    goal_handle, deadline, "NAVIGATE_TO_PLACE", 0.55, retries,
                )
            if code == TaskError.OK:
                self._feedback(goal_handle, "WAIT_FOR_BASE_STOP", 0.73)
                if not await self._wait_for_base_stop(goal_handle, deadline):
                    code = self._fail(
                        TaskError.CANCELLED
                        if self._cancelled(goal_handle)
                        else TaskError.BASE_NOT_STOPPED,
                        "task cancelled while waiting at the destination"
                        if self._cancelled(goal_handle)
                        else "base did not remain stopped at the destination",
                    )
            if code == TaskError.OK:
                code = await self._phase_with_retries(
                    lambda: self._dock(destination, goal_handle, deadline, 0.80),
                    goal_handle, deadline, "FINE_DOCK_PLACE", 0.75, retries,
                )
            if code == TaskError.OK:
                self._feedback(goal_handle, "WAIT_AFTER_DOCK", 0.84)
                if not await self._wait_for_base_stop(goal_handle, deadline):
                    code = self._fail(
                        TaskError.CANCELLED
                        if self._cancelled(goal_handle)
                        else TaskError.BASE_NOT_STOPPED,
                        "task cancelled while waiting after destination alignment"
                        if self._cancelled(goal_handle)
                        else "base did not remain stopped after destination alignment",
                    )
            if code == TaskError.OK:
                self._feedback(goal_handle, "PLACE_OBJECT", 0.90)
                code = await self._place(request, destination, slot, goal_handle, deadline)
            retreat_after_place = (
                self.enable_post_pick_retreat and destination.retreat_after_place
            )
            if code == TaskError.OK and retreat_after_place:
                code = await self._retreat_from_station(
                    destination,
                    goal_handle,
                    deadline,
                    "RETREAT_FROM_PLACE",
                    0.94,
                )

            if code == TaskError.OK:
                self._feedback(goal_handle, "DONE", 1.0)
                goal_handle.succeed()
                return self._result(True, TaskError.OK, "transport task completed")
            if code == TaskError.CANCELLED or self._cancelled(goal_handle):
                goal_handle.canceled()
                return self._result(False, TaskError.CANCELLED, "transport task cancelled")
            goal_handle.abort()
            return self._result(
                False,
                code,
                self._last_failure_message
                or f"transport task failed with error code {code}",
            )
        except Exception as exc:  # Keep the public action deterministic on unexpected failures.
            self.get_logger().error(f"Unhandled task executor failure: {exc}")
            goal_handle.abort()
            return self._result(False, TaskError.INTERNAL_ERROR, str(exc))
        finally:
            self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskExecutorNode()
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
