"""Execute guarded, taught shelf postures without invoking MoveIt planning."""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Dict, Iterable, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from openarmx_task_msgs.action import MoveToShelfView
from openarmx_task_msgs.msg import TaskError
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .station_db import StationDatabase


ARM_JOINTS = {
    side: tuple(f"openarmx_{side}_joint{index}" for index in range(1, 8))
    for side in ("left", "right")
}
JOINT_POSITION_LIMITS = {
    "lift_joint": (-0.650, 0.300),
    "openarmx_left_joint1": (-3.344396, 0.905604),
    "openarmx_left_joint2": (-3.270796, 0.129204),
    "openarmx_right_joint1": (-1.25, 3.0),
    "openarmx_right_joint2": (-0.129204, 3.270796),
}
for _side in ("left", "right"):
    JOINT_POSITION_LIMITS.update(
        {
            f"openarmx_{_side}_joint3": (-1.57, 1.57),
            f"openarmx_{_side}_joint4": (0.0, 1.8),
            f"openarmx_{_side}_joint5": (-1.5, 1.5),
            f"openarmx_{_side}_joint6": (-0.75, 0.75),
            f"openarmx_{_side}_joint7": (-1.4, 1.4),
        }
    )
CURRENT_POSITION_LIMIT_TOLERANCE = 0.02


def build_direct_trajectory_goal(
    joint_names: Iterable[str],
    joint_positions: Iterable[float],
    duration_sec: float,
) -> FollowJointTrajectory.Goal:
    """Build a one-point trajectory that the bridge interpolates monotonically."""
    names = [str(name) for name in joint_names]
    positions = [float(value) for value in joint_positions]
    duration = float(duration_sec)
    if not names or len(names) != len(positions):
        raise ValueError("direct posture needs matching joint names and positions")
    if len(set(names)) != len(names):
        raise ValueError("direct posture contains duplicate joint names")
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("direct posture duration must be positive and finite")
    for name, value in zip(names, positions):
        limits = JOINT_POSITION_LIMITS.get(name)
        if limits is None:
            raise ValueError(f"unsupported direct-posture joint '{name}'")
        if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
            raise ValueError(
                f"direct-posture target {name}={value} is outside "
                f"[{limits[0]}, {limits[1]}]"
            )

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = positions
    seconds = int(duration)
    point.time_from_start.sec = seconds
    point.time_from_start.nanosec = int(round((duration - seconds) * 1e9))
    if point.time_from_start.nanosec >= 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec -= 1_000_000_000
    goal.trajectory.points = [point]
    return goal


def direct_posture_duration(
    current_by_name: Dict[str, float],
    target_by_name: Dict[str, float],
    *,
    velocity_scale: float,
    maximum_arm_speed: float,
    maximum_lift_speed: float,
    minimum_duration: float,
    lift_joint_name: str = "lift_joint",
) -> float:
    """Return one shared duration so all direct posture domains finish together."""
    scale = float(velocity_scale)
    if not math.isfinite(scale) or scale <= 0.0:
        scale = 0.1
    scale = min(1.0, scale)
    duration = float(minimum_duration)
    for name, target in target_by_name.items():
        current = float(current_by_name[name])
        speed = maximum_lift_speed if name == lift_joint_name else maximum_arm_speed
        duration = max(duration, abs(float(target) - current) / (speed * scale))
    return duration


class ShelfViewNode(Node):
    """Hold the base and execute one or both taught arms plus the lift directly."""

    def __init__(self) -> None:
        super().__init__("shelf_view")
        self.declare_parameter("station_config", "")
        self.declare_parameter("station_mission", "")
        self.declare_parameter("action_name", "/competition/move_to_shelf_view")
        self.declare_parameter(
            "left_trajectory_action_name",
            "/left_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "right_trajectory_action_name",
            "/right_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "lift_trajectory_action_name",
            "/lift_joint_controller/follow_joint_trajectory",
        )
        self.declare_parameter("odom_topic", "/odom_safe")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("default_arm_name", "right")
        self.declare_parameter("base_stop_linear_speed", 0.02)
        self.declare_parameter("base_stop_angular_speed", 0.03)
        self.declare_parameter("base_stop_hold_sec", 0.5)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("joint_state_timeout_sec", 0.5)
        self.declare_parameter("lift_joint_name", "lift_joint")
        self.declare_parameter("lift_joint_min_position", -0.650)
        self.declare_parameter("lift_joint_max_position", 0.300)
        self.declare_parameter("lift_position_tolerance", 0.005)
        self.declare_parameter("arm_position_tolerance", 0.03)
        self.declare_parameter("controller_wait_sec", 3.0)
        self.declare_parameter("maximum_arm_speed", 5.0)
        self.declare_parameter("maximum_lift_speed", 0.10)
        self.declare_parameter("minimum_trajectory_sec", 0.5)
        self.declare_parameter("default_velocity_scale", 0.10)
        self.declare_parameter("default_timeout_sec", 30.0)

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

        self.default_arm_name = str(
            self.get_parameter("default_arm_name").value
        ).strip().lower()
        self.base_linear_limit = float(
            self.get_parameter("base_stop_linear_speed").value
        )
        self.base_angular_limit = float(
            self.get_parameter("base_stop_angular_speed").value
        )
        self.base_hold_sec = float(self.get_parameter("base_stop_hold_sec").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.joint_state_timeout_sec = float(
            self.get_parameter("joint_state_timeout_sec").value
        )
        self.lift_joint_name = str(self.get_parameter("lift_joint_name").value)
        self.lift_min = float(
            self.get_parameter("lift_joint_min_position").value
        )
        self.lift_max = float(
            self.get_parameter("lift_joint_max_position").value
        )
        self.lift_tolerance = float(
            self.get_parameter("lift_position_tolerance").value
        )
        self.arm_tolerance = float(
            self.get_parameter("arm_position_tolerance").value
        )
        self.controller_wait_sec = float(
            self.get_parameter("controller_wait_sec").value
        )
        self.maximum_arm_speed = float(
            self.get_parameter("maximum_arm_speed").value
        )
        self.maximum_lift_speed = float(
            self.get_parameter("maximum_lift_speed").value
        )
        self.minimum_trajectory_sec = float(
            self.get_parameter("minimum_trajectory_sec").value
        )
        self.default_velocity_scale = float(
            self.get_parameter("default_velocity_scale").value
        )
        self.default_timeout_sec = float(
            self.get_parameter("default_timeout_sec").value
        )
        positive = (
            self.base_linear_limit,
            self.base_angular_limit,
            self.base_hold_sec,
            self.odom_timeout_sec,
            self.joint_state_timeout_sec,
            self.lift_tolerance,
            self.arm_tolerance,
            self.controller_wait_sec,
            self.maximum_arm_speed,
            self.maximum_lift_speed,
            self.minimum_trajectory_sec,
            self.default_velocity_scale,
            self.default_timeout_sec,
        )
        if (
            self.default_arm_name not in ("left", "right")
            or self.lift_min >= self.lift_max
            or any(value <= 0.0 for value in positive)
        ):
            raise ValueError("shelf-view task parameters are invalid")

        # Keep the configured lift limits authoritative for this task.
        JOINT_POSITION_LIMITS[self.lift_joint_name] = (self.lift_min, self.lift_max)
        self.last_odom_time = 0.0
        self.last_linear_speed = math.inf
        self.last_angular_speed = math.inf
        self.last_joint_time = 0.0
        self.joint_positions: Dict[str, float] = {}
        self._active_goal = False
        self._goal_lock = threading.Lock()
        self._group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
            callback_group=self._group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_state_callback,
            30,
            callback_group=self._group,
        )
        self.trajectory_clients = {
            "left": ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("left_trajectory_action_name").value),
                callback_group=self._group,
            ),
            "right": ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("right_trajectory_action_name").value),
                callback_group=self._group,
            ),
            "lift": ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("lift_trajectory_action_name").value),
                callback_group=self._group,
            ),
        }
        self._server = ActionServer(
            self,
            MoveToShelfView,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._group,
        )
        self.get_logger().info(
            "Shelf-view task ready: taught posture -> direct arm/lift trajectories"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.last_linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        self.last_angular_speed = math.sqrt(
            angular.x * angular.x
            + angular.y * angular.y
            + angular.z * angular.z
        )
        self.last_odom_time = time.monotonic()

    def _joint_state_callback(self, msg: JointState) -> None:
        updated = False
        for index, name in enumerate(msg.name):
            if index >= len(msg.position):
                continue
            value = float(msg.position[index])
            if math.isfinite(value):
                self.joint_positions[str(name)] = value
                updated = True
        if updated:
            self.last_joint_time = time.monotonic()

    def goal_callback(self, request: MoveToShelfView.Goal) -> GoalResponse:
        valid = (
            bool(request.station_id)
            and request.arm
            in (
                MoveToShelfView.Goal.LEFT,
                MoveToShelfView.Goal.RIGHT,
                MoveToShelfView.Goal.BOTH,
                MoveToShelfView.Goal.AUTO,
            )
            and len(request.joint_names) == len(request.joint_positions)
            and all(math.isfinite(float(value)) for value in request.joint_positions)
            and math.isfinite(float(request.velocity_scale))
            and math.isfinite(float(request.timeout_sec))
            and float(request.timeout_sec) >= 0.0
            and (
                not request.use_explicit_lift_position
                or math.isfinite(float(request.lift_position))
            )
        )
        with self._goal_lock:
            if self._active_goal or not valid:
                return GoalResponse.REJECT
            self._active_goal = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _sleep(self, seconds: float) -> None:
        future = Future()
        timer_holder = []

        def wake() -> None:
            if timer_holder:
                timer_holder[0].cancel()
            if not future.done():
                future.set_result(None)

        timer_holder.append(self.create_timer(max(0.001, seconds), wake))
        try:
            await future
        finally:
            timer_holder[0].cancel()
            self.destroy_timer(timer_holder[0])

    def _odom_is_stopped(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            now - self.last_odom_time <= self.odom_timeout_sec
            and self.last_linear_speed <= self.base_linear_limit
            and self.last_angular_speed <= self.base_angular_limit
        )

    def _feedback(self, goal_handle, phase: str, lift_position: float) -> None:
        feedback = MoveToShelfView.Feedback()
        feedback.phase = phase
        feedback.lift_position = float(lift_position)
        feedback.base_linear_speed = (
            math.nan
            if not math.isfinite(self.last_linear_speed)
            else float(self.last_linear_speed)
        )
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        success: bool,
        code: int,
        lift_position: float,
        message: str,
        joint_names=None,
        joint_positions=None,
    ) -> MoveToShelfView.Result:
        result = MoveToShelfView.Result()
        result.success = bool(success)
        result.error_code = int(code)
        result.lift_position = float(lift_position)
        result.joint_names = list(joint_names or [])
        result.joint_positions = [float(value) for value in (joint_positions or [])]
        result.message = message
        return result

    async def _wait_for_base_stop(
        self, goal_handle, deadline: float, lift_position: float
    ) -> Tuple[bool, int, str]:
        stable_since: Optional[float] = None
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, TaskError.CANCELLED, "taught posture request cancelled"
            now = time.monotonic()
            self._feedback(goal_handle, "WAITING_FOR_BASE_STOP", lift_position)
            if self._odom_is_stopped(now):
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.base_hold_sec:
                    return True, TaskError.OK, ""
            else:
                stable_since = None
            await self._sleep(0.05)
        if time.monotonic() - self.last_odom_time > self.odom_timeout_sec:
            return False, TaskError.SAFETY_STOP, "safe odometry is missing or stale"
        return False, TaskError.BASE_NOT_STOPPED, "base did not remain stopped"

    def _resolved_arms(self, request, station) -> Tuple[str, ...]:
        if request.arm == MoveToShelfView.Goal.BOTH:
            return ("left", "right")
        if request.arm == MoveToShelfView.Goal.LEFT:
            return ("left",)
        if request.arm == MoveToShelfView.Goal.RIGHT:
            return ("right",)
        wildcard = station.slots.get("*")
        arm_name = (
            wildcard.arm_name.strip().lower()
            if wildcard is not None and wildcard.arm_name
            else self.default_arm_name
        )
        return ("left",) if arm_name == "left" else ("right",)

    def _resolve_targets(self, request, station, lift_position: float):
        arms = self._resolved_arms(request, station)
        target_by_name: Dict[str, float] = {
            self.lift_joint_name: float(lift_position)
        }
        explicit = {
            str(name): float(value)
            for name, value in zip(request.joint_names, request.joint_positions)
        }
        if explicit:
            target_by_name.update(explicit)
        else:
            for arm_name in arms:
                calibrated = self.database.level_arm_joint_positions(
                    station.station_id, int(request.shelf_level), arm_name
                )
                expected = set(ARM_JOINTS[arm_name])
                if set(calibrated) != expected:
                    raise ValueError(
                        f"station '{station.station_id}' level {request.shelf_level} "
                        f"needs all seven {arm_name} taught joints"
                    )
                target_by_name.update(calibrated)

        expected_names = {self.lift_joint_name}
        for arm_name in arms:
            expected_names.update(ARM_JOINTS[arm_name])
        if set(target_by_name) != expected_names:
            raise ValueError(
                "explicit taught posture must contain every selected arm joint "
                "exactly once and must not include other joints"
            )
        for name, value in target_by_name.items():
            limits = JOINT_POSITION_LIMITS.get(name)
            if limits is None or not math.isfinite(value):
                raise ValueError(f"unsupported or non-finite taught joint '{name}'")
            if not limits[0] <= value <= limits[1]:
                raise ValueError(
                    f"taught target {name}={value} is outside "
                    f"[{limits[0]}, {limits[1]}]"
                )
        return arms, target_by_name

    def _joint_state_problem(self, names: Iterable[str]) -> Optional[str]:
        if time.monotonic() - self.last_joint_time > self.joint_state_timeout_sec:
            return "joint_states are missing or stale"
        for name in names:
            current = self.joint_positions.get(name)
            if current is None or not math.isfinite(current):
                return f"joint_states are missing {name}"
            lower, upper = JOINT_POSITION_LIMITS[name]
            if (
                current < lower - CURRENT_POSITION_LIMIT_TOLERANCE
                or current > upper + CURRENT_POSITION_LIMIT_TOLERANCE
            ):
                return (
                    f"current {name}={current:.5f} is outside the safe "
                    f"[{lower:.5f}, {upper:.5f}] range"
                )
        return None

    async def _wait_for_joint_state(
        self, goal_handle, deadline: float, lift_position: float, names
    ) -> bool:
        wait_deadline = min(deadline, time.monotonic() + self.controller_wait_sec)
        while time.monotonic() < wait_deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self._joint_state_problem(names) is None:
                return True
            self._feedback(goal_handle, "WAITING_FOR_JOINT_STATE", lift_position)
            await self._sleep(0.05)
        return False

    async def _wait_for_controllers(
        self, goal_handle, deadline: float, lift_position: float, domains
    ) -> bool:
        wait_deadline = min(deadline, time.monotonic() + self.controller_wait_sec)
        while time.monotonic() < wait_deadline:
            if goal_handle.is_cancel_requested:
                return False
            if all(self.trajectory_clients[name].server_is_ready() for name in domains):
                return True
            self._feedback(goal_handle, "WAITING_FOR_DIRECT_CONTROLLERS", lift_position)
            for name in domains:
                self.trajectory_clients[name].wait_for_server(timeout_sec=0.05)
            await self._sleep(0.02)
        return False

    async def _cancel_child(self, child_handle) -> None:
        if child_handle is None:
            return
        future = child_handle.cancel_goal_async()
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            await self._sleep(0.02)

    async def _cancel_children(self, child_handles) -> None:
        for child_handle in child_handles.values():
            await self._cancel_child(child_handle)

    async def _cancel_pending_sends(self, send_futures) -> None:
        deadline = time.monotonic() + 1.0
        while (
            not all(future.done() for future in send_futures.values())
            and time.monotonic() < deadline
        ):
            await self._sleep(0.02)
        for future in send_futures.values():
            if future.done():
                handle = future.result()
                if handle is not None and handle.accepted:
                    await self._cancel_child(handle)

    def _targets_reached(self, target_by_name: Dict[str, float]) -> bool:
        if time.monotonic() - self.last_joint_time > self.joint_state_timeout_sec:
            return False
        for name, target in target_by_name.items():
            current = self.joint_positions.get(name)
            if current is None:
                return False
            tolerance = (
                self.lift_tolerance
                if name == self.lift_joint_name
                else self.arm_tolerance
            )
            if abs(current - float(target)) > tolerance:
                return False
        return True

    def _controller_goals(self, arms, target_by_name, duration):
        goals = {}
        for arm_name in arms:
            names = ARM_JOINTS[arm_name]
            goals[arm_name] = build_direct_trajectory_goal(
                names, [target_by_name[name] for name in names], duration
            )
        goals["lift"] = build_direct_trajectory_goal(
            [self.lift_joint_name],
            [target_by_name[self.lift_joint_name]],
            duration,
        )
        return goals

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        lift_position = math.nan
        child_handles = {}
        target_by_name: Dict[str, float] = {}
        try:
            station = self.database.get(str(request.station_id))
            if station is None:
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.STATION_NOT_FOUND,
                    lift_position,
                    f"unknown station '{request.station_id}'",
                )
            if request.use_explicit_lift_position:
                lift_position = float(request.lift_position)
            else:
                calibrated = self.database.level_lift_position(
                    station.station_id, None, int(request.shelf_level)
                )
                if calibrated is None:
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.INVALID_REQUEST,
                        lift_position,
                        f"station '{station.station_id}' has no measured "
                        f"lift_joint for level {request.shelf_level}",
                    )
                lift_position = float(calibrated)
            if not math.isfinite(lift_position) or not (
                self.lift_min <= lift_position <= self.lift_max
            ):
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.INVALID_REQUEST,
                    lift_position,
                    f"lift target is outside [{self.lift_min}, {self.lift_max}]",
                )

            try:
                arms, target_by_name = self._resolve_targets(
                    request, station, lift_position
                )
            except ValueError as exc:
                goal_handle.abort()
                return self._result(
                    False, TaskError.INVALID_REQUEST, lift_position, str(exc)
                )
            names = list(target_by_name)
            positions = [target_by_name[name] for name in names]
            timeout = float(request.timeout_sec)
            if timeout <= 0.0:
                timeout = self.default_timeout_sec
            deadline = time.monotonic() + timeout

            stopped, code, message = await self._wait_for_base_stop(
                goal_handle, deadline, lift_position
            )
            if not stopped:
                if code == TaskError.CANCELLED:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(
                    False, code, lift_position, message, names, positions
                )
            if not request.execute:
                goal_handle.succeed()
                return self._result(
                    True,
                    TaskError.OK,
                    lift_position,
                    "Taught posture validated without execution",
                    names,
                    positions,
                )
            if not await self._wait_for_joint_state(
                goal_handle, deadline, lift_position, names
            ):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    code = TaskError.CANCELLED
                    message = "taught posture request cancelled"
                else:
                    goal_handle.abort()
                    code = TaskError.SAFETY_STOP
                    message = self._joint_state_problem(names) or "joint state unavailable"
                return self._result(
                    False, code, lift_position, message, names, positions
                )
            domains = (*arms, "lift")
            if not await self._wait_for_controllers(
                goal_handle, deadline, lift_position, domains
            ):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    code = TaskError.CANCELLED
                    message = "taught posture request cancelled"
                else:
                    goal_handle.abort()
                    code = TaskError.MOVEIT_UNAVAILABLE
                    message = "one or more direct posture controllers are unavailable"
                return self._result(
                    False, code, lift_position, message, names, positions
                )

            duration = direct_posture_duration(
                self.joint_positions,
                target_by_name,
                velocity_scale=(
                    request.velocity_scale
                    if request.velocity_scale > 0.0
                    else self.default_velocity_scale
                ),
                maximum_arm_speed=self.maximum_arm_speed,
                maximum_lift_speed=self.maximum_lift_speed,
                minimum_duration=self.minimum_trajectory_sec,
                lift_joint_name=self.lift_joint_name,
            )
            if time.monotonic() + duration >= deadline:
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.TIMEOUT,
                    lift_position,
                    f"direct posture needs {duration:.2f}s but the request deadline is too short",
                    names,
                    positions,
                )
            goals = self._controller_goals(arms, target_by_name, duration)
            self._feedback(goal_handle, "SENDING_TAUGHT_POSTURE", lift_position)
            send_futures = {
                domain: self.trajectory_clients[domain].send_goal_async(goal)
                for domain, goal in goals.items()
            }
            while (
                not all(future.done() for future in send_futures.values())
                and time.monotonic() < deadline
            ):
                if goal_handle.is_cancel_requested:
                    await self._cancel_pending_sends(send_futures)
                    goal_handle.canceled()
                    return self._result(
                        False,
                        TaskError.CANCELLED,
                        lift_position,
                        "taught posture request cancelled",
                        names,
                        positions,
                    )
                await self._sleep(0.02)
            if not all(future.done() for future in send_futures.values()):
                await self._cancel_pending_sends(send_futures)
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.TIMEOUT,
                    lift_position,
                    "sending direct posture goals timed out",
                    names,
                    positions,
                )
            child_handles = {
                domain: future.result() for domain, future in send_futures.items()
            }
            rejected = [
                domain
                for domain, handle in child_handles.items()
                if handle is None or not handle.accepted
            ]
            if rejected:
                await self._cancel_children(
                    {
                        domain: handle
                        for domain, handle in child_handles.items()
                        if handle is not None and handle.accepted
                    }
                )
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.MOVEIT_UNAVAILABLE,
                    lift_position,
                    "direct posture controller rejected: " + ", ".join(rejected),
                    names,
                    positions,
                )

            result_futures = {
                domain: handle.get_result_async()
                for domain, handle in child_handles.items()
            }
            while (
                not all(future.done() for future in result_futures.values())
                and time.monotonic() < deadline
            ):
                if goal_handle.is_cancel_requested:
                    await self._cancel_children(child_handles)
                    goal_handle.canceled()
                    return self._result(
                        False,
                        TaskError.CANCELLED,
                        lift_position,
                        "taught posture request cancelled",
                        names,
                        positions,
                    )
                if not self._odom_is_stopped():
                    await self._cancel_children(child_handles)
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        lift_position,
                        "base moved or safe odometry became stale during taught posture",
                        names,
                        positions,
                    )
                self._feedback(goal_handle, "EXECUTING_TAUGHT_POSTURE", lift_position)
                await self._sleep(0.05)
            if not all(future.done() for future in result_futures.values()):
                await self._cancel_children(child_handles)
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.TIMEOUT,
                    lift_position,
                    "direct posture execution timed out",
                    names,
                    positions,
                )

            failures = []
            for domain, future in result_futures.items():
                completed = future.result()
                payload = completed.result
                if (
                    completed.status != GoalStatus.STATUS_SUCCEEDED
                    or payload.error_code != FollowJointTrajectory.Result.SUCCESSFUL
                ):
                    failures.append(
                        f"{domain}: {payload.error_string or payload.error_code}"
                    )
            if failures:
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.PICK_FAILED,
                    lift_position,
                    "direct taught posture failed (" + "; ".join(failures) + ")",
                    names,
                    positions,
                )

            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        False,
                        TaskError.CANCELLED,
                        lift_position,
                        "taught posture verification cancelled",
                        names,
                        positions,
                    )
                if not self._odom_is_stopped():
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        lift_position,
                        "base moved while verifying the taught posture",
                        names,
                        positions,
                    )
                self._feedback(goal_handle, "VERIFYING_JOINT_TARGETS", lift_position)
                if self._targets_reached(target_by_name):
                    goal_handle.succeed()
                    return self._result(
                        True,
                        TaskError.OK,
                        lift_position,
                        "Direct taught shelf posture reached; perception may start",
                        names,
                        positions,
                    )
                await self._sleep(0.05)

            goal_handle.abort()
            return self._result(
                False,
                TaskError.TIMEOUT,
                lift_position,
                "joint_states did not confirm the taught posture",
                names,
                positions,
            )
        except Exception as exc:  # pragma: no cover - ROS action boundary
            self.get_logger().error(f"Shelf-view task failed: {exc}")
            if child_handles:
                await self._cancel_children(child_handles)
            goal_handle.abort()
            return self._result(
                False,
                TaskError.INTERNAL_ERROR,
                lift_position,
                str(exc),
                list(target_by_name),
                list(target_by_name.values()),
            )
        finally:
            with self._goal_lock:
                self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShelfViewNode()
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
