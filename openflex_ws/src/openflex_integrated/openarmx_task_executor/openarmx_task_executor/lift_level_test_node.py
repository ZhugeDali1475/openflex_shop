"""Guarded shelf-level lift test used after navigation and fine alignment."""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from openarmx_task_msgs.action import MoveLiftToLevel
from openarmx_task_msgs.msg import TaskError
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .station_db import StationDatabase


class LiftLevelTestNode(Node):
    """Move only ``lift_joint`` after proving that the base is stopped."""

    def __init__(self) -> None:
        super().__init__("lift_level_test")
        self.declare_parameter("station_config", "")
        self.declare_parameter("station_mission", "")
        self.declare_parameter("action_name", "/competition/move_lift_to_level")
        self.declare_parameter(
            "trajectory_action_name",
            "/lift_joint_controller/follow_joint_trajectory",
        )
        self.declare_parameter("odom_topic", "/odom_safe")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("lift_joint_name", "lift_joint")
        self.declare_parameter("lift_joint_min_position", -0.650)
        self.declare_parameter("lift_joint_max_position", 0.300)
        self.declare_parameter("base_stop_linear_speed", 0.02)
        self.declare_parameter("base_stop_angular_speed", 0.03)
        self.declare_parameter("base_stop_hold_sec", 0.5)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("joint_state_timeout_sec", 0.5)
        self.declare_parameter("joint_position_tolerance", 0.005)
        self.declare_parameter("controller_wait_sec", 2.0)
        self.declare_parameter("maximum_lift_speed", 0.08)
        self.declare_parameter("minimum_trajectory_sec", 0.5)
        self.declare_parameter("default_timeout_sec", 20.0)

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

        self.lift_joint_name = str(self.get_parameter("lift_joint_name").value)
        self.lift_min = float(
            self.get_parameter("lift_joint_min_position").value
        )
        self.lift_max = float(
            self.get_parameter("lift_joint_max_position").value
        )
        self.base_linear_limit = float(
            self.get_parameter("base_stop_linear_speed").value
        )
        self.base_angular_limit = float(
            self.get_parameter("base_stop_angular_speed").value
        )
        self.base_hold_sec = float(self.get_parameter("base_stop_hold_sec").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.joint_timeout_sec = float(
            self.get_parameter("joint_state_timeout_sec").value
        )
        self.joint_tolerance = float(
            self.get_parameter("joint_position_tolerance").value
        )
        self.controller_wait_sec = float(
            self.get_parameter("controller_wait_sec").value
        )
        self.maximum_lift_speed = float(
            self.get_parameter("maximum_lift_speed").value
        )
        self.minimum_trajectory_sec = float(
            self.get_parameter("minimum_trajectory_sec").value
        )
        self.default_timeout_sec = float(
            self.get_parameter("default_timeout_sec").value
        )

        positive_limits = (
            self.base_linear_limit,
            self.base_angular_limit,
            self.base_hold_sec,
            self.odom_timeout_sec,
            self.joint_timeout_sec,
            self.joint_tolerance,
            self.controller_wait_sec,
            self.maximum_lift_speed,
            self.minimum_trajectory_sec,
            self.default_timeout_sec,
        )
        if self.lift_min >= self.lift_max or any(
            value <= 0.0 for value in positive_limits
        ):
            raise ValueError("lift-level test limits must be positive and ordered")

        self.last_odom_time = 0.0
        self.last_linear_speed = math.inf
        self.last_angular_speed = math.inf
        self.last_joint_time = 0.0
        self.last_joint_position: Optional[float] = None
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
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action_name").value),
            callback_group=self._group,
        )
        self._server = ActionServer(
            self,
            MoveLiftToLevel,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._group,
        )
        self.get_logger().info(
            "Guarded lift-level test ready; it will command "
            f"{self.get_parameter('trajectory_action_name').value} only after "
            "the base has stopped"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.last_linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        self.last_angular_speed = math.sqrt(
            angular.x * angular.x + angular.y * angular.y + angular.z * angular.z
        )
        self.last_odom_time = time.monotonic()

    def _joint_state_callback(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self.lift_joint_name)
        except ValueError:
            return
        if index >= len(msg.position) or not math.isfinite(msg.position[index]):
            return
        self.last_joint_position = float(msg.position[index])
        self.last_joint_time = time.monotonic()

    def goal_callback(self, request: MoveLiftToLevel.Goal) -> GoalResponse:
        timeout = float(request.timeout_sec)
        valid = (
            bool(request.station_id)
            and math.isfinite(timeout)
            and timeout >= 0.0
            and (
                not request.use_explicit_position
                or math.isfinite(float(request.target_position))
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

    def _odom_is_fresh_and_stopped(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            now - self.last_odom_time <= self.odom_timeout_sec
            and self.last_linear_speed <= self.base_linear_limit
            and self.last_angular_speed <= self.base_angular_limit
        )

    def _joint_is_fresh(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return (
            self.last_joint_position is not None
            and now - self.last_joint_time <= self.joint_timeout_sec
        )

    def _feedback(self, goal_handle, phase: str, target: float) -> None:
        feedback = MoveLiftToLevel.Feedback()
        feedback.phase = phase
        feedback.target_position = float(target)
        feedback.current_position = (
            math.nan
            if self.last_joint_position is None
            else float(self.last_joint_position)
        )
        feedback.base_linear_speed = (
            math.nan
            if not math.isfinite(self.last_linear_speed)
            else float(self.last_linear_speed)
        )
        goal_handle.publish_feedback(feedback)

    def _result(
        self,
        success: bool,
        code: int,
        target: float,
        message: str,
    ) -> MoveLiftToLevel.Result:
        result = MoveLiftToLevel.Result()
        result.success = bool(success)
        result.error_code = int(code)
        result.target_position = float(target)
        result.final_position = (
            math.nan
            if self.last_joint_position is None
            else float(self.last_joint_position)
        )
        result.message = message
        return result

    async def _wait_for_base_stop(
        self, goal_handle, deadline: float, target: float
    ) -> Tuple[bool, int, str]:
        stable_since: Optional[float] = None
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, TaskError.CANCELLED, "lift move cancelled"
            now = time.monotonic()
            self._feedback(goal_handle, "WAITING_FOR_BASE_STOP", target)
            if self._odom_is_fresh_and_stopped(now):
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

    async def _wait_for_joint_state(
        self, goal_handle, deadline: float, target: float
    ) -> bool:
        wait_deadline = min(deadline, time.monotonic() + self.controller_wait_sec)
        while time.monotonic() < wait_deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self._joint_is_fresh():
                return True
            self._feedback(goal_handle, "WAITING_FOR_JOINT_STATE", target)
            await self._sleep(0.05)
        return False

    async def _wait_for_controller(
        self, goal_handle, deadline: float, target: float
    ) -> bool:
        wait_deadline = min(deadline, time.monotonic() + self.controller_wait_sec)
        while time.monotonic() < wait_deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self.trajectory_client.server_is_ready():
                return True
            self._feedback(goal_handle, "WAITING_FOR_CONTROLLER", target)
            self.trajectory_client.wait_for_server(timeout_sec=0.1)
            await self._sleep(0.02)
        return False

    async def _cancel_child(self, child_handle) -> None:
        if child_handle is None:
            return
        cancel_future = child_handle.cancel_goal_async()
        cancel_deadline = time.monotonic() + 1.0
        while not cancel_future.done() and time.monotonic() < cancel_deadline:
            await self._sleep(0.02)

    async def _cancel_pending_send(self, send_future) -> None:
        """Avoid leaving a controller goal behind if the parent is cancelled."""
        wait_deadline = time.monotonic() + 1.0
        while not send_future.done() and time.monotonic() < wait_deadline:
            await self._sleep(0.02)
        if send_future.done():
            pending_handle = send_future.result()
            if pending_handle is not None and pending_handle.accepted:
                await self._cancel_child(pending_handle)

    def _trajectory_goal(self, target: float) -> FollowJointTrajectory.Goal:
        current = float(self.last_joint_position)
        duration = max(
            self.minimum_trajectory_sec,
            abs(target - current) / self.maximum_lift_speed,
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.lift_joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(target)]
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int((duration - seconds) * 1e9)
        goal.trajectory.points = [point]
        return goal

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        target = math.nan
        child_handle = None
        try:
            station = self.database.get(str(request.station_id))
            if station is None:
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.STATION_NOT_FOUND,
                    target,
                    f"unknown station '{request.station_id}'",
                )

            if request.use_explicit_position:
                target = float(request.target_position)
            else:
                target_value = self.database.level_lift_position(
                    station.station_id, None, int(request.shelf_level)
                )
                if target_value is None:
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.INVALID_REQUEST,
                        target,
                        f"station '{station.station_id}' has no measured lift_joint "
                        f"position for shelf level {request.shelf_level}",
                    )
                target = float(target_value)

            if not math.isfinite(target) or not self.lift_min <= target <= self.lift_max:
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.INVALID_REQUEST,
                    target,
                    f"target {target} is outside lift limits "
                    f"[{self.lift_min}, {self.lift_max}]",
                )

            timeout = float(request.timeout_sec)
            if timeout <= 0.0:
                timeout = self.default_timeout_sec
            deadline = time.monotonic() + timeout

            stopped, code, message = await self._wait_for_base_stop(
                goal_handle, deadline, target
            )
            if not stopped:
                if code == TaskError.CANCELLED:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(False, code, target, message)

            if not await self._wait_for_joint_state(goal_handle, deadline, target):
                if goal_handle.is_cancel_requested:
                    await self._cancel_pending_send(send_future)
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, target, "lift move cancelled"
                    )
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.SAFETY_STOP,
                    target,
                    f"fresh {self.lift_joint_name} joint state is unavailable",
                )

            if not await self._wait_for_controller(goal_handle, deadline, target):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, target, "lift move cancelled"
                    )
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.MOVEIT_UNAVAILABLE,
                    target,
                    "lift trajectory controller action is unavailable",
                )

            send_future = self.trajectory_client.send_goal_async(
                self._trajectory_goal(target)
            )
            while not send_future.done() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, target, "lift move cancelled"
                    )
                await self._sleep(0.02)
            if not send_future.done():
                await self._cancel_pending_send(send_future)
                goal_handle.abort()
                return self._result(
                    False, TaskError.TIMEOUT, target, "sending lift goal timed out"
                )
            child_handle = send_future.result()
            if not child_handle.accepted:
                goal_handle.abort()
                return self._result(
                    False,
                    TaskError.MOVEIT_UNAVAILABLE,
                    target,
                    "lift trajectory controller rejected the goal",
                )

            result_future = child_handle.get_result_async()
            while not result_future.done() and time.monotonic() < deadline:
                self._feedback(goal_handle, "MOVING_LIFT", target)
                if goal_handle.is_cancel_requested:
                    await self._cancel_child(child_handle)
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, target, "lift move cancelled"
                    )
                if not self._odom_is_fresh_and_stopped():
                    await self._cancel_child(child_handle)
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        target,
                        "base moved or safe odometry became stale during lift motion",
                    )
                await self._sleep(0.05)

            if not result_future.done():
                await self._cancel_child(child_handle)
                goal_handle.abort()
                return self._result(
                    False, TaskError.TIMEOUT, target, "lift trajectory timed out"
                )
            child_result = result_future.result()
            if (
                child_result.status != GoalStatus.STATUS_SUCCEEDED
                or child_result.result.error_code
                != FollowJointTrajectory.Result.SUCCESSFUL
            ):
                goal_handle.abort()
                error_string = getattr(child_result.result, "error_string", "")
                return self._result(
                    False,
                    TaskError.MOVEIT_UNAVAILABLE,
                    target,
                    "lift trajectory failed" + (f": {error_string}" if error_string else ""),
                )

            while time.monotonic() < deadline:
                self._feedback(goal_handle, "VERIFYING_POSITION", target)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, target, "lift move cancelled"
                    )
                if not self._odom_is_fresh_and_stopped():
                    goal_handle.abort()
                    return self._result(
                        False,
                        TaskError.SAFETY_STOP,
                        target,
                        "base moved or safe odometry became stale while verifying lift",
                    )
                if (
                    self._joint_is_fresh()
                    and abs(float(self.last_joint_position) - target)
                    <= self.joint_tolerance
                ):
                    goal_handle.succeed()
                    return self._result(
                        True,
                        TaskError.OK,
                        target,
                        f"{self.lift_joint_name} reached shelf level "
                        f"{request.shelf_level}",
                    )
                await self._sleep(0.05)

            goal_handle.abort()
            return self._result(
                False,
                TaskError.TIMEOUT,
                target,
                f"{self.lift_joint_name} did not reach target tolerance",
            )
        except Exception as exc:  # pragma: no cover - last-resort ROS boundary
            self.get_logger().error(f"Lift-level test failed: {exc}")
            if child_handle is not None:
                await self._cancel_child(child_handle)
            goal_handle.abort()
            return self._result(
                False, TaskError.INTERNAL_ERROR, target, str(exc)
            )
        finally:
            with self._goal_lock:
                self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiftLevelTestNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
