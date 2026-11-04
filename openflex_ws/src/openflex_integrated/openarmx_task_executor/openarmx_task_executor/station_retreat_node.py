"""Guarded straight retreat action used after close-station manipulation."""

import math
import struct
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from openarmx_task_msgs.action import RetreatFromStation
from openarmx_task_msgs.msg import TaskError
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, UInt8

from .post_pick_retreat_core import PlanarPose, measure_retreat


def _as_float32(value: float) -> float:
    """Match the precision used by RetreatFromStation numeric fields."""
    return struct.unpack("!f", struct.pack("!f", float(value)))[0]


class StationRetreatNode(Node):
    """Own the only task-level reverse primitive behind the existing safety chain."""

    def __init__(self) -> None:
        super().__init__("station_retreat")
        self.declare_parameter("action_name", "/retreat_from_station")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom_safe")
        self.declare_parameter("localization_state_topic", "/localization_ok")
        self.declare_parameter(
            "reverse_authorization_topic",
            "/competition/post_pick_reverse_authorized",
        )
        self.declare_parameter(
            "localization_guard_state_topic",
            "/shelf_fine_align/localization_guard_state",
        )
        self.declare_parameter("control_period_sec", 0.05)
        self.declare_parameter("odom_timeout_sec", 0.50)
        self.declare_parameter("maximum_distance", 0.30)
        self.declare_parameter("maximum_speed", 0.06)
        self.declare_parameter("lateral_tolerance", 0.03)
        self.declare_parameter("yaw_tolerance", math.radians(3.0))
        self.declare_parameter("stopped_linear_speed", 0.02)
        self.declare_parameter("stopped_angular_speed", 0.03)
        self.declare_parameter("stopped_hold_sec", 0.30)

        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        # RetreatFromStation uses float32 fields. Quantize the configured limits
        # the same way so an exact boundary request such as 0.30 m is not
        # received as 0.3000000119 and incorrectly rejected.
        self.maximum_distance = _as_float32(
            self.get_parameter("maximum_distance").value
        )
        self.maximum_speed = _as_float32(
            self.get_parameter("maximum_speed").value
        )
        self.lateral_tolerance = float(self.get_parameter("lateral_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.stopped_linear_speed = float(
            self.get_parameter("stopped_linear_speed").value
        )
        self.stopped_angular_speed = float(
            self.get_parameter("stopped_angular_speed").value
        )
        self.stopped_hold_sec = float(self.get_parameter("stopped_hold_sec").value)
        limits = (
            self.control_period_sec,
            self.odom_timeout_sec,
            self.maximum_distance,
            self.maximum_speed,
            self.lateral_tolerance,
            self.yaw_tolerance,
            self.stopped_linear_speed,
            self.stopped_angular_speed,
            self.stopped_hold_sec,
        )
        if any(value <= 0.0 for value in limits):
            raise ValueError("station retreat limits must be positive")

        self.last_odom_time = 0.0
        self.last_pose: Optional[PlanarPose] = None
        self.last_linear_speed = math.inf
        self.last_angular_speed = math.inf
        self.localization_ok = False
        self.localization_state_seen = False
        self._active_goal = False

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.authorization_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("reverse_authorization_topic").value),
            10,
        )
        guard_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.localization_guard_pub = self.create_publisher(
            UInt8,
            str(self.get_parameter("localization_guard_state_topic").value),
            guard_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("localization_state_topic").value),
            self._localization_callback,
            state_qos,
        )
        self._server = ActionServer(
            self,
            RetreatFromStation,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            "Station retreat ready: guarded straight reverse on "
            f"{self.get_parameter('cmd_vel_topic').value}"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        linear = msg.twist.twist.linear
        self.last_pose = PlanarPose(position.x, position.y, yaw)
        self.last_linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        self.last_angular_speed = abs(msg.twist.twist.angular.z)
        self.last_odom_time = time.monotonic()

    def _localization_callback(self, msg: Bool) -> None:
        self.localization_state_seen = True
        self.localization_ok = bool(msg.data)

    def goal_callback(self, request: RetreatFromStation.Goal) -> GoalResponse:
        distance = float(request.distance)
        speed = float(request.max_speed)
        timeout = float(request.timeout_sec)
        valid = (
            0.0 < distance <= self.maximum_distance
            and 0.0 < speed <= self.maximum_speed
            and timeout > 0.0
        )
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

    def _authorize(self, enabled: bool) -> None:
        message = Bool()
        message.data = bool(enabled)
        self.authorization_pub.publish(message)

    def _command(self, linear_x: float) -> None:
        message = Twist()
        message.linear.x = float(linear_x)
        self.cmd_pub.publish(message)

    def _release_fine_alignment_guard(self) -> None:
        message = UInt8()
        message.data = 0
        self.localization_guard_pub.publish(message)

    @staticmethod
    def _result(
        success: bool,
        code: int,
        traveled: float,
        message: str,
    ) -> RetreatFromStation.Result:
        result = RetreatFromStation.Result()
        result.success = bool(success)
        result.error_code = int(code)
        result.traveled_distance = float(max(0.0, traveled))
        result.message = message
        return result

    @staticmethod
    def _feedback(
        goal_handle,
        phase: str,
        traveled: float,
        lateral: float = 0.0,
        yaw_error: float = 0.0,
    ) -> None:
        feedback = RetreatFromStation.Feedback()
        feedback.phase = phase
        feedback.traveled_distance = float(max(0.0, traveled))
        feedback.lateral_drift = float(lateral)
        feedback.yaw_error = float(yaw_error)
        goal_handle.publish_feedback(feedback)

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        distance = float(request.distance)
        speed = float(request.max_speed)
        deadline = time.monotonic() + float(request.timeout_sec)
        traveled = 0.0
        try:
            now = time.monotonic()
            if not self.localization_state_seen or not self.localization_ok:
                goal_handle.abort()
                return self._result(
                    False, TaskError.SAFETY_STOP, 0.0, "localization is not safe"
                )
            if self.last_pose is None or now - self.last_odom_time > self.odom_timeout_sec:
                goal_handle.abort()
                return self._result(
                    False, TaskError.SAFETY_STOP, 0.0, "safe odometry is stale"
                )

            start_pose = self.last_pose
            self._authorize(True)
            await self._sleep(self.control_period_sec)
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, traveled, "station retreat cancelled"
                    )
                now = time.monotonic()
                if not self.localization_ok:
                    goal_handle.abort()
                    return self._result(
                        False, TaskError.SAFETY_STOP, traveled, "localization became unsafe"
                    )
                if self.last_pose is None or now - self.last_odom_time > self.odom_timeout_sec:
                    goal_handle.abort()
                    return self._result(
                        False, TaskError.SAFETY_STOP, traveled, "safe odometry became stale"
                    )

                progress = measure_retreat(start_pose, self.last_pose)
                traveled = progress.backward_distance
                self._feedback(
                    goal_handle,
                    "RETREATING",
                    traveled,
                    progress.lateral_drift,
                    progress.yaw_error,
                )
                if abs(progress.lateral_drift) > self.lateral_tolerance:
                    goal_handle.abort()
                    return self._result(
                        False, TaskError.SAFETY_STOP, traveled, "lateral drift exceeded tolerance"
                    )
                if abs(progress.yaw_error) > self.yaw_tolerance:
                    goal_handle.abort()
                    return self._result(
                        False, TaskError.SAFETY_STOP, traveled, "yaw drift exceeded tolerance"
                    )
                if traveled >= distance:
                    break

                self._authorize(True)
                self._command(-speed)
                await self._sleep(self.control_period_sec)
            else:
                goal_handle.abort()
                return self._result(
                    False, TaskError.TIMEOUT, traveled, "station retreat timed out"
                )

            self._command(0.0)
            self._authorize(False)
            stable_since: Optional[float] = None
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        False, TaskError.CANCELLED, traveled, "station retreat cancelled"
                    )
                now = time.monotonic()
                fresh = now - self.last_odom_time <= self.odom_timeout_sec
                stopped = (
                    self.last_linear_speed <= self.stopped_linear_speed
                    and self.last_angular_speed <= self.stopped_angular_speed
                )
                self._feedback(goal_handle, "SETTLING", traveled)
                if fresh and stopped:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= self.stopped_hold_sec:
                        self._release_fine_alignment_guard()
                        goal_handle.succeed()
                        return self._result(
                            True,
                            TaskError.OK,
                            traveled,
                            "station retreat complete",
                        )
                else:
                    stable_since = None
                self._command(0.0)
                await self._sleep(self.control_period_sec)

            goal_handle.abort()
            return self._result(
                False, TaskError.TIMEOUT, traveled, "base did not settle after retreat"
            )
        finally:
            self._command(0.0)
            self._authorize(False)
            self._command(0.0)
            self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StationRetreatNode()
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
