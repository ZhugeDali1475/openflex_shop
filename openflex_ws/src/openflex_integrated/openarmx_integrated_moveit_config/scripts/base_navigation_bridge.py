#!/usr/bin/env python3

"""Minimal odometry-to-velocity implementation of Nav2's NavigateToPose API.

This bridge intentionally provides no map, obstacle avoidance, or global path
planning. It makes the active chassis move to an ``odom`` target using the
already deployed /cmd_vel interface, so MuJoCo, fake hardware, and the real
robot share one action contract.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def duration_from_seconds(seconds: float) -> Duration:
    seconds = max(0.0, seconds)
    whole_seconds = int(seconds)
    return Duration(sec=whole_seconds, nanosec=int((seconds - whole_seconds) * 1e9))


class BaseNavigationBridge(Node):
    """Action server that drives a holonomic base to a target in the odom frame."""

    def __init__(self) -> None:
        super().__init__("base_navigation_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("max_linear_acceleration", 0.35)
        self.declare_parameter("max_angular_acceleration", 0.80)
        self.declare_parameter("position_gain", 0.90)
        self.declare_parameter("yaw_gain", 1.20)
        # Keep a command above the deployed swerve module deadband until the
        # measured odometry reaches the final one-centimetre goal tolerance.
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("min_linear_speed", 0.025)
        self.declare_parameter("yaw_tolerance", 0.035)
        self.declare_parameter("odom_stale_timeout", 0.50)
        self.declare_parameter("goal_timeout", 90.0)

        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._odom_frame = self.get_parameter("odom_frame").value.lstrip("/")
        self._rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._max_linear_acceleration = float(self.get_parameter("max_linear_acceleration").value)
        self._max_angular_acceleration = float(self.get_parameter("max_angular_acceleration").value)
        self._position_gain = float(self.get_parameter("position_gain").value)
        self._yaw_gain = float(self.get_parameter("yaw_gain").value)
        self._position_tolerance = float(self.get_parameter("position_tolerance").value)
        self._min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self._yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self._odom_stale_timeout = float(self.get_parameter("odom_stale_timeout").value)
        self._goal_timeout = float(self.get_parameter("goal_timeout").value)

        if self._rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")

        self._lock = threading.RLock()
        self._odom: Odometry | None = None
        self._odom_received_at = 0.0
        self._active_goal = None
        self._last_command = Twist()
        self._last_command_at = time.monotonic()
        self._callback_group = ReentrantCallbackGroup()

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_callback,
            30,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            handle_accepted_callback=self._handle_accepted,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"Serving /navigate_to_pose: odom feedback -> {self._cmd_vel_topic} "
            "(no map or obstacle avoidance)"
        )

    def destroy_node(self):
        if rclpy.ok():
            self._publish_stop()
        self._action_server.destroy()
        return super().destroy_node()

    def _odom_callback(self, message: Odometry) -> None:
        with self._lock:
            self._odom = message
            self._odom_received_at = time.monotonic()

    def _goal_callback(self, goal_request: NavigateToPose.Goal) -> GoalResponse:
        frame_id = goal_request.pose.header.frame_id.lstrip("/")
        pose = goal_request.pose.pose
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w)
        quaternion_norm = math.sqrt(sum(value * value for value in values[2:]))
        if frame_id != self._odom_frame or not all(math.isfinite(value) for value in values):
            self.get_logger().error(
                f"Rejecting navigation goal: expected finite pose in '{self._odom_frame}', "
                f"received '{frame_id}'"
            )
            return GoalResponse.REJECT
        if quaternion_norm < 1e-6:
            self.get_logger().error("Rejecting navigation goal with a zero orientation quaternion")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_accepted(self, goal_handle) -> None:
        # A later goal replaces the active command. The previous execution loop
        # notices that it no longer owns _active_goal and aborts without stopping
        # the new goal's velocity stream.
        with self._lock:
            previous_goal = self._active_goal
            self._active_goal = goal_handle
        if previous_goal is not None and previous_goal.is_active:
            self.get_logger().info("Preempting the active /navigate_to_pose goal")
        goal_handle.execute()

    def _execute(self, goal_handle):
        target = goal_handle.request.pose.pose
        target_yaw = yaw_from_quaternion(target.orientation)
        started_at = time.monotonic()
        result = NavigateToPose.Result()

        while rclpy.ok():
            if not self._is_active_goal(goal_handle):
                goal_handle.abort()
                return result
            if goal_handle.is_cancel_requested:
                self._publish_stop_if_active(goal_handle)
                goal_handle.canceled()
                return result

            odom = self._current_odom()
            elapsed = time.monotonic() - started_at
            if odom is None:
                if elapsed >= self._goal_timeout:
                    self.get_logger().error("Aborting navigation: no recent /odom feedback")
                    self._publish_stop_if_active(goal_handle)
                    goal_handle.abort()
                    return result
                time.sleep(1.0 / self._rate_hz)
                continue

            current = odom.pose.pose
            current_yaw = yaw_from_quaternion(current.orientation)
            error_x = target.position.x - current.position.x
            error_y = target.position.y - current.position.y
            position_error = math.hypot(error_x, error_y)
            yaw_error = normalize_angle(target_yaw - current_yaw)
            self._publish_feedback(goal_handle, odom, elapsed, position_error)

            if position_error <= self._position_tolerance and abs(yaw_error) <= self._yaw_tolerance:
                self._publish_stop_if_active(goal_handle)
                goal_handle.succeed()
                return result
            if elapsed >= self._goal_timeout:
                self.get_logger().error(f"Aborting navigation: goal timed out after {elapsed:.1f} s")
                self._publish_stop_if_active(goal_handle)
                goal_handle.abort()
                return result

            command = self._command_for_error(error_x, error_y, yaw_error, current_yaw)
            self._publish_command(command)
            time.sleep(1.0 / self._rate_hz)

        self._publish_stop_if_active(goal_handle)
        goal_handle.abort()
        return result

    def _current_odom(self) -> Odometry | None:
        with self._lock:
            if self._odom is None or time.monotonic() - self._odom_received_at > self._odom_stale_timeout:
                return None
            return self._odom

    def _is_active_goal(self, goal_handle) -> bool:
        with self._lock:
            return self._active_goal is goal_handle

    def _command_for_error(self, error_x: float, error_y: float, yaw_error: float, current_yaw: float) -> Twist:
        world_vx = self._position_gain * error_x
        world_vy = self._position_gain * error_y
        world_speed = math.hypot(world_vx, world_vy)
        if world_speed > self._max_linear_speed:
            scale = self._max_linear_speed / world_speed
            world_vx *= scale
            world_vy *= scale
        elif 0.0 < world_speed < self._min_linear_speed:
            scale = self._min_linear_speed / world_speed
            world_vx *= scale
            world_vy *= scale

        command = Twist()
        command.linear.x = math.cos(current_yaw) * world_vx + math.sin(current_yaw) * world_vy
        command.linear.y = -math.sin(current_yaw) * world_vx + math.cos(current_yaw) * world_vy
        command.angular.z = clamp(self._yaw_gain * yaw_error, self._max_angular_speed)
        return self._limit_acceleration(command)

    def _limit_acceleration(self, command: Twist) -> Twist:
        now = time.monotonic()
        with self._lock:
            dt = max(1e-3, now - self._last_command_at)
            previous = self._last_command
        max_linear_delta = self._max_linear_acceleration * dt
        max_angular_delta = self._max_angular_acceleration * dt
        command.linear.x = previous.linear.x + clamp(command.linear.x - previous.linear.x, max_linear_delta)
        command.linear.y = previous.linear.y + clamp(command.linear.y - previous.linear.y, max_linear_delta)
        command.angular.z = previous.angular.z + clamp(command.angular.z - previous.angular.z, max_angular_delta)
        return command

    def _publish_command(self, command: Twist) -> None:
        self._cmd_vel_pub.publish(command)
        with self._lock:
            self._last_command = command
            self._last_command_at = time.monotonic()

    def _publish_stop(self) -> None:
        self._publish_command(Twist())

    def _publish_stop_if_active(self, goal_handle) -> None:
        if self._is_active_goal(goal_handle):
            self._publish_stop()
            with self._lock:
                if self._active_goal is goal_handle:
                    self._active_goal = None

    def _publish_feedback(self, goal_handle, odom: Odometry, elapsed: float, distance_remaining: float) -> None:
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = PoseStamped()
        feedback.current_pose.header = odom.header
        feedback.current_pose.pose = odom.pose.pose
        feedback.navigation_time = duration_from_seconds(elapsed)
        feedback.estimated_time_remaining = duration_from_seconds(
            distance_remaining / max(1e-6, self._max_linear_speed)
        )
        feedback.distance_remaining = float(distance_remaining)
        goal_handle.publish_feedback(feedback)


def main() -> None:
    rclpy.init()
    node = BaseNavigationBridge()
    # The action loop blocks while it tracks a goal, so one additional worker
    # is required for /odom callbacks. More threads do not improve control.
    executor = MultiThreadedExecutor(num_threads=2)
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
