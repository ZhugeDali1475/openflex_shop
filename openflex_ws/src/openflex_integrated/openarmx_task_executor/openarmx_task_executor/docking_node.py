"""TF-based local station docking action for a swerve base."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from openarmx_task_msgs.action import DockToStation
from openarmx_task_msgs.msg import TaskError
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class StationDockingNode(Node):
    """Align the robot so a station marker reaches a configured pose in base_link."""

    def __init__(self) -> None:
        super().__init__("station_docking")
        self.declare_parameter("action_name", "/dock_to_station")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("marker_frame_prefix", "station_marker_")
        # Publish before collision_monitor. Its output is /cmd_vel_safe_in and
        # the final localization safety gate publishes /cmd_vel_safe.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("target_marker_x", 0.65)
        self.declare_parameter("target_marker_y", 0.0)
        self.declare_parameter("target_marker_yaw", 0.0)
        self.declare_parameter("kp_x", 0.8)
        self.declare_parameter("kp_y", 1.2)
        self.declare_parameter("kp_yaw", 1.5)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("max_lateral_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.25)
        self.declare_parameter("default_timeout_sec", 20.0)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("control_period_sec", 0.05)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.marker_frame_prefix = str(self.get_parameter("marker_frame_prefix").value)
        self.target_x = float(self.get_parameter("target_marker_x").value)
        self.target_y = float(self.get_parameter("target_marker_y").value)
        self.target_yaw = float(self.get_parameter("target_marker_yaw").value)
        self.kp_x = float(self.get_parameter("kp_x").value)
        self.kp_y = float(self.get_parameter("kp_y").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_linear_speed = abs(float(self.get_parameter("max_linear_speed").value))
        self.max_lateral_speed = abs(float(self.get_parameter("max_lateral_speed").value))
        self.max_angular_speed = abs(float(self.get_parameter("max_angular_speed").value))
        self.default_timeout_sec = float(self.get_parameter("default_timeout_sec").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self._active_goal = False
        self._server = ActionServer(
            self,
            DockToStation,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            f"Station docking ready; marker prefix={self.marker_frame_prefix}, "
            f"cmd_vel={self.get_parameter('cmd_vel_topic').value}"
        )

    def goal_callback(self, _request: DockToStation.Goal) -> GoalResponse:
        if self._active_goal:
            return GoalResponse.REJECT
        self._active_goal = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_cmd(self, vx: float, vy: float, omega: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        self.cmd_pub.publish(msg)

    async def _sleep(self, seconds: float) -> None:
        future = Future()
        timer_holder = []

        def wake() -> None:
            if not future.done():
                future.set_result(None)
            if timer_holder:
                timer_holder[0].cancel()

        timer_holder.append(self.create_timer(max(0.001, seconds), wake))
        await future

    def _result(
        self,
        success: bool,
        code: int,
        message: str,
        position_error: float = 0.0,
        yaw_error: float = 0.0,
    ) -> DockToStation.Result:
        result = DockToStation.Result()
        result.success = success
        result.error_code = int(code)
        result.final_position_error = float(position_error)
        result.final_yaw_error = float(yaw_error)
        result.message = message
        return result

    def _lookup_marker(self, marker_frame: str):
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            marker_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            translation.x,
            translation.y,
            quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        marker_id = int(request.marker_id)
        marker_frame = f"{self.marker_frame_prefix}{marker_id}"
        timeout = (
            float(request.timeout_sec)
            if request.timeout_sec > 0.0
            else self.default_timeout_sec
        )
        deadline = time.monotonic() + timeout
        position_tolerance = max(0.001, float(request.position_tolerance))
        yaw_tolerance = max(0.005, float(request.yaw_tolerance))
        target_x = self.target_x
        target_y = self.target_y
        target_yaw = self.target_yaw
        target_pose = request.target_marker_pose
        if request.use_target_marker_pose:
            target_x = target_pose.position.x
            target_y = target_pose.position.y
            target_yaw = quaternion_to_yaw(
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            )
        try:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    goal_handle.canceled()
                    return self._result(False, TaskError.CANCELLED, "docking cancelled")
                try:
                    current_x, current_y, current_yaw = self._lookup_marker(marker_frame)
                except TransformException:
                    feedback = DockToStation.Feedback()
                    feedback.phase = "WAIT_FOR_MARKER_TF"
                    goal_handle.publish_feedback(feedback)
                    await self._sleep(self.control_period_sec)
                    continue

                x_error = current_x - target_x
                y_error = current_y - target_y
                yaw_error = normalize_angle(current_yaw - target_yaw)
                position_error = math.hypot(x_error, y_error)
                feedback = DockToStation.Feedback()
                feedback.phase = "ALIGNING"
                feedback.position_error = position_error
                feedback.yaw_error = abs(yaw_error)
                goal_handle.publish_feedback(feedback)

                if position_error <= position_tolerance and abs(yaw_error) <= yaw_tolerance:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    goal_handle.succeed()
                    return self._result(
                        True,
                        TaskError.OK,
                        "station docking complete",
                        position_error,
                        abs(yaw_error),
                    )

                self._publish_cmd(
                    max(-self.max_linear_speed, min(self.max_linear_speed, self.kp_x * x_error)),
                    max(-self.max_lateral_speed, min(self.max_lateral_speed, self.kp_y * y_error)),
                    max(
                        -self.max_angular_speed,
                        min(self.max_angular_speed, self.kp_yaw * yaw_error),
                    ),
                )
                await self._sleep(self.control_period_sec)

            self._publish_cmd(0.0, 0.0, 0.0)
            goal_handle.abort()
            return self._result(False, TaskError.DOCKING_FAILED, "station docking timed out")
        finally:
            self._publish_cmd(0.0, 0.0, 0.0)
            self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StationDockingNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
