"""Collision-unaware navigation baseline for simulator integration; not Nav2."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .math_utils import clamp, normalize_angle


class SimNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_navigation_baseline_not_nav2")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("output_topic", "/thesis/nav_cmd_vel")
        self.declare_parameter("odometry_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("linear_gain", 0.9)
        self.declare_parameter("angular_gain", 1.5)
        self.declare_parameter("max_linear_mps", 0.35)
        self.declare_parameter("max_angular_rps", 0.50)
        self.declare_parameter("position_tolerance_m", 0.04)
        self.declare_parameter("yaw_tolerance_rad", 0.05)
        self.declare_parameter("timeout_sec", 60.0)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            20,
        )
        self.callback_group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.odometry = None
        self.busy = False
        self.get_logger().warn(
            "SIM NAVIGATION BASELINE active: proportional, collision-unaware and NOT Nav2"
        )

    def on_odometry(self, message: Odometry) -> None:
        self.odometry = deepcopy(message)

    def goal_callback(self, request) -> GoalResponse:
        valid = request.pose.header.frame_id in ("map", "odom")
        return GoalResponse.ACCEPT if valid and not self.busy else GoalResponse.REJECT

    @staticmethod
    def yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def velocity(self, current, target):
        command = Twist()
        yaw = self.yaw(current.orientation)
        dx = target.position.x - current.position.x
        dy = target.position.y - current.position.y
        distance = math.hypot(dx, dy)
        body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        yaw_error = normalize_angle(self.yaw(target.orientation) - yaw)
        if distance <= float(self.get_parameter("position_tolerance_m").value):
            body_x = 0.0
            body_y = 0.0
        if abs(yaw_error) <= float(self.get_parameter("yaw_tolerance_rad").value):
            yaw_error = 0.0
        gain = float(self.get_parameter("linear_gain").value)
        angular_gain = float(self.get_parameter("angular_gain").value)
        maximum_linear = float(self.get_parameter("max_linear_mps").value)
        maximum_angular = float(self.get_parameter("max_angular_rps").value)
        command.linear.x = clamp(gain * body_x, -maximum_linear, maximum_linear)
        command.linear.y = clamp(gain * body_y, -maximum_linear, maximum_linear)
        command.angular.z = clamp(
            angular_gain * yaw_error, -maximum_angular, maximum_angular
        )
        return command, distance, abs(yaw_error)

    def execute(self, goal_handle):
        self.busy = True
        started = time.monotonic()
        result = NavigateToPose.Result()
        try:
            while time.monotonic() - started <= float(
                self.get_parameter("timeout_sec").value
            ):
                if goal_handle.is_cancel_requested:
                    self.publisher.publish(Twist())
                    goal_handle.canceled()
                    return result
                odometry = self.odometry
                if odometry is None:
                    time.sleep(0.02)
                    continue
                command, distance, yaw_error = self.velocity(
                    odometry.pose.pose, goal_handle.request.pose.pose
                )
                feedback = NavigateToPose.Feedback()
                feedback.current_pose.header = deepcopy(odometry.header)
                feedback.current_pose.pose = deepcopy(odometry.pose.pose)
                feedback.distance_remaining = float(distance)
                goal_handle.publish_feedback(feedback)
                if (
                    distance <= float(self.get_parameter("position_tolerance_m").value)
                    and yaw_error <= float(self.get_parameter("yaw_tolerance_rad").value)
                ):
                    self.publisher.publish(Twist())
                    goal_handle.succeed()
                    return result
                self.publisher.publish(command)
                time.sleep(0.02)
            self.publisher.publish(Twist())
            goal_handle.abort()
            return result
        finally:
            self.busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimNavigationNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
