"""Transparent proportional local-alignment baseline; this node is not ACT."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.msg import PolicyControl
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .math_utils import clamp, normalize_angle


class ScriptedLocalPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("scripted_local_alignment_baseline")
        self.declare_parameter("policy_control_topic", "/thesis/act/policy_control")
        self.declare_parameter("output_topic", "/thesis/act_cmd_vel")
        self.declare_parameter("odometry_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("linear_gain", 1.1)
        self.declare_parameter("angular_gain", 1.8)
        self.declare_parameter("max_linear_mps", 0.18)
        self.declare_parameter("max_angular_rps", 0.30)
        self.declare_parameter("position_tolerance_m", 0.025)
        self.declare_parameter("yaw_tolerance_rad", 0.035)
        self.declare_parameter("odom_timeout_sec", 0.25)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PolicyControl,
            str(self.get_parameter("policy_control_topic").value),
            self.on_policy_control,
            qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            20,
        )
        self.control = None
        self.odometry = None
        self.odom_receipt = 0.0
        self.create_timer(1.0 / 30.0, self.on_timer)
        self.get_logger().warn(
            "SCRIPTED proportional local policy active. It is a baseline, not ACT, "
            "and its trials must be labeled separately."
        )

    def on_policy_control(self, message: PolicyControl) -> None:
        self.control = deepcopy(message) if message.enabled else None
        if not message.enabled:
            self.publisher.publish(Twist())

    def on_odometry(self, message: Odometry) -> None:
        self.odometry = deepcopy(message)
        self.odom_receipt = time.monotonic()

    @staticmethod
    def yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def on_timer(self) -> None:
        command = Twist()
        control = self.control
        odometry = self.odometry
        if (
            control is None
            or odometry is None
            or time.monotonic() - self.odom_receipt
            > float(self.get_parameter("odom_timeout_sec").value)
        ):
            self.publisher.publish(command)
            return
        target = control.target_base_pose.pose
        current = odometry.pose.pose
        yaw = self.yaw(current.orientation)
        dx = target.position.x - current.position.x
        dy = target.position.y - current.position.y
        body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        target_yaw = self.yaw(target.orientation)
        yaw_error = normalize_angle(target_yaw - yaw)
        if math.hypot(dx, dy) <= float(
            self.get_parameter("position_tolerance_m").value
        ):
            body_x = 0.0
            body_y = 0.0
        if abs(yaw_error) <= float(self.get_parameter("yaw_tolerance_rad").value):
            yaw_error = 0.0
        linear_gain = float(self.get_parameter("linear_gain").value)
        angular_gain = float(self.get_parameter("angular_gain").value)
        maximum_linear = float(self.get_parameter("max_linear_mps").value)
        maximum_angular = float(self.get_parameter("max_angular_rps").value)
        command.linear.x = clamp(linear_gain * body_x, -maximum_linear, maximum_linear)
        command.linear.y = clamp(linear_gain * body_y, -maximum_linear, maximum_linear)
        command.angular.z = clamp(
            angular_gain * yaw_error, -maximum_angular, maximum_angular
        )
        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScriptedLocalPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
