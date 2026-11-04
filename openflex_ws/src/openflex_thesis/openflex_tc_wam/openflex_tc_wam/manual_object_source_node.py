"""Publish a calibrated manual object pose for offline interface validation."""

import math

import rclpy
from openflex_thesis_msgs.msg import ObjectObservation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .geometry import yaw_to_quaternion


class ManualObjectSourceNode(Node):
    def __init__(self) -> None:
        super().__init__("manual_object_source")
        self.declare_parameter("topic", "/thesis/object_observation")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("object_id", "thesis_item_01")
        self.declare_parameter("position", [1.2, 0.0, 0.85])
        self.declare_parameter("yaw_rad", math.pi)
        self.declare_parameter("confidence", 0.95)
        self.declare_parameter("position_variance", 0.0004)
        self.declare_parameter("yaw_variance", 0.0025)
        self.declare_parameter("publish_rate_hz", 5.0)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            ObjectObservation, str(self.get_parameter("topic").value), qos
        )
        self.sequence = 0
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.create_timer(1.0 / rate, self.publish_observation)

    def publish_observation(self) -> None:
        position = [float(value) for value in self.get_parameter("position").value]
        if len(position) != 3:
            self.get_logger().error("position must contain exactly three values")
            return
        yaw = float(self.get_parameter("yaw_rad").value)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        variance = float(self.get_parameter("position_variance").value)
        yaw_variance = float(self.get_parameter("yaw_variance").value)
        msg = ObjectObservation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.object_id = str(self.get_parameter("object_id").value)
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = variance
        msg.pose.covariance[7] = variance
        msg.pose.covariance[14] = variance
        msg.pose.covariance[35] = yaw_variance
        msg.confidence = float(self.get_parameter("confidence").value)
        msg.visible = True
        self.sequence += 1
        msg.sequence = self.sequence
        msg.source = "manual_calibrated_pose"
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualObjectSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
