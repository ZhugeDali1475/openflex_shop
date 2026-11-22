"""Estimator-only FAST-LIO odometry TF bridge."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class LioOdomTfBridge(Node):
    def __init__(self):
        super().__init__("lio_odom_tf_bridge")
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 20)

    def on_odom(self, msg):
        if msg.header.frame_id != self.odom_frame:
            self.get_logger().warning(
                f"rejecting odometry frame {msg.header.frame_id!r}; expected {self.odom_frame!r}")
            return
        if msg.child_frame_id != self.base_frame:
            self.get_logger().warning(
                f"rejecting odometry child {msg.child_frame_id!r}; expected {self.base_frame!r}")
            return
        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = LioOdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
