"""ROS adapter for conservative, event-triggered object memory."""

import rclpy
from openflex_thesis_msgs.msg import ObjectMemory, ObjectObservation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .geometry import quaternion_to_yaw, yaw_to_quaternion
from .memory import MemoryConfig, ObjectMemoryFilter
from .observation_adapter import normalized_quaternion


class ObjectMemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("object_memory")
        self.declare_parameter("observation_topic", "/thesis/object_observation")
        self.declare_parameter("memory_topic", "/thesis/object_memory")
        self.declare_parameter("freshness_tau_sec", 2.0)
        self.declare_parameter("lost_timeout_sec", 1.0)
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("translation_move_threshold_m", 0.05)
        self.declare_parameter("yaw_move_threshold_rad", 0.15)
        self.declare_parameter("move_confirm_frames", 3)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.filter = ObjectMemoryFilter(
            MemoryConfig(
                freshness_tau_sec=float(self.get_parameter("freshness_tau_sec").value),
                lost_timeout_sec=float(self.get_parameter("lost_timeout_sec").value),
                min_confidence=float(self.get_parameter("min_confidence").value),
                translation_move_threshold_m=float(
                    self.get_parameter("translation_move_threshold_m").value
                ),
                yaw_move_threshold_rad=float(
                    self.get_parameter("yaw_move_threshold_rad").value
                ),
                move_confirm_frames=int(self.get_parameter("move_confirm_frames").value),
            )
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            ObjectMemory, str(self.get_parameter("memory_topic").value), qos
        )
        self.create_subscription(
            ObjectObservation,
            str(self.get_parameter("observation_topic").value),
            self.on_observation,
            qos,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self.on_timer)

    @staticmethod
    def stamp_seconds(msg) -> float:
        return float(msg.sec) + float(msg.nanosec) * 1.0e-9

    def on_observation(self, msg: ObjectObservation) -> None:
        if not msg.visible:
            return
        orientation = msg.pose.pose.orientation
        variances = (
            float(msg.pose.covariance[0]),
            float(msg.pose.covariance[7]),
            float(msg.pose.covariance[14]),
        )
        yaw_variance = float(msg.pose.covariance[35])
        try:
            normalized = normalized_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
            self.filter.observe(
                msg.object_id,
                msg.header.frame_id,
                (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ),
                quaternion_to_yaw(
                    *normalized
                ),
                variances,
                yaw_variance,
                msg.confidence,
                self.stamp_seconds(msg.header.stamp),
                msg.sequence,
                msg.source,
            )
        except ValueError as error:
            self.get_logger().warn(f"object observation rejected: {error}")

    def on_timer(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        state = self.filter.tick(now_sec)
        if state is None:
            return
        qx, qy, qz, qw = yaw_to_quaternion(state.yaw)
        msg = ObjectMemory()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = state.frame_id
        msg.object_id = state.object_id
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = state.position
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = state.variances[0]
        msg.pose.covariance[7] = state.variances[1]
        msg.pose.covariance[14] = state.variances[2]
        msg.pose.covariance[35] = state.yaw_variance
        msg.confidence = state.confidence
        msg.freshness = self.filter.freshness(now_sec)
        msg.visible = state.visible
        msg.grasp_authorized = self.filter.grasp_authorized(now_sec)
        msg.map_version = state.map_version
        msg.event_flags = state.event_flags
        msg.observation_sequence = state.sequence
        msg.source = state.source
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
