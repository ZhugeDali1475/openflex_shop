"""Convert an external target pose (for example DREAM) to ObjectObservation."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from openflex_thesis_msgs.msg import ObjectObservation
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from .observation_adapter import (
    default_pose_covariance,
    rotate_pose_covariance,
    transform_pose,
)


class ObjectObservationAdapterNode(Node):
    """Fail-closed pose adapter with explicit object identity and covariance."""

    def __init__(self) -> None:
        super().__init__("object_observation_adapter")
        defaults = {
            "pose_topic": "/dream/active_object_pose",
            "pose_covariance_topic": "",
            "confidence_topic": "",
            "output_topic": "/thesis/object_observation",
            "output_frame": "map",
            "expected_source_frame": "",
            "object_id": "thesis_item_01",
            "source_name": "dream_pose_adapter",
            "fixed_confidence": 0.90,
            "confidence_timeout_sec": 0.50,
            "max_pose_age_sec": 0.50,
            "tf_timeout_sec": 0.25,
            "position_variance": 0.0004,
            "yaw_variance": 0.0025,
            "allow_zero_stamp": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        pose_topic = self._string("pose_topic")
        covariance_topic = self._string("pose_covariance_topic")
        if bool(pose_topic) == bool(covariance_topic):
            raise ValueError(
                "configure exactly one of pose_topic or pose_covariance_topic"
            )
        if not self._string("object_id") or not self._string("output_frame"):
            raise ValueError("object_id and output_frame must be non-empty")
        fixed_confidence = self._float("fixed_confidence")
        if not 0.0 <= fixed_confidence <= 1.0:
            raise ValueError("fixed_confidence must be in [0, 1]")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sequence = 0
        self.latest_confidence = fixed_confidence
        self.latest_confidence_time = float("-inf")

        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            ObjectObservation, self._string("output_topic"), output_qos
        )
        if pose_topic:
            self.create_subscription(PoseStamped, pose_topic, self.on_pose, source_qos)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped,
                covariance_topic,
                self.on_pose_with_covariance,
                source_qos,
            )
        confidence_topic = self._string("confidence_topic")
        if confidence_topic:
            self.create_subscription(
                Float32, confidence_topic, self.on_confidence, source_qos
            )
        self.get_logger().info(
            f"adapting {pose_topic or covariance_topic} -> {self._string('output_topic')} "
            f"in frame {self._string('output_frame')}"
        )

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value).strip()

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def on_confidence(self, msg: Float32) -> None:
        value = float(msg.data)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            self.get_logger().warn("discarding confidence outside [0, 1]")
            return
        self.latest_confidence = value
        self.latest_confidence_time = self.get_clock().now().nanoseconds * 1.0e-9

    def on_pose(self, msg: PoseStamped) -> None:
        covariance = default_pose_covariance(
            self._float("position_variance"), self._float("yaw_variance")
        )
        self._convert(msg.header, msg.pose, covariance)

    def on_pose_with_covariance(self, msg: PoseWithCovarianceStamped) -> None:
        self._convert(msg.header, msg.pose.pose, list(msg.pose.covariance))

    def _convert(self, header, pose, covariance: list[float]) -> None:
        try:
            source_frame = str(header.frame_id).strip()
            expected = self._string("expected_source_frame")
            if not source_frame:
                raise ValueError("source pose has an empty frame_id")
            if expected and source_frame != expected:
                raise ValueError(
                    f"source frame {source_frame!r} does not match expected {expected!r}"
                )
            stamp_sec = self._stamp_seconds(header.stamp)
            if stamp_sec <= 0.0 and not bool(
                self.get_parameter("allow_zero_stamp").value
            ):
                raise ValueError("source pose has a zero timestamp")
            now_sec = self.get_clock().now().nanoseconds * 1.0e-9
            age = now_sec - stamp_sec
            maximum_age = self._float("max_pose_age_sec")
            if age < -0.1 or age > maximum_age:
                raise ValueError(f"source pose age {age:.3f}s exceeds allowed range")

            output_frame = self._string("output_frame")
            if source_frame == output_frame:
                translation = (0.0, 0.0, 0.0)
                rotation = (0.0, 0.0, 0.0, 1.0)
            else:
                tf = self.tf_buffer.lookup_transform(
                    output_frame,
                    source_frame,
                    Time.from_msg(header.stamp),
                    timeout=Duration(seconds=self._float("tf_timeout_sec")),
                ).transform
                translation = (tf.translation.x, tf.translation.y, tf.translation.z)
                rotation = (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
            position, orientation = transform_pose(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                translation,
                rotation,
            )
            target_covariance = rotate_pose_covariance(covariance, rotation)
            confidence = self._float("fixed_confidence")
            if (
                now_sec - self.latest_confidence_time
                <= self._float("confidence_timeout_sec")
            ):
                confidence = self.latest_confidence

            output = ObjectObservation()
            output.header.stamp = header.stamp
            output.header.frame_id = output_frame
            output.object_id = self._string("object_id")
            output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = position
            output.pose.pose.orientation.x, output.pose.pose.orientation.y = orientation[:2]
            output.pose.pose.orientation.z, output.pose.pose.orientation.w = orientation[2:]
            output.pose.covariance = target_covariance
            output.confidence = float(confidence)
            output.visible = True
            self.sequence += 1
            output.sequence = self.sequence
            output.source = self._string("source_name")
            self.publisher.publish(output)
        except (TransformException, ValueError) as error:
            self.get_logger().warn(f"object pose rejected: {error}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectObservationAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
