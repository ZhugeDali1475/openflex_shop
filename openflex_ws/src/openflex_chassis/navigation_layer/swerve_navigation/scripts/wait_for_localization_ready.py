#!/usr/bin/env python3

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class LocalizationReadinessGate(Node):
    """Exit only after LIO data and ICP localization are continuously healthy."""

    def __init__(self) -> None:
        super().__init__('wait_for_localization_ready')

        self.declare_parameter('body_cloud_topic', '/fastlio2/body_cloud')
        self.declare_parameter('lio_odom_topic', '/fastlio2/lio_odom')
        self.declare_parameter('localization_topic', '/localization_ok')
        self.declare_parameter('freshness_sec', 1.0)
        self.declare_parameter('stable_duration_sec', 1.5)
        self.declare_parameter('diagnostic_period_sec', 2.0)

        self.body_cloud_topic = str(
            self.get_parameter('body_cloud_topic').value)
        self.lio_odom_topic = str(self.get_parameter('lio_odom_topic').value)
        self.localization_topic = str(
            self.get_parameter('localization_topic').value)
        self.freshness_sec = max(
            0.1, float(self.get_parameter('freshness_sec').value))
        self.stable_duration_sec = max(
            0.0, float(self.get_parameter('stable_duration_sec').value))
        diagnostic_period = max(
            0.5, float(self.get_parameter('diagnostic_period_sec').value))

        self.last_cloud_time = None
        self.last_odom_time = None
        self.localization_ok = False
        self.localization_seen = False
        self.ready_since = None
        self.ready = False

        self.create_subscription(
            PointCloud2,
            self.body_cloud_topic,
            self._on_body_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.lio_odom_topic,
            self._on_lio_odom,
            qos_profile_sensor_data,
        )
        localization_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool,
            self.localization_topic,
            self._on_localization,
            localization_qos,
        )
        self.check_timer = self.create_timer(0.1, self._check_ready)
        self.diagnostic_timer = self.create_timer(
            diagnostic_period, self._report_waiting)

        self.get_logger().info(
            'Holding Nav2 startup until fresh LIO odometry, body cloud, and '
            f'{self.localization_topic}=true remain stable for '
            f'{self.stable_duration_sec:.1f}s')

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_body_cloud(self, _msg: PointCloud2) -> None:
        self.last_cloud_time = self._now()

    def _on_lio_odom(self, _msg: Odometry) -> None:
        self.last_odom_time = self._now()

    def _on_localization(self, msg: Bool) -> None:
        self.localization_seen = True
        self.localization_ok = bool(msg.data)
        if not self.localization_ok:
            self.ready_since = None

    def _age(self, stamp, now: float):
        if stamp is None:
            return None
        return max(0.0, now - stamp)

    def _inputs_ready(self, now: float) -> bool:
        cloud_age = self._age(self.last_cloud_time, now)
        odom_age = self._age(self.last_odom_time, now)
        return (
            cloud_age is not None
            and cloud_age <= self.freshness_sec
            and odom_age is not None
            and odom_age <= self.freshness_sec
            and self.localization_ok
        )

    def _check_ready(self) -> None:
        now = self._now()
        if not self._inputs_ready(now):
            self.ready_since = None
            return
        if self.ready_since is None:
            self.ready_since = now
            return
        if now - self.ready_since < self.stable_duration_sec:
            return
        self.ready = True
        self.get_logger().info(
            'Localization input chain is stable; Nav2 may now start')

    def _report_waiting(self) -> None:
        now = self._now()
        cloud_age = self._age(self.last_cloud_time, now)
        odom_age = self._age(self.last_odom_time, now)

        def describe(age) -> str:
            return 'never received' if age is None else f'age={age:.2f}s'

        localization = (
            str(self.localization_ok).lower()
            if self.localization_seen
            else 'never received'
        )
        self.get_logger().warn(
            'Waiting for localization chain: '
            f'{self.body_cloud_topic} {describe(cloud_age)}, '
            f'{self.lio_odom_topic} {describe(odom_age)}, '
            f'{self.localization_topic}={localization}')


def main() -> None:
    rclpy.init()
    node = LocalizationReadinessGate()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if node.ready else 1)


if __name__ == '__main__':
    main()
