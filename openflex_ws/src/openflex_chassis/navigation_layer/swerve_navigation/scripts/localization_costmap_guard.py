#!/usr/bin/env python3

from functools import partial
from threading import Lock
from typing import Dict, Optional, Set

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty


class LocalizationCostmapGuard(Node):
    """Clear stale obstacle marks after localization becomes trustworthy again."""

    def __init__(self) -> None:
        super().__init__('localization_costmap_guard')

        self.declare_parameter('localization_state_topic', '/localization_ok')
        self.declare_parameter(
            'global_clear_service',
            '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter(
            'local_clear_service',
            '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter(
            'cleared_event_topic', '/localization_costmap_cleared')
        self.declare_parameter('clear_delay', 0.10)
        self.declare_parameter('retry_period', 0.25)
        self.declare_parameter('max_attempts', 40)

        state_topic = str(self.get_parameter('localization_state_topic').value)
        self.clear_delay = max(0.0, float(self.get_parameter('clear_delay').value))
        retry_period = max(0.05, float(self.get_parameter('retry_period').value))
        self.max_attempts = max(1, int(self.get_parameter('max_attempts').value))

        service_names = (
            str(self.get_parameter('global_clear_service').value),
            str(self.get_parameter('local_clear_service').value),
        )
        self.clear_clients = {
            name: self.create_client(ClearEntireCostmap, name)
            for name in service_names
        }

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Bool, state_topic, self._on_localization_state, state_qos)
        self.cleared_pub = self.create_publisher(
            Empty, str(self.get_parameter('cleared_event_topic').value), event_qos)

        self.localization_ok = False
        self.state_seen = False
        self.generation = 0
        self.state_lock = Lock()
        self.completion_generation: Optional[int] = None
        self.pending: Set[str] = set()
        self.inflight: Dict[str, int] = {}
        self.attempts: Dict[str, int] = {}
        self.exhausted: Set[str] = set()
        self.clear_due = None
        self.timer = self.create_timer(retry_period, self._on_timer)

        self.get_logger().info(
            f'Localization costmap guard started: state={state_topic}, '
            f'services={list(service_names)}')

    def _on_localization_state(self, msg: Bool) -> None:
        was_ok: Optional[bool] = self.localization_ok if self.state_seen else None
        self.state_seen = True
        self.localization_ok = bool(msg.data)

        if not self.localization_ok:
            self.generation += 1
            self.pending.clear()
            self.inflight.clear()
            self.attempts.clear()
            self.exhausted.clear()
            self.clear_due = None
            return

        if was_ok is True:
            return

        self.generation += 1
        self.completion_generation = None
        self.pending = set(self.clear_clients)
        self.attempts = {name: 0 for name in self.clear_clients}
        self.exhausted.clear()
        self.clear_due = self.get_clock().now() + Duration(seconds=self.clear_delay)
        self.get_logger().info(
            'Localization became healthy; scheduling global and local costmap clear')

    def _on_timer(self) -> None:
        if not self.localization_ok or not self.pending or self.clear_due is None:
            return
        if self.get_clock().now() < self.clear_due:
            return

        for service_name in tuple(self.pending):
            if service_name in self.inflight or service_name in self.exhausted:
                continue

            attempts = self.attempts.get(service_name, 0)
            if attempts >= self.max_attempts:
                self.exhausted.add(service_name)
                self.get_logger().error(
                    f'Costmap clear service failed after {attempts} attempts: '
                    f'{service_name}')
                continue

            self.attempts[service_name] = attempts + 1
            client = self.clear_clients[service_name]
            if not client.service_is_ready():
                if attempts == 0 or (attempts + 1) % 10 == 0:
                    self.get_logger().warn(
                        f'Waiting for costmap clear service: {service_name} '
                        f'(attempt {attempts + 1}/{self.max_attempts})')
                continue

            generation = self.generation
            self.inflight[service_name] = generation
            future = client.call_async(ClearEntireCostmap.Request())
            future.add_done_callback(
                partial(self._on_clear_done, service_name, generation))

    def _on_clear_done(self, service_name: str, generation: int, future) -> None:
        if self.inflight.get(service_name) == generation:
            self.inflight.pop(service_name, None)
        if generation != self.generation or not self.localization_ok:
            return

        try:
            future.result()
        except Exception as exc:  # rclpy service errors are surfaced through Future
            self.get_logger().warn(
                f'Costmap clear call failed for {service_name}: {exc}; retrying')
            return

        publish_completion = False
        with self.state_lock:
            self.pending.discard(service_name)
            if (not self.pending and
                    self.completion_generation != generation):
                self.completion_generation = generation
                self.clear_due = None
                publish_completion = True

        self.get_logger().info(f'Costmap cleared after localization recovery: {service_name}')
        if publish_completion:
            self.cleared_pub.publish(Empty())
            self.get_logger().info(
                'Localization recovery costmap clear completed for both costmaps')


def main() -> None:
    rclpy.init()
    node = LocalizationCostmapGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
