#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty

from cmd_vel_safety_gate_core import authorization_is_fresh, reverse_policy_violation


class CmdVelSafetyGate(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_safety_gate')

        self.declare_parameter('input_cmd_topic', '/cmd_vel_safe_in')
        self.declare_parameter('output_cmd_topic', '/cmd_vel_safe')
        self.declare_parameter('hold_topic', '/localization_hold')
        self.declare_parameter('localization_state_topic', '/localization_ok')
        self.declare_parameter('require_localization_state', True)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('hold_duration', 1.5)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter(
            'reverse_authorization_topic',
            '/competition/post_pick_reverse_authorized')
        self.declare_parameter('reverse_authorization_timeout', 0.25)
        self.declare_parameter('authorized_reverse_max_speed', 0.06)
        self.declare_parameter('authorized_reverse_lateral_tolerance', 1.0e-3)
        self.declare_parameter('authorized_reverse_angular_tolerance', 1.0e-3)

        self.input_cmd_topic = str(self.get_parameter('input_cmd_topic').value)
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.hold_topic = str(self.get_parameter('hold_topic').value)
        self.localization_state_topic = str(
            self.get_parameter('localization_state_topic').value)
        self.require_localization_state = bool(
            self.get_parameter('require_localization_state').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.hold_duration = float(self.get_parameter('hold_duration').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.reverse_authorization_topic = str(
            self.get_parameter('reverse_authorization_topic').value)
        self.reverse_authorization_timeout = float(
            self.get_parameter('reverse_authorization_timeout').value)
        self.authorized_reverse_max_speed = float(
            self.get_parameter('authorized_reverse_max_speed').value)
        self.authorized_reverse_lateral_tolerance = float(
            self.get_parameter('authorized_reverse_lateral_tolerance').value)
        self.authorized_reverse_angular_tolerance = float(
            self.get_parameter('authorized_reverse_angular_tolerance').value)
        self.reverse_epsilon = 1.0e-3
        if (
            self.reverse_authorization_timeout <= 0.0
            or self.authorized_reverse_max_speed <= 0.0
            or self.authorized_reverse_lateral_tolerance < 0.0
            or self.authorized_reverse_angular_tolerance < 0.0
        ):
            raise ValueError('authorized reverse safety parameters are invalid')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, qos)
        self.create_subscription(Twist, self.input_cmd_topic, self._on_cmd, qos)
        self.create_subscription(Empty, self.hold_topic, self._on_hold, qos)
        self.create_subscription(
            Bool,
            self.reverse_authorization_topic,
            self._on_reverse_authorization,
            qos)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool, self.localization_state_topic, self._on_localization_state, state_qos)

        self.last_cmd: Optional[Twist] = None
        self.last_cmd_time = None
        self.hold_until = None
        self.hold_active = False
        self.localization_ok = not self.require_localization_state
        self.reverse_block_active = False
        self.reverse_block_reason: Optional[str] = None
        self.last_reverse_authorization_time = None
        self.reverse_authorization_active = False

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'cmd_vel safety gate started: in={self.input_cmd_topic}, '
            f'out={self.output_cmd_topic}, hold={self.hold_topic}, '
            f'localization_state={self.localization_state_topic}, '
            f'allow_reverse={self.allow_reverse}, '
            f'reverse_authorization={self.reverse_authorization_topic}, '
            f'hold_duration={self.hold_duration:.2f}s, '
            f'initially_allowed={self.localization_ok}'
        )

    def _on_cmd(self, msg: Twist) -> None:
        safe_msg = self._apply_direction_policy(msg)
        self.last_cmd = safe_msg
        self.last_cmd_time = self.get_clock().now()
        if self._motion_allowed():
            self.cmd_pub.publish(safe_msg)

    def _apply_direction_policy(self, msg: Twist) -> Twist:
        violation = reverse_policy_violation(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            allow_reverse=self.allow_reverse,
            reverse_authorized=self._reverse_authorization_is_fresh(),
            reverse_epsilon=self.reverse_epsilon,
            authorized_reverse_max_speed=self.authorized_reverse_max_speed,
            authorized_reverse_lateral_tolerance=(
                self.authorized_reverse_lateral_tolerance),
            authorized_reverse_angular_tolerance=(
                self.authorized_reverse_angular_tolerance),
        )
        if violation is not None:
            if not self.reverse_block_active:
                self.get_logger().error(
                    f'Reverse cmd_vel rejected: {violation}')
            elif violation != self.reverse_block_reason:
                self.get_logger().error(
                    f'Reverse cmd_vel still rejected: {violation}')
            self.reverse_block_active = True
            self.reverse_block_reason = violation
            return Twist()

        if self.reverse_block_active:
            self.get_logger().info('Reverse cmd_vel rejection cleared')
            self.reverse_block_active = False
            self.reverse_block_reason = None
        return msg

    def _reverse_authorization_is_fresh(self) -> bool:
        return authorization_is_fresh(
            self.last_reverse_authorization_time,
            self.get_clock().now().nanoseconds * 1e-9,
            self.reverse_authorization_timeout,
        )

    def _on_reverse_authorization(self, msg: Bool) -> None:
        was_active = self._reverse_authorization_is_fresh()
        if msg.data:
            self.last_reverse_authorization_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )
            self.reverse_authorization_active = True
            if not was_active:
                self.get_logger().warn(
                    'Short post-pick straight-reverse authorization enabled')
            return

        self.last_reverse_authorization_time = None
        self.reverse_authorization_active = False
        self.last_cmd = Twist()
        self.cmd_pub.publish(Twist())
        if was_active:
            self.get_logger().info('Post-pick reverse authorization revoked')

    def _expire_reverse_authorization(self) -> bool:
        if (
            not self.reverse_authorization_active
            or self._reverse_authorization_is_fresh()
        ):
            return False
        self.reverse_authorization_active = False
        self.last_reverse_authorization_time = None
        self.last_cmd = Twist()
        self.cmd_pub.publish(Twist())
        self.get_logger().warn(
            'Post-pick reverse authorization expired; forcing zero cmd_vel')
        return True

    def _on_hold(self, _msg: Empty) -> None:
        self.hold_until = self.get_clock().now() + Duration(seconds=self.hold_duration)
        if not self.hold_active:
            self.get_logger().warn(
                f'Localization hold received, forcing zero cmd_vel for {self.hold_duration:.2f}s')
        self.hold_active = True
        self.cmd_pub.publish(Twist())

    def _on_localization_state(self, msg: Bool) -> None:
        was_ok = self.localization_ok
        self.localization_ok = bool(msg.data)
        if not self.localization_ok:
            if was_ok:
                self.get_logger().error(
                    'Localization lost/not stable; cmd_vel is latched at zero')
            self.cmd_pub.publish(Twist())
        elif not was_ok:
            self.get_logger().info(
                'Localization stable; cmd_vel relay is enabled')

    def _hold_is_active(self) -> bool:
        if self.hold_until is None:
            return False
        return self.get_clock().now() < self.hold_until

    def _cmd_is_fresh(self) -> bool:
        if self.last_cmd is None or self.last_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age <= self.cmd_timeout

    def _motion_allowed(self) -> bool:
        return self.localization_ok and not self._hold_is_active()

    def _on_timer(self) -> None:
        if self._expire_reverse_authorization():
            return
        if not self.localization_ok or self._hold_is_active():
            self.cmd_pub.publish(Twist())
            return

        if self.hold_active:
            self.hold_active = False
            self.get_logger().info('Timed localization hold cleared')

        if not self._cmd_is_fresh():
            self.cmd_pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = CmdVelSafetyGate()
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
