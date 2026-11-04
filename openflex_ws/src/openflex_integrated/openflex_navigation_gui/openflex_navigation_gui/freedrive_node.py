#!/usr/bin/env python3
"""Continuous gravity-compensated free-drive mode for OpenArmX arms."""

import math
import signal
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


POSITION_JOINTS = {
    side: [f'openarmx_{side}_joint{i}' for i in range(1, 8)]
    + [f'openarmx_{side}_finger_joint1']
    for side in ('left', 'right')
}
PARAMETER_NODES = {
    'left': '/openarmx_left_hardware_params',
    'right': '/openarmx_right_hardware_params',
}


class FreeDriveNode(Node):
    """Set arm stiffness to zero while gravity compensation remains healthy."""

    def __init__(self) -> None:
        super().__init__('openarmx_freedrive')

        self.declare_parameter('arm_selection', 'both')
        self.declare_parameter('follow_rate_hz', 100.0)
        self.declare_parameter('startup_timeout_sec', 30.0)
        self.declare_parameter('status_timeout_sec', 0.5)
        self.declare_parameter('restore_kp_on_exit', True)
        self.declare_parameter('restore_settle_sec', 0.5)

        selection = str(self.get_parameter('arm_selection').value)
        if selection not in ('left', 'right', 'both'):
            raise ValueError("arm_selection must be 'left', 'right', or 'both'")
        self.sides = ['left', 'right'] if selection == 'both' else [selection]

        self.startup_timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.status_timeout = float(self.get_parameter('status_timeout_sec').value)
        self.restore_on_exit = bool(self.get_parameter('restore_kp_on_exit').value)
        self.restore_settle_sec = float(self.get_parameter('restore_settle_sec').value)
        if self.startup_timeout <= 0.0 or self.status_timeout <= 0.0:
            raise ValueError('timeout parameters must be positive')

        self._joint_positions: Dict[str, float] = {}
        self._last_joint_state_time = 0.0
        self._last_gravity_time = {side: 0.0 for side in self.sides}
        self._original_kp: Dict[str, Dict[str, float]] = {}
        self._original_modes: Dict[str, str] = {}
        self._changed_sides: List[str] = []
        self._switched_sides: List[str] = []
        self._mirror_enabled = False
        self._moveit_disabled_by_us = False
        self.active = False

        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )
        self._position_publishers = {
            side: self.create_publisher(
                Float64MultiArray,
                f'/{side}_forward_position_controller/commands',
                10,
            )
            for side in self.sides
        }
        self._gravity_subscriptions = []
        for side in self.sides:
            subscription = self.create_subscription(
                Float64MultiArray,
                f'/{side}_forward_effort_controller/commands',
                lambda msg, selected_side=side: self._gravity_cb(selected_side, msg),
                10,
            )
            self._gravity_subscriptions.append(subscription)

        self._get_clients = {
            side: self.create_client(
                GetParameters,
                f'{PARAMETER_NODES[side]}/get_parameters',
            )
            for side in self.sides
        }
        self._set_clients = {
            side: self.create_client(
                SetParameters,
                f'{PARAMETER_NODES[side]}/set_parameters',
            )
            for side in self.sides
        }
        self._moveit_enabled_client = self.create_client(
            SetBool, '/moveit_command_bridge/set_enabled'
        )

        follow_rate = float(self.get_parameter('follow_rate_hz').value)
        if follow_rate <= 0.0:
            raise ValueError('follow_rate_hz must be positive')
        self._follow_timer = self.create_timer(
            1.0 / follow_rate, self._publish_current_positions
        )

    @staticmethod
    def _now_monotonic() -> float:
        return time.monotonic()

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if math.isfinite(position):
                self._joint_positions[name] = float(position)
        self._last_joint_state_time = self._now_monotonic()

    def _gravity_cb(self, side: str, msg: Float64MultiArray) -> None:
        if len(msg.data) == 7 and all(math.isfinite(value) for value in msg.data):
            self._last_gravity_time[side] = self._now_monotonic()

    def _publish_current_positions(self) -> None:
        if not self._mirror_enabled:
            return
        for side, publisher in self._position_publishers.items():
            joints = POSITION_JOINTS[side]
            if not all(joint in self._joint_positions for joint in joints):
                continue
            message = Float64MultiArray()
            message.data = [self._joint_positions[joint] for joint in joints]
            publisher.publish(message)

    def _stream_health_reason(self) -> Optional[str]:
        now = self._now_monotonic()
        if now - self._last_joint_state_time > self.status_timeout:
            return 'joint state stream is missing or stale'
        for side in self.sides:
            if now - self._last_gravity_time[side] > self.status_timeout:
                return f'{side} gravity compensation stream is missing or stale'
        return None

    def _startup_reason(self) -> Optional[str]:
        for side in self.sides:
            missing = [
                joint for joint in POSITION_JOINTS[side]
                if joint not in self._joint_positions
            ]
            if missing:
                return f'waiting for joint states ({side}: {", ".join(missing)})'
            if self._last_gravity_time[side] == 0.0:
                return f'waiting for {side} gravity compensation commands'
            if self._position_publishers[side].get_subscription_count() == 0:
                return f'waiting for {side}_forward_position_controller'
            if not self._get_clients[side].service_is_ready():
                return f'waiting for {PARAMETER_NODES[side]} parameter services'
            if not self._set_clients[side].service_is_ready():
                return f'waiting for {PARAMETER_NODES[side]} parameter services'
        return self._stream_health_reason()

    def _coordinate_moveit(self) -> None:
        """Disable MoveIt goals before taking over the shared position topics."""
        bridge_present = any(
            info.node_name == 'moveit_command_bridge'
            for side in self.sides
            for info in self.get_publishers_info_by_topic(
                f'/{side}_forward_position_controller/commands'
            )
        )
        if self._moveit_enabled_client.wait_for_service(timeout_sec=0.5):
            request = SetBool.Request()
            request.data = False
            response = self._wait_for_future(
                self._moveit_enabled_client.call_async(request), timeout_sec=2.0
            )
            if not response.success:
                raise RuntimeError(
                    f'MoveIt refused free-drive handoff: {response.message}'
                )
            self._moveit_disabled_by_us = True
            self.get_logger().info('MoveIt command bridge disabled for free-drive')
        elif bridge_present:
            raise RuntimeError(
                'moveit_command_bridge is publishing but its free-drive interlock service '
                'is unavailable; rebuild/restart the MoveIt bridge first'
            )

        for side in self.sides:
            topic = f'/{side}_forward_position_controller/commands'
            other_nodes = sorted({
                info.node_name
                for info in self.get_publishers_info_by_topic(topic)
                if info.node_name != self.get_name()
            })
            if other_nodes:
                self.get_logger().warn(
                    f'{topic} also has idle publisher(s): {", ".join(other_nodes)}; '
                    'do not send slider/teleop commands during free-drive'
                )

    def _spin_for(self, duration_sec: float) -> None:
        deadline = self._now_monotonic() + max(duration_sec, 0.0)
        while rclpy.ok() and self._now_monotonic() < deadline:
            remaining = max(deadline - self._now_monotonic(), 0.0)
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    def wait_until_ready(self) -> bool:
        deadline = self._now_monotonic() + self.startup_timeout
        while rclpy.ok() and self._now_monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(self._get_clients[side].service_is_ready() for side in self.sides):
                break
        else:
            self.get_logger().error('hardware parameter services are unavailable')
            return False

        last_reason = None
        while rclpy.ok() and self._now_monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            reason = self._startup_reason()
            if reason is None:
                try:
                    self._coordinate_moveit()
                except RuntimeError as exc:
                    self.get_logger().error(str(exc))
                    return False
                self._mirror_enabled = True
                self._spin_for(0.25)
                return True
            if reason != last_reason:
                self.get_logger().info(reason)
                last_reason = reason
        self.get_logger().error(last_reason or 'free-drive startup timed out')
        return False

    def _wait_for_future(self, future, timeout_sec: float = 5.0):
        deadline = self._now_monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and self._now_monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError('ROS parameter service request timed out')
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    def _get_kp(self, side: str) -> Dict[str, float]:
        names = [f'kp_joint{index}' for index in range(1, 8)]
        request = GetParameters.Request()
        request.names = names
        response = self._wait_for_future(self._get_clients[side].call_async(request))
        if len(response.values) != len(names):
            raise RuntimeError(f'{side} parameter service returned an incomplete response')

        values = {}
        for name, value in zip(names, response.values):
            if value.type != ParameterType.PARAMETER_DOUBLE:
                raise RuntimeError(f'{PARAMETER_NODES[side]}/{name} is not a double')
            values[name] = float(value.double_value)
        return values

    def _get_arm_control_mode(self, side: str) -> str:
        request = GetParameters.Request()
        request.names = ['arm_control_mode']
        response = self._wait_for_future(self._get_clients[side].call_async(request))
        if len(response.values) != 1:
            raise RuntimeError(f'{side} hardware did not report arm_control_mode')
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            raise RuntimeError(f'{side} arm_control_mode is unavailable; rebuild the hardware package')
        return str(value.string_value).lower()

    def _set_kp(self, side: str, values: Dict[str, float]) -> None:
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=float(value)).to_parameter_msg()
            for name, value in values.items()
        ]
        response = self._wait_for_future(self._set_clients[side].call_async(request))
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            raise RuntimeError(f'{side} KP update failed: {"; ".join(failures)}')

    def _set_arm_control_mode(self, side: str, mode: str) -> None:
        requested_mode = str(mode).lower()
        if requested_mode not in ('mit', 'csp'):
            raise ValueError(f'unsupported arm control mode: {mode}')
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name='arm_control_mode', value=requested_mode).to_parameter_msg()
        ]
        response = self._wait_for_future(
            self._set_clients[side].call_async(request), timeout_sec=10.0
        )
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            raise RuntimeError(
                f'{side} arm mode switch to {requested_mode} failed: {"; ".join(failures)}'
            )
        self.get_logger().info(f'{side} arm switched to {requested_mode}')

    def activate(self) -> bool:
        if not self.wait_until_ready():
            return False
        try:
            for side in self.sides:
                self._original_modes[side] = self._get_arm_control_mode(side)
                self._original_kp[side] = self._get_kp(side)
            for side in self.sides:
                if self._original_modes[side] != 'mit':
                    self._set_arm_control_mode(side, 'mit')
                    self._switched_sides.append(side)
            self._spin_for(0.2)
            zero_kp = {f'kp_joint{index}': 0.0 for index in range(1, 8)}
            for side in self.sides:
                self._set_kp(side, zero_kp)
                self._changed_sides.append(side)
            self.active = True
            self.get_logger().warn(
                'FREE-DRIVE ACTIVE: KP=0, existing KD retained; support the arms '
                'and keep the emergency stop accessible'
            )
            return True
        except Exception as exc:
            self.get_logger().error(f'failed to enter free-drive: {exc}')
            self.restore_original_kp()
            return False

    def health_reason(self) -> Optional[str]:
        return self._stream_health_reason() if self.active else None

    def restore_original_kp(self) -> None:
        if not self._changed_sides and not self._original_modes:
            self._mirror_enabled = False
            self._enable_moveit()
            return
        self.get_logger().info('capturing the current pose before restoring stiffness')
        self._spin_for(self.restore_settle_sec)
        restore_failed = False
        for side in list(self._changed_sides):
            try:
                self._set_kp(side, self._original_kp[side])
                self.get_logger().info(f'restored original {side} arm KP')
                self._changed_sides.remove(side)
            except Exception as exc:
                restore_failed = True
                self.get_logger().error(f'failed to restore {side} arm KP: {exc}')
        self._spin_for(self.restore_settle_sec)
        for side in reversed(self.sides):
            if side not in self._original_modes:
                continue
            try:
                original_mode = self._original_modes[side]
                if self._get_arm_control_mode(side) != original_mode:
                    self._set_arm_control_mode(side, original_mode)
            except Exception as exc:
                restore_failed = True
                self.get_logger().error(
                    f'failed to restore {side} arm mode: {exc}'
                )
        self._switched_sides.clear()
        self._spin_for(self.restore_settle_sec)
        self.active = False
        if restore_failed:
            self.get_logger().error(
                'free-drive recovery is incomplete; MoveIt remains disabled'
            )
            return
        self._mirror_enabled = False
        self._enable_moveit()

    def _enable_moveit(self) -> None:
        if not self._moveit_disabled_by_us:
            return
        if not self._moveit_enabled_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('MoveIt interlock service disappeared; bridge remains disabled')
            return
        request = SetBool.Request()
        request.data = True
        try:
            response = self._wait_for_future(
                self._moveit_enabled_client.call_async(request), timeout_sec=2.0
            )
            if response.success:
                self.get_logger().info('MoveIt command bridge re-enabled')
                self._moveit_disabled_by_us = False
            else:
                self.get_logger().error(f'failed to re-enable MoveIt: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'failed to re-enable MoveIt: {exc}')


def main() -> None:
    rclpy.init()
    stop_requested = threading.Event()

    def request_stop(_signum, _frame):
        stop_requested.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    node = None
    exit_code = 0
    try:
        node = FreeDriveNode()
        if not node.activate():
            exit_code = 1
        else:
            while rclpy.ok() and not stop_requested.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
                health_reason = node.health_reason()
                if health_reason is not None:
                    node.get_logger().error(
                        f'leaving free-drive because {health_reason}'
                    )
                    exit_code = 2
                    break
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f'free-drive stopped with an error: {exc}')
        else:
            print(f'free-drive initialization failed: {exc}')
        exit_code = 1
    finally:
        if node is not None:
            if node.restore_on_exit:
                node.restore_original_kp()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
