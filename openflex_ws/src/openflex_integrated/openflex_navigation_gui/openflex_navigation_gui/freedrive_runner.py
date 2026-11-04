#!/usr/bin/env python3
"""Console free-drive entry point that restores both arm KP and KD."""

import signal
import threading

import rclpy
from .freedrive_node import FreeDriveNode
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger


class ConsoleFreeDriveNode(FreeDriveNode):
    """Extend the hardware gain snapshot to include damping parameters."""

    def __init__(self):
        self._home_requested = False
        self._home_cancel_requested = False
        self._home_state = "IDLE"
        self._home_detail = ""
        self._home_targets = {}
        self._home_saved_gains = {}
        self._home_stable_samples = 0
        super().__init__()
        self.declare_parameter("home_target_step_rad", 0.005)
        self.declare_parameter("home_position_tolerance_rad", 0.03)
        self.declare_parameter("home_stable_samples", 10)
        self._home_target_step = float(
            self.get_parameter("home_target_step_rad").value
        )
        self._home_tolerance = float(
            self.get_parameter("home_position_tolerance_rad").value
        )
        self._home_required_stable_samples = int(
            self.get_parameter("home_stable_samples").value
        )
        if not 0.0001 <= self._home_target_step <= 0.05:
            raise ValueError("home_target_step_rad must be within [0.0001, 0.05]")
        if not 0.001 <= self._home_tolerance <= 0.10:
            raise ValueError(
                "home_position_tolerance_rad must be within [0.001, 0.10]"
            )
        if not 1 <= self._home_required_stable_samples <= 100:
            raise ValueError("home_stable_samples must be within [1, 100]")
        self._home_status_pub = self.create_publisher(
            String, "/openarmx_freedrive/home_status", 10
        )
        self._home_service = self.create_service(
            Trigger, "/openarmx_freedrive/home_arms", self._request_home
        )
        self._home_cancel_service = self.create_service(
            Trigger, "/openarmx_freedrive/cancel_home", self._request_home_cancel
        )
        self._publish_home_status("IDLE", "拖动模式双臂回零待命")

    def _publish_home_status(self, state, detail):
        self._home_state = str(state)
        self._home_detail = str(detail)
        self._home_status_pub.publish(
            String(data=f"{self._home_state}:{self._home_detail}")
        )

    def _request_home(self, _request, response):
        if not self.active:
            response.success = False
            response.message = "free-drive is not active"
        elif tuple(self.sides) != ("left", "right"):
            response.success = False
            response.message = "dual-arm Home requires arm_selection=both"
        elif self._home_state in ("STARTING", "RUNNING", "RESTORING"):
            response.success = False
            response.message = "dual-arm Home is already running"
        else:
            self._home_requested = True
            self._home_cancel_requested = False
            self._publish_home_status("STARTING", "正在保存拖动 Kp/Kd")
            response.success = True
            response.message = "dual-arm MIT Home accepted"
        return response

    def _request_home_cancel(self, _request, response):
        if self._home_state not in ("STARTING", "RUNNING", "RESTORING"):
            response.success = False
            response.message = "dual-arm Home is not running"
        else:
            self._home_cancel_requested = True
            response.success = True
            response.message = "dual-arm Home cancellation accepted"
        return response

    @staticmethod
    def _step_towards(current, target, maximum_step):
        delta = float(target) - float(current)
        if abs(delta) <= maximum_step:
            return float(target)
        return float(current) + (maximum_step if delta > 0.0 else -maximum_step)

    def _publish_current_positions(self):
        if not self._mirror_enabled:
            return
        for side, publisher in self._position_publishers.items():
            joints = self._home_targets.get(side)
            if joints is None:
                names = self._position_joint_names(side)
                if not all(name in self._joint_positions for name in names):
                    continue
                values = [self._joint_positions[name] for name in names]
            else:
                values = list(joints)
                for index in range(7):
                    values[index] = self._step_towards(
                        values[index], 0.0, self._home_target_step
                    )
                finger_name = self._position_joint_names(side)[7]
                if finger_name in self._joint_positions:
                    values[7] = self._joint_positions[finger_name]
                self._home_targets[side] = values
            publisher.publish(Float64MultiArray(data=values))

    @staticmethod
    def _position_joint_names(side):
        return [f"openarmx_{side}_joint{index}" for index in range(1, 8)] + [
            f"openarmx_{side}_finger_joint1"
        ]

    def _restore_drag_gains(self):
        failures = []
        for side, gains in self._home_saved_gains.items():
            try:
                self._set_kp(side, gains)
            except Exception as exc:
                failures.append(f"{side}: {exc}")
        self._home_targets.clear()
        self._home_saved_gains.clear()
        self._home_requested = False
        self._home_cancel_requested = False
        self._home_stable_samples = 0
        if failures:
            self._publish_home_status(
                "FAILED", "恢复拖动 Kp/Kd 失败：" + "; ".join(failures)
            )
            return False
        return True

    def step_home(self):
        if self._home_state == "STARTING" and self._home_requested:
            try:
                self._home_saved_gains = {
                    side: self._get_kp(side) for side in self.sides
                }
                self._home_targets = {
                    side: [
                        float(self._joint_positions[name])
                        for name in self._position_joint_names(side)
                    ]
                    for side in self.sides
                }
                for side in self.sides:
                    self._set_kp(side, self._original_kp[side])
            except Exception as exc:
                self._restore_drag_gains()
                self._publish_home_status("FAILED", f"启用回零 Kp/Kd 失败：{exc}")
                return
            self._publish_home_status(
                "RUNNING", "保持 MIT，双臂正在渐进回零"
            )
            return
        if self._home_state != "RUNNING":
            return
        if self._home_cancel_requested:
            self._publish_home_status("RESTORING", "正在取消并恢复拖动 Kp/Kd")
            if self._restore_drag_gains():
                self._publish_home_status("CANCELED", "已恢复拖动 Kp/Kd")
            return
        at_zero = all(
            abs(float(self._joint_positions.get(name, float("inf"))))
            <= self._home_tolerance
            for side in self.sides
            for name in self._position_joint_names(side)[:7]
        )
        self._home_stable_samples = (
            self._home_stable_samples + 1 if at_zero else 0
        )
        if self._home_stable_samples < self._home_required_stable_samples:
            return
        self._publish_home_status("RESTORING", "双臂已到零位，正在恢复拖动 Kp/Kd")
        if self._restore_drag_gains():
            self._publish_home_status(
                "SUCCEEDED", "双臂已回零，MIT 拖动模式保持启用"
            )

    def _get_kp(self, side):
        names = [
            f"{gain}_joint{joint}"
            for joint in range(1, 8)
            for gain in ("kp", "kd")
        ]
        request = GetParameters.Request()
        request.names = names
        response = self._wait_for_future(self._get_clients[side].call_async(request))
        if len(response.values) != len(names):
            raise RuntimeError(
                f"{side} parameter service returned an incomplete gain response"
            )
        values = {}
        for name, value in zip(names, response.values):
            if value.type != ParameterType.PARAMETER_DOUBLE:
                raise RuntimeError(f"{side}/{name} is not a double")
            values[name] = float(value.double_value)
        return values


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
        node = ConsoleFreeDriveNode()
        if not node.activate():
            exit_code = 1
        else:
            while rclpy.ok() and not stop_requested.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
                node.step_home()
                health_reason = node.health_reason()
                if health_reason is not None:
                    node.get_logger().error(
                        f"leaving free-drive because {health_reason}"
                    )
                    exit_code = 2
                    break
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"free-drive stopped with an error: {exc}")
        else:
            print(f"free-drive initialization failed: {exc}")
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


if __name__ == "__main__":
    main()
