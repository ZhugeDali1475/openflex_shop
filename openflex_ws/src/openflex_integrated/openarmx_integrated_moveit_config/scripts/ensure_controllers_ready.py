#!/usr/bin/env python3

"""Prepare the required controllers before starting the MoveIt frontend."""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.node import Node


class ControllerReadiness(Node):
    ACTIVATION_ORDER = (
        "joint_state_broadcaster",
        "swerve_drive_controller",
        "lift_state_controller",
        "left_forward_position_controller",
        "right_forward_position_controller",
        "head_forward_position_controller",
        "left_forward_effort_controller",
        "right_forward_effort_controller",
    )

    def __init__(self) -> None:
        super().__init__("ensure_controllers_ready")
        self.list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.load_client = self.create_client(
            LoadController, "/controller_manager/load_controller"
        )
        self.configure_client = self.create_client(
            ConfigureController, "/controller_manager/configure_controller"
        )
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

    def _call(self, client, request, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=min(timeout_sec, 1.0)):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def controller_states(self, timeout_sec: float = 2.0) -> dict[str, str] | None:
        response = self._call(
            self.list_client, ListControllers.Request(), timeout_sec
        )
        if response is None:
            return None
        return {
            controller.name: controller.state
            for controller in response.controller
        }

    def _load(self, name: str) -> bool:
        request = LoadController.Request()
        request.name = name
        response = self._call(self.load_client, request, 5.0)
        return bool(response and response.ok)

    def _configure(self, name: str) -> bool:
        request = ConfigureController.Request()
        request.name = name
        response = self._call(self.configure_client, request, 5.0)
        return bool(response and response.ok)

    def _activate(self, names: list[str]) -> bool:
        if not names:
            return True
        request = SwitchController.Request()
        request.activate_controllers = names
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10
        response = self._call(self.switch_client, request, 12.0)
        return bool(response and response.ok)

    def ensure(
        self,
        active: set[str],
        configured: set[str],
        timeout_sec: float,
        repair_delay_sec: float,
    ) -> bool:
        required = active | configured
        started = time.monotonic()
        deadline = started + timeout_sec
        last_summary = None
        while rclpy.ok() and time.monotonic() < deadline:
            states = self.controller_states()
            if states is None:
                time.sleep(0.2)
                continue
            summary = tuple(sorted((name, states.get(name, "missing")) for name in required))
            if summary != last_summary:
                self.get_logger().info(
                    "Controller readiness: "
                    + ", ".join(f"{name}={state}" for name, state in summary)
                )
                last_summary = summary
            if all(states.get(name) == "active" for name in active) and all(
                states.get(name) in ("inactive", "active") for name in configured
            ):
                self.get_logger().info("Required controllers are ready")
                return True
            if time.monotonic() - started < repair_delay_sec:
                time.sleep(0.2)
                continue

            for name in sorted(required):
                state = states.get(name)
                if state is None:
                    self.get_logger().warn(f"Loading missing controller {name}")
                    self._load(name)
                elif state == "unconfigured":
                    self.get_logger().warn(f"Configuring controller {name}")
                    self._configure(name)

            states = self.controller_states() or {}
            activation_rank = {
                name: index for index, name in enumerate(self.ACTIVATION_ORDER)
            }
            inactive = sorted(
                (name for name in active if states.get(name) == "inactive"),
                key=lambda name: (activation_rank.get(name, len(activation_rank)), name),
            )
            if inactive:
                # controller_manager can report a successful bulk switch while
                # a broadcaster remains active but never enters its update
                # loop on real hardware. Activate one controller per service
                # call, matching the field-proven spawner behavior.
                for name in inactive:
                    self.get_logger().warn(f"Activating controller {name}")
                    self._activate([name])
                    time.sleep(0.05)
            time.sleep(0.2)

        self.get_logger().error("Timed out preparing required controllers")
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--active", action="append", default=[])
    parser.add_argument("--configured", action="append", default=[])
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--repair-delay", type=float, default=15.0)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = ControllerReadiness()
    try:
        ready = node.ensure(
            set(args.active),
            set(args.configured),
            args.timeout,
            args.repair_delay,
        )
        return 0 if ready else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
