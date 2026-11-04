#!/usr/bin/env python3

"""Atomically hand lift ownership between manual control and MoveIt."""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node


class LiftControllerSwitch(Node):
    def __init__(self) -> None:
        super().__init__("switch_lift_controller")
        self.list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

    def wait_until_loaded(self, timeout_sec: float) -> dict[str, str] | None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            remaining = deadline - time.monotonic()
            if not self.list_client.wait_for_service(timeout_sec=min(1.0, remaining)):
                continue
            future = self.list_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=min(2.0, max(0.0, remaining))
            )
            if not future.done() or future.result() is None:
                continue
            states = {
                controller.name: controller.state
                for controller in future.result().controller
            }
            required = {
                "lift_position_controller",
                "lift_manual_position_controller",
            }
            if required <= states.keys():
                return states
            # Let the controller spawners settle without blocking ROS event
            # processing for an entire polling interval.
            time.sleep(0.2)
        return None

    def switch(self, mode: str, timeout_sec: float) -> bool:
        states = self.wait_until_loaded(timeout_sec)
        if states is None:
            self.get_logger().error("Timed out waiting for both lift controllers")
            return False
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("switch_controller service is unavailable")
            return False
        manual = mode == "manual"
        target = (
            "lift_manual_position_controller" if manual else "lift_position_controller"
        )
        other = (
            "lift_position_controller" if manual else "lift_manual_position_controller"
        )
        if states.get(target) == "active" and states.get(other) != "active":
            self.get_logger().info(
                f"Lift controller ownership is already {mode}"
            )
            return True
        request = SwitchController.Request()
        request.activate_controllers = [target]
        request.deactivate_controllers = [other] if states.get(other) == "active" else []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=7.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("Lift controller switch timed out")
            return False
        if not future.result().ok:
            self.get_logger().error("controller_manager rejected the lift handoff")
            return False
        self.get_logger().info(f"Lift controller ownership switched to {mode}")
        return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("manual", "moveit"), required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    # launch_ros appends its own --ros-args section to Python executables.
    # Strip that section before passing application arguments to argparse.
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = LiftControllerSwitch()
    try:
        return 0 if node.switch(args.mode, args.timeout) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
