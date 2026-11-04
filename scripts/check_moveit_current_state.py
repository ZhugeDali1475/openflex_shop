#!/usr/bin/env python3
"""Validate the complete current robot state against a MoveIt planning group."""

from copy import deepcopy
import argparse
import sys
import time

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CurrentStateChecker(Node):
    def __init__(self) -> None:
        super().__init__("moveit_current_state_checker")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)
        self.client = self.create_client(GetStateValidity, "/check_state_validity")

    def on_joint_state(self, message: JointState) -> None:
        self.joint_state = deepcopy(message)

    def run(self, group_name: str, timeout_sec: float) -> int:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is not None and self.client.service_is_ready():
                break
        if self.joint_state is None:
            print("current_state_check_failed: no /joint_states", file=sys.stderr)
            return 1
        if not self.client.service_is_ready():
            print("current_state_check_failed: /check_state_validity unavailable", file=sys.stderr)
            return 1

        positions = dict(
            zip(self.joint_state.name, self.joint_state.position)
        )
        left_names = [
            f"openarmx_left_joint{index}" for index in range(1, 8)
        ]
        print(
            "left_arm_positions="
            + str([round(float(positions[name]), 6) for name in left_names if name in positions])
        )

        state = RobotState()
        state.joint_state = deepcopy(self.joint_state)
        state.is_diff = False
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = group_name
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            print("current_state_check_failed: no service response", file=sys.stderr)
            return 1
        print(f"group={group_name} valid={bool(response.valid)} contacts={len(response.contacts)}")
        for contact in response.contacts:
            print(f"contact {contact.contact_body_1} <-> {contact.contact_body_2}")
        return 0 if response.valid else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--group", default="right_arm_with_lift")
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init()
    node = CurrentStateChecker()
    try:
        return node.run(args.group, args.timeout_sec)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
