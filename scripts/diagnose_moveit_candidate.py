#!/usr/bin/env python3
"""Inspect validity contacts for the first evaluated TC-WAM candidate."""

from copy import deepcopy
import sys
import time

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from openflex_thesis_msgs.msg import AffordanceCandidateArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CandidateDiagnostic(Node):
    def __init__(self) -> None:
        super().__init__("moveit_candidate_diagnostic")
        self.latest = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            AffordanceCandidateArray,
            "/thesis/tc_wam/candidates",
            self.on_candidates,
            qos,
        )
        self.client = self.create_client(GetStateValidity, "/check_state_validity")

    def on_candidates(self, message: AffordanceCandidateArray) -> None:
        self.latest = deepcopy(message)

    def run(self) -> int:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None and self.client.service_is_ready():
                break
        if self.latest is None or not self.client.service_is_ready():
            print("candidate_diagnostic_no_data", file=sys.stderr)
            return 1
        candidate = next(
            (
                item
                for item in self.latest.candidates
                if item.grasp_joint_positions
            ),
            None,
        )
        if candidate is None:
            print("candidate_diagnostic_no_ik_solution", file=sys.stderr)
            return 1
        for label, positions in (
            ("pregrasp", candidate.pregrasp_joint_positions),
            ("grasp", candidate.grasp_joint_positions),
        ):
            state = RobotState()
            state.joint_state.name = list(candidate.joint_names)
            state.joint_state.position = list(positions)
            state.is_diff = False
            request = GetStateValidity.Request()
            request.robot_state = state
            request.group_name = candidate.planning_group
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response is None:
                print(label, "no_response")
                continue
            print(
                label,
                "valid=" + str(bool(response.valid)),
                "contacts=" + str(len(response.contacts)),
            )
            for contact in response.contacts:
                print(
                    " contact",
                    contact.contact_body_1,
                    contact.contact_body_2,
                    contact.position.x,
                    contact.position.y,
                    contact.position.z,
                )
        return 0


def main() -> int:
    rclpy.init()
    node = CandidateDiagnostic()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
