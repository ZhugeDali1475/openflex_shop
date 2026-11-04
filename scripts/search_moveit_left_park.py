#!/usr/bin/env python3
"""Search a collision-free parked posture for the inactive left arm."""

from copy import deepcopy
from itertools import product
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LEFT_JOINT_NAMES = [f"openarmx_left_joint{index}" for index in range(1, 8)]


class LeftParkSearch(Node):
    def __init__(self) -> None:
        super().__init__("moveit_left_park_search")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)
        self.client = self.create_client(GetStateValidity, "/check_state_validity")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fixed_positions = None

    def on_joint_state(self, message: JointState) -> None:
        self.joint_state = deepcopy(message)

    def wait_until_ready(self, timeout_sec: float = 20.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.joint_state is not None
                and self.client.service_is_ready()
                and self.ik_client.service_is_ready()
            ):
                return
        raise RuntimeError("joint states or /check_state_validity unavailable")

    def solve_reference_right_arm(self, lift_position: float) -> None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = "right_arm_with_lift"
        request.ik_request.ik_link_name = "openarmx_right_hand_tcp"
        request.ik_request.robot_state.joint_state = deepcopy(self.joint_state)
        request.ik_request.robot_state.is_diff = True
        lift_constraint = JointConstraint()
        lift_constraint.joint_name = "lift_joint"
        lift_constraint.position = lift_position
        lift_constraint.tolerance_above = 0.01
        lift_constraint.tolerance_below = 0.01
        lift_constraint.weight = 1.0
        request.ik_request.constraints.joint_constraints.append(lift_constraint)
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 1
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.55
        pose.pose.position.y = -0.15
        pose.pose.position.z = 0.80
        pose.pose.orientation.y = math.sqrt(0.5)
        pose.pose.orientation.w = math.sqrt(0.5)
        request.ik_request.pose_stamped = pose
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("reference right-arm IK failed")
        self.fixed_positions = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        print("RIGHT_REFERENCE_IK_OK", flush=True)

    def state_for(self, left_positions, lift_position) -> RobotState:
        positions = dict(self.fixed_positions)
        positions.update(dict(zip(LEFT_JOINT_NAMES, left_positions)))
        positions["lift_joint"] = float(lift_position)
        state = RobotState()
        state.joint_state.name = sorted(positions)
        state.joint_state.position = [float(positions[name]) for name in state.joint_state.name]
        state.is_diff = False
        return state

    def query(self, left_positions, lift_position):
        request = GetStateValidity.Request()
        request.robot_state = self.state_for(left_positions, lift_position)
        request.group_name = "right_arm_with_lift"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            return None
        return bool(response.valid), [
            (contact.contact_body_1, contact.contact_body_2)
            for contact in response.contacts
        ]


def main() -> int:
    rclpy.init()
    node = LeftParkSearch()
    try:
        node.wait_until_ready()
        node.solve_reference_right_arm(-0.15)
        q1_values = (-1.5, -1.0, -0.7, -0.4, -0.2, 0.0, 0.2, 0.5)
        q2_values = (-1.2, -0.8, -0.5, -0.3, -0.15, 0.0)
        q3_values = (-0.8, -0.4, 0.0, 0.4, 0.8)
        q4_values = (1.8, 1.2, 0.6)
        lift_values = (-0.15,)
        checked = 0
        valid_all_lifts = []
        first_failure_reported = False
        for q1, q2, q3, q4 in product(q1_values, q2_values, q3_values, q4_values):
            positions = (q1, q2, q3, q4, 0.0, 0.0, 0.0)
            collision_free_all = True
            for lift_position in lift_values:
                result = node.query(positions, lift_position)
                checked += 1
                if result is None or not result[0]:
                    if not first_failure_reported:
                        print(
                            f"PARK_REJECTED {list(positions)} "
                            f"lift={lift_position} contacts="
                            f"{[] if result is None else result[1]}",
                            flush=True,
                        )
                        first_failure_reported = True
                    collision_free_all = False
                    break
            if collision_free_all:
                valid_all_lifts.append(positions)
                print(
                    f"PARK_OK_ALL_LIFTS {list(positions)} "
                    f"lifts={list(lift_values)}",
                    flush=True,
                )
        print(
            f"park_search_checked={checked} valid_all_lifts={len(valid_all_lifts)}",
            flush=True,
        )
        return 0 if valid_all_lifts else 1
    except RuntimeError as error:
        print(f"park_search_failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
