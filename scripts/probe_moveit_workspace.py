#!/usr/bin/env python3
"""Probe the right lift-arm IK workspace without collision filtering."""

from copy import deepcopy
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


GROUP = "right_arm_with_lift"
IK_LINK = "openarmx_right_hand_tcp"


class WorkspaceProbe(Node):
    def __init__(self) -> None:
        super().__init__("moveit_workspace_probe")
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)
        self.client = self.create_client(GetPositionIK, "/compute_ik")

    def on_joint_state(self, message: JointState) -> None:
        self.latest_joint_state = message

    def wait_until_ready(self, timeout_sec: float = 20.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is not None and self.client.service_is_ready():
                return
        raise RuntimeError("IK service or joint state did not become ready")

    def seed(self, lift_position: float) -> RobotState:
        state = RobotState()
        state.joint_state = deepcopy(self.latest_joint_state)
        state.is_diff = False
        positions = list(state.joint_state.position)
        names = list(state.joint_state.name)
        if "lift_joint" in names:
            positions[names.index("lift_joint")] = lift_position
        else:
            names.append("lift_joint")
            positions.append(lift_position)
        state.joint_state.name = names
        state.joint_state.position = positions
        return state

    def query(self, x: float, y: float, z: float, lift_position: float) -> bool:
        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP
        request.ik_request.ik_link_name = IK_LINK
        request.ik_request.robot_state = self.seed(lift_position)
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 80_000_000
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = math.sqrt(0.5)
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = math.sqrt(0.5)
        request.ik_request.pose_stamped = pose
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        return bool(
            response is not None
            and response.error_code.val == MoveItErrorCodes.SUCCESS
        )


def main() -> int:
    rclpy.init()
    node = WorkspaceProbe()
    try:
        node.wait_until_ready()
        xs = (0.45, 0.55, 0.65, 0.75, 0.85)
        ys = (-0.30, -0.15, 0.0, 0.15, 0.30)
        zs = (0.70, 0.80, 0.90, 1.00)
        lifts = (-0.45, -0.15, 0.15)
        successes = []
        for lift in lifts:
            for x in xs:
                for y in ys:
                    for z in zs:
                        if node.query(x, y, z, lift):
                            successes.append((lift, x, y, z))
                            print("IK_OK", lift, x, y, z, flush=True)
        print("workspace_probe_total", len(successes), flush=True)
        return 0 if successes else 1
    except RuntimeError as error:
        print(f"workspace_probe_failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
