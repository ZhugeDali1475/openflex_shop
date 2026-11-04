#!/usr/bin/env python3
"""Validate MoveIt IK and state validity against the running MuJoCo bridge."""

from copy import deepcopy
import sys
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class MoveItSmokeTest(Node):
    def __init__(self) -> None:
        super().__init__("moveit_ik_smoke_test")
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_joint_state(self, message: JointState) -> None:
        self.latest_joint_state = message

    def wait_until_ready(self, timeout_sec: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.latest_joint_state is not None
                and self.ik_client.service_is_ready()
                and self.validity_client.service_is_ready()
                and self.tf_buffer.can_transform(
                    "base_link", "openarmx_right_hand_tcp", rclpy.time.Time()
                )
            ):
                return
        raise RuntimeError("MoveIt, joint state, or TCP TF did not become ready")

    def current_state(self) -> RobotState:
        state = RobotState()
        state.joint_state = deepcopy(self.latest_joint_state)
        state.is_diff = False
        return state

    def test_state_validity(self) -> None:
        request = GetStateValidity.Request()
        request.robot_state = self.current_state()
        request.group_name = "right_arm_with_lift"
        future = self.validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            raise RuntimeError("state validity service returned no response")
        print(
            "state_validity",
            "valid=" + str(bool(response.valid)),
            "contacts=" + str(len(response.contacts)),
        )

    def current_tcp_pose(self) -> PoseStamped:
        transform = self.tf_buffer.lookup_transform(
            "base_link", "openarmx_right_hand_tcp", rclpy.time.Time()
        )
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = deepcopy(transform.transform.rotation)
        return pose

    def test_ik(self) -> None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = "right_arm_with_lift"
        request.ik_request.ik_link_name = "openarmx_right_hand_tcp"
        request.ik_request.pose_stamped = self.current_tcp_pose()
        request.ik_request.robot_state = self.current_state()
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 2
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            raise RuntimeError("IK service returned no response")
        print(
            "compute_ik",
            "error=" + str(response.error_code.val),
            f"success={response.error_code.val == MoveItErrorCodes.SUCCESS}",
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"IK failed with MoveIt error code {response.error_code.val}"
            )


def main() -> int:
    rclpy.init()
    node = MoveItSmokeTest()
    try:
        node.wait_until_ready()
        node.test_state_validity()
        node.test_ik()
        print("moveit_ik_smoke_test_ok")
        return 0
    except (RuntimeError, TransformException) as error:
        print(f"moveit_ik_smoke_test_failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
