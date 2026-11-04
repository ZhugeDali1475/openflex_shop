#!/usr/bin/env python3
"""Plan and execute a small right-arm motion through MoveIt and MuJoCo."""

from copy import deepcopy
import sys
import time

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import JointState


GROUP = "right_arm_with_lift"
JOINT_NAMES = (
    "lift_joint",
    "openarmx_right_joint1",
    "openarmx_right_joint2",
    "openarmx_right_joint3",
    "openarmx_right_joint4",
    "openarmx_right_joint5",
    "openarmx_right_joint6",
    "openarmx_right_joint7",
)
ACTIVE_JOINT = "openarmx_right_joint1"


class MoveItMoveSmokeTest(Node):
    def __init__(self) -> None:
        super().__init__("moveit_move_smoke_test")
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_joint_trajectory_controller/follow_joint_trajectory",
        )

    def on_joint_state(self, message: JointState) -> None:
        self.latest_joint_state = message

    def wait_until_ready(self, timeout_sec: float = 20.0) -> dict:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.latest_joint_state is not None
                and self.move_client.server_is_ready()
                and self.trajectory_client.server_is_ready()
            ):
                positions = dict(
                    zip(
                        self.latest_joint_state.name,
                        self.latest_joint_state.position,
                    )
                )
                if all(name in positions for name in JOINT_NAMES):
                    return positions
        raise RuntimeError("MoveGroup or MuJoCo trajectory action did not become ready")

    def plan_and_execute(self, current: dict) -> float:
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = "care_smoke_small_right_arm_motion"
        target_active = float(current[ACTIVE_JOINT]) + 0.08
        for name in JOINT_NAMES:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target_active if name == ACTIVE_JOINT else float(current[name])
            constraint.tolerance_above = 0.015
            constraint.tolerance_below = 0.015
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveGroup goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        wrapper = result_future.result()
        if wrapper is None:
            raise RuntimeError("MoveGroup execution timed out")
        error_code = int(wrapper.result.error_code.val)
        print(
            "move_group",
            f"error={error_code}",
            f"success={error_code == MoveItErrorCodes.SUCCESS}",
            f"planned_points={len(wrapper.result.planned_trajectory.joint_trajectory.points)}",
        )
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveGroup failed with code {error_code}")
        return target_active

    def wait_for_joint(self, target: float, timeout_sec: float = 8.0) -> float:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is None:
                continue
            positions = dict(
                zip(
                    self.latest_joint_state.name,
                    self.latest_joint_state.position,
                )
            )
            actual = float(positions[ACTIVE_JOINT])
            if abs(actual - target) <= 0.03:
                return actual
        raise RuntimeError("MuJoCo joint did not track the planned target")


def main() -> int:
    rclpy.init()
    node = MoveItMoveSmokeTest()
    try:
        current = node.wait_until_ready()
        print("start_joint", float(current[ACTIVE_JOINT]))
        target = node.plan_and_execute(current)
        actual = node.wait_for_joint(target)
        print("final_joint", actual)
        print("moveit_move_smoke_test_ok")
        return 0
    except RuntimeError as error:
        print(f"moveit_move_smoke_test_failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
