#!/usr/bin/env python3

"""MoveIt execution bridge for the existing OpenArmX position topics.

This node deliberately does not create a ros2_control controller. It exposes
the action endpoints expected by MoveIt's controller manager and forwards
sampled goals to the already deployed forward-position command topics.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Iterable

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from lift_slide_msgs.msg import MotorStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import SetBool


LEFT_ARM = [f"openarmx_left_joint{i}" for i in range(1, 8)]
RIGHT_ARM = [f"openarmx_right_joint{i}" for i in range(1, 8)]
LIFT = ["lift_joint"]
LEFT_GRIPPER = ["openarmx_left_finger_joint1"]
RIGHT_GRIPPER = ["openarmx_right_finger_joint1"]
TRAJECTORY_DOMAINS = ("left", "right", "lift")
# Each trajectory execute callback blocks while interpolating. Keep two
# control-plane workers free for joint feedback, service responses, and cancel.
BRIDGE_EXECUTOR_THREADS = len(TRAJECTORY_DOMAINS) + 2
GRIPPER_MIN_POSITION_M = 0.0
GRIPPER_MAX_POSITION_M = 0.044
GRIPPER_STATE_VALID_TOLERANCE_M = 0.005
CURRENT_POSITION_LIMIT_TOLERANCE = 0.02
# MoveIt can preserve the measured start state a few 1e-4 rad beyond a URDF
# bound. Accept only that numerical spillover, then clamp every command back to
# the physical limit before publishing it to the hardware controller.
TRAJECTORY_POSITION_LIMIT_TOLERANCE = 0.001
JOINT_POSITION_LIMITS = {
    # Final execution guard uses the physical URDF/driver limits. Most MoveIt
    # joints keep a separate planning inset; right joint7 uses the physical
    # range so a recorded basket photo configuration can be restored exactly.
    "lift_joint": (-0.750, 0.400),
    "openarmx_left_joint1": (-3.344396, 0.905604),
    "openarmx_left_joint2": (-3.270796, 0.129204),
    "openarmx_right_joint1": (-1.25, 3.0),
    "openarmx_right_joint2": (-0.129204, 3.270796),
}
for side in ("left", "right"):
    JOINT_POSITION_LIMITS.update(
        {
            f"openarmx_{side}_joint3": (-1.57, 1.57),
            f"openarmx_{side}_joint4": (0.0, 1.8),
            f"openarmx_{side}_joint5": (-1.5, 1.5),
            f"openarmx_{side}_joint6": (-0.75, 0.75),
            f"openarmx_{side}_joint7": (-1.4, 1.4),
        }
    )


def gripper_position_is_valid(position: float) -> bool:
    value = float(position)
    return (
        math.isfinite(value)
        and GRIPPER_MIN_POSITION_M - GRIPPER_STATE_VALID_TOLERANCE_M
        <= value
        <= GRIPPER_MAX_POSITION_M + GRIPPER_STATE_VALID_TOLERANCE_M
    )


def trajectory_position_is_valid(joint: str, position: float) -> bool:
    value = float(position)
    if not math.isfinite(value):
        return False
    limits = JOINT_POSITION_LIMITS.get(joint)
    if limits is None:
        return True
    lower, upper = limits
    return (
        lower - TRAJECTORY_POSITION_LIMIT_TOLERANCE
        <= value
        <= upper + TRAJECTORY_POSITION_LIMIT_TOLERANCE
    )


def clamp_trajectory_position(joint: str, position: float) -> float:
    value = float(position)
    limits = JOINT_POSITION_LIMITS.get(joint)
    if limits is None:
        return value
    lower, upper = limits
    return min(upper, max(lower, value))


class MoveItCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("moveit_command_bridge")
        self._lock = threading.RLock()
        self._positions: dict[str, float] = {}
        self._joint_revisions: dict[str, int] = {}
        self._gripper_commands: dict[str, float] = {}
        self._position_revision = 0
        self._active: set[str] = set()
        self._enabled = True
        self._group = ReentrantCallbackGroup()
        self._lift_status: MotorStatus | None = None

        # Match the goal tolerance used by the OpenArmX trajectory-controller
        # configuration. The bridge must not add a tighter per-joint gate than
        # the controller itself when forwarding standard MoveIt trajectories.
        self.declare_parameter("trajectory_goal_tolerance_rad", 0.030)
        self.declare_parameter("trajectory_distal_goal_tolerance_rad", 0.050)
        self.declare_parameter("lift_goal_tolerance_m", 0.005)
        self.declare_parameter("trajectory_settle_timeout_s", 8.0)
        self.declare_parameter("trajectory_stable_samples", 3)
        self.declare_parameter("trajectory_poll_period_s", 0.02)
        self.declare_parameter("gripper_goal_tolerance_m", 0.001)
        self.declare_parameter("gripper_settle_timeout_s", 3.0)
        self.declare_parameter("gripper_stall_timeout_s", 0.5)
        self.declare_parameter("gripper_stall_position_delta_m", 0.0001)
        self.declare_parameter("require_exclusive_command_publisher", True)
        self.declare_parameter("require_lift_homed", True)
        self.declare_parameter("lift_profile_speed_margin", 1.25)
        self.declare_parameter("lift_profile_speed_min_m_s", 0.01)
        self.declare_parameter("lift_profile_speed_max_m_s", 0.10)
        self.declare_parameter(
            "allowed_idle_command_publishers",
            ["/rviz2", "/openarmx_joint_slider_panel"],
        )
        self._trajectory_goal_tolerance = float(
            self.get_parameter("trajectory_goal_tolerance_rad").value
        )
        self._trajectory_distal_goal_tolerance = float(
            self.get_parameter("trajectory_distal_goal_tolerance_rad").value
        )
        self._lift_goal_tolerance = float(
            self.get_parameter("lift_goal_tolerance_m").value
        )
        self._trajectory_settle_timeout = float(
            self.get_parameter("trajectory_settle_timeout_s").value
        )
        self._trajectory_stable_samples = int(
            self.get_parameter("trajectory_stable_samples").value
        )
        self._trajectory_poll_period = float(
            self.get_parameter("trajectory_poll_period_s").value
        )
        self._gripper_goal_tolerance = float(
            self.get_parameter("gripper_goal_tolerance_m").value
        )
        self._gripper_settle_timeout = float(
            self.get_parameter("gripper_settle_timeout_s").value
        )
        self._gripper_stall_timeout = float(
            self.get_parameter("gripper_stall_timeout_s").value
        )
        self._gripper_stall_position_delta = float(
            self.get_parameter("gripper_stall_position_delta_m").value
        )
        self._require_exclusive_command_publisher = bool(
            self.get_parameter("require_exclusive_command_publisher").value
        )
        self._require_lift_homed = bool(
            self.get_parameter("require_lift_homed").value
        )
        self._lift_profile_speed_margin = float(
            self.get_parameter("lift_profile_speed_margin").value
        )
        self._lift_profile_speed_min = float(
            self.get_parameter("lift_profile_speed_min_m_s").value
        )
        self._lift_profile_speed_max = float(
            self.get_parameter("lift_profile_speed_max_m_s").value
        )
        self._allowed_idle_command_publishers = {
            "/" + str(name).strip("/")
            for name in self.get_parameter("allowed_idle_command_publishers").value
            if str(name).strip("/")
        }
        if (
            not math.isfinite(self._trajectory_goal_tolerance)
            or self._trajectory_goal_tolerance <= 0.0
            or not math.isfinite(self._trajectory_distal_goal_tolerance)
            or self._trajectory_distal_goal_tolerance <= 0.0
            or not math.isfinite(self._lift_goal_tolerance)
            or self._lift_goal_tolerance <= 0.0
            or not math.isfinite(self._trajectory_settle_timeout)
            or self._trajectory_settle_timeout <= 0.0
            or self._trajectory_stable_samples < 1
            or not math.isfinite(self._trajectory_poll_period)
            or self._trajectory_poll_period <= 0.0
            or not math.isfinite(self._gripper_goal_tolerance)
            or self._gripper_goal_tolerance <= 0.0
            or not math.isfinite(self._gripper_settle_timeout)
            or self._gripper_settle_timeout <= 0.0
            or not math.isfinite(self._gripper_stall_timeout)
            or self._gripper_stall_timeout <= 0.0
            or not math.isfinite(self._gripper_stall_position_delta)
            or self._gripper_stall_position_delta <= 0.0
            or not math.isfinite(self._lift_profile_speed_margin)
            or self._lift_profile_speed_margin < 1.0
            or not math.isfinite(self._lift_profile_speed_min)
            or not math.isfinite(self._lift_profile_speed_max)
            or self._lift_profile_speed_min <= 0.0
            or self._lift_profile_speed_min > self._lift_profile_speed_max
        ):
            raise ValueError(
                "invalid trajectory or gripper feedback convergence parameters"
            )

        self._enabled_service = self.create_service(
            SetBool,
            "/moveit_command_bridge/set_enabled",
            self._set_enabled_callback,
            callback_group=self._group,
        )

        self._left_pub = self.create_publisher(
            Float64MultiArray, "/left_forward_position_controller/commands", 10
        )
        self._right_pub = self.create_publisher(
            Float64MultiArray, "/right_forward_position_controller/commands", 10
        )
        self._lift_pub = self.create_publisher(
            Float64MultiArray, "/lift_position_controller/commands", 10
        )
        self._lift_profile_speed_pub = self.create_publisher(
            Float64, "/lift_state_controller/profile_speed_cmd", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 30,
            callback_group=self._group,
        )
        self.create_subscription(
            MotorStatus,
            "/lift_slide_driver/motor_status",
            self._lift_status_callback,
            10,
            callback_group=self._group,
        )
        self._list_controllers_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
            callback_group=self._group,
        )

        self._servers = [
            self._trajectory_server("lift_joint_controller", LIFT, "lift"),
            self._trajectory_server("left_joint_trajectory_controller", LEFT_ARM, "left"),
            self._trajectory_server("right_joint_trajectory_controller", RIGHT_ARM, "right"),
            self._gripper_server("left_gripper_controller", LEFT_GRIPPER, "left"),
            self._gripper_server("right_gripper_controller", RIGHT_GRIPPER, "right"),
        ]
        self.get_logger().info(
            "MoveIt bridge is forwarding to existing position command topics; "
            f"executor_threads={BRIDGE_EXECUTOR_THREADS}; "
            f"completion requires {self._trajectory_stable_samples} feedback samples within "
            f"{self._trajectory_goal_tolerance:.4f} rad for joints 1-4 and "
            f"{self._trajectory_distal_goal_tolerance:.4f} rad for joints 5-7; "
            f"lift tolerance={self._lift_goal_tolerance:.4f} m"
        )

    def _set_enabled_callback(self, request, response):
        with self._lock:
            if not request.data and self._active:
                response.success = False
                response.message = (
                    "cannot disable MoveIt while trajectories are active: "
                    + ", ".join(sorted(self._active))
                )
                return response
            self._enabled = bool(request.data)
        response.success = True
        response.message = (
            "MoveIt command bridge enabled"
            if self._enabled
            else "MoveIt command bridge disabled"
        )
        self.get_logger().info(response.message)
        return response

    def _is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._lock:
            updates = []
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    updates.append((name, float(msg.position[index])))
            if updates:
                self._position_revision += 1
                for name, position in updates:
                    self._positions[name] = position
                    self._joint_revisions[name] = self._position_revision
                if LEFT_GRIPPER[0] in self._positions:
                    self._gripper_commands.setdefault(
                        "left", self._positions[LEFT_GRIPPER[0]]
                    )
                if RIGHT_GRIPPER[0] in self._positions:
                    self._gripper_commands.setdefault(
                        "right", self._positions[RIGHT_GRIPPER[0]]
                    )

    def _lift_status_callback(self, msg: MotorStatus) -> None:
        with self._lock:
            self._lift_status = msg

    def _trajectory_server(self, controller: str, joints: list[str], domain: str) -> ActionServer:
        return ActionServer(
            self,
            FollowJointTrajectory,
            f"/{controller}/follow_joint_trajectory",
            execute_callback=lambda handle: self._execute_trajectory(handle, joints, domain),
            goal_callback=lambda goal: self._validate_trajectory_goal(goal, joints, domain),
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    def _gripper_server(self, controller: str, joints: list[str], domain: str) -> ActionServer:
        return ActionServer(
            self,
            GripperCommand,
            f"/{controller}/gripper_cmd",
            execute_callback=lambda handle: self._execute_gripper(handle, joints, domain),
            goal_callback=lambda goal: self._validate_gripper_goal(goal, domain),
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    def _validate_trajectory_goal(self, goal, joints: list[str], domain: str) -> GoalResponse:
        if not self._is_enabled():
            self.get_logger().warning(
                f"Rejecting {domain} MoveIt goal while command bridge is disabled"
            )
            return GoalResponse.REJECT
        names = list(goal.trajectory.joint_names)
        if set(names) != set(joints) or len(names) != len(joints):
            self.get_logger().error(
                f"Rejecting {domain} trajectory with joints {names}"
            )
            return GoalResponse.REJECT
        if not goal.trajectory.points:
            return GoalResponse.REJECT
        if any(len(point.positions) != len(names) for point in goal.trajectory.points):
            return GoalResponse.REJECT
        with self._lock:
            current_positions = dict(self._positions)
        for joint in names:
            if joint not in current_positions or joint not in JOINT_POSITION_LIMITS:
                continue
            current = current_positions[joint]
            lower, upper = JOINT_POSITION_LIMITS[joint]
            if (
                not math.isfinite(current)
                or current < lower - CURRENT_POSITION_LIMIT_TOLERANCE
                or current > upper + CURRENT_POSITION_LIMIT_TOLERANCE
            ):
                self.get_logger().error(
                    f"Rejecting {domain} trajectory: current {joint}={current:.5f} "
                    f"is outside the safe [{lower:.5f}, {upper:.5f}] range; "
                    "inspect the arm before restarting motion"
                )
                return GoalResponse.REJECT
        for point_index, point in enumerate(goal.trajectory.points):
            for joint, position in zip(names, point.positions):
                value = float(position)
                if not math.isfinite(value):
                    self.get_logger().error(
                        f"Rejecting {domain} trajectory: point {point_index} has "
                        f"non-finite {joint} position"
                    )
                    return GoalResponse.REJECT
                limits = JOINT_POSITION_LIMITS.get(joint)
                if limits is not None and not trajectory_position_is_valid(
                    joint, value
                ):
                    self.get_logger().error(
                        f"Rejecting {domain} trajectory: point {point_index} commands "
                        f"{joint}={value:.5f} outside [{limits[0]:.5f}, "
                        f"{limits[1]:.5f}] beyond numerical tolerance "
                        f"{TRAJECTORY_POSITION_LIMIT_TOLERANCE:.5f}"
                    )
                    return GoalResponse.REJECT
        if domain == "lift":
            preflight_error = self._lift_preflight_error()
            if preflight_error:
                self.get_logger().error(f"Rejecting lift trajectory: {preflight_error}")
                return GoalResponse.REJECT
        with self._lock:
            if domain in self._active:
                self.get_logger().warning(
                    f"Rejecting overlapping {domain} MoveIt goal"
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _lift_preflight_error(self) -> str | None:
        with self._lock:
            status = self._lift_status
        if status is None:
            return "no /lift_slide_driver/motor_status received"
        if not status.is_enabled:
            return (
                "lift drive is not enabled; call /lift_slide_driver/enable and "
                "verify motor_status before planning"
            )
        if not self._list_controllers_client.wait_for_service(timeout_sec=0.5):
            return "/controller_manager/list_controllers is unavailable"
        future = self._list_controllers_client.call_async(ListControllers.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=1.0):
            return "timed out checking lift_position_controller state"
        if future.exception() is not None:
            return f"failed to check controller state: {future.exception()}"
        if future.result() is None:
            return "controller state query returned no result"
        states = {
            controller.name: controller.state.lower()
            for controller in future.result().controller
        }
        actual = states.get("lift_position_controller", "missing")
        if actual != "active":
            return (
                "lift_position_controller is not active "
                f"(state={actual}); lift commands would be dropped"
            )
        if self._require_lift_homed and not status.homing_complete:
            return (
                "lift has not been homed; call /lift_slide_driver/start_homing "
                "and wait for homing_complete=true"
            )
        return None

    def _validate_gripper_goal(self, goal, domain: str) -> GoalResponse:
        if not self._is_enabled():
            self.get_logger().warning(
                f"Rejecting {domain} gripper goal while command bridge is disabled"
            )
            return GoalResponse.REJECT
        with self._lock:
            if domain in self._active:
                return GoalResponse.REJECT
        position = float(goal.command.position)
        if (
            not math.isfinite(position)
            or position < GRIPPER_MIN_POSITION_M
            or position > GRIPPER_MAX_POSITION_M
        ):
            self.get_logger().error(
                f"Rejecting {domain} gripper target {position}; calibrated range is "
                f"[{GRIPPER_MIN_POSITION_M}, {GRIPPER_MAX_POSITION_M}] m"
            )
            return GoalResponse.REJECT
        joint = LEFT_GRIPPER[0] if domain == "left" else RIGHT_GRIPPER[0]
        with self._lock:
            current = self._positions.get(joint)
        if current is not None and not gripper_position_is_valid(current):
            self.get_logger().error(
                f"Rejecting {domain} gripper goal because feedback is invalid: "
                f"{current:.4f} m"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_trajectory(self, handle, joints: list[str], domain: str):
        result = FollowJointTrajectory.Result()
        goal = handle.request.trajectory
        ordered_points = self._reorder_points(goal.joint_names, goal.points, joints)
        if not self._has_positions(joints):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Waiting for joint_states"
            handle.abort()
            return result

        ownership_error = self._command_ownership_error(domain)
        if ownership_error:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = ownership_error
            self.get_logger().error(ownership_error)
            handle.abort()
            return result

        self._mark_active(domain, True)
        try:
            start = time.monotonic()
            final = ordered_points[-1][1]
            current = self._current_vector(joints)
            if domain == "lift":
                profile_speed = self._lift_profile_speed(ordered_points, current[0])
                self._lift_profile_speed_pub.publish(Float64(data=profile_speed))
                self.get_logger().info(
                    f"lift profile speed set to {profile_speed:.4f} m/s for trajectory"
                )
                # Give the state controller one update cycle before the first
                # position setpoint reaches the hardware interface.
                time.sleep(0.03)
                start = time.monotonic()
            self.get_logger().info(
                f"{domain} trajectory started: duration={ordered_points[-1][0]:.3f} s, "
                f"current={current}, target={final}"
            )
            previous_time = 0.0
            previous = current
            for target_time, target in ordered_points:
                while True:
                    if not self._is_enabled():
                        self._publish_hold(domain)
                        handle.abort()
                        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                        result.error_string = "MoveIt command bridge disabled"
                        return result
                    if handle.is_cancel_requested:
                        self._publish_hold(domain)
                        handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        return result
                    elapsed = time.monotonic() - start
                    remaining = target_time - elapsed
                    if remaining <= 0.0:
                        break
                    alpha = min(1.0, max(0.0, (elapsed - previous_time) /
                                         max(1e-6, target_time - previous_time)))
                    interpolated = [
                        old + (new - old) * alpha for old, new in zip(previous, target)
                    ]
                    self._publish_domain(
                        domain, self._vector_for_domain(domain, joints, interpolated)
                    )
                    time.sleep(min(0.01, remaining))
                self._publish_domain(domain, self._vector_for_domain(domain, joints, target))
                previous_time = target_time
                previous = target

            completion, max_error = self._wait_for_trajectory_target(
                handle,
                joints,
                domain,
                final,
            )
            if completion == "disabled":
                self._publish_hold(domain)
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "MoveIt command bridge disabled while settling"
                handle.abort()
                return result
            if completion == "canceled":
                self._publish_hold(domain)
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory canceled while settling"
                handle.canceled()
                return result
            if completion == "timeout":
                self._publish_hold(domain)
                current = self._current_vector(joints)
                joint_errors = self._format_joint_errors(joints, current, final)
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                if domain == "lift":
                    result.error_string = (
                        f"lift final target did not settle within "
                        f"{self._trajectory_settle_timeout:.2f} s; "
                        f"max joint error={max_error:.5f} m, "
                        f"tolerance={self._lift_goal_tolerance:.5f} m; "
                        f"joint_errors=[{joint_errors}]"
                    )
                else:
                    result.error_string = (
                        f"{domain} final target did not settle within "
                        f"{self._trajectory_settle_timeout:.2f} s; "
                        f"max joint error={max_error:.5f} rad, "
                        f"proximal_tolerance={self._trajectory_goal_tolerance:.5f} rad, "
                        f"distal_tolerance={self._trajectory_distal_goal_tolerance:.5f} rad; "
                        f"joint_errors=[{joint_errors}]"
                    )
                self.get_logger().error(result.error_string)
                handle.abort()
                return result

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info(
                f"{domain} trajectory reached target: actual={self._current_vector(joints)}, "
                f"target={final}, max_error={max_error:.5f}"
            )
            handle.succeed()
            return result
        finally:
            self._mark_active(domain, False)

    def _execute_gripper(self, handle, joints: list[str], domain: str):
        result = GripperCommand.Result()
        target = float(handle.request.command.position)
        if not self._has_positions(joints):
            handle.abort()
            result.reached_goal = False
            result.stalled = True
            return result

        self._mark_active(domain, True)
        try:
            with self._lock:
                current = self._positions.get(joints[0], target)
            if not gripper_position_is_valid(current):
                self.get_logger().error(
                    f"Aborting {domain} gripper command because feedback is invalid: "
                    f"{current:.4f} m"
                )
                handle.abort()
                result.position = current
                result.reached_goal = False
                result.stalled = False
                return result
            self.get_logger().info(
                f"{domain} gripper command: current={current:.4f} m, "
                f"target={target:.4f} m"
            )
            start = time.monotonic()
            duration = max(0.05, min(2.0, abs(target - current) / 0.5))
            while True:
                if not self._is_enabled():
                    self._publish_gripper(domain, current)
                    handle.abort()
                    result.position = current
                    result.reached_goal = False
                    result.stalled = True
                    return result
                if handle.is_cancel_requested:
                    self._publish_gripper(domain, current)
                    handle.canceled()
                    result.position = current
                    result.reached_goal = False
                    return result
                alpha = min(1.0, (time.monotonic() - start) / duration)
                value = current + (target - current) * alpha
                self._publish_gripper(domain, value)
                if alpha >= 1.0:
                    break
                time.sleep(0.02)

            completion, actual = self._wait_for_gripper_target(
                handle,
                joints[0],
                domain,
                target,
            )
            result.position = actual
            result.effort = 0.0
            result.reached_goal = completion == "reached"
            result.stalled = completion == "stalled"
            self.get_logger().info(
                f"{domain} gripper command completed: state={completion}, "
                f"actual={actual:.4f} m, target={target:.4f} m"
            )
            if completion in {"reached", "stalled"}:
                handle.succeed()
            elif completion == "canceled":
                handle.canceled()
            else:
                handle.abort()
            return result
        finally:
            self._mark_active(domain, False)

    def _wait_for_gripper_target(
        self,
        handle,
        joint: str,
        domain: str,
        target: float,
    ) -> tuple[str, float]:
        deadline = time.monotonic() + self._gripper_settle_timeout
        last_motion = time.monotonic()
        last_position = None
        last_revision = -1
        current = self._current_vector([joint])[0]

        while True:
            if not self._is_enabled():
                self._publish_gripper(domain, current)
                return "disabled", current
            if handle.is_cancel_requested:
                self._publish_gripper(domain, current)
                return "canceled", current

            self._publish_gripper(domain, target)
            with self._lock:
                current = self._positions.get(joint, current)
                revision = self._joint_revisions.get(joint, 0)

            if not gripper_position_is_valid(current):
                return "invalid", current

            now = time.monotonic()
            error = abs(current - target)
            if error <= self._gripper_goal_tolerance:
                return "reached", current

            if revision != last_revision:
                if (
                    last_position is None
                    or abs(current - last_position) > self._gripper_stall_position_delta
                ):
                    last_motion = now
                last_position = current
                last_revision = revision

            feedback = GripperCommand.Feedback()
            feedback.position = current
            feedback.effort = 0.0
            feedback.reached_goal = False
            feedback.stalled = now - last_motion >= self._gripper_stall_timeout
            handle.publish_feedback(feedback)
            if feedback.stalled:
                return "stalled", current
            if now >= deadline:
                return "timeout", current
            time.sleep(min(self._trajectory_poll_period, deadline - now))

    def _reorder_points(self, names: Iterable[str], points, joints: list[str]):
        indices = {name: index for index, name in enumerate(names)}
        ordered_points = []
        clamped = {}
        for point in points:
            positions = []
            for joint in joints:
                value = float(point.positions[indices[joint]])
                bounded = clamp_trajectory_position(joint, value)
                if bounded != value:
                    clamped[joint] = max(
                        clamped.get(joint, 0.0), abs(value - bounded)
                    )
                positions.append(bounded)
            ordered_points.append(
                (
                    point.time_from_start.sec + point.time_from_start.nanosec * 1e-9,
                    positions,
                )
            )
        if clamped:
            details = ", ".join(
                f"{joint} by at most {amount:.6f}"
                for joint, amount in sorted(clamped.items())
            )
            self.get_logger().warning(
                "Clamped numerical trajectory limit spillover before execution: "
                + details
            )
        return ordered_points

    def _lift_profile_speed(self, ordered_points, current_position: float) -> float:
        required_speed = 0.0
        previous_time = 0.0
        previous_position = float(current_position)
        for target_time, target in ordered_points:
            duration = float(target_time) - previous_time
            if duration > 1e-6:
                required_speed = max(
                    required_speed,
                    abs(float(target[0]) - previous_position) / duration,
                )
            previous_time = float(target_time)
            previous_position = float(target[0])
        requested = required_speed * self._lift_profile_speed_margin
        return min(
            self._lift_profile_speed_max,
            max(self._lift_profile_speed_min, requested),
        )

    def _has_positions(self, joints: list[str]) -> bool:
        with self._lock:
            return all(joint in self._positions for joint in joints)

    def _current_vector(self, joints: list[str]) -> list[float]:
        with self._lock:
            return [self._positions.get(joint, 0.0) for joint in joints]

    def _current_vector_with_revision(self, joints: list[str]) -> tuple[list[float], int]:
        with self._lock:
            return (
                [self._positions.get(joint, 0.0) for joint in joints],
                min(self._joint_revisions.get(joint, 0) for joint in joints),
            )

    def _wait_for_trajectory_target(
        self,
        handle,
        joints: list[str],
        domain: str,
        target: list[float],
    ) -> tuple[str, float]:
        deadline = time.monotonic() + self._trajectory_settle_timeout
        stable_samples = 0
        last_revision = -1
        max_error = math.inf

        while True:
            if not self._is_enabled():
                return "disabled", max_error
            if handle.is_cancel_requested:
                return "canceled", max_error

            self._publish_domain(
                domain,
                self._vector_for_domain(domain, joints, target),
            )
            current, revision = self._current_vector_with_revision(joints)
            if revision != last_revision:
                last_revision = revision
                errors = [
                    abs(actual - expected)
                    for actual, expected in zip(current, target)
                ]
                max_error = max(errors)
                if all(
                    error <= self._goal_tolerance(joint)
                    for joint, error in zip(joints, errors)
                ):
                    stable_samples += 1
                    if stable_samples >= self._trajectory_stable_samples:
                        return "reached", max_error
                else:
                    stable_samples = 0

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return "timeout", max_error
            time.sleep(min(self._trajectory_poll_period, remaining))

    def _mark_active(self, domain: str, active: bool) -> None:
        with self._lock:
            if active:
                self._active.add(domain)
            else:
                self._active.discard(domain)

    def _vector_for_domain(self, domain: str, joints: list[str], target: list[float]) -> list[float]:
        with self._lock:
            positions = dict(self._positions)
            gripper_command = self._gripper_commands.get(domain)
        for joint, value in zip(joints, target):
            positions[joint] = value
        if domain == "left":
            return [positions.get(joint, 0.0) for joint in LEFT_ARM] + [
                positions.get(LEFT_GRIPPER[0], 0.0)
                if gripper_command is None
                else gripper_command
            ]
        if domain == "right":
            return [positions.get(joint, 0.0) for joint in RIGHT_ARM] + [
                positions.get(RIGHT_GRIPPER[0], 0.0)
                if gripper_command is None
                else gripper_command
            ]
        return [positions.get(LIFT[0], 0.0)]

    def _publish_domain(self, domain: str, values: list[float]) -> None:
        msg = Float64MultiArray(data=values)
        if domain == "left":
            self._left_pub.publish(msg)
        elif domain == "right":
            self._right_pub.publish(msg)
        else:
            self._lift_pub.publish(msg)

    def _command_topic(self, domain: str) -> str:
        if domain == "left":
            return "/left_forward_position_controller/commands"
        if domain == "right":
            return "/right_forward_position_controller/commands"
        return "/lift_position_controller/commands"

    def _command_ownership_error(self, domain: str) -> str | None:
        if not self._require_exclusive_command_publisher:
            return None

        topic = self._command_topic(domain)
        publishers = self.get_publishers_info_by_topic(topic)
        own_namespace = self.get_namespace().rstrip("/")
        own_name = f"{own_namespace}/{self.get_name()}" if own_namespace else f"/{self.get_name()}"
        external = sorted(
            {
                f"{info.node_namespace.rstrip('/')}/{info.node_name}"
                if info.node_namespace.rstrip("/")
                else f"/{info.node_name}"
                for info in publishers
            }
            - {own_name}
            - self._allowed_idle_command_publishers
        )
        if external:
            return (
                f"{domain} command topic {topic} has competing publishers: "
                + ", ".join(external)
                + "; close the direct controller publisher before executing MoveIt"
            )
        return None

    def _goal_tolerance(self, joint: str) -> float:
        if joint == "lift_joint":
            return self._lift_goal_tolerance
        if joint.endswith(("joint5", "joint6", "joint7")):
            return self._trajectory_distal_goal_tolerance
        return self._trajectory_goal_tolerance

    def _format_joint_errors(
        self, joints: list[str], current: list[float], target: list[float]
    ) -> str:
        return ", ".join(
            f"{joint}:target={expected:.5f},actual={actual:.5f},"
            f"error={actual - expected:+.5f},limit={self._goal_tolerance(joint):.5f}"
            for joint, actual, expected in zip(joints, current, target)
        )

    def _publish_gripper(self, domain: str, value: float) -> None:
        with self._lock:
            self._gripper_commands[domain] = value
            arm = [
                self._positions.get(joint, 0.0)
                for joint in (LEFT_ARM if domain == "left" else RIGHT_ARM)
            ]
        self._publish_domain(domain, arm + [value])

    def _publish_hold(self, domain: str) -> None:
        if domain == "left":
            self._publish_domain(domain, self._vector_for_domain(domain, LEFT_ARM, self._current_vector(LEFT_ARM)))
        elif domain == "right":
            self._publish_domain(domain, self._vector_for_domain(domain, RIGHT_ARM, self._current_vector(RIGHT_ARM)))
        else:
            self._publish_domain(domain, self._current_vector(LIFT))


def main() -> None:
    rclpy.init()
    node = MoveItCommandBridge()
    executor = MultiThreadedExecutor(num_threads=BRIDGE_EXECUTOR_THREADS)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
