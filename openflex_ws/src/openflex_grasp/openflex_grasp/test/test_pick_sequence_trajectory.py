import asyncio
import math
import threading
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from openarmx_integrated_moveit_msgs.action import CartesianApproach
from openarmx_task_msgs.action import PlanPick
from rclpy.action import GoalResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from openflex_grasp.pick_sequence_server import (
    MAX_APPROACH_DISTANCE_M,
    MAX_HEIGHT_OFFSET_M,
    PickSequenceServer,
    apply_approach_distance,
    offset_pose_height,
    require_pose_above_minimum,
    resolve_height_offsets,
    reverse_arm_trajectory,
    validate_height_offset,
    validate_approach_distance,
)


def _point(position, velocity, seconds, joint_count=7):
    point = JointTrajectoryPoint()
    point.positions = [position] * joint_count
    point.velocities = [velocity] * joint_count
    point.accelerations = [0.25] * joint_count
    point.effort = [1.0] * joint_count
    point.time_from_start.sec = seconds
    return point


def test_reverse_arm_trajectory_reverses_points_time_and_velocity():
    source = RobotTrajectory()
    names = [f"openarmx_right_joint{index}" for index in range(1, 8)]
    source.joint_trajectory.joint_names = names
    source.joint_trajectory.points = [
        _point(1.0, 0.5, 0),
        _point(2.0, 0.25, 1),
        _point(3.0, 0.0, 2),
    ]

    result = reverse_arm_trajectory(source, names)

    assert [point.positions[0] for point in result.joint_trajectory.points] == [3.0, 2.0, 1.0]
    assert [point.time_from_start.sec for point in result.joint_trajectory.points] == [0, 1, 2]
    assert [point.velocities[0] for point in result.joint_trajectory.points] == [-0.0, -0.25, -0.5]
    assert all(not point.effort for point in result.joint_trajectory.points)
    assert source.joint_trajectory.points[0].positions[0] == 1.0


def test_reverse_arm_trajectory_supports_arm_with_lift():
    source = RobotTrajectory()
    names = ["lift_joint"] + [
        f"openarmx_right_joint{index}" for index in range(1, 8)
    ]
    source.joint_trajectory.joint_names = names
    source.joint_trajectory.points = [
        _point(1.0, 0.5, 0, 8),
        _point(2.0, 0.0, 1, 8),
    ]

    result = reverse_arm_trajectory(source, names)

    assert result.joint_trajectory.joint_names == names
    assert list(result.joint_trajectory.points[0].positions) == [2.0] * 8
    assert list(result.joint_trajectory.points[-1].velocities) == [-0.5] * 8


def test_reverse_arm_trajectory_rejects_wrong_lift_order():
    source = RobotTrajectory()
    arm_names = [f"openarmx_right_joint{index}" for index in range(1, 8)]
    names = arm_names + ["lift_joint"]
    source.joint_trajectory.joint_names = names
    source.joint_trajectory.points = [
        _point(1.0, 0.0, 0, 8),
        _point(2.0, 0.0, 1, 8),
    ]

    with pytest.raises(RuntimeError, match="start with lift_joint"):
        reverse_arm_trajectory(source, names)


def test_staged_height_offsets_are_independent():
    pregrasp = PoseStamped()
    pregrasp.header.frame_id = "odom"
    pregrasp.pose.position.z = 0.72
    grasp = PoseStamped()
    grasp.header.frame_id = "odom"
    grasp.pose.position.z = 0.69
    goal = SimpleNamespace(
        height_offset_m=0.0,
        initial_raise_m=0.01,
        pregrasp_height_offset_m=0.02,
        grasp_height_offset_m=0.03,
        retreat_height_offset_m=0.04,
    )

    assert resolve_height_offsets(goal) == pytest.approx((0.01, 0.02, 0.03, 0.04))
    elevated_pregrasp = offset_pose_height(
        pregrasp, goal.pregrasp_height_offset_m, "pregrasp_height_offset_m"
    )
    elevated_grasp = offset_pose_height(
        grasp, goal.grasp_height_offset_m, "grasp_height_offset_m"
    )

    assert elevated_pregrasp.pose.position.z == pytest.approx(0.74)
    assert elevated_grasp.pose.position.z == pytest.approx(0.72)
    assert pregrasp.pose.position.z == pytest.approx(0.72)
    assert grasp.pose.position.z == pytest.approx(0.69)


def test_legacy_uniform_height_offset_remains_supported():
    goal = SimpleNamespace(
        height_offset_m=0.035,
        initial_raise_m=0.0,
        pregrasp_height_offset_m=0.0,
        grasp_height_offset_m=0.0,
        retreat_height_offset_m=0.0,
    )

    assert resolve_height_offsets(goal) == pytest.approx((0.035,) * 4)


def test_approach_distance_keeps_grasp_fixed_and_moves_pregrasp():
    pregrasp = PoseStamped()
    pregrasp.pose.position.x = 0.40
    pregrasp.pose.position.z = 0.70
    grasp = PoseStamped()
    grasp.pose.position.x = 0.50
    grasp.pose.position.z = 0.70

    adjusted, actual_distance = apply_approach_distance(
        pregrasp, grasp, 0.15
    )

    assert actual_distance == pytest.approx(0.15)
    assert adjusted.pose.position.x == pytest.approx(0.35)
    assert adjusted.pose.position.z == pytest.approx(0.70)
    assert grasp.pose.position.x == pytest.approx(0.50)
    assert pregrasp.pose.position.x == pytest.approx(0.40)


def test_zero_approach_distance_keeps_graspnet_distance():
    pregrasp = PoseStamped()
    grasp = PoseStamped()
    grasp.pose.position.y = 0.12

    adjusted, actual_distance = apply_approach_distance(pregrasp, grasp, 0.0)

    assert actual_distance == pytest.approx(0.12)
    assert adjusted.pose.position.y == pytest.approx(0.0)


@pytest.mark.parametrize("value", [-0.01, 0.001, MAX_APPROACH_DISTANCE_M + 0.01])
def test_approach_distance_rejects_unsafe_values(value):
    with pytest.raises(ValueError, match="approach_distance_m"):
        validate_approach_distance(value)


@pytest.mark.parametrize("value", [-0.001, MAX_HEIGHT_OFFSET_M + 0.001, math.inf])
def test_height_offset_rejects_unsafe_values(value):
    with pytest.raises(ValueError, match="height_offset_m"):
        validate_height_offset(value)


def test_product_bottom_height_rejects_low_tcp_pose():
    pose = PoseStamped()
    pose.pose.position.z = 0.64

    with pytest.raises(ValueError, match="below the product-cloud"):
        require_pose_above_minimum(pose, 0.65, "grasp")


def test_product_bottom_height_accepts_pose_at_clearance():
    pose = PoseStamped()
    pose.pose.position.z = 0.65

    require_pose_above_minimum(pose, 0.65, "grasp")


def _plan_goal(max_dof):
    return SimpleNamespace(
        arm_name="right",
        max_dof=max_dof,
        object_id="可乐",
        detection_id="detection-1",
        height_offset_m=0.0,
        initial_raise_m=0.0,
        pregrasp_height_offset_m=0.03,
        grasp_height_offset_m=0.03,
        retreat_height_offset_m=0.04,
        approach_distance_m=0.0,
    )


def test_pick_sequence_accepts_only_arm_with_lift_scope():
    server = object.__new__(PickSequenceServer)

    assert (
        PickSequenceServer._plan_goal(
            server, _plan_goal(PlanPick.Goal.ARM_WITH_LIFT)
        )
        == GoalResponse.ACCEPT
    )
    assert (
        PickSequenceServer._plan_goal(server, _plan_goal(PlanPick.Goal.ARM_ONLY))
        == GoalResponse.REJECT
    )


def test_cartesian_transit_prefers_strict_path():
    calls = []
    server = object.__new__(PickSequenceServer)
    handle = SimpleNamespace(is_cancel_requested=False)
    expected = SimpleNamespace(trajectory=RobotTrajectory())

    async def plan(*_args, **kwargs):
        calls.append(
            kwargs.get("path_mode", CartesianApproach.Goal.STRICT_CARTESIAN)
        )
        return expected

    server._plan_cartesian = plan

    result = asyncio.run(
        PickSequenceServer._plan_cartesian_prefer_straight(
            server,
            None,
            None,
            "right",
            None,
            0.1,
            0.1,
            handle,
            label="test transit",
        )
    )

    assert result is expected
    assert calls == [CartesianApproach.Goal.STRICT_CARTESIAN]


def test_cartesian_goal_carries_product_bottom_height():
    captured = []
    server = object.__new__(PickSequenceServer)
    handle = SimpleNamespace(is_cancel_requested=False)
    server.get_parameter = lambda name: SimpleNamespace(
        value={
            "cartesian_start_position_tolerance_m": 0.04,
            "cartesian_start_orientation_tolerance_deg": 20.0,
        }[name]
    )

    class Client:
        @staticmethod
        def wait_for_server(timeout_sec):
            return timeout_sec == pytest.approx(2.0)

        @staticmethod
        async def send_goal_async(goal):
            captured.append(goal)
            return SimpleNamespace(accepted=True)

    server._cartesian = Client()

    async def child_result(_sent, _handle, _label):
        return SimpleNamespace(
            error_code=SimpleNamespace(val=1),
            path_fraction=1.0,
            trajectory=RobotTrajectory(),
            message="ok",
        )

    server._child_result = child_result
    start = PoseStamped()
    start.header.frame_id = "odom"
    start.pose.orientation.w = 1.0
    end = PoseStamped()
    end.header.frame_id = "odom"
    end.pose.position.x = 0.1
    end.pose.orientation.w = 1.0

    asyncio.run(
        PickSequenceServer._plan_cartesian(
            server,
            start,
            end,
            "right",
            JointState(),
            0.1,
            0.1,
            handle,
            minimum_tcp_z=0.65,
        )
    )

    assert captured[0].use_minimum_tcp_z
    assert captured[0].minimum_tcp_z == pytest.approx(0.65)


def test_cartesian_transit_falls_back_to_moveit():
    calls = []
    warnings = []
    server = object.__new__(PickSequenceServer)
    handle = SimpleNamespace(is_cancel_requested=False)
    fallback = SimpleNamespace(trajectory=RobotTrajectory())
    server.get_logger = lambda: SimpleNamespace(warning=warnings.append)

    async def plan(*_args, **kwargs):
        mode = kwargs.get("path_mode", CartesianApproach.Goal.STRICT_CARTESIAN)
        calls.append((mode, kwargs.get("end_joint_positions")))
        if mode == CartesianApproach.Goal.STRICT_CARTESIAN:
            raise RuntimeError("straight path unavailable")
        return fallback

    server._plan_cartesian = plan

    result = asyncio.run(
        PickSequenceServer._plan_cartesian_prefer_straight(
            server,
            None,
            None,
            "right",
            None,
            0.1,
            0.1,
            handle,
            label="test transit",
            end_joint_positions=[0.1] * 8,
        )
    )

    assert result is fallback
    assert calls == [
        (CartesianApproach.Goal.STRICT_CARTESIAN, None),
        (CartesianApproach.Goal.UNCONSTRAINED_MOVEIT, [0.1] * 8),
    ]
    assert warnings and "falling back" in warnings[0]


def test_pregrasp_transit_falls_back_to_height_validated_moveit():
    calls = []
    warnings = []
    server = object.__new__(PickSequenceServer)
    handle = SimpleNamespace(is_cancel_requested=False)
    fallback_trajectory = RobotTrajectory()
    server.get_logger = lambda: SimpleNamespace(warning=warnings.append)

    async def cartesian(*_args, **kwargs):
        mode = kwargs.get("path_mode", CartesianApproach.Goal.STRICT_CARTESIAN)
        calls.append((mode, kwargs.get("minimum_tcp_z")))
        if mode == CartesianApproach.Goal.STRICT_CARTESIAN:
            raise RuntimeError("straight path unavailable")
        return SimpleNamespace(trajectory=fallback_trajectory)

    server._plan_cartesian = cartesian

    result = asyncio.run(
        PickSequenceServer._plan_pregrasp_prefer_straight(
            server,
            None,
            None,
            "right",
            None,
            8,
            0.1,
            0.1,
            handle,
            minimum_tcp_z=0.65,
        )
    )

    assert result is fallback_trajectory
    assert calls == [
        (CartesianApproach.Goal.STRICT_CARTESIAN, 0.65),
        (CartesianApproach.Goal.UNCONSTRAINED_MOVEIT, 0.65),
    ]
    assert warnings and "falling back" in warnings[0]


def test_height_offset_execution_raises_before_pregrasp_and_lowers_last():
    stages = []
    plan = SimpleNamespace(
        plan_id="plan-1",
        detection_id="detection-1",
        object_id="可乐",
        arm_name="right",
        raise_trajectory="raise",
        pregrasp_trajectory="pregrasp",
        approach_trajectory="approach",
        retreat_trajectory="retreat",
        photo_return_trajectory="photo_return",
        lower_trajectory="lower",
    )

    class Handle:
        request = SimpleNamespace(
            plan_id="plan-1", object_id="可乐", arm_name="right"
        )
        is_cancel_requested = False

        @staticmethod
        def publish_feedback(_message):
            pass

        @staticmethod
        def succeed():
            pass

        @staticmethod
        def abort():
            raise AssertionError("execution unexpectedly aborted")

        @staticmethod
        def canceled():
            raise AssertionError("execution unexpectedly canceled")

    server = object.__new__(PickSequenceServer)
    server._busy = threading.Lock()
    server._plans = SimpleNamespace(take=lambda _plan_id: plan)
    server._grippers = {"right": object()}
    server._publish_execution_debug = lambda *_args: None
    server._validate_robot_start = lambda _plan: None
    server.get_parameter = lambda name: SimpleNamespace(
        value={
            "gripper_open_m": 0.044,
            "gripper_attached_min_position_m": 0.002,
            "gripper_attached_max_position_m": 0.041,
        }[name]
    )

    async def reverify(_detection_id):
        return SimpleNamespace(success=True, message="ok")

    async def execute_motion(trajectory, _handle, stage):
        stages.append((trajectory, stage))

    async def gripper(_client, position, _handle):
        return SimpleNamespace(stalled=position == 0.0, position=0.01)

    async def snapshot(_detection_id, consume=False):
        assert consume
        return SimpleNamespace(success=True)

    server._reverify_call = reverify
    server._execute_motion = execute_motion
    server._gripper = gripper
    server._snapshot_call = snapshot

    result = asyncio.run(PickSequenceServer._execute(server, Handle()))

    assert result.success
    assert [trajectory for trajectory, _stage in stages] == [
        "raise",
        "pregrasp",
        "approach",
        "retreat",
        "photo_return",
        "lower",
    ]


class _GripperResultClient:
    def __init__(self, wrapped):
        self.wrapped = wrapped
        self.goals = []

    @staticmethod
    def wait_for_server(timeout_sec):
        return timeout_sec == pytest.approx(2.0)

    async def send_goal_async(self, goal):
        self.goals.append(goal)
        wrapped = self.wrapped

        class SentGoal:
            accepted = True

            async def get_result_async(self):
                return wrapped

        return SentGoal()


def _gripper_test_server():
    server = object.__new__(PickSequenceServer)
    server._active_child_lock = threading.Lock()
    server._active_child_goal = None
    server.get_parameter = lambda name: SimpleNamespace(
        value={
            "gripper_max_effort": 10.0,
            "gripper_attached_max_position_m": 0.041,
        }[name]
    )
    return server


def _gripper_wrapped(status, *, position, reached_goal, stalled):
    return SimpleNamespace(
        status=status,
        result=SimpleNamespace(
            position=position,
            reached_goal=reached_goal,
            stalled=stalled,
        ),
    )


def test_gripper_rejects_failed_action_terminal_state():
    server = _gripper_test_server()
    client = _GripperResultClient(
        _gripper_wrapped(
            GoalStatus.STATUS_ABORTED,
            position=0.044,
            reached_goal=True,
            stalled=False,
        )
    )

    with pytest.raises(RuntimeError, match="terminal status"):
        asyncio.run(
            PickSequenceServer._gripper(
                server, client, 0.044, SimpleNamespace(is_cancel_requested=False)
            )
        )


@pytest.mark.parametrize(
    ("target", "position", "reached_goal", "stalled", "message"),
    [
        (0.044, 0.020, False, True, "failed to open"),
        (0.044, 0.020, True, False, "failed to open"),
        (0.0, 0.020, False, False, "failed to close"),
        (0.0, 0.020, True, False, "failed to close"),
        (0.0, 0.0438, False, True, "before meaningful closure"),
        (0.0, -0.5271, False, True, "invalid position"),
    ],
)
def test_gripper_rejects_unsuccessful_or_invalid_results(
    target, position, reached_goal, stalled, message
):
    server = _gripper_test_server()
    client = _GripperResultClient(
        _gripper_wrapped(
            GoalStatus.STATUS_SUCCEEDED,
            position=position,
            reached_goal=reached_goal,
            stalled=stalled,
        )
    )

    with pytest.raises(RuntimeError, match=message):
        asyncio.run(
            PickSequenceServer._gripper(
                server, client, target, SimpleNamespace(is_cancel_requested=False)
            )
        )


@pytest.mark.parametrize(
    ("target", "position", "reached_goal", "stalled"),
    [
        (0.044, 0.0435, True, False),
        (0.044, 0.0430, False, True),
        (0.0, 0.020, False, True),
    ],
)
def test_gripper_accepts_verified_open_or_contact_close(
    target, position, reached_goal, stalled
):
    server = _gripper_test_server()
    client = _GripperResultClient(
        _gripper_wrapped(
            GoalStatus.STATUS_SUCCEEDED,
            position=position,
            reached_goal=reached_goal,
            stalled=stalled,
        )
    )

    result = asyncio.run(
        PickSequenceServer._gripper(
            server, client, target, SimpleNamespace(is_cancel_requested=False)
        )
    )

    assert result.position == pytest.approx(position)
    assert client.goals[0].command.position == pytest.approx(target)


@pytest.mark.parametrize("failed_target", [0.044, 0.0])
def test_gripper_failure_stops_normal_pick_sequence(failed_target):
    motions = []
    gripper_targets = []
    consumed_snapshots = []
    plan = SimpleNamespace(
        plan_id="plan-gripper-failure",
        detection_id="detection-gripper-failure",
        object_id="饮料",
        arm_name="right",
        raise_trajectory=None,
        pregrasp_trajectory="pregrasp",
        approach_trajectory="approach",
        retreat_trajectory="retreat",
        photo_return_trajectory="photo_return",
        lower_trajectory=None,
    )

    class Handle:
        request = SimpleNamespace(
            plan_id=plan.plan_id,
            object_id=plan.object_id,
            arm_name=plan.arm_name,
        )
        is_cancel_requested = False

        def __init__(self):
            self.aborted = False

        @staticmethod
        def publish_feedback(_message):
            pass

        @staticmethod
        def succeed():
            raise AssertionError("failed pick unexpectedly succeeded")

        def abort(self):
            self.aborted = True

        @staticmethod
        def canceled():
            raise AssertionError("failed pick unexpectedly canceled")

    server = object.__new__(PickSequenceServer)
    server._busy = threading.Lock()
    server._plans = SimpleNamespace(take=lambda _plan_id: plan)
    server._grippers = {"right": object()}
    server._publish_execution_debug = lambda *_args: None
    server._validate_robot_start = lambda _plan: None
    server.get_parameter = lambda name: SimpleNamespace(
        value={
            "gripper_open_m": 0.044,
            "gripper_attached_min_position_m": 0.002,
            "gripper_attached_max_position_m": 0.041,
        }[name]
    )

    async def reverify(_detection_id):
        return SimpleNamespace(success=True, message="ok")

    async def execute_motion(trajectory, _handle, stage):
        motions.append((trajectory, stage))

    async def gripper(_client, position, _handle):
        gripper_targets.append(position)
        if position == failed_target:
            raise RuntimeError("injected gripper failure")
        return SimpleNamespace(stalled=False, position=position)

    async def snapshot(detection_id, consume=False):
        consumed_snapshots.append((detection_id, consume))
        return SimpleNamespace(success=True)

    server._reverify_call = reverify
    server._execute_motion = execute_motion
    server._gripper = gripper
    server._snapshot_call = snapshot
    handle = Handle()

    result = asyncio.run(PickSequenceServer._execute(server, handle))

    assert not result.success
    assert handle.aborted
    assert consumed_snapshots == []
    if failed_target == 0.044:
        assert gripper_targets == pytest.approx([0.044])
        assert not any(trajectory == "approach" for trajectory, _stage in motions)
    else:
        assert gripper_targets == pytest.approx([0.044, 0.0])
        assert any(trajectory == "approach" for trajectory, _stage in motions)
        assert any(trajectory == "retreat" for trajectory, _stage in motions)


class _StartValidator:
    def __init__(self, joints):
        self._joints = joints

    @staticmethod
    def get_parameter(name):
        values = {
            "max_arm_start_error_deg": 1.5,
            "max_lift_start_error_m": 0.003,
        }
        return SimpleNamespace(value=values[name])


def test_start_validation_covers_arm_and_lift_joint_state():
    joints = {
        f"openarmx_right_joint{index}": 0.0
        for index in range(1, 8)
    }
    joints["lift_joint"] = 0.0
    plan = SimpleNamespace(
        arm_name="right",
        start_joints=dict(joints),
    )
    validator = _StartValidator(dict(joints))

    PickSequenceServer._validate_robot_start(validator, plan)

    validator._joints["openarmx_right_joint1"] = 0.1
    with pytest.raises(RuntimeError, match="joint openarmx_right_joint1 moved"):
        PickSequenceServer._validate_robot_start(validator, plan)

    validator._joints["openarmx_right_joint1"] = 0.0
    validator._joints["lift_joint"] = 0.01
    with pytest.raises(RuntimeError, match="joint lift_joint moved"):
        PickSequenceServer._validate_robot_start(validator, plan)
