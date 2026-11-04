import pytest

from openarmx_task_executor.shelf_view_node import (
    build_direct_trajectory_goal,
    direct_posture_duration,
)


def test_build_direct_trajectory_goal_uses_one_bounded_endpoint():
    names = [f"openarmx_right_joint{index}" for index in range(1, 8)]
    positions = [-1.0, 0.1, 0.2, 1.8, 0.3, 0.1, 0.5]

    goal = build_direct_trajectory_goal(names, positions, 3.25)

    assert list(goal.trajectory.joint_names) == names
    assert len(goal.trajectory.points) == 1
    assert list(goal.trajectory.points[0].positions) == pytest.approx(positions)
    assert goal.trajectory.points[0].time_from_start.sec == 3
    assert goal.trajectory.points[0].time_from_start.nanosec == 250_000_000


def test_build_direct_trajectory_goal_rejects_joint_limit_violation():
    with pytest.raises(ValueError, match="openarmx_right_joint4"):
        build_direct_trajectory_goal(
            ["openarmx_right_joint4"],
            [1.8001],
            1.0,
        )


def test_direct_posture_duration_synchronizes_arm_and_lift():
    current = {"openarmx_right_joint4": 0.0, "lift_joint": 0.0}
    target = {"openarmx_right_joint4": 1.8, "lift_joint": 0.183}

    duration = direct_posture_duration(
        current,
        target,
        velocity_scale=0.1,
        maximum_arm_speed=5.0,
        maximum_lift_speed=0.1,
        minimum_duration=0.5,
    )

    assert duration == pytest.approx(18.3)
