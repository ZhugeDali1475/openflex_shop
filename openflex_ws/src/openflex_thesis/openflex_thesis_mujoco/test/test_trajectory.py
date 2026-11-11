import pytest

from openflex_thesis_mujoco.trajectory import TrajectoryPoint, TrajectorySampler, measured_goal_reached


def test_piecewise_linear_interpolation_and_completion():
    sampler = TrajectorySampler(
        ("lift", "joint"),
        {"lift": 0.0, "joint": -1.0},
        [
            TrajectoryPoint(1.0, (0.2, 0.0)),
            TrajectoryPoint(3.0, (0.4, 1.0)),
        ],
    )
    values, done = sampler.sample(2.0)
    assert not done
    assert values == pytest.approx({"lift": 0.3, "joint": 0.5})
    values, done = sampler.sample(4.0)
    assert done
    assert values == pytest.approx({"lift": 0.4, "joint": 1.0})


def test_rejects_wrong_position_count():
    with pytest.raises(ValueError):
        TrajectorySampler(
            ("a", "b"), {"a": 0.0, "b": 0.0}, [TrajectoryPoint(1.0, (1.0,))]
        )


@pytest.mark.parametrize('time_sec,position', [(float('nan'), 0), (float('inf'), 0), (1, float('nan'))])
def test_rejects_non_finite_trajectory(time_sec, position):
    with pytest.raises(ValueError):
        TrajectorySampler(['joint'], {'joint': 0.0}, [TrajectoryPoint(time_sec, (position,))])


def test_requested_positions_alone_do_not_prove_real_arrival():
    assert measured_goal_reached(['lift_joint', 'arm'], [0.1, 0.2], [0.101, 0.201], [0, 0])
    assert not measured_goal_reached(['lift_joint'], [0.1], [0.11], [0])
    assert not measured_goal_reached(['arm'], [0.2], [0.2], [0.1])
    assert not measured_goal_reached(['arm'], [0.2], [float('nan')], [0])
    assert not measured_goal_reached(['arm'], [0.2], [], [])
