import pytest

from openflex_thesis_mujoco.trajectory import TrajectoryPoint, TrajectorySampler


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

