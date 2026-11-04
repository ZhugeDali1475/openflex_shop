import math

import pytest

from openarmx_task_executor.head_pitch_motion import (
    next_position_step,
    position_reached,
)


def test_interpolation_steps_down_without_overshoot():
    values = [0.0]
    while values[-1] != pytest.approx(-1.2):
        values.append(next_position_step(values[-1], -1.2, 0.25))
        if len(values) > 10:
            raise AssertionError("interpolation did not terminate")

    assert values == pytest.approx([0.0, -0.25, -0.5, -0.75, -1.0, -1.2])


def test_interpolation_steps_back_up_without_overshoot():
    assert next_position_step(-1.2, -0.1, 0.5) == pytest.approx(-0.7)
    assert next_position_step(-0.2, -0.1, 0.5) == pytest.approx(-0.1)


def test_reached_rejects_non_finite_feedback():
    assert position_reached(-1.21, -1.2, 0.02)
    assert not position_reached(math.nan, -1.2, 0.02)


def test_interpolation_rejects_invalid_step():
    with pytest.raises(ValueError, match="positive"):
        next_position_step(0.0, -1.0, 0.0)
