import math

import pytest

from openarmx_task_executor.post_pick_retreat_core import (
    PlanarPose,
    measure_retreat,
)


def test_retreat_distance_is_measured_along_initial_heading():
    start = PlanarPose(1.2, -0.4, math.radians(90.0))
    current = PlanarPose(1.2, -0.5, math.radians(90.0))

    progress = measure_retreat(start, current)

    assert progress.backward_distance == pytest.approx(0.10)
    assert progress.lateral_drift == pytest.approx(0.0)
    assert progress.yaw_error == pytest.approx(0.0)


def test_forward_motion_does_not_count_as_retreat():
    progress = measure_retreat(
        PlanarPose(0.0, 0.0, 0.0),
        PlanarPose(0.03, 0.0, 0.0),
    )

    assert progress.backward_distance == pytest.approx(-0.03)


def test_retreat_reports_lateral_drift_separately():
    progress = measure_retreat(
        PlanarPose(0.0, 0.0, math.radians(45.0)),
        PlanarPose(-0.05 - 0.02, -0.05 + 0.02, math.radians(45.0)),
    )

    assert progress.backward_distance == pytest.approx(math.sqrt(2.0) * 0.05)
    assert progress.lateral_drift == pytest.approx(math.sqrt(2.0) * 0.02)


def test_yaw_error_wraps_at_pi():
    progress = measure_retreat(
        PlanarPose(0.0, 0.0, math.radians(179.0)),
        PlanarPose(0.0, 0.0, math.radians(-179.0)),
    )

    assert progress.yaw_error == pytest.approx(math.radians(2.0))
