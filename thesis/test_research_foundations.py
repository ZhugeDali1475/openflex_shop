import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_research_foundations import compose_pose, evaluate, local_pose, wrap_angle


@pytest.mark.parametrize("frame", [(0, 0, 0), (9, -3, 0.4), (-5, 2, -2.4)])
def test_episode_local_pose_is_common_frame_invariant(frame):
    origin = (2.0, -1.0, 2.9)
    current = (-0.7, 3.0, -3.0)
    expected = local_pose(origin, current)
    actual = local_pose(compose_pose(frame, origin), compose_pose(frame, current))
    assert actual[:2] == pytest.approx(expected[:2], abs=1.0e-12)
    assert wrap_angle(actual[2] - expected[2]) == pytest.approx(0.0, abs=1.0e-12)


def test_yaw_boundary_and_adjacency_limit_are_explicit():
    report = evaluate()
    assert report["yaw_boundary"]["scalar_distance"] > 6.0
    assert report["yaw_boundary"]["circular_distance"] < 3.0e-6
    assert report["yaw_boundary"]["sine_cosine_distance"] < 3.0e-6
    edge = report["geometric_adjacency_not_motion_certificate"]
    assert edge["component"] == [1, 2]
    assert edge["barrier_x_interval"][0] < edge["straight_segment_midpoint"][0] < edge["barrier_x_interval"][1]
    assert edge["midpoint_is_blocked"] is True


def test_local_pose_reconstructs_current_pose():
    origin = (-2.0, 3.0, 2.0)
    current = (4.0, 5.0, -2.0)
    restored = compose_pose(origin, local_pose(origin, current))
    assert restored[:2] == pytest.approx(current[:2])
    assert math.cos(restored[2]) == pytest.approx(math.cos(current[2]))
    assert math.sin(restored[2]) == pytest.approx(math.sin(current[2]))
