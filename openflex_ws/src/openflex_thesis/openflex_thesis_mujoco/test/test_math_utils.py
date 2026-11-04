import math

import pytest

from openflex_thesis_mujoco.math_utils import (
    compose_pose,
    relative_pose,
    yaw_to_quaternion,
)


def test_relative_and_compose_pose_round_trip():
    parent_position = (0.4, -0.2, 0.7)
    parent_orientation = yaw_to_quaternion(0.6)
    child_position = (1.1, 0.3, 0.9)
    child_orientation = yaw_to_quaternion(-0.3)
    local = relative_pose(
        parent_position, parent_orientation, child_position, child_orientation
    )
    reconstructed = compose_pose(
        parent_position, parent_orientation, local[0], local[1]
    )
    assert reconstructed[0] == pytest.approx(child_position)
    assert reconstructed[1] == pytest.approx(child_orientation)


def test_yaw_quaternion_is_normalized():
    quaternion = yaw_to_quaternion(math.pi)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)

