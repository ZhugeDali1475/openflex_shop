import math

import pytest

from openflex_tc_wam.observation_adapter import (
    default_pose_covariance,
    normalized_quaternion,
    rotate_pose_covariance,
    transform_pose,
)


def test_transform_pose_rotates_position_and_orientation():
    half = math.sqrt(0.5)
    position, orientation = transform_pose(
        (1.0, 0.0, 0.5),
        (0.0, 0.0, 0.0, 2.0),
        (2.0, 3.0, 0.5),
        (0.0, 0.0, half, half),
    )
    assert position == pytest.approx((2.0, 4.0, 1.0))
    assert orientation == pytest.approx((0.0, 0.0, half, half))


def test_covariance_rotation_swaps_planar_axes():
    covariance = [0.0] * 36
    covariance[0] = 1.0
    covariance[7] = 4.0
    covariance[14] = 9.0
    half = math.sqrt(0.5)
    rotated = rotate_pose_covariance(covariance, (0.0, 0.0, half, half))
    assert rotated[0] == pytest.approx(4.0)
    assert rotated[7] == pytest.approx(1.0)
    assert rotated[14] == pytest.approx(9.0)


def test_invalid_quaternion_and_covariance_fail_closed():
    with pytest.raises(ValueError):
        normalized_quaternion((0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        rotate_pose_covariance([0.0] * 35, (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        default_pose_covariance(0.0, 0.1)
