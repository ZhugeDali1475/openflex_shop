import math

from openflex_tc_wam.geometry import (
    quaternion_multiply,
    rotate_vector,
    yaw_to_quaternion,
)


def test_candidate_frame_rotation_maps_world_vector_to_body():
    yaw_90 = yaw_to_quaternion(math.pi / 2.0)
    rotated = rotate_vector(yaw_90, (1.0, 0.0, 0.0))
    assert abs(rotated[0]) < 1.0e-7
    assert abs(rotated[1] - 1.0) < 1.0e-7


def test_quaternion_identity_composition():
    value = yaw_to_quaternion(0.3)
    assert quaternion_multiply((0.0, 0.0, 0.0, 1.0), value) == value
