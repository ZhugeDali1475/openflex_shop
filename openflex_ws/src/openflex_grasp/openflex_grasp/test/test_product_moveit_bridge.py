import math

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped

from openflex_grasp.product_moveit_bridge import (
    multiply_quaternions_xyzw,
    parse_arm,
    parse_max_dof,
    rotate_vector_by_quaternion,
    transform_pose_fields,
    transform_pose_fields_via_planar_base,
    transform_pose_via_planar_base,
    yaw_only_quaternion_xyzw,
)


def test_parse_moveit_request_options():
    assert parse_arm("right") == 1
    assert parse_arm("left") == 0
    assert parse_max_dof("auto") == 0
    assert parse_max_dof("8") == 8
    with pytest.raises(ValueError):
        parse_max_dof("9")


def test_rotates_vector_by_quaternion():
    half = math.sqrt(0.5)
    rotated = rotate_vector_by_quaternion((1.0, 0.0, 0.0), (0.0, 0.0, half, half))
    np.testing.assert_allclose(rotated, (0.0, 1.0, 0.0), atol=1e-8)


def test_transform_pose_fields_applies_tf_transform():
    half = math.sqrt(0.5)
    position, orientation = transform_pose_fields(
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.2, -0.1, 0.3),
        (0.0, 0.0, half, half),
    )
    np.testing.assert_allclose(position, (0.2, 0.9, 0.3), atol=1e-8)
    np.testing.assert_allclose(orientation, (0.0, 0.0, half, half), atol=1e-8)


def test_multiply_quaternions_normalizes_result():
    q = multiply_quaternions_xyzw((0.0, 0.0, 1.0, 1.0), (0.0, 0.0, 0.0, 2.0))
    np.testing.assert_allclose(np.linalg.norm(q), 1.0, atol=1e-8)


def test_yaw_only_quaternion_removes_roll_and_pitch():
    roll = math.radians(20.0)
    pitch = math.radians(-15.0)
    yaw = math.radians(35.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

    result = yaw_only_quaternion_xyzw(quaternion)

    np.testing.assert_allclose(result, (0.0, 0.0, sy, cy), atol=1e-8)


def test_planar_base_transform_ignores_navigation_height():
    half = math.sqrt(0.5)
    position, orientation = transform_pose_fields_via_planar_base(
        (0.10, 0.0, 0.20),
        (0.0, 0.0, 0.0, 1.0),
        (0.50, 0.0, 0.80),
        (0.0, 0.0, 0.0, 1.0),
        (1.0, 2.0, -0.113),
        (0.0, 0.0, half, half),
    )

    np.testing.assert_allclose(position, (1.0, 2.6, 1.0), atol=1e-8)
    np.testing.assert_allclose(orientation, (0.0, 0.0, half, half), atol=1e-8)


def test_pose_transform_routes_through_planar_base():
    half = math.sqrt(0.5)
    base_from_camera = TransformStamped()
    base_from_camera.transform.translation.x = 0.50
    base_from_camera.transform.translation.z = 0.80
    base_from_camera.transform.rotation.w = 1.0
    odom_from_base = TransformStamped()
    odom_from_base.transform.translation.x = 1.0
    odom_from_base.transform.translation.y = 2.0
    odom_from_base.transform.translation.z = -0.113
    odom_from_base.transform.rotation.z = half
    odom_from_base.transform.rotation.w = half

    class FakeBuffer:
        def __init__(self):
            self.calls = []

        def lookup_transform(self, target, source, lookup_time, timeout):
            self.calls.append((target, source, lookup_time, timeout))
            return {
                ("base_link", "camera"): base_from_camera,
                ("odom", "base_link"): odom_from_base,
            }[(target, source)]

    source = PoseStamped()
    source.header.frame_id = "camera"
    source.pose.position.x = 0.10
    source.pose.position.z = 0.20
    source.pose.orientation.w = 1.0
    buffer = FakeBuffer()

    result = transform_pose_via_planar_base(
        buffer,
        source,
        target_frame="odom",
        lookup_time="stamp",
        timeout="timeout",
    )

    assert [(target, source) for target, source, _, _ in buffer.calls] == [
        ("base_link", "camera"),
        ("odom", "base_link"),
    ]
    assert result.header.frame_id == "odom"
    np.testing.assert_allclose(
        (
            result.pose.position.x,
            result.pose.position.y,
            result.pose.position.z,
        ),
        (1.0, 2.6, 1.0),
        atol=1e-8,
    )
