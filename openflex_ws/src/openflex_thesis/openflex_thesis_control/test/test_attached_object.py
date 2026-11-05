"""Regression tests for the G5 attached-object geometry defect.

The first G5 implementation attached the grasp box at the identity pose in the
TCP frame. With the oracle's horizontal approach that rotated the box's 0.16 m
axis onto the approach axis, moving the back face to 0.1001 m from the link7
origin while the link7 collision mesh reaches 0.11051 m. MoveIt then refused the
transport start state with `link7 <-> thesis_grasped_...`, and OMPL reported the
unhelpful `Motion planning start tree could not be initialized`.
"""

import math

import pytest

from openflex_thesis_control.attached_object import (
    axes_alignment,
    attached_box_clearance,
    box_extent_along,
    quaternion_multiply,
    relative_pose,
    rotate,
    summarize_box,
)

# Measured from openarmx_description/meshes/arm/v10/collision/link7_symp.stl
LINK7_FORWARD_REACH_M = 0.11051
# openarmx_right_link7 -> openarmx_right_hand (0.1001) -> hand_tcp (0.08)
TCP_OFFSET_ALONG_LINK7_M = 0.1001 + 0.08
HALF_SIZE = (0.035, 0.030, 0.080)


def _yaw_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def _oracle_tcp_orientation(object_yaw: float):
    """TCP orientation used by mujoco_oracle_anygrasp_node for a shelf grasp."""
    outward = (math.cos(object_yaw), math.sin(object_yaw))
    inward = (-outward[0], -outward[1])
    root_half = math.sqrt(0.5)
    return (-inward[1] * root_half, inward[0] * root_half, 0.0, root_half)


def _orientation_in_link(object_yaw: float):
    _, orientation = relative_pose(
        (0.0, 0.0, 0.0),
        _oracle_tcp_orientation(object_yaw),
        (0.0, 0.0, 0.0),
        _yaw_quaternion(object_yaw),
    )
    return orientation


def test_identity_attachment_is_the_original_g5_defect():
    """Reproduces the recorded failure: 10.4 mm of penetration."""
    clearance = attached_box_clearance(
        TCP_OFFSET_ALONG_LINK7_M, (0.0, 0.0, 0.0, 1.0), HALF_SIZE, LINK7_FORWARD_REACH_M
    )
    assert clearance == pytest.approx(-0.0104, abs=1e-4)


def test_real_relative_pose_clears_link7():
    """The object's true world orientation yields a 34.6 mm gap."""
    clearance = attached_box_clearance(
        TCP_OFFSET_ALONG_LINK7_M, _orientation_in_link(math.pi), HALF_SIZE, LINK7_FORWARD_REACH_M
    )
    assert clearance == pytest.approx(0.0346, abs=1e-4)
    assert clearance > 0.0


def test_clearance_holds_for_every_shelf_yaw():
    for degrees in range(-180, 181, 15):
        clearance = attached_box_clearance(
            TCP_OFFSET_ALONG_LINK7_M,
            _orientation_in_link(math.radians(degrees)),
            HALF_SIZE,
            LINK7_FORWARD_REACH_M,
        )
        assert clearance > 0.03, f"yaw {degrees} deg gave {clearance:.4f} m"


def test_identity_attachment_rotates_the_long_axis_onto_the_approach_axis():
    """Why the identity pose is wrong even beyond the clearance number."""
    aligned = axes_alignment((0.0, 0.0, 0.0, 1.0))
    assert aligned == pytest.approx((1.0, 1.0, 1.0), abs=1e-9)

    real = box_extent_along(_orientation_in_link(math.pi), HALF_SIZE, (0.0, 0.0, 1.0))
    identity = box_extent_along((0.0, 0.0, 0.0, 1.0), HALF_SIZE, (0.0, 0.0, 1.0))
    # The 0.16 m axis is 0.080 m half length, so identity shows that as the
    # backward extent; the correct orientation only shows the 0.035 m half.
    assert identity == pytest.approx(0.080, abs=1e-9)
    assert real == pytest.approx(0.035, abs=1e-9)


def test_tcp_z_points_at_the_shelf_for_the_oracle_orientation():
    """Documents the assumption the geometry fix depends on."""
    tcp = _oracle_tcp_orientation(math.pi)
    # Object yaw = pi puts its +x on free space, so the shelf is at +x in the
    # object frame and "inward" is +x. TCP +z follows the inward approach.
    assert rotate(tcp, (0.0, 0.0, 1.0)) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    # Free space is therefore behind the TCP, where the pregrasp pose sits.
    assert rotate(tcp, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_relative_pose_is_its_own_inverse():
    parent_position = (1.0, -2.0, 0.5)
    parent_orientation = _oracle_tcp_orientation(math.pi)
    child_position = (0.7, -2.1, 0.9)
    child_orientation = _yaw_quaternion(math.pi)

    offset, orientation = relative_pose(
        parent_position, parent_orientation, child_position, child_orientation
    )
    rotated = rotate(parent_orientation, offset)
    rebuilt = tuple(float(parent_position[i]) + rotated[i] for i in range(3))
    rebuilt_orientation = quaternion_multiply(parent_orientation, orientation)
    assert rebuilt == pytest.approx(child_position, abs=1e-9)
    assert rebuilt_orientation == pytest.approx(child_orientation, abs=1e-9)


def test_summary_reports_the_backward_extent_used_by_the_gate():
    summary = summarize_box(HALF_SIZE, _orientation_in_link(math.pi))
    assert summary["dimensions_m"] == pytest.approx([0.07, 0.06, 0.16], abs=1e-9)
    assert summary["extent_along_minus_z"] == pytest.approx(0.035, abs=1e-6)
