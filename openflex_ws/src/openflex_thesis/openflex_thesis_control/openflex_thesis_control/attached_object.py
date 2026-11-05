"""Pure geometry for an object rigidly attached to a robot link.

The attached collision object must be expressed in the frame of its parent
link. Placing it at the identity (as the first G5 implementation did) silently
rotates the object's long axis onto the link's own axis, which for the OpenFlex
wrist produces a real interpenetration between `openarmx_right_link7` and the
grasped box. Everything here is ROS-free so the failure mode stays unit-tested.

All quaternions are (x, y, z, w) and all rotations are active rotations of a
vector from the parent frame into the child frame.
"""

from __future__ import annotations

import itertools
import math
from typing import Iterable, Sequence, Tuple

Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


def quaternion_multiply(a: Sequence[float], b: Sequence[float]) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_conjugate(q: Sequence[float]) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_normalize(q: Sequence[float]) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0:
        raise ValueError("cannot normalize a zero quaternion")
    return tuple(float(value) / norm for value in q)  # type: ignore[return-value]


def rotate(q: Sequence[float], vector: Sequence[float]) -> Vector3:
    """Rotate `vector` by `q`."""
    rotated = quaternion_multiply(
        quaternion_multiply(q, (vector[0], vector[1], vector[2], 0.0)),
        quaternion_conjugate(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def relative_pose(
    parent_position: Sequence[float],
    parent_orientation: Sequence[float],
    child_position: Sequence[float],
    child_orientation: Sequence[float],
) -> Tuple[Vector3, Quaternion]:
    """Express a child pose in the parent frame."""
    inverse = quaternion_conjugate(quaternion_normalize(parent_orientation))
    delta = tuple(float(child_position[i]) - float(parent_position[i]) for i in range(3))
    return rotate(inverse, delta), quaternion_multiply(inverse, child_orientation)


def box_extent_along(
    orientation_in_parent: Sequence[float], half_size: Sequence[float], axis: Sequence[float]
) -> float:
    """Half-extent of an oriented box projected onto `axis`.

    For an axis-aligned box with half sizes `h`, the support distance along a
    unit vector `d` is `sum_i h_i * |d_i|` where `d_i` is the world direction of
    the box axis `i`. Taking the maximum over the eight corners gives the same
    number and is easier to trust.
    """
    best = 0.0
    for signs in itertools.product((-1.0, 1.0), repeat=3):
        corner = tuple(signs[i] * float(half_size[i]) for i in range(3))
        projected = sum(rotate(orientation_in_parent, corner)[i] * float(axis[i]) for i in range(3))
        best = max(best, abs(projected))
    return best


def box_axis_directions(
    orientation_in_parent: Sequence[float],
) -> Tuple[Vector3, Vector3, Vector3]:
    """The three box axes expressed in the parent frame."""
    return (
        rotate(orientation_in_parent, (1.0, 0.0, 0.0)),
        rotate(orientation_in_parent, (0.0, 1.0, 0.0)),
        rotate(orientation_in_parent, (0.0, 0.0, 1.0)),
    )


def axes_alignment(orientation_in_parent: Sequence[float]) -> Tuple[float, float, float]:
    """Largest |cosine| between each box axis and each parent axis.

    A rigidly attached object that is axis-aligned with its parent link should
    report a permutation-ish profile near 1.0; a box that was attached with a
    wrong relative rotation shows up here immediately.
    """
    values = []
    axes = box_axis_directions(orientation_in_parent)
    for parent_axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        values.append(max(abs(sum(axis[i] * parent_axis[i] for i in range(3))) for axis in axes))
    return tuple(values)  # type: ignore[return-value]


def attached_box_clearance(
    grasp_link_offset_along_z: float,
    orientation_in_link: Sequence[float],
    half_size: Sequence[float],
    link_forward_reach: float,
) -> float:
    """Clearance between a link's forward tip and the attached box's back face.

    `grasp_link_offset_along_z` is the distance along the link's +z from the
    link origin to the attach frame origin (the TCP). The link is modelled as
    reaching `link_forward_reach` metres along +z; the box back face sits at
    `offset - extent_along(-z)`. Positive clearance means no overlap.
    """
    back_extent = box_extent_along(orientation_in_link, half_size, (0.0, 0.0, -1.0))
    back_face = float(grasp_link_offset_along_z) - back_extent
    return back_face - float(link_forward_reach)


def summarize_box(
    half_size: Sequence[float], orientation_in_parent: Sequence[float]
) -> dict:
    """Human readable descriptor used in logs and acceptance evidence."""
    axes = box_axis_directions(orientation_in_parent)
    return {
        "dimensions_m": [2.0 * float(value) for value in half_size],
        "axis_directions": [[round(value, 6) for value in axis] for axis in axes],
        "parent_axis_alignment": [round(value, 6) for value in axes_alignment(orientation_in_parent)],
        "extent_along_plus_z": round(
            box_extent_along(orientation_in_parent, half_size, (0.0, 0.0, 1.0)), 6
        ),
        "extent_along_minus_z": round(
            box_extent_along(orientation_in_parent, half_size, (0.0, 0.0, -1.0)), 6
        ),
    }
