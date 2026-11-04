"""Dependency-light validation and transform helpers for object observations."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

from .geometry import add_vector, quaternion_multiply, rotate_vector


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


def finite_vector(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def normalized_quaternion(values: Sequence[float]) -> Quaternion:
    if len(values) != 4 or not finite_vector(values):
        raise ValueError("orientation must contain four finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm < 1.0e-6:
        raise ValueError("orientation quaternion has near-zero norm")
    return tuple(float(value) / norm for value in values)  # type: ignore[return-value]


def transform_pose(
    position: Sequence[float],
    orientation: Sequence[float],
    translation_target_source: Sequence[float],
    rotation_target_source: Sequence[float],
) -> tuple[Vector3, Quaternion]:
    """Transform ``source`` pose into ``target`` using a target<-source transform."""

    if len(position) != 3 or not finite_vector(position):
        raise ValueError("position must contain three finite values")
    if len(translation_target_source) != 3 or not finite_vector(
        translation_target_source
    ):
        raise ValueError("transform translation must contain three finite values")
    source_orientation = normalized_quaternion(orientation)
    target_source_rotation = normalized_quaternion(rotation_target_source)
    target_position = add_vector(
        tuple(float(value) for value in translation_target_source),  # type: ignore[arg-type]
        rotate_vector(
            target_source_rotation,
            tuple(float(value) for value in position),  # type: ignore[arg-type]
        ),
    )
    target_orientation = normalized_quaternion(
        quaternion_multiply(target_source_rotation, source_orientation)
    )
    return target_position, target_orientation


def quaternion_rotation_matrix(values: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    x, y, z, w = normalized_quaternion(values)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def rotate_pose_covariance(
    covariance: Sequence[float], rotation_target_source: Sequence[float]
) -> list[float]:
    """Rotate a ROS 6x6 pose covariance into the target frame.

    Position and small-angle orientation errors are both rotated by the same
    3-D frame rotation. Cross-covariance terms are preserved.
    """

    if len(covariance) != 36 or not finite_vector(covariance):
        raise ValueError("pose covariance must contain 36 finite values")
    rotation = quaternion_rotation_matrix(rotation_target_source)
    transform = [[0.0 for _ in range(6)] for _ in range(6)]
    for block in (0, 3):
        for row in range(3):
            for column in range(3):
                transform[block + row][block + column] = rotation[row][column]
    source = [
        list(map(float, covariance[row * 6:(row + 1) * 6])) for row in range(6)
    ]
    intermediate = [
        [sum(transform[row][k] * source[k][column] for k in range(6)) for column in range(6)]
        for row in range(6)
    ]
    target = [
        [
            sum(intermediate[row][k] * transform[column][k] for k in range(6))
            for column in range(6)
        ]
        for row in range(6)
    ]
    return [target[row][column] for row in range(6) for column in range(6)]


def default_pose_covariance(position_variance: float, yaw_variance: float) -> list[float]:
    if not math.isfinite(position_variance) or position_variance <= 0.0:
        raise ValueError("position_variance must be positive and finite")
    if not math.isfinite(yaw_variance) or yaw_variance <= 0.0:
        raise ValueError("yaw_variance must be positive and finite")
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[14] = float(position_variance)
    covariance[21] = covariance[28] = float(yaw_variance)
    covariance[35] = float(yaw_variance)
    return covariance
