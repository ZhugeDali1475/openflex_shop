"""Small dependency-free pose helpers shared by the simulator and tests."""

import math
from typing import Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # x, y, z, w


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_normalize(q: Sequence[float]) -> Quaternion:
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(value) / norm for value in q)  # type: ignore[return-value]


def quaternion_conjugate(q: Sequence[float]) -> Quaternion:
    return (-float(q[0]), -float(q[1]), -float(q[2]), float(q[3]))


def quaternion_multiply(a: Sequence[float], b: Sequence[float]) -> Quaternion:
    ax, ay, az, aw = (float(value) for value in a)
    bx, by, bz, bw = (float(value) for value in b)
    return quaternion_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(q: Sequence[float], vector: Sequence[float]) -> Vector3:
    qn = quaternion_normalize(q)
    vx, vy, vz = (float(value) for value in vector)
    # Expanded q * v * q^-1 avoids normalizing the pure-vector quaternion.
    qx, qy, qz, qw = qn
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def relative_pose(
    parent_position: Sequence[float],
    parent_orientation: Sequence[float],
    child_position: Sequence[float],
    child_orientation: Sequence[float],
) -> Tuple[Vector3, Quaternion]:
    inverse = quaternion_conjugate(quaternion_normalize(parent_orientation))
    delta = tuple(float(child_position[i]) - float(parent_position[i]) for i in range(3))
    return rotate_vector(inverse, delta), quaternion_multiply(inverse, child_orientation)


def compose_pose(
    parent_position: Sequence[float],
    parent_orientation: Sequence[float],
    child_position: Sequence[float],
    child_orientation: Sequence[float],
) -> Tuple[Vector3, Quaternion]:
    rotated = rotate_vector(parent_orientation, child_position)
    position = tuple(float(parent_position[i]) + rotated[i] for i in range(3))
    return position, quaternion_multiply(parent_orientation, child_orientation)  # type: ignore[arg-type]


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))

