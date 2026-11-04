"""Small, dependency-free geometry helpers used by the TC-WAM core."""

from dataclasses import dataclass
import math
from typing import Tuple


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def quaternion_multiply(first: Quaternion, second: Quaternion) -> Quaternion:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    return -value[0], -value[1], -value[2], value[3]


def rotate_vector(rotation: Quaternion, vector: Vector3) -> Vector3:
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(rotation, vector_quaternion),
        quaternion_conjugate(rotation),
    )
    return rotated[0], rotated[1], rotated[2]


def add_vector(first: Vector3, second: Vector3) -> Vector3:
    return first[0] + second[0], first[1] + second[1], first[2] + second[2]


def subtract_vector(first: Vector3, second: Vector3) -> Vector3:
    return first[0] - second[0], first[1] - second[1], first[2] - second[2]


def compose_planar(parent: PlanarPose, child: PlanarPose) -> PlanarPose:
    c = math.cos(parent.yaw)
    s = math.sin(parent.yaw)
    return PlanarPose(
        parent.x + c * child.x - s * child.y,
        parent.y + s * child.x + c * child.y,
        normalize_angle(parent.yaw + child.yaw),
    )


def relative_planar(reference: PlanarPose, target: PlanarPose) -> PlanarPose:
    dx = target.x - reference.x
    dy = target.y - reference.y
    c = math.cos(reference.yaw)
    s = math.sin(reference.yaw)
    return PlanarPose(
        c * dx + s * dy,
        -s * dx + c * dy,
        normalize_angle(target.yaw - reference.yaw),
    )


def planar_distance(first: PlanarPose, second: PlanarPose) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)
