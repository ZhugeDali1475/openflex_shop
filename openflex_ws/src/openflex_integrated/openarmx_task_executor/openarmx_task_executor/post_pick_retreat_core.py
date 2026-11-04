"""Planar odometry helpers for the post-pick shelf retreat."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RetreatProgress:
    """Motion relative to the base heading captured before retreating."""

    backward_distance: float
    lateral_drift: float
    yaw_error: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def measure_retreat(start: PlanarPose, current: PlanarPose) -> RetreatProgress:
    """Project an odometry displacement onto the initial base axes.

    ``backward_distance`` is positive only for motion behind the initial base
    heading. ``lateral_drift`` is signed positive to the initial base left.
    """

    delta_x = current.x - start.x
    delta_y = current.y - start.y
    forward_distance = delta_x * math.cos(start.yaw) + delta_y * math.sin(start.yaw)
    lateral_drift = -delta_x * math.sin(start.yaw) + delta_y * math.cos(start.yaw)
    return RetreatProgress(
        backward_distance=-forward_distance,
        lateral_drift=lateral_drift,
        yaw_error=normalize_angle(current.yaw - start.yaw),
    )
