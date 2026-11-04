"""Deterministic object-centred base candidate generation."""

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence

from .geometry import PlanarPose, normalize_angle


def inclusive_values(start: float, stop: float, step: float) -> List[float]:
    if step <= 0.0:
        raise ValueError("step must be positive")
    if stop < start:
        raise ValueError("stop must not be smaller than start")
    count = int(math.floor((stop - start) / step + 1.0e-9))
    values = [start + index * step for index in range(count + 1)]
    if not values or values[-1] < stop - 1.0e-8:
        values.append(stop)
    return values


@dataclass(frozen=True)
class CandidateGeneratorConfig:
    distance_min_m: float = 0.55
    distance_max_m: float = 1.00
    distance_step_m: float = 0.10
    lateral_min_m: float = -0.40
    lateral_max_m: float = 0.40
    lateral_step_m: float = 0.10
    yaw_offsets_rad: Sequence[float] = (0.0,)

    def validate(self) -> None:
        inclusive_values(self.distance_min_m, self.distance_max_m, self.distance_step_m)
        inclusive_values(self.lateral_min_m, self.lateral_max_m, self.lateral_step_m)
        if not self.yaw_offsets_rad:
            raise ValueError("at least one yaw offset is required")


def generate_candidates(
    object_pose: PlanarPose, config: CandidateGeneratorConfig
) -> List[PlanarPose]:
    """Generate poses in front of the object and orient the base toward it.

    The object's +x axis points from the shelf toward free space. Candidates are
    placed along +x with a lateral offset in the object frame. If a detector uses
    another convention, its object-to-shelf calibration must adapt the pose first.
    """

    config.validate()
    c = math.cos(object_pose.yaw)
    s = math.sin(object_pose.yaw)
    result: List[PlanarPose] = []
    for distance in inclusive_values(
        config.distance_min_m, config.distance_max_m, config.distance_step_m
    ):
        for lateral in inclusive_values(
            config.lateral_min_m, config.lateral_max_m, config.lateral_step_m
        ):
            x = object_pose.x + c * distance - s * lateral
            y = object_pose.y + s * distance + c * lateral
            face_yaw = math.atan2(object_pose.y - y, object_pose.x - x)
            for yaw_offset in config.yaw_offsets_rad:
                result.append(PlanarPose(x, y, normalize_angle(face_yaw + yaw_offset)))
    return result
