"""Wall/simulation-time independent interpolation for trajectory Action adapters."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


@dataclass(frozen=True)
class TrajectoryPoint:
    time_sec: float
    positions: Tuple[float, ...]


class TrajectorySampler:
    def __init__(
        self,
        joint_names: Sequence[str],
        start_positions: Mapping[str, float],
        points: Iterable[TrajectoryPoint],
    ) -> None:
        self.joint_names = tuple(joint_names)
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("trajectory joint names must be non-empty and unique")
        self.start = tuple(float(start_positions[name]) for name in self.joint_names)
        if not all(math.isfinite(value) for value in self.start):
            raise ValueError('trajectory start positions must be finite')
        self.points: List[TrajectoryPoint] = list(points)
        if not self.points:
            raise ValueError("trajectory must contain at least one point")
        previous = -1.0
        for point in self.points:
            if not math.isfinite(point.time_sec) or not all(math.isfinite(value) for value in point.positions):
                raise ValueError('trajectory points and times must be finite')
            if point.time_sec < 0.0 or point.time_sec < previous:
                raise ValueError("trajectory point times must be non-negative and ordered")
            if len(point.positions) != len(self.joint_names):
                raise ValueError("trajectory point position count does not match joints")
            previous = point.time_sec

    @property
    def duration_sec(self) -> float:
        return self.points[-1].time_sec

    @staticmethod
    def _blend(a: Sequence[float], b: Sequence[float], alpha: float) -> Tuple[float, ...]:
        alpha = max(0.0, min(1.0, alpha))
        return tuple(float(x) + alpha * (float(y) - float(x)) for x, y in zip(a, b))

    def sample(self, elapsed_sec: float) -> Tuple[Dict[str, float], bool]:
        elapsed = max(0.0, float(elapsed_sec))
        previous_time = 0.0
        previous_positions = self.start
        for point in self.points:
            if elapsed <= point.time_sec:
                span = point.time_sec - previous_time
                alpha = 1.0 if span <= 1.0e-12 else (elapsed - previous_time) / span
                positions = self._blend(previous_positions, point.positions, alpha)
                return dict(zip(self.joint_names, positions)), False
            previous_time = point.time_sec
            previous_positions = point.positions
        return dict(zip(self.joint_names, self.points[-1].positions)), True


def duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def measured_goal_reached(names, target, positions, velocities):
    if not names or any(len(values) != len(names) for values in [target, positions, velocities]):
        return False
    if not all(math.isfinite(value) for values in [target, positions, velocities] for value in values):
        return False
    return all(abs(actual - requested) <= (0.005 if name == 'lift_joint' else 0.01)
               and abs(velocity) <= (0.01 if name == 'lift_joint' else 0.03)
               for name, requested, actual, velocity in zip(names, target, positions, velocities))
