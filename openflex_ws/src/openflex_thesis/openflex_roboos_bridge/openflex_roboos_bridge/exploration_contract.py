"""ROS-free frontier / information-gain exploration contract (phase C2).

Chooses the next observation stand from the current occupancy map **without any
preset route or station list**: frontiers (free cells that border unknown space)
are clustered, scored by how much unknown area stands within sensor range, and
the best one is returned.  Every failure mode is fail-closed: lost localization,
an incomplete map with no frontier left, an unreachable goal, a deadline, a step
budget or a cancellation all end the search with a named, non-authorizing
outcome.

It is middleware-free and deterministic; the caller supplies the map snapshot,
the robot pose and a clock, so an exploration run can be replayed offline.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Mapping, Optional, Sequence, Tuple

from .mapping_contract import FREE, UNKNOWN, MapSnapshot, _finite

# Non-terminal / terminal outcomes.  Only ``next_position`` asks the caller to
# move; ``target_found`` is the only successful terminal outcome.
NEXT_POSITION = "next_position"
TARGET_FOUND = "target_found"
TARGET_NOT_FOUND = "target_not_found"
MAP_INCOMPLETE = "map_incomplete"
LOCALIZATION_LOST = "localization_lost"
TIMEOUT = "timeout"
STEPS_EXHAUSTED = "steps_exhausted"
CANCELLED = "cancelled"

_EIGHT = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


class ExplorationError(ValueError):
    """Raised when the exploration machine is driven out of contract."""


@dataclass(frozen=True)
class FrontierCluster:
    centroid_xy: Tuple[float, float]
    cell_count: int
    information_gain: float

    def to_dict(self) -> Mapping[str, object]:
        return {"centroid_xy": list(self.centroid_xy), "cell_count": int(self.cell_count),
                "information_gain": float(self.information_gain)}


@dataclass(frozen=True)
class FrontierCandidate:
    goal_xy: Tuple[float, float]
    goal_yaw_rad: float
    cell_count: int
    information_gain: float
    distance_m: float

    def to_dict(self) -> Mapping[str, object]:
        return {"goal_xy": list(self.goal_xy), "goal_yaw_rad": float(self.goal_yaw_rad),
                "cell_count": int(self.cell_count),
                "information_gain": float(self.information_gain),
                "distance_m": float(self.distance_m)}


@dataclass(frozen=True)
class ExplorationDecision:
    finished: bool
    outcome: str
    detail: str
    goal_xy: Optional[Tuple[float, float]] = None
    goal_yaw_rad: Optional[float] = None
    information_gain: Optional[float] = None
    map_version: Optional[int] = None
    steps: int = 0

    def to_dict(self) -> Mapping[str, object]:
        return {"finished": bool(self.finished), "outcome": self.outcome, "detail": self.detail,
                "goal_xy": list(self.goal_xy) if self.goal_xy is not None else None,
                "goal_yaw_rad": self.goal_yaw_rad,
                "information_gain": self.information_gain,
                "map_version": self.map_version, "steps": int(self.steps),
                "robot_motion_executed": False}


def _frontier_cells(snapshot: MapSnapshot) -> List[Tuple[int, int]]:
    cells: List[Tuple[int, int]] = []
    for j in range(snapshot.height):
        for i in range(snapshot.width):
            if snapshot.cell(i, j) != FREE:
                continue
            for di, dj in _EIGHT:
                ni, nj = i + di, j + dj
                if 0 <= ni < snapshot.width and 0 <= nj < snapshot.height \
                        and snapshot.cell(ni, nj) == UNKNOWN:
                    cells.append((i, j))
                    break
    return cells


def information_gain(snapshot: MapSnapshot, center_xy: Tuple[float, float], radius_m: float) -> float:
    """Count unknown cells within ``radius_m`` of ``center_xy`` (cell area weighted)."""

    radius = _finite(radius_m, "radius_m")
    cell_area = snapshot.resolution_m ** 2
    i0, j0 = snapshot.world_to_grid(*center_xy) or (0, 0)
    span = int(math.ceil(radius / snapshot.resolution_m))
    count = 0
    for j in range(j0 - span, j0 + span + 1):
        for i in range(i0 - span, i0 + span + 1):
            if not (0 <= i < snapshot.width and 0 <= j < snapshot.height):
                continue
            wx, wy = snapshot.grid_to_world(i, j)
            if math.hypot(wx - center_xy[0], wy - center_xy[1]) <= radius \
                    and snapshot.cell(i, j) == UNKNOWN:
                count += 1
    return count * cell_area


def find_frontier_clusters(snapshot: MapSnapshot, *, sensor_radius_m: float = 1.2,
                           min_cluster_size: int = 1) -> Tuple[FrontierCluster, ...]:
    """Cluster frontier cells and score each by information gain."""

    if int(min_cluster_size) < 1:
        raise ExplorationError("min_cluster_size must be at least 1")
    cells = _frontier_cells(snapshot)
    remaining = set(cells)
    clusters: List[FrontierCluster] = []
    while remaining:
        seed = min(remaining)
        stack, members = [seed], []
        remaining.discard(seed)
        while stack:
            i, j = stack.pop()
            members.append((i, j))
            for di, dj in _EIGHT:
                neighbor = (i + di, j + dj)
                if neighbor in remaining:
                    remaining.discard(neighbor)
                    stack.append(neighbor)
        if len(members) < int(min_cluster_size):
            continue
        mean_i = sum(cell[0] for cell in members) / float(len(members))
        mean_j = sum(cell[1] for cell in members) / float(len(members))
        centroid = snapshot.grid_to_world(mean_i, mean_j)
        clusters.append(FrontierCluster(
            centroid_xy=(float(centroid[0]), float(centroid[1])),
            cell_count=len(members),
            information_gain=information_gain(snapshot, centroid, sensor_radius_m)))
    clusters.sort(key=lambda cluster: (cluster.centroid_xy[0], cluster.centroid_xy[1]))
    return tuple(clusters)


def select_frontier(snapshot: MapSnapshot, robot_xy: Tuple[float, float], *,
                    sensor_radius_m: float = 1.2, min_cluster_size: int = 1,
                    max_reach_m: Optional[float] = None) -> Optional[FrontierCandidate]:
    """Pick the highest-information reachable frontier; nearest wins ties."""

    _finite(robot_xy[0], "robot_xy.x")
    _finite(robot_xy[1], "robot_xy.y")
    clusters = find_frontier_clusters(snapshot, sensor_radius_m=sensor_radius_m,
                                      min_cluster_size=min_cluster_size)
    best: Optional[FrontierCandidate] = None
    for cluster in clusters:
        distance = math.hypot(cluster.centroid_xy[0] - robot_xy[0],
                              cluster.centroid_xy[1] - robot_xy[1])
        if max_reach_m is not None and distance > float(max_reach_m):
            continue
        yaw = math.atan2(cluster.centroid_xy[1] - robot_xy[1],
                         cluster.centroid_xy[0] - robot_xy[0])
        candidate = FrontierCandidate(goal_xy=(float(cluster.centroid_xy[0]),
                                               float(cluster.centroid_xy[1])),
                                      goal_yaw_rad=float(yaw), cell_count=cluster.cell_count,
                                      information_gain=float(cluster.information_gain),
                                      distance_m=float(distance))
        if best is None or (candidate.information_gain, -candidate.distance_m) > \
                (best.information_gain, -best.distance_m):
            best = candidate
    return best


@dataclass
class ExplorationSearch:
    """Fail-closed exploration loop over successive map snapshots."""

    frame_id: str
    min_known_fraction: float = 0.6
    max_position_rmse_m: float = 0.05
    max_yaw_rmse_rad: float = 0.05
    deadline_sec: Optional[float] = None
    max_steps: int = 50
    sensor_radius_m: float = 1.2
    min_cluster_size: int = 1
    max_reach_m: Optional[float] = None
    history: list = field(default_factory=list)
    _steps: int = 0
    _finished: bool = False
    _outcome: Optional[str] = None
    _detail: str = "exploration not started"
    _cancelled: bool = False

    def __post_init__(self) -> None:
        if not str(self.frame_id).strip():
            raise ExplorationError("frame_id must be a non-empty string")
        if not 0.0 <= float(self.min_known_fraction) <= 1.0:
            raise ExplorationError("min_known_fraction must be within [0, 1]")
        for name in ("max_position_rmse_m", "max_yaw_rmse_rad"):
            if _finite(getattr(self, name), name) < 0.0:
                raise ExplorationError(f"{name} must be non-negative")
        if int(self.max_steps) < 1:
            raise ExplorationError("max_steps must be at least 1")
        if self.deadline_sec is not None and not math.isfinite(float(self.deadline_sec)):
            raise ExplorationError("deadline_sec must be finite when supplied")

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def outcome(self) -> Optional[str]:
        return self._outcome

    @property
    def steps(self) -> int:
        return self._steps

    def cancel(self, detail: str = "exploration cancelled") -> ExplorationDecision:
        self._cancelled, self._finished, self._outcome, self._detail = True, True, CANCELLED, detail
        return ExplorationDecision(True, CANCELLED, detail, steps=self._steps)

    def _finish(self, outcome: str, detail: str,
                snapshot: Optional[MapSnapshot] = None, **extra) -> ExplorationDecision:
        self._finished, self._outcome, self._detail = True, outcome, detail
        return ExplorationDecision(True, outcome, detail,
                                   map_version=snapshot.map_version if snapshot else None,
                                   steps=self._steps, **extra)

    def plan(self, snapshot: MapSnapshot, *, robot_xy: Tuple[float, float], now_sec: float,
             target_observed: bool = False) -> ExplorationDecision:
        """Consume one map snapshot and decide the next observation position."""

        if self._finished:
            raise ExplorationError("exploration already finished")
        if not isinstance(snapshot, MapSnapshot):
            raise ExplorationError("snapshot must be a MapSnapshot")
        if snapshot.frame_id != self.frame_id:
            raise ExplorationError(
                f"snapshot frame {snapshot.frame_id!r} does not match {self.frame_id!r}")
        _finite(now_sec, "now_sec")
        _finite(robot_xy[0], "robot_xy.x")
        _finite(robot_xy[1], "robot_xy.y")
        if not snapshot.localization.is_usable(max_position_rmse_m=self.max_position_rmse_m,
                                               max_yaw_rmse_rad=self.max_yaw_rmse_rad):
            return self._finish(
                LOCALIZATION_LOST,
                f"localization quality {snapshot.localization.source} is not usable "
                f"(rmse {snapshot.localization.position_rmse_m:.3f} m); refusing",
                snapshot)
        if target_observed:
            return self._finish(TARGET_FOUND, "target observed from the current stand", snapshot)
        if self.deadline_sec is not None and float(now_sec) > float(self.deadline_sec):
            return self._finish(TIMEOUT, "exploration deadline exceeded", snapshot)
        if self._steps >= int(self.max_steps):
            return self._finish(STEPS_EXHAUSTED,
                               f"step budget {int(self.max_steps)} reached", snapshot)
        candidate = select_frontier(snapshot, robot_xy, sensor_radius_m=self.sensor_radius_m,
                                    min_cluster_size=self.min_cluster_size,
                                    max_reach_m=self.max_reach_m)
        coverage = snapshot.coverage_fraction()
        if candidate is None:
            if coverage < float(self.min_known_fraction):
                return self._finish(
                    MAP_INCOMPLETE,
                    f"no frontier left but map coverage {coverage:.3f} < "
                    f"{self.min_known_fraction}; refusing", snapshot)
            return self._finish(TARGET_NOT_FOUND,
                               f"map complete (coverage {coverage:.3f}) and no target observed",
                               snapshot)
        self._steps += 1
        self.history.append({"step": self._steps, "map_version": snapshot.map_version,
                             "coverage_fraction": round(coverage, 6),
                             "goal_xy": [round(candidate.goal_xy[0], 6),
                                         round(candidate.goal_xy[1], 6)],
                             "information_gain": round(candidate.information_gain, 6),
                             "distance_m": round(candidate.distance_m, 6)})
        return ExplorationDecision(False, NEXT_POSITION,
                                   f"frontier at ({candidate.goal_xy[0]:.2f}, "
                                   f"{candidate.goal_xy[1]:.2f}) gain {candidate.information_gain:.3f}",
                                   goal_xy=candidate.goal_xy, goal_yaw_rad=candidate.goal_yaw_rad,
                                   information_gain=candidate.information_gain,
                                   map_version=snapshot.map_version, steps=self._steps)

    def report(self) -> Mapping[str, object]:
        return {"outcome": self._outcome, "finished": self._finished, "detail": self._detail,
                "steps": self._steps, "history": list(self.history),
                "physical_verification": False, "robot_motion_executed": False}
