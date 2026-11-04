"""ROS-independent hard gates and soft TC-WAM score."""

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple

from .geometry import clamp


@dataclass(frozen=True)
class ScoreWeights:
    reachability: float = 0.22
    joint_margin: float = 0.18
    robustness: float = 0.25
    grasp_quality: float = 0.15
    visibility: float = 0.10
    move: float = 0.10

    def normalized(self) -> "ScoreWeights":
        values = (
            self.reachability,
            self.joint_margin,
            self.robustness,
            self.grasp_quality,
            self.visibility,
            self.move,
        )
        if any(value < 0.0 for value in values):
            raise ValueError("weights must be non-negative")
        total = sum(values)
        if total <= 0.0:
            raise ValueError("at least one score weight must be positive")
        return ScoreWeights(*(value / total for value in values))


@dataclass(frozen=True)
class CandidateMetrics:
    candidate_id: int
    x: float
    y: float
    yaw: float
    evaluated: bool
    ground_free: bool
    path_valid: bool
    ik_valid: bool
    collision_free: bool
    reachability: float
    joint_margin: float
    robustness: float
    move_score: float
    grasp_id: int = 0
    lift_seed: float = 0.0
    lift_position: float = 0.0
    planning_group: str = ""
    grasp_quality: float = 0.0
    visibility_score: float = 0.0


@dataclass(frozen=True)
class CandidateScore:
    candidate_id: int
    score: float
    feasible: bool
    rejection_reason: str


@dataclass(frozen=True)
class GridProjection:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: Tuple[int, ...]


def score_candidate(
    candidate: CandidateMetrics,
    weights: ScoreWeights,
    reachability_threshold: float,
    require_navigation: bool = False,
) -> CandidateScore:
    weights = weights.normalized()
    gates = (
        (candidate.evaluated, "not_evaluated"),
        (candidate.ground_free, "ground_collision"),
        (not require_navigation or candidate.path_valid, "no_navigation_path"),
        (candidate.ik_valid, "ik_invalid"),
        (candidate.collision_free, "collision"),
        (candidate.reachability >= reachability_threshold, "low_reachability"),
    )
    for passed, reason in gates:
        if not passed:
            return CandidateScore(candidate.candidate_id, 0.0, False, reason)
    score = (
        weights.reachability * clamp(candidate.reachability, 0.0, 1.0)
        + weights.joint_margin * clamp(candidate.joint_margin, 0.0, 1.0)
        + weights.robustness * clamp(candidate.robustness, 0.0, 1.0)
        + weights.grasp_quality * clamp(candidate.grasp_quality, 0.0, 1.0)
        + weights.visibility * clamp(candidate.visibility_score, 0.0, 1.0)
        + weights.move * clamp(candidate.move_score, 0.0, 1.0)
    )
    return CandidateScore(candidate.candidate_id, clamp(score, 0.0, 1.0), True, "")


def select_best(scores: Sequence[CandidateScore]) -> Optional[CandidateScore]:
    feasible = [score for score in scores if score.feasible]
    if not feasible:
        return None
    return max(feasible, key=lambda score: (score.score, -score.candidate_id))


def ready_candidate_ids(
    scores: Iterable[CandidateScore], absolute_threshold: float, relative_ratio: float
) -> List[int]:
    scores = list(scores)
    best = select_best(scores)
    if best is None:
        return []
    threshold = max(absolute_threshold, relative_ratio * best.score)
    return [score.candidate_id for score in scores if score.feasible and score.score >= threshold]


def conditioned_ready_candidate_ids(
    candidates: Sequence[CandidateMetrics],
    scores: Sequence[CandidateScore],
    best_candidate_id: int,
    absolute_threshold: float,
    relative_ratio: float,
    neighbor_distance_m: float,
    neighbor_yaw_rad: float,
    lift_seed_tolerance_m: float = 1.0e-4,
    lift_solution_tolerance_m: float = 0.02,
) -> List[int]:
    """Return the connected ready component bound to the best (grasp, lift seed).

    Score grids from other grasps or lift strata are deliberately excluded. The
    connected-component gate also prevents disjoint high-score islands from being
    presented to ACT as one target set.
    """

    if neighbor_distance_m <= 0.0 or neighbor_yaw_rad < 0.0:
        raise ValueError("ready-region neighborhood tolerances are invalid")
    metric_by_id = {item.candidate_id: item for item in candidates}
    score_by_id = {item.candidate_id: item for item in scores}
    best_metric = metric_by_id.get(best_candidate_id)
    best_score = score_by_id.get(best_candidate_id)
    if best_metric is None or best_score is None or not best_score.feasible:
        return []
    threshold = max(absolute_threshold, relative_ratio * best_score.score)
    eligible = {
        identifier
        for identifier, metric in metric_by_id.items()
        if metric.grasp_id == best_metric.grasp_id
        and metric.planning_group == best_metric.planning_group
        and abs(metric.lift_seed - best_metric.lift_seed) <= lift_seed_tolerance_m
        and abs(metric.lift_position - best_metric.lift_position) <= lift_solution_tolerance_m
        and identifier in score_by_id
        and score_by_id[identifier].feasible
        and score_by_id[identifier].score >= threshold
    }
    if best_candidate_id not in eligible:
        return []
    connected = {best_candidate_id}
    frontier = [best_candidate_id]
    while frontier:
        current = metric_by_id[frontier.pop()]
        for identifier in eligible - connected:
            other = metric_by_id[identifier]
            distance = math.hypot(other.x - current.x, other.y - current.y)
            yaw_delta = abs(math.atan2(math.sin(other.yaw - current.yaw), math.cos(other.yaw - current.yaw)))
            if distance <= neighbor_distance_m + 1.0e-9 and yaw_delta <= neighbor_yaw_rad:
                connected.add(identifier)
                frontier.append(identifier)
    return sorted(connected)


def project_score_grid(
    points: Sequence[Tuple[float, float, float, bool]],
    resolution: float,
    padding: float = 0.10,
) -> GridProjection:
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if not points:
        return GridProjection(resolution, 0.0, 0.0, 0, 0, tuple())
    min_x = min(point[0] for point in points) - padding
    max_x = max(point[0] for point in points) + padding
    min_y = min(point[1] for point in points) - padding
    max_y = max(point[1] for point in points) + padding
    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution
    width = max(1, int(math.ceil((max_x - origin_x) / resolution)) + 1)
    height = max(1, int(math.ceil((max_y - origin_y) / resolution)) + 1)
    data = [-1] * (width * height)
    for x, y, score, feasible in points:
        ix = int(math.floor((x - origin_x) / resolution))
        iy = int(math.floor((y - origin_y) / resolution))
        if 0 <= ix < width and 0 <= iy < height:
            value = int(round(100.0 * clamp(score, 0.0, 1.0))) if feasible else 0
            index = iy * width + ix
            data[index] = max(data[index], value)
    return GridProjection(resolution, origin_x, origin_y, width, height, tuple(data))
