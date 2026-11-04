"""Competition-navigation benchmark profile loading and result evaluation."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence

import yaml


@dataclass(frozen=True)
class BenchmarkWaypoint:
    waypoint_id: str
    label: str
    x: float
    y: float
    yaw: float
    clearance_m: float
    role: str
    # Per-map station action policy. ``None`` keeps legacy mission/profile
    # files compatible: shelf approach points align by default, while other
    # roles do not. Newly saved competition missions always write an explicit
    # boolean so role and runtime behaviour are no longer coupled.
    fine_align_enabled: bool | None = None
    fine_align_mode: str = "front_surface_scan"
    target_clearance_m: float = 0.05
    clearance_tolerance_m: float = 0.01
    fine_align_yaw_tolerance_rad: float = 0.017453292519943295
    fine_align_timeout_sec: float = 20.0
    retreat_after_pick: bool | None = None
    retreat_after_place: bool | None = None
    retreat_distance_m: float = 0.15
    retreat_speed_mps: float = 0.04

    def __post_init__(self) -> None:
        mode = self.fine_align_mode.strip().lower()
        if mode == "shelf_scan":
            mode = "front_surface_scan"
        if mode not in ("front_surface_scan", "none"):
            raise ValueError(f"不支持的精调模式: {self.fine_align_mode}")
        object.__setattr__(self, "fine_align_mode", mode)
        if not 0.035 <= self.target_clearance_m <= 0.45:
            raise ValueError("精调目标净距必须在 0.035~0.45 m 内")
        if not 0.0 < self.clearance_tolerance_m < self.target_clearance_m:
            raise ValueError("精调距离容差必须大于 0 且小于目标净距")
        if not math.radians(0.5) <= self.fine_align_yaw_tolerance_rad <= math.radians(5.0):
            raise ValueError("精调角度容差必须在 0.5~5.0 度内")
        if not 0.0 < self.fine_align_timeout_sec <= 60.0:
            raise ValueError("精调超时必须在 0~60 秒内")
        if not 0.0 < self.retreat_distance_m <= 0.30:
            raise ValueError("退避距离必须在 0~0.30 m 内")
        if not 0.0 < self.retreat_speed_mps <= 0.06:
            raise ValueError("退避速度必须在 0~0.06 m/s 内")

    @property
    def requires_fine_alignment(self) -> bool:
        if self.fine_align_enabled is not None:
            return self.fine_align_enabled
        return self.role == "shelf_approach"

    @property
    def should_retreat_after_pick(self) -> bool:
        if self.retreat_after_pick is not None:
            return self.retreat_after_pick
        return self.role == "shelf_approach"

    @property
    def should_retreat_after_place(self) -> bool:
        if self.retreat_after_place is not None:
            return self.retreat_after_place
        return self.role in ("delivery_table", "replenishment")


@dataclass(frozen=True)
class BenchmarkAcceptance:
    goal_position_m: float
    goal_yaw_rad: float
    max_goal_position_m: float
    max_goal_yaw_rad: float
    path_cross_track_m: float
    max_icp_step_m: float
    max_icp_step_yaw_rad: float
    stopped_linear_mps: float
    stopped_angular_radps: float
    stopped_hold_sec: float
    minimum_success_rate: float
    required_zero_collision_tours: int


@dataclass(frozen=True)
class BenchmarkTour:
    tour_id: str
    label: str
    sequence: tuple[str, ...]
    settle_sec: float


@dataclass(frozen=True)
class BenchmarkProfile:
    profile_id: str
    display_name: str
    purpose: str
    map_root: Path
    map_name: str
    launch_file: str
    initial_x: float
    initial_y: float
    initial_yaw: float
    acceptance: BenchmarkAcceptance
    waypoints: Mapping[str, BenchmarkWaypoint]
    tours: Mapping[str, BenchmarkTour]
    transport_length_m: float = 0.92
    transport_width_m: float = 0.90
    transport_padding_m: float = 0.04
    goal_obstacle_clearance_m: float = 0.05


@dataclass(frozen=True)
class GoalEvaluation:
    passed: bool
    position_error_m: float
    yaw_error_rad: float


def _positive_float(mapping: Mapping, key: str) -> float:
    value = float(mapping[key])
    if value <= 0.0:
        raise ValueError(f"{key} 必须大于 0")
    return value


def load_benchmark_profile(path: str | Path) -> BenchmarkProfile:
    source = Path(path).expanduser()
    with source.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or int(data.get("schema_version", 0)) != 1:
        raise ValueError("不支持的比赛基准配置 schema_version")

    profile_data = data["profile"]
    acceptance_data = data["acceptance"]
    initial_pose = profile_data["initial_pose"]
    acceptance = BenchmarkAcceptance(
        goal_position_m=_positive_float(acceptance_data, "goal_position_m"),
        goal_yaw_rad=_positive_float(acceptance_data, "goal_yaw_rad"),
        max_goal_position_m=_positive_float(
            acceptance_data, "max_goal_position_m"
        ),
        max_goal_yaw_rad=_positive_float(acceptance_data, "max_goal_yaw_rad"),
        path_cross_track_m=_positive_float(acceptance_data, "path_cross_track_m"),
        max_icp_step_m=_positive_float(acceptance_data, "max_icp_step_m"),
        max_icp_step_yaw_rad=_positive_float(
            acceptance_data, "max_icp_step_yaw_rad"
        ),
        stopped_linear_mps=_positive_float(
            acceptance_data, "stopped_linear_mps"
        ),
        stopped_angular_radps=_positive_float(
            acceptance_data, "stopped_angular_radps"
        ),
        stopped_hold_sec=_positive_float(acceptance_data, "stopped_hold_sec"),
        minimum_success_rate=float(acceptance_data["minimum_success_rate"]),
        required_zero_collision_tours=int(
            acceptance_data["required_zero_collision_tours"]
        ),
    )
    if not 0.0 < acceptance.minimum_success_rate <= 1.0:
        raise ValueError("minimum_success_rate 必须在 (0, 1] 内")

    waypoints = {}
    for waypoint_id, waypoint_data in data["waypoints"].items():
        pose = waypoint_data["pose"]
        action = waypoint_data.get("action", {}) or {}
        fine_align = action.get("fine_align", {}) or {}
        retreat = action.get("retreat", {}) or {}
        mode = str(fine_align.get("mode", "front_surface_scan")).strip().lower()
        if mode == "shelf_scan":
            mode = "front_surface_scan"
        waypoints[waypoint_id] = BenchmarkWaypoint(
            waypoint_id=waypoint_id,
            label=str(waypoint_data["label"]),
            x=float(pose["x"]),
            y=float(pose["y"]),
            yaw=float(pose["yaw"]),
            clearance_m=_positive_float(waypoint_data, "clearance_m"),
            role=str(waypoint_data.get("role", "waypoint")),
            fine_align_enabled=(
                None
                if "enabled" not in fine_align
                else bool(fine_align["enabled"])
            ),
            fine_align_mode=mode,
            target_clearance_m=float(
                fine_align.get("target_clearance_m", 0.05)
            ),
            clearance_tolerance_m=float(
                fine_align.get("clearance_tolerance_m", 0.01)
            ),
            fine_align_yaw_tolerance_rad=math.radians(
                float(fine_align.get("yaw_tolerance_deg", 1.0))
            ),
            fine_align_timeout_sec=float(fine_align.get("timeout_sec", 20.0)),
            retreat_after_pick=(
                None
                if "after_pick" not in retreat
                else bool(retreat["after_pick"])
            ),
            retreat_after_place=(
                None
                if "after_place" not in retreat
                else bool(retreat["after_place"])
            ),
            retreat_distance_m=float(retreat.get("distance_m", 0.15)),
            retreat_speed_mps=float(retreat.get("speed_mps", 0.04)),
        )
    if not waypoints:
        raise ValueError("比赛基准配置至少需要一个航点")

    tours = {}
    for tour_id, tour_data in data["tours"].items():
        sequence = tuple(str(item) for item in tour_data["sequence"])
        missing = [item for item in sequence if item not in waypoints]
        if missing:
            raise ValueError(f"巡游 {tour_id} 引用了未知航点: {missing}")
        if not sequence:
            raise ValueError(f"巡游 {tour_id} 不能为空")
        tours[tour_id] = BenchmarkTour(
            tour_id=tour_id,
            label=str(tour_data["label"]),
            sequence=sequence,
            settle_sec=float(tour_data.get("settle_sec", 1.0)),
        )

    map_root = Path(str(profile_data["map_root"])).expanduser()
    footprint_data = profile_data.get("transport_footprint_m", {})
    transport_length_m = float(footprint_data.get("length", 0.92))
    transport_width_m = float(footprint_data.get("width", 0.90))
    transport_padding_m = float(footprint_data.get("padding", 0.04))
    goal_obstacle_clearance_m = float(
        profile_data.get("goal_obstacle_clearance_m", 0.05)
    )
    if transport_length_m <= 0.0 or transport_width_m <= 0.0:
        raise ValueError("transport_footprint_m 长宽必须大于 0")
    if transport_padding_m < 0.0 or goal_obstacle_clearance_m < 0.0:
        raise ValueError("足迹 padding 和目标障碍净距不能小于 0")
    return BenchmarkProfile(
        profile_id=str(profile_data["id"]),
        display_name=str(profile_data["display_name"]),
        purpose=str(profile_data["purpose"]),
        map_root=map_root,
        map_name=str(profile_data["map_name"]),
        launch_file=str(profile_data["launch_file"]),
        initial_x=float(initial_pose["x"]),
        initial_y=float(initial_pose["y"]),
        initial_yaw=float(initial_pose["yaw"]),
        acceptance=acceptance,
        waypoints=waypoints,
        tours=tours,
        transport_length_m=transport_length_m,
        transport_width_m=transport_width_m,
        transport_padding_m=transport_padding_m,
        goal_obstacle_clearance_m=goal_obstacle_clearance_m,
    )


def expand_tour(
    profile: BenchmarkProfile, tour_id: str, repeats: int
) -> list[BenchmarkWaypoint]:
    if repeats < 1:
        raise ValueError("巡游次数必须至少为 1")
    tour = profile.tours[tour_id]
    return [
        profile.waypoints[waypoint_id]
        for _ in range(repeats)
        for waypoint_id in tour.sequence
    ]


def evaluate_goal(
    acceptance: BenchmarkAcceptance,
    position_error_m: float,
    yaw_error_rad: float,
) -> GoalEvaluation:
    position = abs(float(position_error_m))
    yaw = abs(float(yaw_error_rad))
    return GoalEvaluation(
        passed=(
            position <= acceptance.goal_position_m
            and yaw <= acceptance.goal_yaw_rad
        ),
        position_error_m=position,
        yaw_error_rad=yaw,
    )


def waypoint_landmarks(
    waypoints: Sequence[BenchmarkWaypoint],
) -> list[tuple[float, float, str]]:
    grouped: dict[tuple[float, float], list[str]] = {}
    for waypoint in waypoints:
        grouped.setdefault((waypoint.x, waypoint.y), []).append(waypoint.waypoint_id)
    return [
        (x, y, "/".join(labels)) for (x, y), labels in grouped.items()
    ]


def find_waypoint_by_role(
    profile: BenchmarkProfile, role: str
) -> BenchmarkWaypoint | None:
    matches = [
        waypoint
        for waypoint in profile.waypoints.values()
        if waypoint.role == role
    ]
    if len(matches) > 1:
        raise ValueError(f"航点角色 {role} 必须唯一")
    return matches[0] if matches else None
