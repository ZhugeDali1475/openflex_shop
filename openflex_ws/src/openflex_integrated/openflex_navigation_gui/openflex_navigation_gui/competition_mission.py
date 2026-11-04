"""Per-map semantic navigation stations for the supermarket competition."""

from __future__ import annotations

import math
import os
import re
from pathlib import Path
from typing import Sequence

import yaml

from .benchmark import BenchmarkWaypoint
from .map_editor_core import GridMap, MapMetadata
from .model import MapPaths


MISSION_FILENAME = "competition_mission.yaml"
MISSION_SCHEMA_VERSION = 2
STATION_ID_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")
STATION_ROLES = (
    ("起点", "start_position"),
    ("交付台", "delivery_table"),
    ("货架接近点", "shelf_approach"),
    ("补货台/补货箱", "replenishment"),
    ("巡查点", "patrol"),
    ("任务判定区", "judge_zone"),
    ("终点", "finish_position"),
    ("通用站点", "waypoint"),
)
ROLE_LABELS = {value: label for label, value in STATION_ROLES}


def mission_path(paths: MapPaths) -> Path:
    return paths.map_2d_dir / MISSION_FILENAME


def validate_station_id(value: str) -> str:
    station_id = value.strip()
    if not station_id or not STATION_ID_PATTERN.fullmatch(station_id):
        raise ValueError("站点 ID 只能包含字母、数字、点、下划线和短横线")
    return station_id


def save_mission(
    path: Path,
    map_name: str,
    stations: Sequence[BenchmarkWaypoint],
) -> None:
    ids = [validate_station_id(station.waypoint_id) for station in stations]
    if len(ids) != len(set(ids)):
        raise ValueError("站点 ID 不能重复")
    start_count = sum(station.role == "start_position" for station in stations)
    if start_count > 1:
        raise ValueError("每张地图最多只能有一个起点")

    document = {
        "schema_version": MISSION_SCHEMA_VERSION,
        "map_name": str(map_name),
        "stations": [
            {
                "id": station.waypoint_id,
                "label": station.label,
                "role": station.role,
                "pose": {
                    "x": float(station.x),
                    "y": float(station.y),
                    "yaw": float(station.yaw),
                },
                "action": {
                    "fine_align": {
                        "enabled": bool(station.requires_fine_alignment),
                        "mode": str(station.fine_align_mode),
                        "target_clearance_m": float(station.target_clearance_m),
                        "clearance_tolerance_m": float(
                            station.clearance_tolerance_m
                        ),
                        "yaw_tolerance_deg": math.degrees(
                            station.fine_align_yaw_tolerance_rad
                        ),
                        "timeout_sec": float(station.fine_align_timeout_sec),
                    },
                    "retreat": {
                        "after_pick": bool(station.should_retreat_after_pick),
                        "after_place": bool(station.should_retreat_after_place),
                        "distance_m": float(station.retreat_distance_m),
                        "speed_mps": float(station.retreat_speed_mps),
                    },
                },
            }
            for station in stations
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def load_mission(path: Path, expected_map_name: str) -> list[BenchmarkWaypoint]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    schema_version = int(document.get("schema_version", 0)) if isinstance(document, dict) else 0
    if schema_version not in (1, MISSION_SCHEMA_VERSION):
        raise ValueError("不支持的比赛站点配置版本")
    if str(document.get("map_name", "")) != expected_map_name:
        raise ValueError("比赛站点配置与当前地图名称不一致")

    stations = []
    seen = set()
    for raw in document.get("stations", []):
        station_id = validate_station_id(str(raw["id"]))
        if station_id in seen:
            raise ValueError(f"站点 ID 重复: {station_id}")
        seen.add(station_id)
        pose = raw["pose"]
        action = raw.get("action", {}) if schema_version >= 2 else {}
        if not isinstance(action, dict):
            raise ValueError(f"站点 {station_id} 的 action 必须是映射")
        fine_align = action.get("fine_align", {}) or {}
        retreat = action.get("retreat", {}) or {}
        if not isinstance(fine_align, dict) or not isinstance(retreat, dict):
            raise ValueError(f"站点 {station_id} 的动作策略格式无效")
        fine_align_enabled = (
            bool(fine_align.get("enabled", False)) if schema_version >= 2 else None
        )
        fine_align_mode = str(
            fine_align.get("mode", "front_surface_scan")
        ).strip().lower()
        if fine_align_mode == "shelf_scan":
            fine_align_mode = "front_surface_scan"
        if fine_align_mode not in ("front_surface_scan", "none"):
            raise ValueError(
                f"站点 {station_id} 使用不支持的精调模式 {fine_align_mode}"
            )
        stations.append(
            BenchmarkWaypoint(
                waypoint_id=station_id,
                label=str(raw.get("label", station_id)),
                x=float(pose["x"]),
                y=float(pose["y"]),
                yaw=float(pose["yaw"]),
                clearance_m=0.0,
                role=str(raw.get("role", "waypoint")),
                fine_align_enabled=fine_align_enabled,
                fine_align_mode=fine_align_mode,
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
                    bool(retreat.get("after_pick", False))
                    if schema_version >= 2
                    else None
                ),
                retreat_after_place=(
                    bool(retreat.get("after_place", False))
                    if schema_version >= 2
                    else None
                ),
                retreat_distance_m=float(retreat.get("distance_m", 0.15)),
                retreat_speed_mps=float(retreat.get("speed_mps", 0.04)),
            )
        )
    if sum(station.role == "start_position" for station in stations) > 1:
        raise ValueError("比赛站点配置包含多个起点")
    return stations


def grid_to_world(
    grid_x: int,
    grid_y: int,
    grid: GridMap,
    metadata: MapMetadata,
) -> tuple[float, float]:
    if metadata.resolution is None or metadata.origin is None:
        raise ValueError("地图 YAML 缺少 resolution 或 origin")
    origin_x, origin_y, _ = metadata.origin
    x = origin_x + (grid_x + 0.5) * metadata.resolution
    y = origin_y + (grid.height - grid_y - 0.5) * metadata.resolution
    return x, y


def world_to_grid(
    world_x: float,
    world_y: float,
    grid: GridMap,
    metadata: MapMetadata,
) -> tuple[int, int] | None:
    if metadata.resolution is None or metadata.origin is None:
        raise ValueError("地图 YAML 缺少 resolution 或 origin")
    origin_x, origin_y, _ = metadata.origin
    grid_x = math.floor((world_x - origin_x) / metadata.resolution)
    map_y = math.floor((world_y - origin_y) / metadata.resolution)
    grid_y = grid.height - 1 - map_y
    if 0 <= grid_x < grid.width and 0 <= grid_y < grid.height:
        return int(grid_x), int(grid_y)
    return None
