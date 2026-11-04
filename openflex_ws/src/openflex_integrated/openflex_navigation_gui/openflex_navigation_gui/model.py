import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


@dataclass(frozen=True)
class MapPaths:
    root: Path
    name: str
    map_3d_dir: Path
    map_2d_dir: Path
    pcd: Path
    poses: Path
    yaml: Path
    pgm: Path


def validate_map_name(name: str) -> str:
    value = name.strip()
    if not value:
        raise ValueError("地图名称不能为空")
    if not MAP_NAME_PATTERN.fullmatch(value):
        raise ValueError("地图名称只能包含字母、数字、点、下划线和短横线")
    return value


def resolve_map_paths(root: str | Path, name: str) -> MapPaths:
    clean_name = validate_map_name(name)
    root_path = Path(root).expanduser().resolve()
    map_3d_dir = root_path / "3d" / clean_name
    map_2d_dir = root_path / "2d" / clean_name
    return MapPaths(
        root=root_path,
        name=clean_name,
        map_3d_dir=map_3d_dir,
        map_2d_dir=map_2d_dir,
        pcd=map_3d_dir / "map.pcd",
        poses=map_3d_dir / "poses.txt",
        yaml=map_2d_dir / "map.yaml",
        pgm=map_2d_dir / "map.pgm",
    )


def discover_map_names(root: str | Path) -> list[str]:
    root_path = Path(root).expanduser()
    names = set()
    for base, filename in ((root_path / "3d", "map.pcd"), (root_path / "2d", "map.yaml")):
        if not base.is_dir():
            continue
        for path in base.glob(f"*/{filename}"):
            names.add(path.parent.name)
    return sorted(names)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def nearest_path_error(
    x: float,
    y: float,
    yaw: float,
    path: Sequence[tuple[float, float, float]],
) -> tuple[float, float]:
    if not path:
        return 0.0, 0.0
    nearest = min(path, key=lambda point: math.hypot(x - point[0], y - point[1]))
    return math.hypot(x - nearest[0], y - nearest[1]), normalize_angle(yaw - nearest[2])


def command_text(program: str, arguments: Iterable[str]) -> str:
    tokens = [program, *arguments]
    return " ".join(_quote_token(token) for token in tokens)


def _quote_token(token: str) -> str:
    if token and all(char.isalnum() or char in "_./:=,[]+-" for char in token):
        return token
    return "'" + token.replace("'", "'\"'\"'") + "'"


RECORD_TOPICS = (
    "/tf",
    "/tf_static",
    "/map",
    "/initialpose",
    "/goal_pose",
    "/openflex_navigation_gui/goal_pose",
    "/plan",
    "/plan_smoothed",
    "/local_plan",
    "/predicted_path",
    "/astar_detour_path",
    "/odom",
    "/odom_safe",
    "/fastlio2/lio_odom",
    "/fastlio2/raw/odom",
    "/fastlio2/body_cloud",
    "/fastlio2/world_cloud",
    "/livox/body_cloud_safety",
    "/livox/body_cloud_nav2",
    "/livox/imu",
    "/cmd_vel_nav",
    "/cmd_vel",
    "/cmd_vel_safe_in",
    "/cmd_vel_safe",
    "/joint_states",
    "/dynamic_joint_states",
    "/swerve_drive_controller/debug_states",
    "/scan",
    "/local_costmap/costmap_raw",
    "/global_costmap/costmap_raw",
    "/global_costmap/voxel_marked_cloud",
    "/dyn_obj",
    "/dyn_obj_removed",
    "/dyn_obj_dbg_hist",
    "/cloud_effected",
    "/Laser_map",
    "/localization_ok",
    "/localization_hold",
    "/localization_costmap_cleared",
    "/collision_monitor/footprint_approach",
    "/speed_limit",
    "/diagnostics",
    "/rosout",
    "/behavior_tree_log",
    "/navigate_to_pose/_action/feedback",
    "/navigate_to_pose/_action/status",
    "/shelf_fine_align/_action/feedback",
    "/shelf_fine_align/_action/status",
    "/return_to_shelf_anchor/_action/feedback",
    "/return_to_shelf_anchor/_action/status",
    "/shelf_fine_align/localization_guard_state",
    "/follow_path/_action/feedback",
    "/follow_path/_action/status",
)
