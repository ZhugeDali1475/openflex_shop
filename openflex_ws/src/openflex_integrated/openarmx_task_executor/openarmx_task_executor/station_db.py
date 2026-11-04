"""Station and object-slot configuration loaded from YAML."""

from dataclasses import dataclass, field
import copy
from pathlib import Path
from typing import Any, Dict, ItemsView, Optional
import math

import yaml
from geometry_msgs.msg import PoseStamped, Quaternion


_REMOVED_GRASP_STRATEGIES = {"anygrasp", "mock_or_anygrasp"}


def _grasp_strategy(value: Any, station_id: str, object_id: str) -> str:
    strategy = str(value if value is not None else "default").strip().lower()
    if not strategy:
        raise ValueError(
            f"slot '{object_id}' in '{station_id}' has an empty grasp_strategy"
        )
    if strategy in _REMOVED_GRASP_STRATEGIES:
        raise ValueError(
            f"slot '{object_id}' in '{station_id}' uses removed grasp_strategy "
            f"'{strategy}'; use 'graspnet_shelf', 'graspnet_basket', or 'mock'"
        )
    return strategy


def _number(value: Any, default: float = 0.0) -> float:
    return float(value) if value is not None else default


@dataclass(frozen=True)
class PoseSpec:
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @classmethod
    def from_mapping(cls, value: Dict[str, Any], default_frame: str) -> "PoseSpec":
        if not isinstance(value, dict):
            raise ValueError("pose must be a mapping")
        frame_id = str(value.get("frame_id", default_frame))
        if not frame_id:
            raise ValueError("pose.frame_id must not be empty")

        orientation = value.get("orientation")
        if orientation is None:
            # A yaw-only representation keeps station files readable.
            import math

            yaw = _number(value.get("yaw"))
            qx = 0.0
            qy = 0.0
            qz = math.sin(yaw * 0.5)
            qw = math.cos(yaw * 0.5)
        else:
            qx = _number(orientation.get("x"))
            qy = _number(orientation.get("y"))
            qz = _number(orientation.get("z"))
            qw = _number(orientation.get("w"), 1.0)

        return cls(
            frame_id=frame_id,
            x=_number(value.get("x")),
            y=_number(value.get("y")),
            z=_number(value.get("z")),
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

    def to_msg(self, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        msg.pose.orientation = Quaternion(
            x=self.qx, y=self.qy, z=self.qz, w=self.qw
        )
        return msg

    @property
    def yaw(self) -> float:
        return math.atan2(
            2.0 * (self.qw * self.qz + self.qx * self.qy),
            1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz),
        )


@dataclass(frozen=True)
class SlotSpec:
    pose: PoseSpec
    arm_name: str = ""
    grasp_strategy: str = "default"
    # -1 means that the slot is not tied to a known shelf layer.  Dynamic
    # slots supplied by the external inventory database carry their layer in
    # ExecuteTransport instead.
    level: int = -1
    nominal_lift_position: Optional[float] = None


@dataclass(frozen=True)
class StationSpec:
    station_id: str
    nav_pose: PoseSpec
    marker_id: int = -1
    docking_enabled: bool = True
    alignment_mode: str = "marker"
    position_tolerance: float = 0.03
    yaw_tolerance: float = 0.05
    target_marker_pose: Optional[PoseSpec] = None
    shelf_target_clearance: float = 0.05
    shelf_clearance_tolerance: float = 0.01
    shelf_alignment_timeout: float = 20.0
    # Reverse motion is opt-in per measured close station.
    retreat_after_pick: bool = False
    retreat_after_place: bool = False
    retreat_distance: float = 0.10
    retreat_speed: float = 0.04
    slots: Dict[str, SlotSpec] = field(default_factory=dict)
    place_pose: Optional[PoseSpec] = None
    place_strategy: str = "default"
    # Calibrated lift_joint positions, deliberately optional.  A missing
    # entry must not be replaced by a guessed motor coordinate.
    level_lift_positions: Dict[int, float] = field(default_factory=dict)
    # Optional calibrated arm joint targets paired with each shelf level.
    # The extra arm key keeps right/left calibration independent.
    level_arm_positions: Dict[int, Dict[str, Dict[str, float]]] = field(
        default_factory=dict
    )


class StationDatabase:
    """Immutable station database used by the task executor."""

    def __init__(self, stations: Dict[str, StationSpec]):
        self._stations = stations

    @classmethod
    def from_yaml(
        cls, path: str, mission_path: str = ""
    ) -> "StationDatabase":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        station_data = copy.deepcopy(raw.get("stations", raw))
        if not isinstance(station_data, dict) or not station_data:
            raise ValueError("stations.yaml must contain a non-empty 'stations' mapping")
        if mission_path:
            station_data = cls._overlay_mission(station_data, mission_path)

        stations: Dict[str, StationSpec] = {}
        for station_id, value in station_data.items():
            if not isinstance(value, dict):
                raise ValueError(f"station '{station_id}' must be a mapping")
            nav_value = value.get("nav_pose")
            if nav_value is None:
                raise ValueError(f"station '{station_id}' is missing nav_pose")
            nav_pose = PoseSpec.from_mapping(nav_value, "map")
            dock = value.get("dock", {}) or {}
            alignment = value.get("alignment", dock) or {}
            alignment_mode = str(alignment.get("mode", "marker")).strip().lower()
            if alignment_mode not in ("marker", "shelf_scan", "none"):
                raise ValueError(
                    f"station '{station_id}' has unsupported alignment mode "
                    f"'{alignment_mode}'"
                )
            target_marker_pose = None
            marker_pose_value = alignment.get(
                "target_marker_pose", dock.get("target_marker_pose")
            )
            if marker_pose_value is not None:
                target_marker_pose = PoseSpec.from_mapping(
                    marker_pose_value, "base_link"
                )
            retreat = value.get("retreat", {}) or {}
            retreat_distance = float(retreat.get("distance", 0.10))
            if retreat_distance <= 0.0:
                raise ValueError(f"station '{station_id}' retreat.distance must be positive")
            retreat_speed = float(retreat.get("speed", 0.04))
            if not 0.0 < retreat_speed <= 0.06:
                raise ValueError(
                    f"station '{station_id}' retreat.speed must be in (0, 0.06]"
                )
            slots: Dict[str, SlotSpec] = {}
            for object_id, slot_value in (value.get("slots", {}) or {}).items():
                if not isinstance(slot_value, dict):
                    raise ValueError(f"slot '{object_id}' in '{station_id}' must be a mapping")
                slot_pose = slot_value.get("pose", slot_value)
                slots[str(object_id)] = SlotSpec(
                    pose=PoseSpec.from_mapping(slot_pose, str(station_id)),
                    arm_name=str(slot_value.get("arm_name", "")),
                    grasp_strategy=_grasp_strategy(
                        slot_value.get("grasp_strategy", "default"),
                        str(station_id),
                        str(object_id),
                    ),
                    level=int(slot_value.get("level", -1)),
                    nominal_lift_position=(
                        None
                        if slot_value.get("nominal_lift_position") is None
                        else float(slot_value["nominal_lift_position"])
                    ),
                )

            place_pose = None
            if value.get("place_pose") is not None:
                place_pose = PoseSpec.from_mapping(value["place_pose"], str(station_id))
            level_lift_positions: Dict[int, float] = {}
            for level, lift_position in (value.get("level_lift_positions", {}) or {}).items():
                level_lift_positions[int(level)] = float(lift_position)
            level_arm_positions: Dict[int, Dict[str, Dict[str, float]]] = {}
            for level, arms in (value.get("level_arm_positions", {}) or {}).items():
                if not isinstance(arms, dict):
                    raise ValueError(
                        f"station '{station_id}' level_arm_positions[{level}] "
                        "must be a mapping"
                    )
                parsed_arms: Dict[str, Dict[str, float]] = {}
                for arm_name, joints in arms.items():
                    normalized_arm = str(arm_name).strip().lower()
                    if normalized_arm not in ("left", "right") or not isinstance(joints, dict):
                        raise ValueError(
                            f"station '{station_id}' shelf level {level} has an "
                            f"invalid arm posture '{arm_name}'"
                        )
                    parsed_joints: Dict[str, float] = {}
                    for joint_name, position in joints.items():
                        name = str(joint_name).strip()
                        value_float = float(position)
                        if not name or not math.isfinite(value_float):
                            raise ValueError(
                                f"station '{station_id}' shelf level {level} has "
                                "an invalid arm joint target"
                            )
                        parsed_joints[name] = value_float
                    if not parsed_joints:
                        raise ValueError(
                            f"station '{station_id}' shelf level {level} has an "
                            f"empty {normalized_arm} arm posture"
                        )
                    parsed_arms[normalized_arm] = parsed_joints
                level_arm_positions[int(level)] = parsed_arms
            stations[str(station_id)] = StationSpec(
                station_id=str(station_id),
                nav_pose=nav_pose,
                marker_id=int(value.get("marker_id", -1)),
                docking_enabled=(
                    alignment_mode != "none"
                    and bool(alignment.get("enabled", dock.get("enabled", True)))
                ),
                alignment_mode=alignment_mode,
                position_tolerance=float(
                    alignment.get("position_tolerance", dock.get("position_tolerance", 0.03))
                ),
                yaw_tolerance=float(
                    alignment.get("yaw_tolerance", dock.get("yaw_tolerance", 0.05))
                ),
                target_marker_pose=target_marker_pose,
                shelf_target_clearance=float(
                    alignment.get("target_clearance", 0.05)
                ),
                shelf_clearance_tolerance=float(
                    alignment.get("clearance_tolerance", 0.01)
                ),
                shelf_alignment_timeout=float(alignment.get("timeout_sec", 20.0)),
                retreat_after_pick=bool(retreat.get("after_pick", False)),
                retreat_after_place=bool(retreat.get("after_place", False)),
                retreat_distance=retreat_distance,
                retreat_speed=retreat_speed,
                slots=slots,
                place_pose=place_pose,
                place_strategy=str(value.get("place_strategy", "default")),
                level_lift_positions=level_lift_positions,
                level_arm_positions=level_arm_positions,
            )
        return cls(stations)

    @staticmethod
    def _overlay_mission(
        station_data: Dict[str, Any], mission_path: str
    ) -> Dict[str, Any]:
        """Overlay per-map navigation/action policy on calibrated station data.

        Calibration-only fields (slots, arm/lift teaching and place poses) stay
        in the station database. The map-owned mission is authoritative for
        navigation poses, fine alignment and post-operation retreat.
        """
        mission_file = Path(mission_path)
        mission = yaml.safe_load(mission_file.read_text(encoding="utf-8")) or {}
        if not isinstance(mission, dict):
            raise ValueError("competition mission must be a mapping")
        schema_version = int(mission.get("schema_version", 0))
        if schema_version not in (1, 2):
            raise ValueError("competition mission schema_version must be 1 or 2")
        raw_stations = mission.get("stations", [])
        if not isinstance(raw_stations, list):
            raise ValueError("competition mission stations must be a list")

        merged = copy.deepcopy(station_data)
        seen = set()
        for raw_station in raw_stations:
            if not isinstance(raw_station, dict):
                raise ValueError("competition mission station must be a mapping")
            station_id = str(raw_station.get("id", "")).strip()
            if not station_id or station_id in seen:
                raise ValueError(
                    "competition mission station IDs must be non-empty and unique"
                )
            seen.add(station_id)
            pose = raw_station.get("pose")
            if not isinstance(pose, dict):
                raise ValueError(f"mission station '{station_id}' is missing pose")
            station = copy.deepcopy(merged.get(station_id, {}))
            station["nav_pose"] = {
                "frame_id": "map",
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": 0.0,
                "yaw": float(pose["yaw"]),
            }

            role = str(raw_station.get("role", "waypoint"))
            if schema_version >= 2:
                action = raw_station.get("action", {}) or {}
                fine_align = action.get("fine_align", {}) or {}
                retreat = action.get("retreat", {}) or {}
                if not isinstance(fine_align, dict) or not isinstance(retreat, dict):
                    raise ValueError(
                        f"mission station '{station_id}' has invalid action policy"
                    )
                enabled = bool(fine_align.get("enabled", False))
                mode = str(
                    fine_align.get("mode", "front_surface_scan")
                ).strip().lower()
                if mode in ("front_surface_scan", "shelf_scan"):
                    task_mode = "shelf_scan" if enabled else "none"
                elif mode == "none":
                    task_mode = "none"
                else:
                    raise ValueError(
                        f"mission station '{station_id}' has unsupported "
                        f"fine-align mode '{mode}'"
                    )
                station["alignment"] = {
                    "enabled": enabled,
                    "mode": task_mode,
                    "target_clearance": float(
                        fine_align.get("target_clearance_m", 0.05)
                    ),
                    "clearance_tolerance": float(
                        fine_align.get("clearance_tolerance_m", 0.01)
                    ),
                    "yaw_tolerance": math.radians(
                        float(fine_align.get("yaw_tolerance_deg", 3.0))
                    ),
                    "timeout_sec": float(fine_align.get("timeout_sec", 20.0)),
                }
                station["retreat"] = {
                    "after_pick": bool(retreat.get("after_pick", False)),
                    "after_place": bool(retreat.get("after_place", False)),
                    "distance": float(retreat.get("distance_m", 0.15)),
                    "speed": float(retreat.get("speed_mps", 0.04)),
                }
            elif station_id not in merged:
                is_shelf = role == "shelf_approach"
                is_counter = role in ("delivery_table", "replenishment")
                station["alignment"] = {
                    "enabled": is_shelf,
                    "mode": "shelf_scan" if is_shelf else "none",
                    "target_clearance": 0.05,
                    "clearance_tolerance": 0.01,
                    "yaw_tolerance": math.radians(3.0),
                    "timeout_sec": 20.0,
                }
                station["retreat"] = {
                    "after_pick": is_shelf,
                    "after_place": is_counter,
                    "distance": 0.15,
                    "speed": 0.04,
                }
            merged[station_id] = station
        return merged

    def get(self, station_id: str) -> Optional[StationSpec]:
        return self._stations.get(station_id)

    def items(self) -> ItemsView[str, StationSpec]:
        return self._stations.items()

    def slot(self, station_id: str, object_id: str) -> Optional[SlotSpec]:
        station = self.get(station_id)
        if station is None:
            return None
        return station.slots.get(object_id) or station.slots.get("*")

    def exact_slot(self, station_id: str, slot_id: str) -> Optional[SlotSpec]:
        """Return a destination slot without silently falling back to '*'."""
        station = self.get(station_id)
        if station is None or not slot_id:
            return None
        return station.slots.get(str(slot_id))

    def level_lift_position(
        self, station_id: str, slot: Optional[SlotSpec], level: int
    ) -> Optional[float]:
        """Return only an explicitly calibrated lift position for a layer."""
        if slot is not None and slot.nominal_lift_position is not None:
            return slot.nominal_lift_position
        station = self.get(station_id)
        if station is None or level < 0:
            return None
        return station.level_lift_positions.get(int(level))

    def level_arm_joint_positions(
        self, station_id: str, level: int, arm_name: str
    ) -> Dict[str, float]:
        """Return a copy of the explicitly calibrated arm target for a layer."""
        station = self.get(station_id)
        normalized_arm = str(arm_name).strip().lower()
        if station is None or level < 0 or normalized_arm not in ("left", "right"):
            return {}
        level_targets = station.level_arm_positions.get(int(level), {})
        return dict(level_targets.get(normalized_arm, {}))
