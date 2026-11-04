"""Persistent, map-bound calibration for proven shelf-face anchors."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import os
from pathlib import Path
from typing import Iterable, Mapping

import yaml

from .benchmark import BenchmarkWaypoint
from .model import normalize_angle


CALIBRATION_FILENAME = "shelf_anchor_calibration.yaml"
SCHEMA_VERSION = 2


@dataclass(frozen=True)
class ShelfAnchorCalibration:
    map_name: str
    map_fingerprint: str
    station_id: str
    shelf_id: int
    face: int
    coarse_x: float
    coarse_y: float
    coarse_yaw: float
    aligned_x: float
    aligned_y: float
    aligned_yaw: float
    shelf_world_x: float
    shelf_world_y: float
    shelf_yaw: float
    captured_at: str

    @property
    def aligned_pose(self) -> tuple[float, float, float]:
        return self.aligned_x, self.aligned_y, self.aligned_yaw

    def compatibility_problem(
        self,
        *,
        map_name: str,
        map_fingerprint: str,
        anchor: BenchmarkWaypoint,
        shelf: Mapping[str, object],
    ) -> str:
        if self.map_name != map_name or self.map_fingerprint != map_fingerprint:
            return "anchor calibration belongs to a different map revision"
        if self.station_id != anchor.waypoint_id:
            return "anchor station ID changed"
        if math.hypot(self.coarse_x - anchor.x, self.coarse_y - anchor.y) > 0.02:
            return "anchor coarse position changed by more than 2 cm"
        if abs(normalize_angle(self.coarse_yaw - anchor.yaw)) > math.radians(1.0):
            return "anchor coarse heading changed by more than 1 degree"
        if int(shelf["id"]) != self.shelf_id:
            return "registered shelf ID changed"
        if math.hypot(
            self.shelf_world_x - float(shelf["world_x"]),
            self.shelf_world_y - float(shelf["world_y"]),
        ) > 0.02:
            return "registered shelf position changed by more than 2 cm"
        if abs(normalize_angle(self.shelf_yaw - float(shelf["yaw"]))) > math.radians(1.0):
            return "registered shelf heading changed by more than 1 degree"
        return ""


def calibration_path(map_2d_dir: Path) -> Path:
    return Path(map_2d_dir) / CALIBRATION_FILENAME


def save_calibrations(
    path: Path,
    calibrations: Iterable[ShelfAnchorCalibration],
) -> None:
    values = sorted(calibrations, key=lambda value: value.station_id)
    station_ids = [value.station_id for value in values]
    if len(station_ids) != len(set(station_ids)):
        raise ValueError("duplicate shelf anchor calibration station ID")
    document = {
        "schema_version": SCHEMA_VERSION,
        "anchors": [asdict(value) for value in values],
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


def load_calibrations(path: Path) -> dict[str, ShelfAnchorCalibration]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("unsupported shelf anchor calibration schema")
    schema_version = int(document.get("schema_version", 0))
    if schema_version == 1:
        raw_values = [document.get("anchor")]
    elif schema_version == SCHEMA_VERSION:
        raw_values = document.get("anchors")
    else:
        raise ValueError("unsupported shelf anchor calibration schema")
    if not isinstance(raw_values, list) or not all(
        isinstance(raw, dict) for raw in raw_values
    ):
        raise ValueError("shelf anchor calibration is missing anchor data")
    result = {}
    for raw in raw_values:
        calibration = ShelfAnchorCalibration(**raw)
        if calibration.station_id in result:
            raise ValueError("duplicate shelf anchor calibration station ID")
        result[calibration.station_id] = calibration
    return result


def save_calibration(path: Path, calibration: ShelfAnchorCalibration) -> None:
    """Insert or replace one anchor while preserving other shelf faces."""
    calibrations = load_calibrations(path)
    calibrations[calibration.station_id] = calibration
    save_calibrations(path, calibrations.values())


def load_calibration(
    path: Path,
    station_id: str | None = None,
) -> ShelfAnchorCalibration | None:
    """Load one anchor; the station ID is optional for legacy callers."""
    calibrations = load_calibrations(path)
    if station_id is not None:
        return calibrations.get(station_id)
    if len(calibrations) == 1:
        return next(iter(calibrations.values()))
    return None
