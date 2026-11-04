"""Pure geometry helpers for manual scene registration on a ROS occupancy map."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

from .map_editor_core import GridMap, MapMetadata


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class MapTransform:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @classmethod
    def from_map(cls, grid: GridMap, metadata: MapMetadata) -> "MapTransform":
        if metadata.resolution is None or metadata.resolution <= 0:
            raise ValueError("地图 YAML 缺少有效的 resolution")
        if metadata.origin is None:
            raise ValueError("地图 YAML 缺少有效的 origin")
        return cls(
            width=grid.width,
            height=grid.height,
            resolution=metadata.resolution,
            origin_x=metadata.origin[0],
            origin_y=metadata.origin[1],
            origin_yaw=metadata.origin[2],
        )

    def pixel_to_world(self, pixel_x: float, pixel_y: float) -> tuple[float, float]:
        local_x = pixel_x * self.resolution
        local_y = (self.height - pixel_y) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + local_x * cosine - local_y * sine,
            self.origin_y + local_x * sine + local_y * cosine,
        )

    def world_to_pixel(self, world_x: float, world_y: float) -> tuple[float, float]:
        delta_x = world_x - self.origin_x
        delta_y = world_y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = delta_x * cosine + delta_y * sine
        local_y = -delta_x * sine + delta_y * cosine
        return local_x / self.resolution, self.height - local_y / self.resolution

    def world_yaw_to_scene_degrees(self, world_yaw: float) -> float:
        return -math.degrees(normalize_angle(world_yaw - self.origin_yaw))

    def scene_degrees_to_world_yaw(self, scene_degrees: float) -> float:
        return normalize_angle(self.origin_yaw - math.radians(scene_degrees))

    def map_center_world(self) -> tuple[float, float]:
        return self.pixel_to_world(self.width * 0.5, self.height * 0.5)


@dataclass(frozen=True)
class EntityFootprint:
    entity_id: int
    kind: str
    world_x: float
    world_y: float
    yaw: float
    size_x: float
    size_y: float

    def corners(self) -> tuple[tuple[float, float], ...]:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        result = []
        for local_x, local_y in (
            (0.0, 0.0),
            (self.size_x, 0.0),
            (self.size_x, self.size_y),
            (0.0, self.size_y),
        ):
            result.append((
                self.world_x + local_x * cosine - local_y * sine,
                self.world_y + local_x * sine + local_y * cosine,
            ))
        return tuple(result)


def center_layout_on_map(entities: Iterable[EntityFootprint],
                         transform: MapTransform) -> list[EntityFootprint]:
    """Translate a layout draft to map center without changing relative poses or yaw."""
    entities = list(entities)
    if not entities:
        return []
    corners = [corner for entity in entities for corner in entity.corners()]
    layout_center = (
        (min(point[0] for point in corners) + max(point[0] for point in corners)) * 0.5,
        (min(point[1] for point in corners) + max(point[1] for point in corners)) * 0.5,
    )
    target_x, target_y = transform.map_center_world()
    delta_x = target_x - layout_center[0]
    delta_y = target_y - layout_center[1]
    return [
        replace(entity, world_x=entity.world_x + delta_x, world_y=entity.world_y + delta_y)
        for entity in entities
    ]


def fingerprint_map_files(yaml_path: Path, pgm_path: Path) -> str:
    """Return a deterministic content fingerprint for one ROS map pair."""
    digest = hashlib.sha256()
    for label, path in ((b"yaml", yaml_path), (b"pgm", pgm_path)):
        digest.update(label)
        digest.update(b"\0")
        with Path(path).open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return f"sha256:{digest.hexdigest()}"
