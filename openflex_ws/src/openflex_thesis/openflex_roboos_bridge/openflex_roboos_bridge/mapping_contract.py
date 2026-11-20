"""ROS-free occupancy mapping and localization-quality contract (phase C2).

Turns depth measurements plus a camera pose into a 2-D occupancy grid, versions
the resulting map and records localization quality.  It is deliberately
sensor-facing but middleware-free: the caller supplies the depth frame, the
camera intrinsics and the camera pose in ``map``; nothing here subscribes to a
topic or reads a planner.

Localization in the isolated MuJoCo stack is ground truth (see audit section
22), so ``LocalizationQuality`` carries an explicit ``source`` and error terms
rather than pretending an estimator ran.  L2 claims built on it must be reported
as "map unknown, localization = simulation ground truth".
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

from .target_search_contract import camera_to_map

UNKNOWN = -1
FREE = 0
OCCUPIED = 100
_STATES = {UNKNOWN: "unknown", FREE: "free", OCCUPIED: "occupied"}


class MappingError(ValueError):
    """Raised when mapping inputs violate the contract."""


def _finite(value, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise MappingError(f"{name} must be finite")
    return number


@dataclass(frozen=True)
class LocalizationQuality:
    """Localization estimate quality for one map snapshot."""

    converged: bool
    source: str
    position_rmse_m: float = 0.0
    yaw_rmse_rad: float = 0.0
    stamp_sec: float = 0.0
    covariance_trace: Optional[float] = None

    def __post_init__(self) -> None:
        if not str(self.source).strip():
            raise MappingError("localization source must be a non-empty string")
        for name in ("position_rmse_m", "yaw_rmse_rad", "stamp_sec"):
            value = _finite(getattr(self, name), name)
            if value < 0.0:
                raise MappingError(f"{name} must be non-negative")
        if self.covariance_trace is not None:
            if _finite(self.covariance_trace, "covariance_trace") < 0.0:
                raise MappingError("covariance_trace must be non-negative")

    def is_usable(self, *, max_position_rmse_m: float, max_yaw_rmse_rad: float) -> bool:
        return (bool(self.converged)
                and self.position_rmse_m <= float(max_position_rmse_m)
                and self.yaw_rmse_rad <= float(max_yaw_rmse_rad))

    def to_dict(self) -> Mapping[str, object]:
        return {"converged": bool(self.converged), "source": self.source,
                "position_rmse_m": float(self.position_rmse_m),
                "yaw_rmse_rad": float(self.yaw_rmse_rad),
                "stamp_sec": float(self.stamp_sec),
                "covariance_trace": self.covariance_trace}


def ground_truth_localization(*, stamp_sec: float = 0.0) -> LocalizationQuality:
    """Localization as produced by the sim bridge: exact, but not an estimator."""

    return LocalizationQuality(converged=True, source="simulation_ground_truth_not_an_estimator",
                               position_rmse_m=0.0, yaw_rmse_rad=0.0, stamp_sec=stamp_sec)


@dataclass(frozen=True)
class MapSnapshot:
    """An immutable, versioned 2-D occupancy grid in a single frame."""

    frame_id: str
    resolution_m: float
    origin_xy: Tuple[float, float]
    width: int
    height: int
    occupancy: Tuple[int, ...]
    map_version: int
    localization: LocalizationQuality
    stamp_sec: float

    def __post_init__(self) -> None:
        if not str(self.frame_id).strip():
            raise MappingError("map frame_id must be a non-empty string")
        if _finite(self.resolution_m, "resolution_m") <= 0.0:
            raise MappingError("resolution_m must be positive")
        _finite(self.origin_xy[0], "origin_xy.x")
        _finite(self.origin_xy[1], "origin_xy.y")
        if self.width <= 0 or self.height <= 0:
            raise MappingError("map width and height must be positive")
        if len(self.occupancy) != self.width * self.height:
            raise MappingError("occupancy length must equal width * height")
        if any(int(cell) not in _STATES for cell in self.occupancy):
            raise MappingError("occupancy cells must be UNKNOWN, FREE or OCCUPIED")
        if int(self.map_version) < 0:
            raise MappingError("map_version must be non-negative")
        _finite(self.stamp_sec, "stamp_sec")

    # -- geometry ----------------------------------------------------------
    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        i = int(round((float(x) - self.origin_xy[0]) / self.resolution_m))
        j = int(round((float(y) - self.origin_xy[1]) / self.resolution_m))
        if 0 <= i < self.width and 0 <= j < self.height:
            return i, j
        return None

    def grid_to_world(self, i: int, j: int) -> Tuple[float, float]:
        return (self.origin_xy[0] + i * self.resolution_m,
                self.origin_xy[1] + j * self.resolution_m)

    def cell(self, i: int, j: int) -> int:
        if not (0 <= i < self.width and 0 <= j < self.height):
            return UNKNOWN
        return int(self.occupancy[j * self.width + i])

    def cell_state(self, i: int, j: int) -> str:
        return _STATES[self.cell(i, j)]

    def coverage_fraction(self) -> float:
        known = sum(1 for cell in self.occupancy if cell != UNKNOWN)
        return known / float(self.width * self.height)

    def to_dict(self, *, include_occupancy: bool = False) -> Mapping[str, object]:
        payload = {"frame_id": self.frame_id, "resolution_m": self.resolution_m,
                   "origin_xy": list(self.origin_xy), "width": self.width,
                   "height": self.height, "map_version": int(self.map_version),
                   "stamp_sec": float(self.stamp_sec),
                   "localization": self.localization.to_dict(),
                   "coverage_fraction": self.coverage_fraction()}
        if include_occupancy:
            payload["occupancy"] = list(self.occupancy)
        return payload


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[Tuple[int, int]]:
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        doubled = 2 * error
        if doubled > -dy:
            error -= dy
            x0 += sx
        if doubled < dx:
            error += dx
            y0 += sy


@dataclass
class OccupancyGridBuilder:
    """Incremental 2-D occupancy grid fed by ray-cast sensor hits."""

    frame_id: str
    resolution_m: float
    origin_xy: Tuple[float, float]
    width: int
    height: int
    _cells: List[int] = field(default_factory=list)
    _version: int = 0

    def __post_init__(self) -> None:
        if _finite(self.resolution_m, "resolution_m") <= 0.0:
            raise MappingError("resolution_m must be positive")
        if self.width <= 0 or self.height <= 0:
            raise MappingError("grid width and height must be positive")
        _finite(self.origin_xy[0], "origin_xy.x")
        _finite(self.origin_xy[1], "origin_xy.y")
        if not self._cells:
            self._cells = [UNKNOWN] * (self.width * self.height)

    @property
    def version(self) -> int:
        return self._version

    def _grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        i = int(round((float(x) - self.origin_xy[0]) / self.resolution_m))
        j = int(round((float(y) - self.origin_xy[1]) / self.resolution_m))
        if 0 <= i < self.width and 0 <= j < self.height:
            return i, j
        return None

    def _mark_free(self, i: int, j: int) -> None:
        if self._cells[j * self.width + i] == UNKNOWN:
            self._cells[j * self.width + i] = FREE

    def integrate_ray(self, sensor_xy: Tuple[float, float],
                      endpoint_xy: Tuple[float, float], *, endpoint_occupied: bool = True) -> bool:
        """Ray-cast one measurement: free along the ray, occupied at the hit."""

        start = self._grid(*sensor_xy)
        end = self._grid(*endpoint_xy)
        if start is None or end is None or start == end:
            return False
        for i, j in _bresenham(*start, *end):
            if (i, j) == end:
                continue
            self._mark_free(i, j)
        if endpoint_occupied:
            self._cells[end[1] * self.width + end[0]] = OCCUPIED
        else:
            self._mark_free(*end)
        self._version += 1
        return True

    def integrate_points_2d(self, sensor_xy: Tuple[float, float],
                            points_xy: Sequence[Tuple[float, float]]) -> int:
        return sum(1 for point in points_xy
                   if self.integrate_ray(sensor_xy, (float(point[0]), float(point[1]))))

    def integrate_depth_frame(self, depth_mm, intrinsics, camera_pose, *, stride: int = 4,
                              min_range_m: float = 0.15, max_range_m: float = 5.0) -> int:
        """Back-project a depth frame to map points and ray-cast each into the grid."""

        import numpy as np

        depth = np.asarray(depth_mm)
        if depth.shape[0] != intrinsics.height or depth.shape[1] != intrinsics.width:
            raise MappingError("depth frame shape does not match camera intrinsics")
        if int(stride) < 1:
            raise MappingError("stride must be a positive integer")
        minimum = _finite(min_range_m, "min_range_m")
        maximum = _finite(max_range_m, "max_range_m")
        if not 0.0 < minimum < maximum:
            raise MappingError("require 0 < min_range_m < max_range_m")
        sensor_xy = (camera_pose.translation_xyz[0], camera_pose.translation_xyz[1])
        hits: List[Tuple[float, float]] = []
        for v in range(0, intrinsics.height, int(stride)):
            for u in range(0, intrinsics.width, int(stride)):
                depth_value = float(depth[v, u]) / 1000.0
                if not minimum <= depth_value <= maximum:
                    continue
                camera_point = ((u - intrinsics.cx) * depth_value / intrinsics.fx,
                                (v - intrinsics.cy) * depth_value / intrinsics.fy,
                                depth_value)
                map_point = camera_to_map(camera_pose, camera_point)
                hits.append((float(map_point[0]), float(map_point[1])))
        return self.integrate_points_2d(sensor_xy, hits)

    def snapshot(self, *, localization: LocalizationQuality, stamp_sec: float) -> MapSnapshot:
        return MapSnapshot(frame_id=self.frame_id, resolution_m=self.resolution_m,
                           origin_xy=(float(self.origin_xy[0]), float(self.origin_xy[1])),
                           width=self.width, height=self.height,
                           occupancy=tuple(int(cell) for cell in self._cells),
                           map_version=self._version, localization=localization,
                           stamp_sec=float(stamp_sec))

    def with_snapshot(self, snapshot: MapSnapshot) -> "OccupancyGridBuilder":
        """Adopt an existing snapshot (keeps version); used for deterministic replays."""

        if snapshot.frame_id != self.frame_id or snapshot.width != self.width \
                or snapshot.height != self.height:
            raise MappingError("snapshot geometry does not match the builder")
        return replace(self, _cells=list(snapshot.occupancy), _version=int(snapshot.map_version))
