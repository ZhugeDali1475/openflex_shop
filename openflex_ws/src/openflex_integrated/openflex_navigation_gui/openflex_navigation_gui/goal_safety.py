"""Static-map goal exclusion geometry for the competition GUI."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .competition_mission import world_to_grid
from .map_editor_core import FREE, GridMap, MapMetadata


@dataclass(frozen=True)
class GoalSafetyGeometry:
    """Robot footprint around base_link plus required obstacle clearance."""

    half_length_m: float
    half_width_m: float
    clearance_m: float

    def __post_init__(self) -> None:
        if self.half_length_m <= 0.0 or self.half_width_m <= 0.0:
            raise ValueError("车体足迹尺寸必须大于 0")
        if self.clearance_m < 0.0:
            raise ValueError("目标障碍净距不能小于 0")


@dataclass(frozen=True)
class CostmapSnapshot:
    """A copy of one global costmap update used by the GUI preflight.

    ``nav2_msgs/Costmap`` stores rows from the map origin upward, so the
    indexing here is the same as a normal Cartesian grid (unlike a PGM image,
    whose rows are top-to-bottom).
    """

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: Sequence[int]
    received_at: float


def costmap_pose_problem(
    snapshot: CostmapSnapshot,
    world_x: float,
    world_y: float,
) -> str:
    """Return a preflight error if the robot centre is unsafe for Nav2.

    The global costmap has already inflated obstacles for the configured robot
    footprint. Applying the physical footprint to that inflated map again
    double-counts the robot geometry and rejects valid starts near aisles.
    SmacPlanner2D evaluates the robot centre in this inflated representation,
    where 253 (inscribed), 254 (lethal), and 255 (unknown) are unsafe.
    """

    if snapshot.resolution <= 0.0 or snapshot.width <= 0 or snapshot.height <= 0:
        return "全局代价地图尺寸或分辨率无效"
    if len(snapshot.data) < snapshot.width * snapshot.height:
        return "全局代价地图数据不完整"
    grid_x = math.floor((world_x - snapshot.origin_x) / snapshot.resolution)
    grid_y = math.floor((world_y - snapshot.origin_y) / snapshot.resolution)
    if not 0 <= grid_x < snapshot.width or not 0 <= grid_y < snapshot.height:
        return "当前 base_link 中心超出全局代价地图范围（按 unknown 处理）"

    value = int(snapshot.data[grid_y * snapshot.width + grid_x])
    if value == 255:
        return "当前 base_link 中心位于 unknown 栅格"
    if value >= 253:
        return f"当前 base_link 中心位于 lethal/inscribed 栅格（代价 {value}）"
    return ""


class GoalSafetyMap:
    """Build an orientation-aware base_link exclusion mask from a ROS map."""

    def __init__(
        self,
        grid: GridMap,
        metadata: MapMetadata,
        geometry: GoalSafetyGeometry,
    ) -> None:
        if metadata.resolution is None or metadata.origin is None:
            raise ValueError("地图 YAML 缺少 resolution 或 origin")
        self.grid = grid
        self.metadata = metadata
        self.geometry = geometry
        self._obstacles = [
            (index % grid.width, index // grid.width)
            for index, value in enumerate(grid.pixels)
            if value < FREE
        ]
        self._obstacle_runs: list[tuple[int, int, int]] = []
        for y in range(grid.height):
            x = 0
            while x < grid.width:
                if grid.value(x, y) >= FREE:
                    x += 1
                    continue
                start = x
                while x + 1 < grid.width and grid.value(x + 1, y) < FREE:
                    x += 1
                self._obstacle_runs.append((y, start, x))
                x += 1
        self._cache_key: int | None = None
        self._cache_masks: tuple[bytearray, bytearray] | None = None

    @staticmethod
    def _footprint_vertices(
        yaw: float,
        half_length: float,
        half_width: float,
        center_x: float = 0.0,
        center_y: float = 0.0,
    ) -> list[tuple[float, float]]:
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        corners = (
            (-half_length, -half_width),
            (-half_length, half_width),
            (half_length, half_width),
            (half_length, -half_width),
        )
        return [
            (
                center_x + local_x * cosine - local_y * sine,
                center_y + local_x * sine + local_y * cosine,
            )
            for local_x, local_y in corners
        ]

    @staticmethod
    def _cell_vertices(
        center_x: float,
        center_y: float,
        half_size: float,
    ) -> list[tuple[float, float]]:
        return [
            (center_x - half_size, center_y - half_size),
            (center_x - half_size, center_y + half_size),
            (center_x + half_size, center_y + half_size),
            (center_x + half_size, center_y - half_size),
        ]

    @staticmethod
    def _point_in_polygon(
        point: tuple[float, float],
        polygon: list[tuple[float, float]],
    ) -> bool:
        x, y = point
        inside = False
        previous_x, previous_y = polygon[-1]
        for current_x, current_y in polygon:
            if (
                (current_y > y) != (previous_y > y)
                and x
                < (previous_x - current_x)
                * (y - current_y)
                / (previous_y - current_y)
                + current_x
            ):
                inside = not inside
            previous_x, previous_y = current_x, current_y
        return inside

    @staticmethod
    def _segments_intersect(
        a0: tuple[float, float],
        a1: tuple[float, float],
        b0: tuple[float, float],
        b1: tuple[float, float],
    ) -> bool:
        def orientation(
            p: tuple[float, float],
            q: tuple[float, float],
            r: tuple[float, float],
        ) -> float:
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (
                r[0] - p[0]
            )

        def on_segment(
            p: tuple[float, float],
            q: tuple[float, float],
            r: tuple[float, float],
        ) -> bool:
            return (
                min(p[0], r[0]) - 1e-12 <= q[0] <= max(p[0], r[0]) + 1e-12
                and min(p[1], r[1]) - 1e-12 <= q[1] <= max(p[1], r[1]) + 1e-12
            )

        o1 = orientation(a0, a1, b0)
        o2 = orientation(a0, a1, b1)
        o3 = orientation(b0, b1, a0)
        o4 = orientation(b0, b1, a1)
        if o1 * o2 < 0.0 and o3 * o4 < 0.0:
            return True
        if abs(o1) <= 1e-12 and on_segment(a0, b0, a1):
            return True
        if abs(o2) <= 1e-12 and on_segment(a0, b1, a1):
            return True
        if abs(o3) <= 1e-12 and on_segment(b0, a0, b1):
            return True
        if abs(o4) <= 1e-12 and on_segment(b0, a1, b1):
            return True
        return False

    @staticmethod
    def _point_segment_distance(
        point: tuple[float, float],
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> float:
        px, py = point
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-18:
            return math.hypot(px - sx, py - sy)
        t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / length_sq))
        return math.hypot(px - (sx + t * dx), py - (sy + t * dy))

    @classmethod
    def _polygon_distance(
        cls,
        polygon_a: list[tuple[float, float]],
        polygon_b: list[tuple[float, float]],
    ) -> float:
        if any(cls._point_in_polygon(point, polygon_b) for point in polygon_a):
            return 0.0
        if any(cls._point_in_polygon(point, polygon_a) for point in polygon_b):
            return 0.0
        edges_a = list(zip(polygon_a, polygon_a[1:] + polygon_a[:1]))
        edges_b = list(zip(polygon_b, polygon_b[1:] + polygon_b[:1]))
        if any(
            cls._segments_intersect(a0, a1, b0, b1)
            for a0, a1 in edges_a
            for b0, b1 in edges_b
        ):
            return 0.0
        return min(
            min(
                cls._point_segment_distance(point, start, end)
                for start, end in edges_b
            )
            for point in polygon_a
        )

    @classmethod
    def _footprint_cell_distance(
        cls,
        obstacle_x: float,
        obstacle_y: float,
        yaw: float,
        half_length: float,
        half_width: float,
        cell_half_size: float,
    ) -> float:
        footprint = cls._footprint_vertices(yaw, half_length, half_width)
        obstacle = cls._cell_vertices(obstacle_x, obstacle_y, cell_half_size)
        return min(
            cls._polygon_distance(footprint, obstacle),
            cls._polygon_distance(obstacle, footprint),
        )

    def masks(self, yaw: float) -> tuple[bytearray, bytearray]:
        """Return forbidden cells and their outer red boundary for one yaw."""
        key = round(math.atan2(math.sin(yaw), math.cos(yaw)) * 10000.0)
        if key == self._cache_key and self._cache_masks is not None:
            return self._cache_masks

        resolution = float(self.metadata.resolution)
        normalized_yaw = key / 10000.0
        half_length = self.geometry.half_length_m
        half_width = self.geometry.half_width_m
        cell_half_size = 0.5 * resolution
        reach = (
            math.hypot(half_length, half_width)
            + self.geometry.clearance_m
            + math.sqrt(2.0) * cell_half_size
        )
        cell_reach = int(math.ceil(reach / resolution))

        kernel_rows: dict[int, tuple[int, int]] = {}
        clearance = self.geometry.clearance_m + 1e-9
        for pixel_dy in range(-cell_reach, cell_reach + 1):
            world_y = -pixel_dy * resolution
            for pixel_dx in range(-cell_reach, cell_reach + 1):
                world_x = pixel_dx * resolution
                if (
                    self._footprint_cell_distance(
                        world_x,
                        world_y,
                        normalized_yaw,
                        half_length,
                        half_width,
                        cell_half_size,
                    )
                    <= clearance
                ):
                    if pixel_dy in kernel_rows:
                        minimum, maximum = kernel_rows[pixel_dy]
                        kernel_rows[pixel_dy] = (
                            min(minimum, pixel_dx),
                            max(maximum, pixel_dx),
                        )
                    else:
                        kernel_rows[pixel_dy] = (pixel_dx, pixel_dx)

        forbidden = bytearray(self.grid.width * self.grid.height)
        for obstacle_y, obstacle_start_x, obstacle_end_x in self._obstacle_runs:
            for delta_y, (minimum_delta_x, maximum_delta_x) in kernel_rows.items():
                goal_y = obstacle_y - delta_y
                if not 0 <= goal_y < self.grid.height:
                    continue
                goal_start_x = max(0, obstacle_start_x - maximum_delta_x)
                goal_end_x = min(
                    self.grid.width - 1,
                    obstacle_end_x - minimum_delta_x,
                )
                if goal_start_x <= goal_end_x:
                    start = goal_y * self.grid.width + goal_start_x
                    length = goal_end_x - goal_start_x + 1
                    forbidden[start:start + length] = b"\x01" * length

        boundary = bytearray(len(forbidden))
        for y in range(self.grid.height):
            for x in range(self.grid.width):
                index = y * self.grid.width + x
                if not forbidden[index] or self.grid.pixels[index] < FREE:
                    continue
                for neighbor_y in range(max(0, y - 1), min(self.grid.height, y + 2)):
                    for neighbor_x in range(max(0, x - 1), min(self.grid.width, x + 2)):
                        neighbor = neighbor_y * self.grid.width + neighbor_x
                        if (
                            not forbidden[neighbor]
                            and self.grid.pixels[neighbor] >= FREE
                        ):
                            boundary[index] = 1
                            break
                    if boundary[index]:
                        break

        self._cache_key = key
        self._cache_masks = forbidden, boundary
        return forbidden, boundary

    def is_safe(self, world_x: float, world_y: float, yaw: float) -> bool:
        cell = world_to_grid(world_x, world_y, self.grid, self.metadata)
        if cell is None:
            return False
        if self.grid.value(cell[0], cell[1]) < FREE:
            return False
        resolution = float(self.metadata.resolution)
        origin_x, origin_y, _ = self.metadata.origin
        reach = (
            math.hypot(self.geometry.half_length_m, self.geometry.half_width_m)
            + self.geometry.clearance_m
            + math.sqrt(2.0) * 0.5 * resolution
        )
        cell_radius = int(math.ceil(reach / resolution))
        for obstacle_x, obstacle_y in self._obstacles:
            if (
                abs(obstacle_x - cell[0]) > cell_radius
                or abs(obstacle_y - cell[1]) > cell_radius
            ):
                continue
            obstacle_world_x = origin_x + (obstacle_x + 0.5) * resolution
            obstacle_world_y = (
                origin_y + (self.grid.height - obstacle_y - 0.5) * resolution
            )
            if (
                self._footprint_cell_distance(
                    obstacle_world_x - world_x,
                    obstacle_world_y - world_y,
                    yaw,
                    self.geometry.half_length_m,
                    self.geometry.half_width_m,
                    0.5 * resolution,
                )
                <= self.geometry.clearance_m + 1e-9
            ):
                return False
        return True

    def rejection_reason(self, world_x: float, world_y: float, yaw: float) -> str:
        cell = world_to_grid(world_x, world_y, self.grid, self.metadata)
        if cell is None:
            return "目标位于二维地图范围外"
        if self.grid.value(cell[0], cell[1]) < FREE:
            return "目标位于障碍或未知栅格"
        if not self.is_safe(world_x, world_y, yaw):
            return (
                "目标位于红色禁设区：该 base_link 位姿不能保证车体与障碍物"
                f"至少 {self.geometry.clearance_m * 100:.0f} cm 净距"
            )
        return ""
