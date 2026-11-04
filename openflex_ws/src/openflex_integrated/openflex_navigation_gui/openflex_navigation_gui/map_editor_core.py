"""Pure-Python PGM editing helpers used by the OpenFlex map editor."""

from __future__ import annotations

import os
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional


OCCUPIED = 0
UNKNOWN = 205
FREE = 254


@dataclass
class GridMap:
    width: int
    height: int
    pixels: bytearray

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("地图尺寸必须为正数")
        if len(self.pixels) != self.width * self.height:
            raise ValueError("PGM 像素数量与地图尺寸不一致")

    def copy(self) -> "GridMap":
        return GridMap(self.width, self.height, bytearray(self.pixels))

    def index(self, x: int, y: int) -> int:
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise IndexError((x, y))
        return y * self.width + x

    def value(self, x: int, y: int) -> int:
        return self.pixels[self.index(x, y)]

    def counts(self) -> dict[int, int]:
        return {
            OCCUPIED: self.pixels.count(OCCUPIED),
            UNKNOWN: self.pixels.count(UNKNOWN),
            FREE: self.pixels.count(FREE),
        }


@dataclass(frozen=True)
class MapMetadata:
    resolution: Optional[float] = None
    origin: Optional[tuple[float, float, float]] = None


def _next_token(data: bytes, index: int) -> tuple[bytes, int]:
    length = len(data)
    while index < length:
        if data[index] in b" \t\r\n":
            index += 1
            continue
        if data[index] == ord("#"):
            while index < length and data[index] not in (10, 13):
                index += 1
            continue
        break
    start = index
    while index < length and data[index] not in b" \t\r\n#":
        index += 1
    return data[start:index], index


def read_pgm(path: Path) -> GridMap:
    data = path.read_bytes()
    magic, index = _next_token(data, 0)
    if magic not in (b"P5", b"P2"):
        raise ValueError(f"不支持的 PGM 格式: {magic!r}")
    width_token, index = _next_token(data, index)
    height_token, index = _next_token(data, index)
    max_token, index = _next_token(data, index)
    width = int(width_token)
    height = int(height_token)
    max_value = int(max_token)
    if max_value <= 0 or max_value > 255:
        raise ValueError("地图编辑器只支持 8-bit PGM")

    if magic == b"P5":
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        raster = data[index:index + width * height]
        if len(raster) != width * height:
            raise ValueError("PGM 数据长度不正确")
        if max_value != 255:
            raster = bytes(round(value * 255 / max_value) for value in raster)
        return GridMap(width, height, bytearray(raster))

    values = []
    while True:
        token, index = _next_token(data, index)
        if not token:
            break
        values.append(round(int(token) * 255 / max_value))
    if len(values) != width * height:
        raise ValueError("P2 PGM 数据长度不正确")
    return GridMap(width, height, bytearray(values))


def write_pgm(path: Path, grid: GridMap) -> None:
    header = (
        b"P5\n"
        b"# CREATOR: openflex_navigation_gui map editor\n" +
        f"{grid.width} {grid.height}\n255\n".encode("ascii")
    )
    path.write_bytes(header + bytes(grid.pixels))


def save_with_backup(path: Path, grid: GridMap) -> Optional[Path]:
    """Atomically replace a map while preserving the previous PGM."""
    path.parent.mkdir(parents=True, exist_ok=True)
    backup = None
    if path.exists():
        backup_dir = path.parent / "backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        backup = backup_dir / f"{path.stem}_{stamp}{path.suffix}"
        shutil.copy2(path, backup)

    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        write_pgm(temporary, grid)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return backup


def parse_map_yaml(path: Path) -> MapMetadata:
    resolution = None
    origin = None
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        key, raw = (part.strip() for part in line.split(":", 1))
        if key == "resolution":
            try:
                resolution = float(raw)
            except ValueError:
                pass
        elif key == "origin" and raw.startswith("[") and raw.endswith("]"):
            try:
                values = tuple(float(value.strip()) for value in raw[1:-1].split(","))
            except ValueError:
                continue
            if len(values) == 3:
                origin = values
    return MapMetadata(resolution=resolution, origin=origin)


def paint_circle(grid: GridMap, x: int, y: int, radius: int, value: int) -> int:
    radius = max(0, radius)
    changed = 0
    radius_sq = radius * radius
    for py in range(max(0, y - radius), min(grid.height - 1, y + radius) + 1):
        for px in range(max(0, x - radius), min(grid.width - 1, x + radius) + 1):
            if radius and (px - x) ** 2 + (py - y) ** 2 > radius_sq:
                continue
            index = py * grid.width + px
            if grid.pixels[index] != value:
                grid.pixels[index] = value
                changed += 1
    return changed


def line_cells(start: tuple[int, int], end: tuple[int, int]) -> Iterable[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        step = error * 2
        if step > -dy:
            error -= dy
            x0 += sx
        if step < dx:
            error += dx
            y0 += sy


def paint_line(
    grid: GridMap,
    start: tuple[int, int],
    end: tuple[int, int],
    radius: int,
    value: int,
) -> int:
    return sum(paint_circle(grid, x, y, radius, value) for x, y in line_cells(start, end))


def fill_component(grid: GridMap, x: int, y: int, value: int) -> int:
    """Replace one 8-connected same-value component."""
    source = grid.value(x, y)
    if source == value:
        return 0
    pending = [(x, y)]
    seen = {(x, y)}
    changed = 0
    while pending:
        px, py = pending.pop()
        index = py * grid.width + px
        if grid.pixels[index] != source:
            continue
        grid.pixels[index] = value
        changed += 1
        for ny in range(max(0, py - 1), min(grid.height - 1, py + 1) + 1):
            for nx in range(max(0, px - 1), min(grid.width - 1, px + 1) + 1):
                if (nx, ny) not in seen and grid.value(nx, ny) == source:
                    seen.add((nx, ny))
                    pending.append((nx, ny))
    return changed


def edit_summary(before: bytes, after: bytes) -> dict[str, int]:
    if len(before) != len(after):
        raise ValueError("编辑前后地图尺寸不一致")
    return {
        "changed": sum(old != new for old, new in zip(before, after)),
        "occupied_to_free": sum(
            old == OCCUPIED and new == FREE for old, new in zip(before, after)
        ),
        "unknown_to_free": sum(old == UNKNOWN and new == FREE for old, new in zip(before, after)),
        "to_occupied": sum(old != OCCUPIED and new == OCCUPIED for old, new in zip(before, after)),
        "to_unknown": sum(old != UNKNOWN and new == UNKNOWN for old, new in zip(before, after)),
    }
