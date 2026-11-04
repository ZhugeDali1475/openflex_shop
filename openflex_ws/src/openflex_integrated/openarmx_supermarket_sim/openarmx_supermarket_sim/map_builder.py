"""Project the competition MJCF into a Nav2 occupancy map and validate stations."""

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


DEFAULT_SCENE = "/home/openflex/openflex_shop/supermarket_scene/supermarket_scene_from_db.xml"


def _numbers(value, length, default=0.0):
    parts = [] if not value else [float(item) for item in value.split()]
    parts.extend([default] * (length - len(parts)))
    return parts[:length]


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @property
    def width(self):
        return int(math.ceil((self.max_x - self.min_x) / self.resolution))

    @property
    def height(self):
        return int(math.ceil((self.max_y - self.min_y) / self.resolution))


@dataclass(frozen=True)
class Rectangle:
    x: float
    y: float
    half_x: float
    half_y: float
    yaw: float = 0.0

    def contains(self, x, y, padding=0.0):
        dx = x - self.x
        dy = y - self.y
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        return (
            abs(local_x) <= self.half_x + padding
            and abs(local_y) <= self.half_y + padding
        )


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float

    def contains(self, x, y, padding=0.0):
        return math.hypot(x - self.x, y - self.y) <= self.radius + padding


class OccupancyGrid:
    def __init__(self, spec, occupied):
        self.spec = spec
        self.occupied = occupied

    def world_to_cell(self, x, y):
        column = int(math.floor((x - self.spec.min_x) / self.spec.resolution))
        row_from_bottom = int(math.floor((y - self.spec.min_y) / self.spec.resolution))
        return column, row_from_bottom

    def occupied_at(self, x, y):
        column, row = self.world_to_cell(x, y)
        if column < 0 or column >= self.spec.width or row < 0 or row >= self.spec.height:
            return True
        return self.occupied[row][column]

    def pose_is_clear(self, x, y, yaw, half_x, half_y, sample_step=None):
        step = sample_step or self.spec.resolution * 0.5
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = -half_x
        while local_x <= half_x + 1e-9:
            local_y = -half_y
            while local_y <= half_y + 1e-9:
                world_x = x + cosine * local_x - sine * local_y
                world_y = y + sine * local_x + cosine * local_y
                if self.occupied_at(world_x, world_y):
                    return False
                local_y += step
            local_x += step
        return True

    def write_pgm(self, path):
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        header = f"P5\n{self.spec.width} {self.spec.height}\n255\n".encode("ascii")
        pixels = bytearray()
        for row in reversed(self.occupied):
            pixels.extend(0 if cell else 254 for cell in row)
        output.write_bytes(header + pixels)


def _body_pose(body, parent_pose, angle_scale):
    parent_x, parent_y, parent_z, parent_yaw = parent_pose
    local_x, local_y, local_z = _numbers(body.get("pos"), 3)
    euler = _numbers(body.get("euler"), 3)
    local_yaw = euler[2] * angle_scale
    cosine = math.cos(parent_yaw)
    sine = math.sin(parent_yaw)
    return (
        parent_x + cosine * local_x - sine * local_y,
        parent_y + sine * local_x + cosine * local_y,
        parent_z + local_z,
        parent_yaw + local_yaw,
    )


def _geom_shape(geom, body_pose, angle_scale, min_height, max_height):
    body_x, body_y, body_z, body_yaw = body_pose
    local_x, local_y, local_z = _numbers(geom.get("pos"), 3)
    euler = _numbers(geom.get("euler"), 3)
    cosine = math.cos(body_yaw)
    sine = math.sin(body_yaw)
    x = body_x + cosine * local_x - sine * local_y
    y = body_y + sine * local_x + cosine * local_y
    z = body_z + local_z
    geom_type = geom.get("type", "sphere")
    size = _numbers(geom.get("size"), 3)
    if geom_type == "box":
        bottom = z - size[2]
        top = z + size[2]
        if top < min_height or bottom > max_height:
            return None
        return Rectangle(x, y, size[0], size[1], body_yaw + euler[2] * angle_scale)
    if geom_type in ("cylinder", "sphere"):
        half_height = size[1] if geom_type == "cylinder" else size[0]
        if z + half_height < min_height or z - half_height > max_height:
            return None
        return Circle(x, y, size[0])
    return None


def _collect_shapes(body, parent_pose, angle_scale, heights, shapes):
    pose = _body_pose(body, parent_pose, angle_scale)
    for geom in body.findall("geom"):
        shape = _geom_shape(geom, pose, angle_scale, heights[0], heights[1])
        if shape is not None:
            shapes.append(shape)
    for child in body.findall("body"):
        _collect_shapes(child, pose, angle_scale, heights, shapes)


def project_scene(scene_path, layout):
    root = ET.parse(scene_path).getroot()
    compiler = root.find("compiler")
    angle_scale = math.pi / 180.0 if compiler is None or compiler.get("angle") == "degree" else 1.0
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF has no worldbody")
    map_config = layout["map"]
    bounds = map_config["bounds"]
    spec = GridSpec(float(map_config["resolution"]), *[float(value) for value in bounds])
    heights = (
        float(map_config.get("obstacle_min_height", 0.04)),
        float(map_config.get("obstacle_max_height", 1.80)),
    )
    roots = tuple(layout["obstacle_roots"])
    shapes = []
    for body in worldbody.findall("body"):
        name = body.get("name", "")
        if any(name == root_name or name.startswith(root_name + "*") for root_name in roots):
            _collect_shapes(body, (0.0, 0.0, 0.0, 0.0), angle_scale, heights, shapes)
    for item in layout.get("virtual_obstacles", []):
        shapes.append(
            Rectangle(
                float(item["x"]),
                float(item["y"]),
                float(item["half_x"]),
                float(item["half_y"]),
                float(item.get("yaw", 0.0)),
            )
        )
    padding = float(map_config.get("geometry_padding", 0.0))
    occupied = []
    for row in range(spec.height):
        y = spec.min_y + (row + 0.5) * spec.resolution
        values = []
        for column in range(spec.width):
            x = spec.min_x + (column + 0.5) * spec.resolution
            values.append(any(shape.contains(x, y, padding) for shape in shapes))
        occupied.append(values)
    return OccupancyGrid(spec, occupied), shapes


def validate_stations(grid, stations, footprint):
    failures = []
    half_x = float(footprint["half_length"])
    half_y = float(footprint["half_width"])
    for station_id, station in stations.get("stations", stations).items():
        pose = station["nav_pose"]
        if pose.get("frame_id", "map") != "map":
            failures.append(f"{station_id}: nav_pose frame must be map")
            continue
        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))
        if not grid.pose_is_clear(x, y, yaw, half_x, half_y):
            failures.append(f"{station_id}: robot footprint intersects an obstacle at ({x}, {y})")
    return failures


def build_nav2_map(scene_path, layout_path, stations_path, output_directory):
    scene = Path(scene_path or DEFAULT_SCENE).expanduser().resolve()
    layout_file = Path(layout_path).expanduser().resolve()
    stations_file = Path(stations_path).expanduser().resolve()
    layout = yaml.safe_load(layout_file.read_text(encoding="utf-8"))
    stations = yaml.safe_load(stations_file.read_text(encoding="utf-8"))
    grid, _shapes = project_scene(scene, layout)
    failures = validate_stations(grid, stations, layout["robot_footprint"])
    if failures:
        raise ValueError("invalid navigation stations:\n  " + "\n  ".join(failures))

    output = Path(output_directory).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    pgm_path = output / "supermarket_bottom.pgm"
    yaml_path = output / "supermarket_bottom.yaml"
    grid.write_pgm(pgm_path)
    map_yaml = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": grid.spec.resolution,
        "origin": [grid.spec.min_x, grid.spec.min_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(yaml.safe_dump(map_yaml, sort_keys=False), encoding="utf-8")
    return yaml_path, pgm_path, grid


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene", default=os.environ.get("OPENFLEX_SUPERMARKET_MODEL", DEFAULT_SCENE)
    )
    parser.add_argument("--layout", required=True)
    parser.add_argument("--stations", required=True)
    parser.add_argument("--output-directory", required=True)
    options = parser.parse_args(args)
    yaml_path, pgm_path, grid = build_nav2_map(
        options.scene, options.layout, options.stations, options.output_directory
    )
    print(f"map_yaml={yaml_path}")
    print(f"map_image={pgm_path}")
    print(f"size={grid.spec.width}x{grid.spec.height} resolution={grid.spec.resolution}")


if __name__ == "__main__":
    main()
