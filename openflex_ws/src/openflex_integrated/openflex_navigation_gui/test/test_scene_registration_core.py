import math
from pathlib import Path

import pytest

from openflex_navigation_gui.map_editor_core import GridMap, MapMetadata
from openflex_navigation_gui.scene_registration_core import (
    EntityFootprint,
    MapTransform,
    center_layout_on_map,
    fingerprint_map_files,
)


def _transform(origin=(0.0, 0.0, 0.0)) -> MapTransform:
    grid = GridMap(200, 100, bytearray([254] * 200 * 100))
    return MapTransform.from_map(grid, MapMetadata(0.05, origin))


def test_world_pixel_round_trip_includes_map_origin_yaw():
    transform = _transform((2.0, -3.0, math.pi / 3.0))

    pixel = transform.world_to_pixel(3.25, 1.5)
    world = transform.pixel_to_world(*pixel)

    assert world == pytest.approx((3.25, 1.5))


def test_scene_rotation_round_trip_accounts_for_inverted_image_y_axis():
    transform = _transform((0.0, 0.0, math.pi / 6.0))
    scene_degrees = transform.world_yaw_to_scene_degrees(math.pi / 2.0)

    assert scene_degrees == pytest.approx(-60.0)
    assert transform.scene_degrees_to_world_yaw(scene_degrees) == pytest.approx(math.pi / 2.0)


def test_center_layout_preserves_relative_pose_and_orientation():
    transform = _transform()
    entities = [
        EntityFootprint(1, "shelf", -4.0, 2.0, 0.25, 0.8, 1.8),
        EntityFootprint(2, "table", -1.0, 5.0, -0.5, 1.2, 0.8),
    ]

    centered = center_layout_on_map(entities, transform)

    assert centered[1].world_x - centered[0].world_x == pytest.approx(3.0)
    assert centered[1].world_y - centered[0].world_y == pytest.approx(3.0)
    assert [item.yaw for item in centered] == [0.25, -0.5]
    corners = [point for entity in centered for point in entity.corners()]
    bbox_center = (
        (min(point[0] for point in corners) + max(point[0] for point in corners)) / 2,
        (min(point[1] for point in corners) + max(point[1] for point in corners)) / 2,
    )
    assert bbox_center == pytest.approx(transform.map_center_world())


def test_entity_origin_is_the_first_rotated_footprint_corner():
    entity = EntityFootprint(3, "shelf", 1.0, 2.0, math.pi / 2.0, 0.8, 1.2)
    corners = entity.corners()

    assert corners[0] == pytest.approx((1.0, 2.0))
    assert corners[1] == pytest.approx((1.0, 2.8))
    assert corners[3] == pytest.approx((-0.2, 2.0))


def test_map_fingerprint_changes_when_either_file_changes(tmp_path: Path):
    yaml_path = tmp_path / "map.yaml"
    pgm_path = tmp_path / "map.pgm"
    yaml_path.write_text("resolution: 0.05\n", encoding="utf-8")
    pgm_path.write_bytes(b"P5\n1 1\n255\n\xfe")
    original = fingerprint_map_files(yaml_path, pgm_path)

    pgm_path.write_bytes(b"P5\n1 1\n255\n\x00")

    assert fingerprint_map_files(yaml_path, pgm_path) != original
