from pathlib import Path

import pytest

from openflex_navigation_gui.model import (
    RECORD_TOPICS,
    nearest_path_error,
    normalize_angle,
    resolve_map_paths,
    validate_map_name,
)


def test_resolve_map_paths():
    paths = resolve_map_paths("/tmp/maps", "shop_01")
    assert paths.pcd == Path("/tmp/maps/3d/shop_01/map.pcd")
    assert paths.yaml == Path("/tmp/maps/2d/shop_01/map.yaml")


@pytest.mark.parametrize("name", ["", "bad name", "../escape", "map/child"])
def test_invalid_map_names(name):
    with pytest.raises(ValueError):
        validate_map_name(name)


def test_path_error_and_angle_normalization():
    distance, heading = nearest_path_error(1.0, 0.1, 3.2, [(0.0, 0.0, 0.0), (1.0, 0.0, -3.1)])
    assert distance == pytest.approx(0.1)
    assert heading == pytest.approx(normalize_angle(3.2 + 3.1))


def test_record_topics_cover_localization_and_global_costmap_diagnostics():
    assert "/localization_ok" in RECORD_TOPICS
    assert "/localization_hold" in RECORD_TOPICS
    assert "/localization_costmap_cleared" in RECORD_TOPICS
    assert "/initialpose" in RECORD_TOPICS
    assert "/goal_pose" in RECORD_TOPICS
    assert "/openflex_navigation_gui/goal_pose" in RECORD_TOPICS
    assert "/livox/imu" in RECORD_TOPICS
    assert "/global_costmap/costmap_raw" in RECORD_TOPICS
    assert "/global_costmap/voxel_marked_cloud" in RECORD_TOPICS
    assert "/fastlio2/body_cloud" in RECORD_TOPICS
    assert "/fastlio2/world_cloud" in RECORD_TOPICS
    assert "/livox/body_cloud_safety" in RECORD_TOPICS
    assert "/livox/body_cloud_nav2" in RECORD_TOPICS
    assert "/collision_monitor/footprint_approach" in RECORD_TOPICS
    assert "/shelf_fine_align/localization_guard_state" in RECORD_TOPICS
    assert "/collision_monitor/emergency_stop" not in RECORD_TOPICS
