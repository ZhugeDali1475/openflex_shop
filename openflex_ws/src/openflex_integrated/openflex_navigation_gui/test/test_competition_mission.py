from pathlib import Path

import pytest

from openflex_navigation_gui.benchmark import BenchmarkWaypoint
from openflex_navigation_gui.competition_mission import (
    grid_to_world,
    load_mission,
    save_mission,
    validate_station_id,
    world_to_grid,
)
from openflex_navigation_gui.map_editor_core import GridMap, MapMetadata


def station(station_id: str, role: str = "waypoint") -> BenchmarkWaypoint:
    return BenchmarkWaypoint(station_id, station_id, 1.0, 2.0, 0.5, 0.0, role)


def test_mission_round_trip_preserves_order_and_pose(tmp_path: Path):
    path = tmp_path / "competition_mission.yaml"
    save_mission(path, "shop", [station("START", "start_position"), station("SHELF_A")])
    loaded = load_mission(path, "shop")
    assert [item.waypoint_id for item in loaded] == ["START", "SHELF_A"]
    assert loaded[0].x == pytest.approx(1.0)
    assert loaded[0].yaw == pytest.approx(0.5)


def test_mission_v2_round_trip_preserves_station_action_policy(tmp_path: Path):
    path = tmp_path / "competition_mission.yaml"
    station_value = BenchmarkWaypoint(
        "DELIVERY",
        "交付台",
        2.32,
        0.37,
        -1.5708,
        0.0,
        "delivery_table",
        True,
        "front_surface_scan",
        0.20,
        0.01,
        0.0174533,
        20.0,
        False,
        True,
        0.15,
        0.04,
    )
    save_mission(path, "shop", [station_value])
    loaded = load_mission(path, "shop")
    assert loaded[0].requires_fine_alignment
    assert loaded[0].target_clearance_m == pytest.approx(0.20)
    assert loaded[0].should_retreat_after_place
    assert not loaded[0].should_retreat_after_pick
    assert loaded[0].retreat_speed_mps == pytest.approx(0.04)


def test_mission_rejects_duplicate_start_and_bad_id(tmp_path: Path):
    with pytest.raises(ValueError):
        save_mission(
            tmp_path / "mission.yaml",
            "shop",
            [station("A", "start_position"), station("B", "start_position")],
        )
    with pytest.raises(ValueError):
        validate_station_id("bad id")


def test_grid_world_conversion_round_trip():
    grid = GridMap(10, 8, bytearray([254] * 80))
    metadata = MapMetadata(0.05, (-1.0, -2.0, 0.0))
    world = grid_to_world(3, 4, grid, metadata)
    assert world_to_grid(*world, grid, metadata) == (3, 4)


def test_waypoint_picker_checks_initial_pose_cell(tmp_path: Path, monkeypatch):
    pytest.importorskip("PyQt5")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication

    from openflex_navigation_gui.waypoint_picker import WaypointPickerDialog

    pgm = tmp_path / "map.pgm"
    pgm.write_text("P2\n3 2\n255\n254 0 254\n205 254 254\n", encoding="ascii")
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n",
        encoding="ascii",
    )
    app = QApplication.instance() or QApplication([])

    occupied = WaypointPickerDialog(pgm, yaml_path, [], (1.5, 1.5, 0.0))
    assert not occupied.selected_is_free
    assert not occupied.ok_button.isEnabled()

    free = WaypointPickerDialog(pgm, yaml_path, [], (0.5, 1.5, 0.0))
    assert free.selected_is_free
    assert free.ok_button.isEnabled()
    occupied.close()
    free.close()
    app.processEvents()
