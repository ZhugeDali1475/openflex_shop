import math

from openflex_navigation_gui.benchmark import BenchmarkWaypoint
from openflex_navigation_gui.shelf_anchor_calibration import (
    ShelfAnchorCalibration,
    load_calibration,
    load_calibrations,
    save_calibration,
)


def calibration():
    return ShelfAnchorCalibration(
        map_name="supermarket_test_09",
        map_fingerprint="sha256:test",
        station_id="SHELF-A",
        shelf_id=2,
        face=1,
        coarse_x=1.945,
        coarse_y=1.323,
        coarse_yaw=math.pi,
        aligned_x=1.845,
        aligned_y=1.323,
        aligned_yaw=math.pi,
        shelf_world_x=0.57,
        shelf_world_y=0.80,
        shelf_yaw=0.06,
        captured_at="2026-08-16T12:00:00+08:00",
    )


def test_anchor_calibration_round_trip_and_context_validation(tmp_path):
    path = tmp_path / "anchor.yaml"
    expected = calibration()
    save_calibration(path, expected)

    loaded = load_calibration(path)

    assert loaded == expected
    anchor = BenchmarkWaypoint(
        "SHELF-A", "A", 1.945, 1.323, math.pi, 0.05, "shelf_approach"
    )
    shelf = {"id": 2, "world_x": 0.57, "world_y": 0.80, "yaw": 0.06}
    assert loaded.compatibility_problem(
        map_name="supermarket_test_09",
        map_fingerprint="sha256:test",
        anchor=anchor,
        shelf=shelf,
    ) == ""


def test_anchor_calibration_rejects_changed_shelf_registration():
    anchor = BenchmarkWaypoint(
        "SHELF-A", "A", 1.945, 1.323, math.pi, 0.05, "shelf_approach"
    )
    problem = calibration().compatibility_problem(
        map_name="supermarket_test_09",
        map_fingerprint="sha256:test",
        anchor=anchor,
        shelf={"id": 2, "world_x": 0.60, "world_y": 0.80, "yaw": 0.06},
    )

    assert "shelf position changed" in problem


def test_saving_second_anchor_preserves_first(tmp_path):
    path = tmp_path / "anchors.yaml"
    first = calibration()
    second = ShelfAnchorCalibration(
        **{
            **first.__dict__,
            "station_id": "SHELF-2-FACE-0",
            "face": 0,
            "coarse_x": -0.12,
            "coarse_y": 1.68,
            "coarse_yaw": 0.06,
            "aligned_x": -0.10,
            "aligned_y": 1.68,
            "aligned_yaw": 0.06,
        }
    )

    save_calibration(path, first)
    save_calibration(path, second)

    loaded = load_calibrations(path)
    assert set(loaded) == {"SHELF-A", "SHELF-2-FACE-0"}
    assert load_calibration(path, "SHELF-A") == first
    assert load_calibration(path, "SHELF-2-FACE-0") == second
    assert load_calibration(path) is None
