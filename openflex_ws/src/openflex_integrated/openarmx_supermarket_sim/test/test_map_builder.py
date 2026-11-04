from pathlib import Path

import yaml

from openarmx_supermarket_sim.map_builder import build_nav2_map, validate_stations


WORKSPACE = Path(__file__).resolve().parents[4]
PACKAGE = Path(__file__).resolve().parents[1]
SCENE = Path("/home/openflex/openflex_shop/supermarket_scene/supermarket_scene_from_db.xml")
LAYOUT = PACKAGE / "config" / "scene_layout_bottom.yaml"
STATIONS = (
    WORKSPACE
    / "src"
    / "openflex_integrated"
    / "openarmx_task_executor"
    / "config"
    / "stations_supermarket_mujoco.yaml"
)


def test_map_projection_and_registered_stations(tmp_path):
    yaml_path, pgm_path, grid = build_nav2_map(
        str(SCENE), str(LAYOUT), str(STATIONS), str(tmp_path)
    )
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    assert pgm_path.is_file()
    assert metadata["resolution"] == 0.05
    assert grid.spec.width == 126
    assert grid.spec.height == 84
    assert grid.occupied_at(-1.1, -1.18)
    assert grid.occupied_at(1.1, -1.18)
    assert grid.occupied_at(-2.4, -3.6)
    assert not grid.occupied_at(0.0, -3.6)


def test_station_validation_rejects_pose_inside_shelf(tmp_path):
    _yaml_path, _pgm_path, grid = build_nav2_map(
        str(SCENE), str(LAYOUT), str(STATIONS), str(tmp_path)
    )
    layout = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    invalid = {
        "stations": {
            "bad": {
                "nav_pose": {"frame_id": "map", "x": -1.1, "y": -1.18, "yaw": 0.0}
            }
        }
    }
    assert validate_stations(grid, invalid, layout["robot_footprint"])
