from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.scene_registry import load_scene_registry


V7 = ROOT / "local_dependencies/openflex_thesis/retail_fixture_single_red_runtime_20261113_v7"


def test_v7_destination_ids_resolve_to_distinct_surfaces_without_enabling_execution():
    registry = load_scene_registry(V7 / "runtime.yaml", V7 / "scene.xml")
    left = registry["affordances"]["sim_delivery_top_left"]
    right = registry["affordances"]["sim_delivery_top_right"]

    assert (left["body"], left["geom"]) == ("thesis_delivery_table", "thesis_delivery_top")
    assert (right["body"], right["geom"]) == ("counter_top_right", "counter_top_right_top")
    assert left["surface_center_xyz"] != right["surface_center_xyz"]
    assert registry["stations"]["sim_delivery_top_left"]["approach_candidate_xyyaw"] != \
        registry["stations"]["sim_delivery_top_right"]["approach_candidate_xyyaw"]
    assert registry["delivery_destinations"]["sim_delivery_top_left"]["validated"] is False
    assert registry["delivery_destinations"]["sim_delivery_top_right"]["validated"] is False
    assert registry["stations"]["sim_delivery_top_right"]["route_waypoints"] == []
    assert registry["affordances"]["delivery_zone"]["geom"] == left["geom"]
    assert registry["execution_enabled"] is False


def test_duplicate_destination_body_geom_mapping_is_rejected(tmp_path):
    config = yaml.safe_load((V7 / "runtime.yaml").read_text())
    bridge = config["openflex_thesis_mujoco_bridge"]["ros__parameters"]
    bridge["delivery_destinations"] = {
        "sim_delivery_top_right": {
            "body": "thesis_delivery_table", "surface_geom": "thesis_delivery_top",
        }
    }
    config_path, model_path = tmp_path / "runtime.yaml", tmp_path / "scene.xml"
    config_path.write_text(yaml.safe_dump(config))
    model_path.write_bytes((V7 / "scene.xml").read_bytes())
    with pytest.raises(ValueError, match="distinct"):
        load_scene_registry(config_path, model_path)
