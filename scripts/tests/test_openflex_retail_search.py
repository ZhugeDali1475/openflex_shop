import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"))
from openflex_thesis_mujoco.retail_search_planner import build_search_plan, resolve_product_instances


def _products():
    return [{
        "instance_id": "product_banana_uid17", "simulation_sku": "banana",
        "display_name": "banana", "shelf_id": "shelf_group_1", "face": "neg",
        "level": 4,
        "support_bounds": {"min": [-1.4, 0.25, 1.24], "max": [-1.1, 2.11, 1.26]},
        "world_mesh_bounds": {"min": [-1.3, 1.7, 1.261], "max": [-1.2, 1.9, 1.3]},
    }, {
        "instance_id": "product_mug_uid44", "simulation_sku": "mug",
        "display_name": "mug", "shelf_id": "shelf_group_2", "face": "pos",
        "support_bounds": {"min": [1.1, 0.25, 1.24], "max": [1.4, 2.11, 1.26]},
        "world_mesh_bounds": {"min": [1.2, 1.0, 1.261], "max": [1.3, 1.2, 1.4]},
    }]


def test_query_alias_keeps_all_instances():
    matches = resolve_product_instances(_products(), "香蕉")
    assert [item["instance_id"] for item in matches] == ["product_banana_uid17"]
    assert resolve_product_instances(_products(), "mug")[0]["simulation_sku"] == "mug"


def test_search_starts_with_head_scan_and_then_target_station():
    plan = build_search_plan(_products(), "banana", current_base=(0, 0, 0))
    assert plan["execution_enabled"] is False
    assert plan["requires_visual_confirmation"] is True
    assert plan["candidate_instance_ids"] == ["product_banana_uid17"]
    assert [item["phase"] for item in plan["observations"][:6]] == ["head_scan"] * 6
    target_observations = [item for item in plan["observations"]
                           if item["phase"] == "target_observation"]
    assert len(target_observations) >= 5
    assert target_observations[-1]["phase"] == "target_observation"
    assert all(abs(item["head_yaw_rad"]) <= 1.45 for item in target_observations)
    assert all(abs(item["head_pitch_rad"]) <= 1.4 for item in target_observations)
    assert max(item["lift_m"] for item in target_observations) == pytest.approx(0.3)


def test_unknown_query_fails_closed():
    with pytest.raises(LookupError):
        build_search_plan(_products(), "矿泉水")


def test_cli_plan_is_json_serialisable(tmp_path):
    plan = build_search_plan(_products(), "马克杯")
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(plan, ensure_ascii=False), encoding="utf-8")
    loaded = json.loads(path.read_text(encoding="utf-8"))
    assert loaded["safety_gates"][-1] == "result_readback"
