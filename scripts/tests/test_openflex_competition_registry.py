from pathlib import Path
import sys

import pytest

BRIDGE = Path(__file__).parents[2] / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"
sys.path.insert(0, str(BRIDGE))

from openflex_roboos_bridge.competition_registry import (
    AFFORDANCE_IDS,
    TARGET_OBJECT_ID,
    load_competition_half_registry,
)


ROOT = Path(__file__).parents[2]
SCENE = ROOT / "local_dependencies/openflex_thesis/retail_fixture_single_red_20261113_v3/scene.xml"


def test_registry_loads_exact_competition_topology():
    registry = load_competition_half_registry(str(SCENE))
    registry.validate()
    assert registry.shelves == ("shelf_group_1", "shelf_group_2")
    assert tuple(registry.objects) == (TARGET_OBJECT_ID,)
    assert registry.objects[TARGET_OBJECT_ID].color == "red"
    target = registry.objects[TARGET_OBJECT_ID]
    assert (target.body, target.geom, target.free_joint) == (
        "thesis_object", "thesis_object_geom", "thesis_object_free")
    assert target.initial_pose_xyzyaw[:3] == pytest.approx((0.9475, 0.53, 0.943))
    assert tuple(sorted(registry.affordances)) == tuple(sorted(AFFORDANCE_IDS))
    assert registry.affordances["sim_delivery_top_left"].surface_center_xyz != registry.affordances["sim_delivery_top_right"].surface_center_xyz


def test_registry_rejects_manifest_to_mjcf_object_mapping_mismatch(tmp_path):
    import json

    scene_text = SCENE.read_text()
    altered_scene = tmp_path / "scene.xml"
    altered_scene.write_text(scene_text.replace(
        'name="thesis_object_geom"', 'name="not_the_registered_geom"', 1))
    manifest = json.loads((SCENE.parent / "manifest.json").read_text())
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="body/geom/freejoint"):
        load_competition_half_registry(str(altered_scene), str(manifest_path))


def test_registry_rejects_unvalidated_delivery_and_limits_legacy_alias():
    registry = load_competition_half_registry(str(SCENE))
    with pytest.raises(RuntimeError, match="not route-validated"):
        registry.resolve_affordance("sim_delivery_top_right")
    assert registry.aliases["delivery_zone"] == "sim_delivery_top_left"
    with pytest.raises(KeyError, match="unknown"):
        registry.resolve_affordance("delivery_zone_b")


def test_registry_rejects_multi_product_parent_scene():
    parent = ROOT / "local_dependencies/openflex_thesis/retail_fixture_half_left_20261113_v5/scene.xml"
    with pytest.raises(ValueError, match="exactly one"):
        load_competition_half_registry(str(parent))
