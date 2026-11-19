from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.competition_registry import load_competition_half_registry  # noqa: E402
from openflex_roboos_bridge.scene_sync_contract import (  # noqa: E402
    SceneSyncError,
    attach_for_planning,
    initial_snapshot,
    release_and_reinsert,
)


SCENE = ROOT / "local_dependencies/openflex_thesis/retail_fixture_single_red_20261113_v3/scene.xml"


@pytest.fixture
def registry():
    return load_competition_half_registry(str(SCENE))


def test_attachment_removes_only_target_from_world_collision(registry):
    initial = initial_snapshot(registry)
    attached = attach_for_planning(initial, registry, grasp_verified=True)
    assert attached.world_collision_ids == ()
    assert attached.attached_object_id == "thesis_item_01"
    assert attached.object_mapping == {"body": "thesis_object", "geom": "thesis_object_geom",
                                      "free_joint": "thesis_object_free"}


def test_release_reinserts_target_and_keeps_left_right_geometry_distinct(registry):
    attached = attach_for_planning(initial_snapshot(registry), registry, grasp_verified=True)
    left = release_and_reinsert(attached, registry, "sim_delivery_top_left",
                                settled=True, retreat_verified=True)
    right = release_and_reinsert(attached, registry, "sim_delivery_top_right",
                                 settled=True, retreat_verified=True)
    assert left.world_collision_ids == ("thesis_item_01",)
    assert left.placed_object_ids == ("thesis_item_01",)
    assert left.destination_geometry != right.destination_geometry
    assert left.destination_geometry["body"] == "counter_top_left"
    assert right.destination_geometry["body"] == "counter_top_right"


def test_uncalibrated_or_unregistered_state_cannot_become_runtime_consumable(registry):
    state = initial_snapshot(registry)
    with pytest.raises(SceneSyncError):
        state.__class__(**{**state.__dict__, "runtime_consumable": True}).validate(registry)
    with pytest.raises(SceneSyncError):
        release_and_reinsert(state, registry, "delivery_zone", settled=True,
                             retreat_verified=True)


def test_release_requires_measured_settle_and_retreat(registry):
    attached = attach_for_planning(initial_snapshot(registry), registry, grasp_verified=True)
    with pytest.raises(SceneSyncError):
        release_and_reinsert(attached, registry, "sim_delivery_top_left",
                             settled=False, retreat_verified=True)
    with pytest.raises(SceneSyncError):
        release_and_reinsert(attached, registry, "sim_delivery_top_left",
                             settled=True, retreat_verified=False)
