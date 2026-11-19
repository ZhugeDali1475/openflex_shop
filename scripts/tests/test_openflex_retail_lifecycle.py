from dataclasses import replace
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.retail_object_lifecycle import (
    LifecycleError,
    attach_target,
    initial_lifecycle,
    load_product_registry,
    offline_planning_scene_projection,
    release_target,
    reset_after_delivery,
    select_target,
)


PRODUCT_EXPORT = ROOT / "local_dependencies/openflex_thesis/retail_moveit_products_20261112_v1/planning_scene_products.json"


@pytest.fixture(scope="module")
def registry():
    return load_product_registry(PRODUCT_EXPORT)


def test_registry_is_offline_and_contains_six_unique_products(registry):
    assert registry.frame_id == "mujoco_sim_world"
    assert len(registry.object_ids) == 6
    assert len(registry.metadata) == 6


def test_lifecycle_preserves_other_products_as_world_obstacles(registry):
    initial = initial_lifecycle(registry)
    target = sorted(registry.object_ids)[0]
    selected = select_target(initial, registry, target)
    assert selected.world_ids == registry.object_ids
    attached = attach_target(selected, registry, precheck_passed=True)
    assert attached.attached_ids == {target}
    assert attached.world_ids == registry.object_ids - {target}
    assert not attached.placed_ids
    placed = release_target(attached, registry, settled=True, retreat_verified=True)
    assert placed.world_ids == registry.object_ids
    assert placed.placed_ids == {target}
    with pytest.raises(LifecycleError, match="not-yet-placed"):
        select_target(reset_after_delivery(placed, registry), registry, target)


def test_offline_projection_removes_only_attached_target(registry):
    target = sorted(registry.object_ids)[0]
    attached = attach_target(
        select_target(initial_lifecycle(registry), registry, target),
        registry,
        precheck_passed=True,
    )
    projection = offline_planning_scene_projection(attached, registry)
    world_ids = {item["id"] for item in projection["world_objects"]}
    assert projection["runtime_consumable"] is False
    assert projection["execution_enabled"] is False
    assert projection["frame_registered_to_live_map"] is False
    assert target not in world_ids
    assert world_ids == registry.object_ids - {target}
    assert projection["attached_object"]["id"] == target
    assert projection["attached_object"]["pose"] is None
    assert projection["attached_object"]["pose_source"] == "runtime_attach_required"


def test_offline_projection_reinserts_placed_target(registry):
    target = sorted(registry.object_ids)[0]
    placed = release_target(
        attach_target(
            select_target(initial_lifecycle(registry), registry, target),
            registry,
            precheck_passed=True,
        ),
        registry,
        settled=True,
        retreat_verified=True,
    )
    projection = offline_planning_scene_projection(placed, registry)
    assert {item["id"] for item in projection["world_objects"]} == registry.object_ids
    assert projection["attached_object"] is None
    assert projection["placed_ids"] == [target]


def test_lifecycle_rejects_multiple_attached_targets(registry):
    object_ids = sorted(registry.object_ids)
    selected = select_target(initial_lifecycle(registry), registry, object_ids[0])
    invalid = replace(
        attach_target(selected, registry, precheck_passed=True),
        world_ids=frozenset(registry.object_ids - set(object_ids[:2])),
        attached_ids=frozenset(object_ids[:2]),
    )
    with pytest.raises(LifecycleError, match="one attached target"):
        offline_planning_scene_projection(invalid, registry)


@pytest.mark.parametrize("bad_target", ["", "unknown", "thesis_item_01"])
def test_unknown_or_legacy_target_is_rejected(registry, bad_target):
    with pytest.raises(LifecycleError):
        select_target(initial_lifecycle(registry), registry, bad_target)


def test_attachment_requires_precheck(registry):
    target = sorted(registry.object_ids)[0]
    selected = select_target(initial_lifecycle(registry), registry, target)
    with pytest.raises(LifecycleError, match="precheck"):
        attach_target(selected, registry, precheck_passed=False)


@pytest.mark.parametrize("precheck", [1, "passed", object()])
def test_attachment_rejects_non_boolean_truthy_precheck(registry, precheck):
    target = sorted(registry.object_ids)[0]
    selected = select_target(initial_lifecycle(registry), registry, target)
    with pytest.raises(LifecycleError, match="precheck"):
        attach_target(selected, registry, precheck_passed=precheck)


def test_release_requires_settling_and_retreat(registry):
    target = sorted(registry.object_ids)[0]
    attached = attach_target(select_target(initial_lifecycle(registry), registry, target),
                             registry, precheck_passed=True)
    for settled, retreat in ((False, True), (True, False), (False, False)):
        with pytest.raises(LifecycleError):
            release_target(attached, registry, settled=settled, retreat_verified=retreat)
    assert attached.attached_ids == {target}


@pytest.mark.parametrize("settled,retreat", [(1, True), (True, "yes"), (True, object())])
def test_release_rejects_non_boolean_truthy_evidence_flags(registry, settled, retreat):
    target = sorted(registry.object_ids)[0]
    attached = attach_target(select_target(initial_lifecycle(registry), registry, target),
                             registry, precheck_passed=True)
    with pytest.raises(LifecycleError, match="settling"):
        release_target(attached, registry, settled=settled, retreat_verified=retreat)


def test_next_task_requires_explicit_placed_state(registry):
    initial = initial_lifecycle(registry)
    with pytest.raises(LifecycleError):
        reset_after_delivery(initial, registry)
    target = sorted(registry.object_ids)[0]
    placed = release_target(
        attach_target(select_target(initial, registry, target), registry, precheck_passed=True),
        registry, settled=True, retreat_verified=True)
    reset = reset_after_delivery(placed, registry)
    assert reset.phase == "world" and reset.target_id is None
    assert reset.world_ids == registry.object_ids


def test_export_role_and_frame_are_fail_closed(tmp_path):
    payload = PRODUCT_EXPORT.read_text(encoding="utf-8")
    altered = tmp_path / "not_runtime.json"
    altered.write_text(payload.replace('"runtime_consumable": false', '"runtime_consumable": true'), encoding="utf-8")
    with pytest.raises(LifecycleError):
        load_product_registry(altered)
