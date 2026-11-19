"""Pure multi-product planning-scene lifecycle contract for offline validation."""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import FrozenSet


class LifecycleError(ValueError):
    """Raised when a product-scene transition would lose safety state."""


@dataclass(frozen=True)
class ProductRegistry:
    object_ids: FrozenSet[str]
    metadata: dict
    source_path: str
    frame_id: str


@dataclass(frozen=True)
class PlanningSceneLifecycle:
    phase: str
    target_id: str | None
    world_ids: FrozenSet[str]
    attached_ids: FrozenSet[str]
    placed_ids: FrozenSet[str]


def _object_projection(metadata: dict, *, pose_source: str) -> dict:
    return {
        "id": metadata["id"],
        "simulation_sku": metadata["simulation_sku"],
        "instance_id": metadata["instance_id"],
        "slot_id": metadata["slot_id"],
        "shape": metadata["shape"],
        "dimensions_m": list(metadata["dimensions_m"]),
        "mass_kg": metadata["mass_kg"],
        "collision_geom": metadata["collision_geom"],
        "pose": dict(metadata["pose"]) if pose_source == "offline_registry_pose" else None,
        "pose_source": pose_source,
    }


def offline_planning_scene_projection(
        state: PlanningSceneLifecycle, registry: ProductRegistry) -> dict:
    """Project a lifecycle state into a non-executable, offline scene contract.

    The projection deliberately carries no live-frame transform or attached TCP pose. It is
    intended for adapter tests and review of object membership, not for MoveIt execution.
    """
    _validate_state(state, registry)
    attached_id = next(iter(state.attached_ids), None)
    world_objects = [
        _object_projection(registry.metadata[object_id], pose_source="offline_registry_pose")
        for object_id in sorted(state.world_ids)
    ]
    attached_object = None
    if attached_id is not None:
        attached_object = _object_projection(
            registry.metadata[attached_id], pose_source="runtime_attach_required")
    return {
        "schema_version": 1,
        "frame_id": registry.frame_id,
        "frame_registered_to_live_map": False,
        "runtime_consumable": False,
        "execution_enabled": False,
        "phase": state.phase,
        "target_id": state.target_id,
        "world_objects": world_objects,
        "attached_object": attached_object,
        "placed_ids": sorted(state.placed_ids),
        "limitations": [
            "No transform from mujoco_sim_world to the live planning frame",
            "Attached TCP-relative pose must be supplied by a runtime verified grasp",
            "No ROS or MoveIt scene update or readback was performed",
        ],
    }


def load_product_registry(path: str | Path) -> ProductRegistry:
    source = Path(path).expanduser().resolve(strict=True)
    payload = json.loads(source.read_text(encoding="utf-8"))
    if payload.get("runtime_consumable") is not False or payload.get("execution_enabled") is not False:
        raise LifecycleError("Only explicitly offline product exports may enter this contract")
    frame_id = payload.get("frame_id")
    if frame_id != "mujoco_sim_world" or payload.get("frame_registered_to_live_map") is not False:
        raise LifecycleError("Product export must remain explicitly unregistered to the live map")
    objects = payload.get("objects")
    if not isinstance(objects, list) or not objects:
        raise LifecycleError("Product export has no objects")
    metadata = {}
    for product in objects:
        identifier = product.get("id")
        if not isinstance(identifier, str) or not identifier or identifier in metadata:
            raise LifecycleError("Product IDs must be unique nonempty strings")
        if product.get("role") != "movable_product_world_object":
            raise LifecycleError(f"Unexpected product role: {identifier}")
        metadata[identifier] = dict(product)
    return ProductRegistry(frozenset(metadata), metadata, str(source), frame_id)


def initial_lifecycle(registry: ProductRegistry) -> PlanningSceneLifecycle:
    if not registry.object_ids:
        raise LifecycleError("Cannot initialize an empty product scene")
    return PlanningSceneLifecycle("world", None, registry.object_ids, frozenset(), frozenset())


def _validate_state(state: PlanningSceneLifecycle, registry: ProductRegistry) -> None:
    all_ids = registry.object_ids
    if (not state.world_ids <= all_ids or not state.attached_ids <= all_ids
            or not state.placed_ids <= all_ids):
        raise LifecycleError("Lifecycle contains an unknown object")
    if state.world_ids & state.attached_ids:
        raise LifecycleError("An object cannot be in world and attached")
    if state.attached_ids & state.placed_ids:
        raise LifecycleError("An object cannot be attached and already placed")
    if len(state.attached_ids) > 1:
        raise LifecycleError("This lifecycle contract supports only one attached target")
    if state.world_ids | state.attached_ids | state.placed_ids != all_ids:
        raise LifecycleError("Lifecycle dropped an object from the scene")
    if not state.placed_ids <= state.world_ids:
        raise LifecycleError("Placed objects must remain in the world scene")
    if state.target_id is not None and state.target_id not in all_ids:
        raise LifecycleError("Target is not in the product registry")
    if state.phase not in {"world", "approach", "transport", "placed"}:
        raise LifecycleError("Unknown lifecycle phase")
    if state.phase == "world" and (state.target_id is not None or state.attached_ids):
        raise LifecycleError("World phase cannot have a selected or attached target")
    if state.phase == "approach" and (
            state.target_id not in state.world_ids or state.attached_ids):
        raise LifecycleError("Approach phase must select an unattached world object")
    if state.phase == "transport" and (
            state.target_id not in state.attached_ids
            or state.attached_ids != frozenset({state.target_id})):
        raise LifecycleError("Transport phase must carry only the selected target")
    if state.phase == "placed" and (
            state.target_id not in state.placed_ids or state.target_id not in state.world_ids
            or state.attached_ids):
        raise LifecycleError("Placed phase must reinsert the released target into the world")


def select_target(state: PlanningSceneLifecycle, registry: ProductRegistry,
                  target_id: str) -> PlanningSceneLifecycle:
    _validate_state(state, registry)
    if state.phase != "world" or state.target_id is not None:
        raise LifecycleError("Target selection requires an unselected world phase")
    if target_id not in state.world_ids or target_id in state.placed_ids:
        raise LifecycleError("Target must be an available, not-yet-placed world object")
    result = PlanningSceneLifecycle("approach", target_id, state.world_ids,
                                    state.attached_ids, state.placed_ids)
    _validate_state(result, registry)
    return result


def attach_target(state: PlanningSceneLifecycle, registry: ProductRegistry,
                  *, precheck_passed: bool) -> PlanningSceneLifecycle:
    _validate_state(state, registry)
    if state.phase != "approach" or state.target_id is None:
        raise LifecycleError("Attachment requires a selected target in approach phase")
    if precheck_passed is not True:
        raise LifecycleError("Attachment requires the complete grasp safety precheck")
    target = state.target_id
    result = PlanningSceneLifecycle("transport", target, state.world_ids - {target},
                                    frozenset({target}), state.placed_ids)
    _validate_state(result, registry)
    return result


def release_target(state: PlanningSceneLifecycle, registry: ProductRegistry,
                   *, settled: bool, retreat_verified: bool) -> PlanningSceneLifecycle:
    _validate_state(state, registry)
    if state.phase != "transport" or state.target_id is None:
        raise LifecycleError("Release requires a selected attached target")
    if settled is not True or retreat_verified is not True:
        raise LifecycleError("Release requires measured settling and verified post-retreat")
    target = state.target_id
    result = PlanningSceneLifecycle("placed", target, state.world_ids | {target},
                                    frozenset(), state.placed_ids | {target})
    _validate_state(result, registry)
    return result


def reset_after_delivery(state: PlanningSceneLifecycle, registry: ProductRegistry) -> PlanningSceneLifecycle:
    """Start another task only after the prior target is explicitly placed."""
    _validate_state(state, registry)
    if state.phase != "placed":
        raise LifecycleError("A new target cannot be selected before placement is complete")
    result = PlanningSceneLifecycle("world", None, state.world_ids, frozenset(), state.placed_ids)
    _validate_state(result, registry)
    return result
