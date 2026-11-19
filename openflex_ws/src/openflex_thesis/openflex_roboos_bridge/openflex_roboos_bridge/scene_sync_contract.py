"""Offline planning-scene synchronization contract for the competition half.

The object membership transitions are represented separately from a live
MoveIt update.  Every snapshot is versioned by the static MJCF hash and keeps
the two delivery-table geometries distinct; no uncalibrated pose is executable.
"""

from dataclasses import dataclass
from typing import Mapping, Tuple

from .competition_registry import CompetitionHalfRegistry, TARGET_OBJECT_ID


class SceneSyncError(ValueError):
    pass


@dataclass(frozen=True)
class SceneSyncSnapshot:
    scene_sha256: str
    object_id: str
    object_mapping: Mapping[str, str]
    world_collision_ids: Tuple[str, ...]
    attached_object_id: str | None
    placed_object_ids: Tuple[str, ...]
    destination_id: str | None
    destination_geometry: Mapping[str, object] | None
    frame_id: str
    runtime_consumable: bool = False
    execution_enabled: bool = False

    def validate(self, registry: CompetitionHalfRegistry) -> None:
        if self.scene_sha256 != registry.scene_sha256:
            raise SceneSyncError("snapshot scene hash does not match registered MJCF")
        if self.object_id != TARGET_OBJECT_ID or self.object_id not in registry.objects:
            raise SceneSyncError("snapshot must contain the unique red target")
        expected = registry.objects[self.object_id]
        if dict(self.object_mapping) != {
                "body": expected.body, "geom": expected.geom, "free_joint": expected.free_joint}:
            raise SceneSyncError("object body/geom/freejoint mapping is inconsistent")
        all_ids = {self.object_id}
        if set(self.world_collision_ids) - all_ids or set(self.placed_object_ids) - all_ids:
            raise SceneSyncError("snapshot contains an unknown collision object")
        if self.attached_object_id not in (None, self.object_id):
            raise SceneSyncError("snapshot attached object is not the registered target")
        if self.attached_object_id and self.attached_object_id in self.world_collision_ids:
            raise SceneSyncError("attached object must be removed from world collisions")
        if self.attached_object_id and self.attached_object_id in self.placed_object_ids:
            raise SceneSyncError("attached object cannot already be placed")
        if self.destination_id is None and self.destination_geometry is not None:
            raise SceneSyncError("destination geometry requires a destination ID")
        if self.destination_id is not None:
            if self.destination_id not in registry.affordances:
                raise SceneSyncError("destination is not a registered affordance")
            expected_geometry = registry.affordances[self.destination_id]
            if self.destination_geometry != {
                    "body": expected_geometry.body,
                    "surface_geom": expected_geometry.surface_geom,
                    "surface_center_xyz": expected_geometry.surface_center_xyz,
                    "surface_dimensions_xy": expected_geometry.surface_dimensions_xy}:
                raise SceneSyncError("destination geometry does not match registry")
            if expected_geometry.navigation_pose_xyyaw is None:
                # Geometry can be inspected offline, but cannot be sent to a
                # runtime planner until a calibrated approach/release pose exists.
                if self.runtime_consumable or self.execution_enabled:
                    raise SceneSyncError("uncalibrated destination cannot be runtime-consumable")
        if self.frame_id != "mujoco_sim_world":
            raise SceneSyncError("unregistered frame cannot enter this sync contract")
        if self.runtime_consumable or self.execution_enabled:
            raise SceneSyncError("offline sync snapshots are never executable")


def initial_snapshot(registry: CompetitionHalfRegistry) -> SceneSyncSnapshot:
    target = registry.objects[TARGET_OBJECT_ID]
    state = SceneSyncSnapshot(
        registry.scene_sha256, TARGET_OBJECT_ID,
        {"body": target.body, "geom": target.geom, "free_joint": target.free_joint},
        (TARGET_OBJECT_ID,), None, (), None, None, "mujoco_sim_world")
    state.validate(registry)
    return state


def attach_for_planning(state: SceneSyncSnapshot, registry: CompetitionHalfRegistry,
                        *, grasp_verified: bool) -> SceneSyncSnapshot:
    state.validate(registry)
    if grasp_verified is not True:
        raise SceneSyncError("attachment synchronization requires independently verified grasp")
    result = SceneSyncSnapshot(
        state.scene_sha256, state.object_id, state.object_mapping, (), state.object_id,
        state.placed_object_ids, state.destination_id, state.destination_geometry,
        state.frame_id)
    result.validate(registry)
    return result


def release_and_reinsert(state: SceneSyncSnapshot, registry: CompetitionHalfRegistry,
                         destination_id: str, *, settled: bool, retreat_verified: bool) -> SceneSyncSnapshot:
    state.validate(registry)
    if state.attached_object_id != TARGET_OBJECT_ID:
        raise SceneSyncError("release requires the target to be attached")
    if settled is not True or retreat_verified is not True:
        raise SceneSyncError("release requires measured settling and retreat evidence")
    if destination_id not in registry.affordances:
        raise SceneSyncError("release destination is not registered")
    affordance = registry.affordances[destination_id]
    geometry = {"body": affordance.body, "surface_geom": affordance.surface_geom,
                "surface_center_xyz": affordance.surface_center_xyz,
                "surface_dimensions_xy": affordance.surface_dimensions_xy}
    result = SceneSyncSnapshot(
        state.scene_sha256, state.object_id, state.object_mapping, (TARGET_OBJECT_ID,),
        None, (TARGET_OBJECT_ID,), destination_id, geometry, state.frame_id)
    result.validate(registry)
    return result
