"""Revision-aware scene state and editable registration drafts."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, replace
from typing import Optional

from .scene_registration_core import EntityFootprint, MapTransform, center_layout_on_map


@dataclass(frozen=True)
class SceneEntity:
    entity_id: int
    kind: str
    name: str
    world_x: float
    world_y: float
    yaw: float
    size_x: float
    size_y: float
    face_0_representatives: tuple[str, ...] = ()
    face_1_representatives: tuple[str, ...] = ()

    def footprint(self) -> EntityFootprint:
        return EntityFootprint(
            self.entity_id,
            self.kind,
            self.world_x,
            self.world_y,
            self.yaw,
            self.size_x,
            self.size_y,
        )


class SceneStore:
    def __init__(self) -> None:
        self.snapshot: Optional[dict] = None
        self.map_name = ""
        self.map_fingerprint = ""
        self.map_transform: Optional[MapTransform] = None
        self.draft: list[SceneEntity] = []
        self._initial_draft: list[SceneEntity] = []
        self._undo: list[list[SceneEntity]] = []
        self._redo: list[list[SceneEntity]] = []

    @property
    def scene_revision(self) -> int:
        return 0 if self.snapshot is None else int(self.snapshot["scene_revision"])

    @property
    def has_snapshot(self) -> bool:
        return self.snapshot is not None

    @property
    def is_dirty(self) -> bool:
        return self.draft != self._initial_draft

    @property
    def can_undo(self) -> bool:
        return bool(self._undo)

    @property
    def can_redo(self) -> bool:
        return bool(self._redo)

    def load_snapshot(self, snapshot: dict) -> None:
        self.snapshot = deepcopy(snapshot)
        self.cancel_draft()

    def begin_registration(self, map_name: str, map_fingerprint: str,
                           transform: MapTransform) -> list[SceneEntity]:
        if self.snapshot is None:
            raise RuntimeError("必须先加载场景快照")
        self.map_name = str(map_name)
        self.map_fingerprint = str(map_fingerprint)
        self.map_transform = transform
        entities = [
            SceneEntity(
                entity_id=value["id"],
                kind="shelf",
                name=value["name"],
                world_x=value["world_x"],
                world_y=value["world_y"],
                yaw=value["yaw"],
                size_x=value["shelf_width"],
                size_y=value["shelf_length"],
                face_0_representatives=tuple(value.get("face_0_representatives", [])),
                face_1_representatives=tuple(value.get("face_1_representatives", [])),
            )
            for value in self.snapshot["shelves"]
        ]
        entities.extend(
            SceneEntity(
                entity_id=value["id"],
                kind="delivery_table",
                name=value["name"],
                world_x=value["world_x"],
                world_y=value["world_y"],
                yaw=value["yaw"],
                size_x=value["length"],
                size_y=value["width"],
            )
            for value in self.snapshot["delivery_tables"]
        )

        binding = self.snapshot.get("map_binding")
        map_matches = bool(
            binding
            and binding.get("map_name") == self.map_name
            and binding.get("map_fingerprint") == self.map_fingerprint
        )
        if entities and not map_matches:
            centered = center_layout_on_map(
                [entity.footprint() for entity in entities], transform
            )
            by_key = {(value.kind, value.entity_id): value for value in centered}
            entities = [
                replace(
                    entity,
                    world_x=by_key[(entity.kind, entity.entity_id)].world_x,
                    world_y=by_key[(entity.kind, entity.entity_id)].world_y,
                )
                for entity in entities
            ]

        self.draft = entities
        self._initial_draft = list(entities)
        self._undo.clear()
        self._redo.clear()
        return list(self.draft)

    def entity(self, kind: str, entity_id: int) -> SceneEntity:
        for entity in self.draft:
            if entity.kind == kind and entity.entity_id == entity_id:
                return entity
        raise KeyError((kind, entity_id))

    def update_pose(self, kind: str, entity_id: int, *, world_x: float,
                    world_y: float, yaw: float, record_history: bool = True) -> SceneEntity:
        index = next(
            (index for index, value in enumerate(self.draft)
             if value.kind == kind and value.entity_id == entity_id),
            None,
        )
        if index is None:
            raise KeyError((kind, entity_id))
        updated = replace(
            self.draft[index], world_x=float(world_x), world_y=float(world_y), yaw=float(yaw)
        )
        if updated == self.draft[index]:
            return updated
        if record_history:
            self._undo.append(list(self.draft))
            self._redo.clear()
        self.draft[index] = updated
        return updated

    def commit_history_state(self, previous: list[SceneEntity]) -> None:
        """Record one completed drag while coalescing all mouse-move previews."""
        if previous != self.draft:
            self._undo.append(list(previous))
            self._redo.clear()

    def undo(self) -> bool:
        if not self._undo:
            return False
        self._redo.append(list(self.draft))
        self.draft = self._undo.pop()
        return True

    def redo(self) -> bool:
        if not self._redo:
            return False
        self._undo.append(list(self.draft))
        self.draft = self._redo.pop()
        return True

    def cancel_draft(self) -> None:
        self.map_name = ""
        self.map_fingerprint = ""
        self.map_transform = None
        self.draft = []
        self._initial_draft = []
        self._undo.clear()
        self._redo.clear()

    def mark_applied(self, snapshot: dict) -> None:
        self.snapshot = deepcopy(snapshot)
        self._initial_draft = list(self.draft)
        self._undo.clear()
        self._redo.clear()

    def layout_request(self) -> dict:
        if not self.map_name or not self.map_fingerprint:
            raise RuntimeError("当前没有已加载的地图草稿")
        return {
            "map_name": self.map_name,
            "map_fingerprint": self.map_fingerprint,
            "expected_scene_revision": self.scene_revision,
            "shelf_updates": [
                {
                    "shelf_id": entity.entity_id,
                    "world_x": entity.world_x,
                    "world_y": entity.world_y,
                    "yaw": entity.yaw,
                }
                for entity in self.draft if entity.kind == "shelf"
            ],
            "delivery_table_updates": [
                {
                    "table_id": entity.entity_id,
                    "world_x": entity.world_x,
                    "world_y": entity.world_y,
                    "yaw": entity.yaw,
                }
                for entity in self.draft if entity.kind == "delivery_table"
            ],
        }
