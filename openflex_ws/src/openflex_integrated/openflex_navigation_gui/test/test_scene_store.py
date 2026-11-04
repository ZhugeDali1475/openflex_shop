import pytest

from openflex_navigation_gui.map_editor_core import GridMap, MapMetadata
from openflex_navigation_gui.scene_registration_core import MapTransform
from openflex_navigation_gui.scene_store import SceneStore


def _snapshot(binding=None):
    return {
        "scene_revision": 12,
        "map_binding": binding,
        "shelf_types": [],
        "shelves": [
            {
                "id": 1,
                "name": "Shelf",
                "world_x": -4.0,
                "world_y": 1.0,
                "yaw": 0.3,
                "shelf_width": 0.8,
                "shelf_length": 1.8,
                "face_0_representatives": ["normal: tea"],
                "face_1_representatives": [],
            }
        ],
        "delivery_tables": [
            {
                "id": 2,
                "name": "Table",
                "world_x": -1.0,
                "world_y": 3.0,
                "yaw": -0.2,
                "length": 1.2,
                "width": 0.8,
            }
        ],
        "skus": [],
        "slots": [],
    }


def _transform():
    return MapTransform.from_map(
        GridMap(200, 100, bytearray([254] * 200 * 100)),
        MapMetadata(0.05, (0.0, 0.0, 0.0)),
    )


def test_matching_map_binding_keeps_database_poses():
    store = SceneStore()
    store.load_snapshot(_snapshot({
        "map_name": "map.yaml", "map_fingerprint": "sha256:a", "updated_at": "now"
    }))

    draft = store.begin_registration("map.yaml", "sha256:a", _transform())

    assert draft[0].world_x == -4.0
    assert draft[1].world_y == 3.0
    assert not store.is_dirty


def test_new_map_centers_layout_without_changing_relative_pose():
    store = SceneStore()
    store.load_snapshot(_snapshot())

    draft = store.begin_registration("new-map.yaml", "sha256:new", _transform())

    assert draft[1].world_x - draft[0].world_x == pytest.approx(3.0)
    assert draft[1].world_y - draft[0].world_y == pytest.approx(2.0)
    assert [value.yaw for value in draft] == [0.3, -0.2]
    assert not store.is_dirty


def test_pose_edit_undo_redo_and_layout_request():
    store = SceneStore()
    store.load_snapshot(_snapshot())
    store.begin_registration("map.yaml", "sha256:map", _transform())
    original = store.entity("shelf", 1)

    store.update_pose("shelf", 1, world_x=2.0, world_y=3.0, yaw=0.75)

    assert store.is_dirty
    assert store.can_undo
    assert store.undo()
    assert store.entity("shelf", 1) == original
    assert store.redo()
    assert store.entity("shelf", 1).world_x == 2.0
    request = store.layout_request()
    assert request["expected_scene_revision"] == 12
    assert request["shelf_updates"][0] == {
        "shelf_id": 1,
        "world_x": 2.0,
        "world_y": 3.0,
        "yaw": 0.75,
    }
    assert request["delivery_table_updates"][0]["table_id"] == 2


def test_drag_preview_can_be_committed_as_one_history_entry():
    store = SceneStore()
    store.load_snapshot(_snapshot())
    store.begin_registration("map.yaml", "sha256:map", _transform())
    previous = list(store.draft)

    store.update_pose("shelf", 1, world_x=1.0, world_y=1.0, yaw=0.3, record_history=False)
    store.update_pose("shelf", 1, world_x=2.0, world_y=2.0, yaw=0.3, record_history=False)
    store.commit_history_state(previous)

    assert store.undo()
    assert store.entity("shelf", 1) == previous[0]
