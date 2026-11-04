from types import SimpleNamespace

from openflex_navigation_gui.scene_client import SceneRosClient


def test_snapshot_conversion_preserves_requested_map_match():
    response = SimpleNamespace(
        success=True,
        error="",
        scene_revision=8,
        has_map_binding=False,
        requested_map_matches=True,
        shelf_types=[],
        shelves=[],
        delivery_tables=[],
        skus=[],
        slots=[],
    )

    snapshot = SceneRosClient.snapshot_from_response(response)

    assert snapshot["scene_revision"] == 8
    assert snapshot["requested_map_matches"] is True
