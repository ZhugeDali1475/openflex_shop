import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.target_search_contract import (  # noqa: E402
    CameraIntrinsics, CameraPose, analyze_frame, color_mask, map_to_camera,
    project_point, resolve_search_target,
)

WIDTH, HEIGHT = 64, 48
INTRINSICS = CameraIntrinsics.from_k(WIDTH, HEIGHT, [50.0, 0.0, 32.0, 0.0, 50.0, 24.0, 0.0, 0.0, 1.0])
# Identity orientation: optical +z maps to map +z (fine for a synthetic unit frame).
CAMERA = CameraPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
SLOT = (0.0, 0.0, 2.0)
SLOT_PX = (32, 24)

GREY = (150, 150, 150)
RED = (220, 40, 40)
BLUE = (40, 60, 220)


def frame(blobs=((RED, 8),), slot_depth_mm=2000, background_mm=3000):
    rgb = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    rgb[:, :] = GREY
    depth = np.full((HEIGHT, WIDTH), background_mm, dtype=np.uint16)
    u, v = SLOT_PX
    depth[v - 10:v + 10, u - 10:u + 10] = slot_depth_mm
    for color, size in blobs:
        if not size:
            continue
        y0, y1 = v - size // 2, v + size // 2
        x0, x1 = u - size // 2, u + size // 2
        rgb[y0:y1, x0:x1] = color
        depth[y0:y1, x0:x1] = slot_depth_mm
    return rgb, depth


def test_background_and_robot_grey_are_never_candidates():
    rgb, depth = frame(blobs=((GREY, 0),))
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    assert analysis.candidates == ()
    # white / grey is not saturated, so no colour mask fires
    assert int(color_mask(rgb, "red").sum()) == 0


def test_visible_target_is_resolved_and_authorized():
    rgb, depth = frame(blobs=((RED, 8),))
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    assert analysis.region.state == "target"
    resolution = resolve_search_target(analysis, "红色的快递盒")
    assert resolution.outcome == "authorized"
    assert resolution.reason == "target_visible"
    assert resolution.selected_object_id == "thesis_item_01"
    assert resolution.to_dict()["selection_basis"] == "image_derived_candidates_not_oracle_object_id"


def test_occluded_slot_fails_closed():
    rgb, depth = frame(blobs=((GREY, 0),), slot_depth_mm=1500)  # nearer surface, no red
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    assert analysis.region.state == "occluded"
    resolution = resolve_search_target(analysis, "红色的快递盒")
    assert resolution.outcome == "failed"
    assert resolution.reason == "target_occluded"
    assert resolution.selected_object_id is None


def test_empty_slot_fails_closed():
    rgb, depth = frame(blobs=((GREY, 0),), slot_depth_mm=3000)  # background only
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    assert analysis.region.state == "empty"
    resolution = resolve_search_target(analysis, "红色的快递盒")
    assert resolution.outcome == "failed"
    assert resolution.reason == "empty_slot"


def test_wrong_class_fails_closed():
    rgb, depth = frame(blobs=((BLUE, 8),))
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    resolution = resolve_search_target(analysis, "红色的快递盒")
    assert resolution.outcome == "failed"
    assert resolution.reason == "wrong_class"
    assert resolution.selected_object_id is None


def test_two_visible_targets_are_ambiguous_and_not_grasped():
    rgb, depth = frame(blobs=((RED, 8),))
    u, v = SLOT_PX
    rgb[v - 3:v + 3, u - 20:u - 14] = RED
    depth[v - 3:v + 3, u - 20:u - 14] = 2000
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    resolution = resolve_search_target(analysis, "红色的快递盒")
    assert resolution.outcome == "failed"
    assert resolution.reason == "ambiguous_candidates"


def test_projection_round_trip_under_rotation():
    from math import sqrt
    # 90 deg about x: camera +z -> map -y, +y -> map +z
    pose = CameraPose((0.5, 0.0, 1.0), (0.7071067811865476, 0.0, 0.0, 0.7071067811865476))
    point_camera = (0.1, 0.05, 2.0)
    from openflex_roboos_bridge.target_search_contract import camera_to_map
    point_map = camera_to_map(pose, point_camera)
    back = map_to_camera(pose, point_map)
    assert back == pytest.approx(point_camera, abs=1e-9)
    u, v, z = project_point(pose, INTRINSICS, point_map)
    assert z == pytest.approx(2.0, abs=1e-9)
    assert u == pytest.approx(32.0 + 50.0 * 0.1 / 2.0, abs=1e-6)


def test_oracle_observation_is_not_an_input_to_selection():
    rgb, depth = frame(blobs=((GREY, 0),))
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    resolution = resolve_search_target(analysis, "红色的快递盒")
    # The resolver signature has no oracle argument, and the record says so.
    assert "oracle" not in resolution.model_output
    assert resolution.to_dict()["selection_basis"].endswith("not_oracle_object_id")


def test_fixture_decoration_elsewhere_is_ignored():
    rgb, depth = frame(blobs=((RED, 8),))
    # saturated fixture trim: a long thin, much nearer red stripe
    rgb[44:46, 0:60] = RED
    depth[44:46, 0:60] = 1200
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    resolution = resolve_search_target(analysis, "红色的快递盒")
    assert resolution.outcome == "authorized"
    # and the stripe itself is filtered by the aspect-ratio gate
    assert all(candidate.depth_m in (None, 2.0) for candidate in analysis.candidates)


def test_near_slot_but_wrong_depth_is_not_authorized():
    rgb, depth = frame(blobs=((GREY, 0),))
    u, v = SLOT_PX
    rgb[v - 4:v + 4, u - 22:u - 14] = RED   # compact red block, near but not at slot
    depth[v - 4:v + 4, u - 22:u - 14] = 1200  # and much nearer than the slot
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    resolution = resolve_search_target(analysis, "红色的快递盒")
    assert resolution.outcome == "failed"
    assert resolution.selected_object_id is None
