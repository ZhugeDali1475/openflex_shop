import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.target_search_contract import (  # noqa: E402
    CameraIntrinsics,
    CameraPose,
    TargetSearchError,
    analyze_frame,
    depth_valid_fraction,
    observation_from_resolution,
    project_point,
    resolve_search_target,
)

WIDTH, HEIGHT = 64, 48
INTRINSICS = CameraIntrinsics.from_k(WIDTH, HEIGHT, [50.0, 0.0, 32.0, 0.0, 50.0, 24.0, 0.0, 0.0, 1.0])
CAMERA = CameraPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
SLOT = (0.0, 0.0, 2.0)
SLOT_PX = (32, 24)
GREY, RED, BLUE = (150, 150, 150), (220, 40, 40), (40, 60, 220)


def scene(*, blob_color=RED, blob_px=SLOT_PX, blob_mm=2000, background_mm=3000, blob_size=10):
    rgb = np.full((HEIGHT, WIDTH, 3), GREY, dtype=np.uint8)
    depth = np.full((HEIGHT, WIDTH), background_mm, dtype=np.uint16)
    u, v = blob_px
    depth[v - 10:v + 10, u - 10:u + 10] = blob_mm
    y0, y1 = v - blob_size // 2, v + blob_size // 2
    x0, x1 = u - blob_size // 2, u + blob_size // 2
    rgb[y0:y1, x0:x1] = blob_color
    depth[y0:y1, x0:x1] = blob_mm
    return rgb, depth


def observed(rgb=None, depth=None):
    rgb, depth = (rgb if rgb is not None else scene()[0], depth if depth is not None else scene()[1])
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA)  # no slot prior
    return analysis, resolve_search_target(analysis, "red box", color="red")


def test_depth_valid_fraction_counts_usable_returns():
    _, depth = scene(background_mm=0)
    assert depth_valid_fraction(depth) == pytest.approx(400.0 / (WIDTH * HEIGHT))
    rgb, full = scene()
    assert depth_valid_fraction(full) == 1.0


def test_authorized_full_frame_resolution_becomes_visual_observation():
    rgb, depth = scene()
    analysis, resolution = observed(rgb, depth)
    assert resolution.outcome == "authorized"
    assert analysis.region.expected_px is None
    observation = observation_from_resolution(
        resolution, intrinsics=INTRINSICS, camera_pose=CAMERA, timestamp_sec=5.0,
        camera_frame="thesis_head_color_optical_frame", map_frame="map", depth_frame_mm=depth)
    assert observation.perception_track_id.startswith("track_")
    assert observation.perception_track_id == resolution.selected_perception_track_id
    assert observation.task_target == "red"
    assert abs(observation.position_camera_xyz[2] - 2.0) < 0.05
    assert abs(observation.position_map_xyz[2] - 2.0) < 0.05
    assert observation.metadata["slot_coordinates_used"] is False
    assert observation.metadata["mujoco_object_id_used"] is False
    assert resolution.selected_object_id is None


@pytest.mark.parametrize("case,expected_reason", [
    ("wrong_class", "wrong_class"),
    ("ambiguous", "ambiguous_candidates"),
    ("absent", "target_absent"),
    ("occluded_looking", "target_absent"),
])
def test_full_frame_authorization_never_reads_a_slot(case, expected_reason):
    """Without any look-at region the search only decides what it can see."""

    if case == "wrong_class":
        rgb, depth = scene(blob_color=BLUE)
    elif case == "ambiguous":
        rgb, depth = scene()
        rgb[SLOT_PX[1] - 4:SLOT_PX[1] + 4, SLOT_PX[0] + 14:SLOT_PX[0] + 22] = RED
        depth[SLOT_PX[1] - 4:SLOT_PX[1] + 4, SLOT_PX[0] + 14:SLOT_PX[0] + 22] = 2000
    else:
        rgb, depth = scene(blob_color=GREY)
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA)
    assert analysis.region.expected_px is None
    resolution = resolve_search_target(analysis, "red box", color="red")
    assert resolution.outcome == "failed"
    assert resolution.reason == expected_reason
    with pytest.raises(TargetSearchError):
        observation_from_resolution(
            resolution, intrinsics=INTRINSICS, camera_pose=CAMERA, timestamp_sec=5.0,
            camera_frame="thesis_head_color_optical_frame", map_frame="map")


@pytest.mark.parametrize("case,expected_reason", [
    ("occluded", "target_occluded"),
    ("empty", "empty_slot"),
])
def test_region_anchored_diagnostics_separate_occluded_and_empty(case, expected_reason):
    """Occluded vs empty needs a caller-supplied look-at region, never a grasp."""

    if case == "occluded":
        rgb, depth = scene(blob_color=GREY, blob_mm=1500)
    else:
        rgb, depth = scene(blob_color=GREY, blob_mm=3000)
    analysis = analyze_frame(rgb, depth, INTRINSICS, CAMERA, expected_point_map=SLOT)
    resolution = resolve_search_target(analysis, "red box", color="red")
    assert analysis.region.state in ("occluded", "empty")
    assert resolution.outcome == "failed"
    assert resolution.reason == expected_reason
    with pytest.raises(TargetSearchError):
        observation_from_resolution(
            resolution, intrinsics=INTRINSICS, camera_pose=CAMERA, timestamp_sec=5.0,
            camera_frame="thesis_head_color_optical_frame", map_frame="map")


def test_invalid_depth_never_authorizes_or_becomes_an_observation():
    rgb, depth = scene(background_mm=0, blob_mm=0)   # no depth return anywhere
    rgb[SLOT_PX[1] - 5:SLOT_PX[1] + 5, SLOT_PX[0] - 5:SLOT_PX[0] + 5] = RED
    analysis, resolution = observed(rgb, depth)
    assert analysis.model_output["depth_valid_fraction"] == 0.0
    assert resolution.outcome == "failed"
    # The target-colour blob exists but carries no depth return, so it is not a
    # depth-confirmed candidate and must not authorize a grasp.
    assert resolution.reason == "below_confidence"
    with pytest.raises(TargetSearchError):
        observation_from_resolution(
            resolution, intrinsics=INTRINSICS, camera_pose=CAMERA, timestamp_sec=5.0,
            camera_frame="thesis_head_color_optical_frame", map_frame="map", depth_frame_mm=depth)


def test_observation_from_resolution_requires_a_live_depth_frame():
    _, resolution = observed()
    with pytest.raises(TargetSearchError):
        observation_from_resolution(
            resolution, intrinsics=INTRINSICS, camera_pose=CAMERA, timestamp_sec=5.0,
            camera_frame="thesis_head_color_optical_frame", map_frame="map",
            depth_frame_mm=np.zeros((HEIGHT, WIDTH), dtype=np.uint16))


def test_random_slots_localize_without_slot_priors():
    rng = np.random.default_rng(20261120)
    for _ in range(24):
        u = int(rng.integers(10, WIDTH - 10))
        v = int(rng.integers(10, HEIGHT - 10))
        depth_mm = int(rng.integers(700, 2500))
        rgb, depth = scene(blob_px=(u, v), blob_mm=depth_mm, blob_size=9)
        analysis, resolution = observed(rgb, depth)
        assert analysis.region.expected_px is None, "search must not use a slot prior"
        assert resolution.outcome == "authorized", resolution.reason
        observation = observation_from_resolution(
            resolution, intrinsics=INTRINSICS, camera_pose=CAMERA, timestamp_sec=1.0,
            camera_frame="thesis_head_color_optical_frame", map_frame="map", depth_frame_mm=depth)
        projected = project_point(CAMERA, INTRINSICS, observation.position_map_xyz)
        assert projected is not None
        assert abs(projected[0] - u) <= 3.0 and abs(projected[1] - v) <= 3.0
        assert abs(observation.position_map_xyz[2] - depth_mm / 1000.0) < 0.08
