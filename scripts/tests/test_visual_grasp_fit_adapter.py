import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.target_observation import TargetObservation  # noqa: E402
from openflex_roboos_bridge.visual_grasp_input import VisualGraspPointCloudInput  # noqa: E402
from openflex_roboos_bridge.visual_grasp_fit_adapter import (  # noqa: E402
    FitGraspRequestPayload,
    FitGraspResult,
    VisualGraspFitError,
    grasp_candidates_from_result,
)
from openflex_roboos_bridge.visual_grasp_scene import (  # noqa: E402
    build_visual_grasp_plan_request,
)

CAMERA_FRAME = "thesis_head_color_optical_frame"
SIM_SHELF_SCENE = (
    ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/sim_shelf_scene.yaml"
)


def observation(**overrides):
    values = dict(
        perception_track_id="track_red_001", task_target="red", color="red",
        bbox_px=(10, 10, 30, 30), position_camera_xyz=(0.1, 0.0, 1.0),
        position_map_xyz=(1.0, 2.0, 0.8), depth_valid_fraction=0.95,
        timestamp_sec=12.5, camera_frame=CAMERA_FRAME, map_frame="map",
        tf_valid=True, confidence=0.9, metadata={"arm_name": "right"},
    )
    values.update(overrides)
    return TargetObservation(**values)


def visual_input():
    product = np.zeros((100, 6), dtype=np.float32)
    product[:, 0] = np.linspace(0.05, 0.15, 100)
    product[:, 1] = np.linspace(-0.10, 0.00, 100)
    product[:, 2] = np.linspace(0.95, 1.05, 100)
    scene = np.zeros((120, 3), dtype=np.float32)
    return VisualGraspPointCloudInput(observation(), product, scene, CAMERA_FRAME, 12.5)


def result(**overrides):
    values = dict(
        success=True, frame_id=CAMERA_FRAME, timestamp_sec=12.5,
        grasp_position_camera_xyz=(0.10, -0.05, 1.00),
        grasp_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        required_width_m=0.05, score=0.9, candidate_count=1,
    )
    values.update(overrides)
    return FitGraspResult(**values)


def test_request_payload_mirrors_fitgrasp_goal_without_simulator_identity():
    payload = FitGraspRequestPayload.from_visual_input(
        visual_input(), grasp_strategy="top_down").to_dict()
    assert payload["detection_id"] == "track_red_001"
    assert payload["arm_name"] == "right"
    assert payload["grasp_strategy"] == "top_down"
    assert payload["frame_id"] == CAMERA_FRAME
    assert payload["oracle_input"] is False
    assert "object_id" not in payload
    assert payload["product_xyz"].shape == (100, 3)


def test_result_becomes_planning_candidate_in_camera_frame():
    candidates = grasp_candidates_from_result(result(), visual_input())
    assert len(candidates) == 1
    candidate = candidates[0]
    assert candidate.candidate_id == "track_red_001_0"
    assert candidate.position_camera_xyz == pytest.approx((0.10, -0.05, 1.00))
    assert candidate.required_width_m == pytest.approx(0.05)


def test_rejects_failed_or_empty_result():
    with pytest.raises(VisualGraspFitError, match="failure"):
        grasp_candidates_from_result(result(success=False, error_code=7), visual_input())
    with pytest.raises(VisualGraspFitError, match="no candidates"):
        grasp_candidates_from_result(result(candidate_count=0), visual_input())


def test_rejects_frame_or_timestamp_mismatch():
    with pytest.raises(VisualGraspFitError, match="frame"):
        grasp_candidates_from_result(result(frame_id="other_frame"), visual_input())
    with pytest.raises(VisualGraspFitError, match="same frame"):
        grasp_candidates_from_result(result(timestamp_sec=13.0), visual_input())


def test_rejects_oracle_source_and_bad_quaternion():
    with pytest.raises(VisualGraspFitError, match="oracle"):
        grasp_candidates_from_result(
            result(source="mujoco_ground_truth_oracle"), visual_input())
    with pytest.raises(VisualGraspFitError, match="unit quaternion"):
        grasp_candidates_from_result(
            result(grasp_quaternion_xyzw=(0.0, 0.0, 0.0, 0.0)), visual_input())


def test_rejects_object_id_string_instead_of_visual_input():
    with pytest.raises(VisualGraspFitError, match="VisualGraspPointCloudInput"):
        FitGraspRequestPayload.from_visual_input("thesis_item_01")
    with pytest.raises(VisualGraspFitError, match="VisualGraspPointCloudInput"):
        grasp_candidates_from_result(result(), "thesis_item_01")


def test_end_to_end_visual_chain_to_plan_request():
    capture = visual_input()
    payload = FitGraspRequestPayload.from_visual_input(capture).to_dict()
    assert payload["detection_id"] == "track_red_001"
    candidates = grasp_candidates_from_result(result(), capture)
    request = build_visual_grasp_plan_request(
        capture, planning_frame="map", shelf_scene=SIM_SHELF_SCENE,
        tf_parent_frame="map", tf_child_frame=CAMERA_FRAME,
        tf_translation_xyz=(0.0, 0.0, 0.0), tf_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        candidates=candidates, joint_names=("openarmx_right_joint1",),
        joint_positions=(0.0,))
    plan = request.to_planning_payload()
    assert plan["planning_request"] is True
    assert plan["object_memory_used"] is False
    assert plan["candidates"][0]["position_planning_xyz"] == pytest.approx([0.10, -0.05, 1.00])
    assert "object_id" not in plan
