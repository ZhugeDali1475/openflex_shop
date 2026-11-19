import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.target_observation import TargetObservation  # noqa: E402
from openflex_roboos_bridge.visual_grasp_evidence import (  # noqa: E402
    VisualGraspEvidenceError,
    planning_evidence_pair,
    planning_evaluation_evidence,
    planning_perception_evidence,
)
from openflex_roboos_bridge.visual_grasp_planning import (  # noqa: E402
    Aabb,
    FrameTransform,
    GraspCandidatePose,
    VisualGraspPlanRequest,
)

CAMERA_FRAME = "thesis_head_color_optical_frame"


def request():
    observation = TargetObservation(
        perception_track_id="track_red_001", task_target="red", color="red",
        bbox_px=(10, 10, 30, 30), position_camera_xyz=(0.1, 0.0, 1.0),
        position_map_xyz=(1.0, 2.0, 0.8), depth_valid_fraction=0.95,
        timestamp_sec=12.5, camera_frame=CAMERA_FRAME, map_frame="map",
        tf_valid=True, confidence=0.9, metadata={"arm_name": "right"})
    return VisualGraspPlanRequest(
        observation=observation, planning_frame="map", camera_frame=CAMERA_FRAME,
        camera_to_planning=FrameTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        target_aabb_planning=Aabb((0.0, -0.1, 0.9), (0.2, 0.1, 1.1)),
        static_obstacles_planning=(Aabb((-0.5, -0.5, 0.0), (0.5, 0.5, 1.0)),),
        candidates=(GraspCandidatePose("red-g5", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), 0.9, 0.05),),
        joint_names=("openarmx_right_joint1",), joint_positions=(0.0,))


def pair(**overrides):
    values = dict(request_id="req-1", attempt=1, frame_sha256="a" * 64,
                  frame_ref="frame.png", depth_ref="depth.png")
    values.update(overrides)
    return planning_evidence_pair(request(), **values)


def test_pair_separates_perception_and_evaluation_layers():
    evidence = pair()
    assert evidence["perception"]["evidence_layer"] == "perception"
    assert evidence["evaluation"]["evidence_layer"] == "simulation_evaluation"
    assert evidence["perception"]["physical_verification"] is False
    assert evidence["evaluation"]["mujoco_object_id"] is None
    assert evidence["evaluation"]["physical_verification"] is False
    assert evidence["perception"]["record"]["boundary"] == "visual_grasp_planning"
    assert "object_id" not in evidence["perception"]
    assert "object_id" not in evidence["perception"]["record"]["visual_grasp_plan"]
    json.dumps(evidence, ensure_ascii=False)


def test_perception_evidence_rejects_injected_simulator_identity(monkeypatch):
    polluted = request().to_planning_payload()
    polluted["object_id"] = "thesis_item_01"
    monkeypatch.setattr(VisualGraspPlanRequest, "to_planning_payload",
                        lambda self: polluted)
    with pytest.raises(VisualGraspEvidenceError, match="simulator identity"):
        pair()


def test_evaluation_requires_evaluator_object_id_for_physical_verification():
    evidence = planning_evaluation_evidence("req-1", 1)
    assert evidence.to_dict()["physical_verification"] is False
    with pytest.raises(VisualGraspEvidenceError):
        planning_evaluation_evidence("req-1", 1, physical_verification=True).to_dict()
    cleared = planning_evaluation_evidence("req-1", 1, mujoco_object_id="thesis_item_01",
                                           physical_verification=True, source="sim_contact_check")
    assert cleared.to_dict()["mujoco_object_id"] == "thesis_item_01"


def test_perception_evidence_requires_planning_request_and_frame_refs():
    with pytest.raises(VisualGraspEvidenceError, match="VisualGraspPlanRequest"):
        planning_perception_evidence("thesis_item_01", request_id="req-1", attempt=1,
                                     frame_sha256="a" * 64, frame_ref="f", depth_ref="d")
    with pytest.raises(VisualGraspEvidenceError):
        planning_perception_evidence(request(), request_id="", attempt=1,
                                     frame_sha256="a" * 64, frame_ref="f", depth_ref="d")
