from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.grasp_plan_contract import (  # noqa: E402
    GraspCandidate,
    validate_visual_grasp_input,
    GraspPlanError,
    evaluate_grasp_candidate,
)
from openflex_roboos_bridge.target_observation import TargetObservation, TargetObservationError  # noqa: E402


def visual_observation(**overrides):
    values = dict(perception_track_id="track_red_001", task_target="red", color="red",
                  bbox_px=(1, 2, 10, 12), position_camera_xyz=(0.1, 0.0, 1.2),
                  position_map_xyz=(1.0, 2.0, 0.8), depth_valid_fraction=0.9,
                  timestamp_sec=1.0, camera_frame="camera", map_frame="map",
                  tf_valid=True, confidence=0.95)
    values.update(overrides)
    return TargetObservation(**values)


def test_visual_grasp_input_is_explicitly_non_oracle():
    result = validate_visual_grasp_input(visual_observation())
    assert result["perception_track_id"] == "track_red_001"
    assert result["oracle_input"] is False
    assert result["physical_verification"] is False


def test_visual_grasp_rejects_simulator_identity_or_invalid_observation():
    with pytest.raises(GraspPlanError):
        validate_visual_grasp_input("thesis_item_01")
    with pytest.raises(GraspPlanError):
        validate_visual_grasp_input(visual_observation(tf_valid=False))


def candidate(**updates):
    values = {"candidate_id": "red-g5", "object_id": "thesis_item_01",
              "object_dimensions_m": (0.07, 0.06, 0.16), "mass_kg": 0.22,
              "shelf_id": "shelf_group_2", "face": "neg", "level": 2,
              "pregrasp_joint_positions": (0.1, 0.2),
              "grasp_joint_positions": (0.2, 0.3)}
    values.update(updates)
    return GraspCandidate(**values)


def test_grasp_candidate_records_all_planning_checks():
    result = evaluate_grasp_candidate(candidate(), full_robot_collision_free=True,
                                       ik_valid=True, tc_wam_valid=True,
                                       non_active_arm_clear=True)
    assert result["graspable"] is True
    assert result["grasp_execution_authorized"] is True
    assert result["physical_execution_validated"] is False


def test_failed_check_is_explicit_and_never_authorizes_execution():
    result = evaluate_grasp_candidate(candidate(), full_robot_collision_free=True,
                                       ik_valid=False, tc_wam_valid=True,
                                       non_active_arm_clear=True)
    assert result["graspable"] is False
    assert result["failed_checks"] == ["ik_valid"]
    assert result["grasp_execution_authorized"] is False


@pytest.mark.parametrize("updates", [
    {"object_id": "other"}, {"shelf_id": ""}, {"face": "unknown"},
    {"mass_kg": 0}, {"object_dimensions_m": (0.0, 0.1, 0.1)},
])
def test_invalid_fixture_metadata_is_rejected(updates):
    with pytest.raises(GraspPlanError):
        evaluate_grasp_candidate(candidate(**updates), full_robot_collision_free=True,
                                 ik_valid=True, tc_wam_valid=True,
                                 non_active_arm_clear=True)
