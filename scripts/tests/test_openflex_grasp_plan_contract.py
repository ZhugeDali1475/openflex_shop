from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.grasp_plan_contract import (  # noqa: E402
    GraspCandidate,
    GraspPlanError,
    evaluate_grasp_candidate,
)


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
