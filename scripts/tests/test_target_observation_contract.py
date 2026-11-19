import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.target_observation import (  # noqa: E402
    TargetObservation,
    TargetObservationError,
)


def valid(**overrides):
    values = dict(
        perception_track_id="track_red_001", task_target="red", color="red",
        bbox_px=(10, 10, 30, 30), position_camera_xyz=(0.1, 0.0, 1.0),
        position_map_xyz=(1.0, 2.0, 0.8), depth_valid_fraction=0.98,
        timestamp_sec=12.5, camera_frame="camera_optical", map_frame="map",
        tf_valid=True, confidence=0.92,
    )
    values.update(overrides)
    return TargetObservation(**values)


def test_visual_observation_is_graspable_and_has_no_sim_identity():
    observation = valid()
    observation.validate_for_grasp()
    record = observation.to_dict()
    assert record["perception_track_id"] == "track_red_001"
    assert record["mujoco_object_id"] is None
    assert record["physical_verification"] is False


def test_candidate_observation_can_record_explicit_manipulation_arm():
    from types import SimpleNamespace
    from openflex_roboos_bridge.target_observation import observation_from_candidate

    candidate = SimpleNamespace(
        pose_xyzyaw=(1.0, 2.0, 0.8, 0.0),
        bbox_px=(10, 10, 30, 30),
        candidate_id="red-1",
        color="red",
        confidence=0.9,
    )
    result = observation_from_candidate(
        candidate, task_target="red", timestamp_sec=12.5,
        camera_frame="camera_optical", map_frame="map",
        position_camera_xyz=(0.1, 0.0, 1.0), arm_name="right",
    )
    assert result.metadata["arm_name"] == "right"
    assert result.to_dict()["mujoco_object_id"] is None


@pytest.mark.parametrize("overrides", [
    {"tf_valid": False},
    {"depth_valid_fraction": 0.0},
    {"position_map_xyz": (float("nan"), 0.0, 1.0)},
    {"source": "mujoco_ground_truth_oracle_NOT_PHYSICAL"},
    {"perception_track_id": "mujoco_object_01"},
])
def test_invalid_oracle_like_observation_is_rejected(overrides):
    with pytest.raises(TargetObservationError):
        valid(**overrides).validate_for_grasp()
