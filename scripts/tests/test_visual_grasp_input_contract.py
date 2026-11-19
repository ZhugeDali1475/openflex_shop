import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.target_observation import (  # noqa: E402
    TargetObservation,
    observation_from_candidate,
)
from openflex_roboos_bridge.visual_grasp_input import (  # noqa: E402
    VisualGraspInputError,
    VisualGraspPointCloudInput,
)


def observation(**overrides):
    values = dict(
        perception_track_id="track_red_001", task_target="red", color="red",
        bbox_px=(10, 10, 30, 30), position_camera_xyz=(0.1, 0.0, 1.0),
        position_map_xyz=(1.0, 2.0, 0.8), depth_valid_fraction=0.95,
        timestamp_sec=12.5, camera_frame="camera_optical", map_frame="map",
        tf_valid=True, confidence=0.9, metadata={"arm_name": "right"},
    )
    values.update(overrides)
    return TargetObservation(**values)


def clouds():
    product = np.zeros((100, 6), dtype=np.float32)
    product[:, 2] = 1.0
    scene = np.zeros((120, 3), dtype=np.float32)
    scene[:, 2] = 1.0
    return product, scene


def test_visual_point_clouds_serialize_without_simulator_identity():
    product, scene = clouds()
    value = VisualGraspPointCloudInput(
        observation(), product, scene, "camera_optical", 12.5
    )
    payload = value.to_fit_grasp_payload()
    assert payload["perception_track_id"] == "track_red_001"
    assert payload["arm_name"] == "right"
    assert payload["oracle_input"] is False
    assert "object_id" not in payload
    assert payload["product_xyz"].shape == (100, 3)


def test_search_candidate_observation_flows_into_visual_grasp_contract():
    from types import SimpleNamespace

    candidate = SimpleNamespace(
        pose_xyzyaw=(1.0, 2.0, 0.8, 0.0), bbox_px=(1, 2, 10, 12),
        candidate_id="red-1", color="red", confidence=0.9,
    )
    found = observation_from_candidate(
        candidate, task_target="red", timestamp_sec=12.5,
        camera_frame="camera_optical", map_frame="map",
        position_camera_xyz=(0.1, 0.0, 1.0), track_id="track_red_001",
        arm_name="right",
    )
    product, scene = clouds()
    adapted = VisualGraspPointCloudInput(
        found, product, scene, "camera_optical", 12.5
    ).to_fit_grasp_payload()
    assert adapted["arm_name"] == "right"
    assert adapted["perception_track_id"] == "track_red_001"


@pytest.mark.parametrize("kwargs", [
    {"frame_id": "other_frame"},
    {"timestamp_sec": 13.0},
])
def test_point_cloud_metadata_must_match_observation(kwargs):
    product, scene = clouds()
    with pytest.raises(VisualGraspInputError):
        VisualGraspPointCloudInput(
            observation(), product, scene,
            kwargs.get("frame_id", "camera_optical"),
            kwargs.get("timestamp_sec", 12.5),
        )


def test_insufficient_or_nonfinite_cloud_is_rejected():
    product, scene = clouds()
    with pytest.raises(VisualGraspInputError, match="at least"):
        VisualGraspPointCloudInput(observation(), product[:99], scene, "camera_optical", 12.5)
    scene[0, 0] = np.nan
    with pytest.raises(VisualGraspInputError, match="non-finite"):
        VisualGraspPointCloudInput(observation(), product, scene, "camera_optical", 12.5)


def test_arm_context_is_required_and_inputs_are_not_mutated():
    product, scene = clouds()
    with pytest.raises(VisualGraspInputError, match="arm"):
        VisualGraspPointCloudInput(
            observation(metadata={}), product, scene, "camera_optical", 12.5
        )
    value = VisualGraspPointCloudInput(
        observation(), product, scene, "camera_optical", 12.5
    )
    assert product.flags.writeable and scene.flags.writeable
    assert not value.product_xyz.flags.writeable
