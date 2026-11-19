import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.target_observation import TargetObservation  # noqa: E402
from openflex_roboos_bridge.visual_grasp_input import VisualGraspPointCloudInput  # noqa: E402
from openflex_roboos_bridge.visual_grasp_planning import (  # noqa: E402
    Aabb,
    FrameTransform,
    GraspCandidatePose,
    VisualGraspPlanRequest,
    VisualGraspPlanningError,
    aabb_from_pointcloud,
    transform_aabb,
)


def observation(**overrides):
    values = dict(
        perception_track_id="track_red_001", task_target="red", color="red",
        bbox_px=(10, 10, 30, 30), position_camera_xyz=(0.1, 0.0, 1.0),
        position_map_xyz=(1.0, 2.0, 0.8), depth_valid_fraction=0.95,
        timestamp_sec=12.5, camera_frame="thesis_head_color_optical_frame",
        map_frame="map", tf_valid=True, confidence=0.9, metadata={"arm_name": "right"},
    )
    values.update(overrides)
    return TargetObservation(**values)


def clouds():
    product = np.zeros((100, 6), dtype=np.float32)
    product[:, 0] = np.linspace(0.05, 0.15, 100)
    product[:, 1] = np.linspace(-0.10, 0.00, 100)
    product[:, 2] = np.linspace(0.95, 1.05, 100)
    scene = np.zeros((120, 3), dtype=np.float32)
    scene[:, 2] = 0.5
    return product, scene


def identity_transform():
    return FrameTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def quarter_turn_z():
    half = math.pi / 4
    return FrameTransform((1.0, 2.0, 3.0), (0.0, 0.0, math.sin(half), math.cos(half)))


def visual_input():
    product, scene = clouds()
    return VisualGraspPointCloudInput(
        observation(), product, scene, "thesis_head_color_optical_frame", 12.5)


def candidate(**overrides):
    values = dict(candidate_id="red-g5", position_camera_xyz=(0.0, 0.0, 1.0),
                  quaternion_xyzw=(0.0, 0.0, 0.0, 1.0), score=0.9, required_width_m=0.05)
    values.update(overrides)
    return GraspCandidatePose(**values)


def shelf_obstacle():
    return Aabb((-0.5, -0.5, 0.0), (0.5, 0.5, 1.0))


def request(**overrides):
    values = dict(
        observation=observation(), planning_frame="world",
        camera_frame="thesis_head_color_optical_frame",
        camera_to_planning=identity_transform(),
        target_aabb_planning=Aabb((0.0, -0.1, 0.9), (0.2, 0.1, 1.1)),
        static_obstacles_planning=(shelf_obstacle(),),
        candidates=(candidate(),),
        joint_names=("openarmx_right_joint1", "lift_joint"),
        joint_positions=(0.1, 0.2),
    )
    values.update(overrides)
    return VisualGraspPlanRequest(**values)


def test_payload_is_a_ros_free_request_without_simulator_identity():
    payload = request().to_planning_payload()
    assert payload["planning_request"] is True
    assert payload["object_memory_used"] is False
    assert payload["oracle_input"] is False
    assert payload["physical_verification"] is False
    assert payload["perception_track_id"] == "track_red_001"
    assert "object_id" not in payload
    assert payload["collision_scene"]["static_obstacles"][0]["min_xyz"] == [-0.5, -0.5, 0.0]


def test_camera_cloud_becomes_independent_target_collision_volume():
    product, _ = clouds()
    spread_min = product[:, :3].min(axis=0)
    spread_max = product[:, :3].max(axis=0)
    box = aabb_from_pointcloud(product, padding_m=0.01)
    assert box.min_xyz == pytest.approx(spread_min - 0.01)
    assert box.max_xyz == pytest.approx(spread_max + 0.01)

    identity = VisualGraspPlanRequest.from_visual_input(
        visual_input(), planning_frame="world", camera_to_planning=identity_transform(),
        static_obstacles_planning=(shelf_obstacle(),), candidates=(candidate(),),
        joint_names=("openarmx_right_joint1",), joint_positions=(0.0,))
    assert identity.target_aabb_planning.min_xyz == pytest.approx(spread_min)
    assert identity.target_aabb_planning.max_xyz == pytest.approx(spread_max)

    rotated = VisualGraspPlanRequest.from_visual_input(
        visual_input(), planning_frame="world", camera_to_planning=quarter_turn_z(),
        static_obstacles_planning=(shelf_obstacle(),), candidates=(candidate(),),
        joint_names=("openarmx_right_joint1",), joint_positions=(0.0,))
    # 90 deg about +z adds translation (1, 2, 3) and swaps the x/y extents
    assert rotated.target_aabb_planning.min_xyz[2] == pytest.approx(3.0 + spread_min[2])
    assert rotated.target_aabb_planning.max_xyz[2] == pytest.approx(3.0 + spread_max[2])


def test_candidate_pose_is_transformed_into_planning_frame():
    payload = request(camera_to_planning=quarter_turn_z(),
                      candidates=(candidate(position_camera_xyz=(0.0, 0.0, 1.0)),)
                      ).to_planning_payload()
    # 90 deg about +z maps (0,0,1) rotated by (0,0,1,0)->(1,0,0)? verify with matrix
    x, y, z = payload["candidates"][0]["position_planning_xyz"]
    assert (x, y, z) == pytest.approx((1.0, 2.0, 4.0), abs=1e-6)
    qx, qy, qz, qw = payload["candidates"][0]["quaternion_xyzw_planning"]
    assert qz == pytest.approx(math.sin(math.pi / 4), abs=1e-6)
    assert qw == pytest.approx(math.cos(math.pi / 4), abs=1e-6)
    assert (qx, qy) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_transform_aabb_is_conservative_for_rotated_box():
    box = Aabb((-1.0, -1.0, 0.0), (1.0, 1.0, 0.5))
    transformed = transform_aabb(box, quarter_turn_z())
    assert transformed.min_xyz[0] == pytest.approx(1.0 - 1.0, abs=1e-6)
    assert transformed.max_xyz[0] == pytest.approx(1.0 + 1.0, abs=1e-6)
    assert transformed.min_xyz[1] == pytest.approx(2.0 - 1.0, abs=1e-6)


def test_planning_requires_independent_shelf_collision_geometry():
    with pytest.raises(VisualGraspPlanningError, match="static shelf"):
        request(static_obstacles_planning=())


def test_planning_requires_at_least_one_visual_candidate():
    with pytest.raises(VisualGraspPlanningError, match="fitted visual candidate"):
        request(candidates=())


def test_camera_frame_must_match_observation():
    with pytest.raises(VisualGraspPlanningError, match="camera_frame"):
        request(camera_frame="some_other_optical_frame")


def test_oracle_observation_is_rejected_as_planning_input():
    with pytest.raises(VisualGraspPlanningError):
        request(observation=observation(source="mujoco_ground_truth_oracle"))


def test_candidate_pose_and_transform_must_be_valid():
    with pytest.raises(VisualGraspPlanningError, match="unit quaternion"):
        candidate(quaternion_xyzw=(0.0, 0.0, 0.0, 0.0))
    with pytest.raises(VisualGraspPlanningError):
        FrameTransform((0.0, 0.0, 0.0), (0.0, 0.0, 2.0, 0.0))
    with pytest.raises(VisualGraspPlanningError, match="finite"):
        candidate(position_camera_xyz=(0.0, float("nan"), 1.0))


def test_degenerate_aabb_and_joint_mismatch_are_rejected():
    with pytest.raises(VisualGraspPlanningError, match="max must exceed min"):
        Aabb((0.0, 0.0, 0.0), (0.0, 1.0, 1.0))
    with pytest.raises(VisualGraspPlanningError, match="joint_positions"):
        request(joint_positions=(0.1,))


def test_forged_simulator_identity_in_candidate_is_rejected():
    for forged in ("mujoco_thesis_item_01", "thesis_item_01", "object_id_7"):
        with pytest.raises(VisualGraspPlanningError, match="simulator identity"):
            candidate(candidate_id=forged)


def test_planning_input_rejects_object_id_string_instead_of_visual_input():
    with pytest.raises(VisualGraspPlanningError, match="VisualGraspPointCloudInput"):
        VisualGraspPlanRequest.from_visual_input(
            "thesis_item_01", planning_frame="world", camera_to_planning=identity_transform(),
            static_obstacles_planning=(shelf_obstacle(),), candidates=(candidate(),),
            joint_names=("openarmx_right_joint1",), joint_positions=(0.0,))


def test_aabb_from_pointcloud_rejects_non_finite_points():
    product, _ = clouds()
    product[0, 1] = np.nan
    with pytest.raises(VisualGraspPlanningError, match="non-finite"):
        aabb_from_pointcloud(product)
