import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.target_observation import TargetObservation  # noqa: E402
from openflex_roboos_bridge.visual_grasp_input import VisualGraspPointCloudInput  # noqa: E402
from openflex_roboos_bridge.visual_grasp_planning import GraspCandidatePose  # noqa: E402
from openflex_roboos_bridge.visual_grasp_scene import (  # noqa: E402
    ShelfSceneError,
    build_visual_grasp_plan_request,
    frame_transform_from_tf,
    load_shelf_scene,
)

SIM_SHELF_SCENE = (
    ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/sim_shelf_scene.yaml"
)
CAMERA_FRAME = "thesis_head_color_optical_frame"


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


def candidate():
    return GraspCandidatePose("red-g5", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), 0.9, 0.05)


def test_loads_tracked_sim_shelf_scene_with_frame_and_calibration():
    scene = load_shelf_scene(SIM_SHELF_SCENE)
    assert scene.frame_id == "map"
    assert scene.calibrated is False
    assert len(scene.object_ids) == 6
    assert scene.object_ids[0] == "mujoco_shelf_back"
    back = scene.obstacles[0]
    assert back.min_xyz == pytest.approx((1.435, -0.65, 0.0))
    assert back.max_xyz == pytest.approx((1.485, 0.65, 2.10))


def test_rotated_box_becomes_a_conservative_aabb():
    scene = load_shelf_scene({
        "frame_id": "map", "calibrated": False, "collision_objects": [{
            "id": "rotated", "shape": "box", "dimensions": [1.0, 2.0, 3.0],
            "position": [0.0, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)],
        }],
    })
    box = scene.obstacles[0]
    assert box.min_xyz == pytest.approx((-1.0, -0.5, -1.5), abs=1e-6)
    assert box.max_xyz == pytest.approx((1.0, 0.5, 1.5), abs=1e-6)


def test_rejects_malformed_scene_documents():
    with pytest.raises(ShelfSceneError, match="frame_id"):
        load_shelf_scene({"frame_id": "", "collision_objects": [{"id": "a", "shape": "box",
            "dimensions": [1, 1, 1], "position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}]})
    with pytest.raises(ShelfSceneError, match="no collision objects"):
        load_shelf_scene({"frame_id": "map", "collision_objects": []})
    with pytest.raises(ShelfSceneError, match="only box"):
        load_shelf_scene({"frame_id": "map", "collision_objects": [{"id": "a", "shape": "sphere",
            "dimensions": [1, 1, 1], "position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}]})
    with pytest.raises(ShelfSceneError, match="positive"):
        load_shelf_scene({"frame_id": "map", "collision_objects": [{"id": "a", "shape": "box",
            "dimensions": [1, 0, 1], "position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}]})
    with pytest.raises(ShelfSceneError, match="unique"):
        load_shelf_scene({"frame_id": "map", "collision_objects": [
            {"id": "a", "shape": "box", "dimensions": [1, 1, 1], "position": [0, 0, 0],
             "orientation_xyzw": [0, 0, 0, 1]},
            {"id": "a", "shape": "box", "dimensions": [1, 1, 1], "position": [1, 1, 1],
             "orientation_xyzw": [0, 0, 0, 1]}]})


def test_frame_transform_requires_planning_parent_and_camera_child():
    transform = frame_transform_from_tf(
        planning_frame="map", camera_frame=CAMERA_FRAME, parent_frame="map",
        child_frame=CAMERA_FRAME, translation_xyz=(1.0, 2.0, 3.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
    assert transform.translation_xyz == (1.0, 2.0, 3.0)
    with pytest.raises(ShelfSceneError, match="parent_frame"):
        frame_transform_from_tf(planning_frame="map", camera_frame=CAMERA_FRAME,
            parent_frame="odom", child_frame=CAMERA_FRAME,
            translation_xyz=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ShelfSceneError, match="child_frame"):
        frame_transform_from_tf(planning_frame="map", camera_frame=CAMERA_FRAME,
            parent_frame="map", child_frame="other_camera",
            translation_xyz=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ShelfSceneError, match="unit quaternion"):
        frame_transform_from_tf(planning_frame="map", camera_frame=CAMERA_FRAME,
            parent_frame="map", child_frame=CAMERA_FRAME,
            translation_xyz=(0.0, 0.0, 0.0), quaternion_xyzw=(0.0, 0.0, 2.0, 0.0))


def test_builds_plan_request_from_measured_tf_and_real_scene():
    request = build_visual_grasp_plan_request(
        visual_input(), planning_frame="map", shelf_scene=SIM_SHELF_SCENE,
        tf_parent_frame="map", tf_child_frame=CAMERA_FRAME,
        tf_translation_xyz=(0.0, 0.0, 0.0), tf_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        candidates=(candidate(),), joint_names=("openarmx_right_joint1",),
        joint_positions=(0.0,))
    payload = request.to_planning_payload()
    assert payload["planning_request"] is True
    assert payload["object_memory_used"] is False
    assert payload["collision_scene"]["static_obstacles"][0]["min_xyz"] == [1.435, -0.65, 0.0]
    assert "object_id" not in payload
    assert request.planning_frame == "map"


def test_build_rejects_scene_frame_mismatch():
    with pytest.raises(ShelfSceneError, match="must match planning frame"):
        build_visual_grasp_plan_request(
            visual_input(), planning_frame="map",
            shelf_scene={"frame_id": "base_link", "collision_objects": [{"id": "a",
                "shape": "box", "dimensions": [1, 1, 1], "position": [0, 0, 0],
                "orientation_xyzw": [0, 0, 0, 1]}]},
            tf_parent_frame="map", tf_child_frame=CAMERA_FRAME,
            tf_translation_xyz=(0.0, 0.0, 0.0), tf_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            candidates=(candidate(),), joint_names=("openarmx_right_joint1",),
            joint_positions=(0.0,))


def test_build_requires_planning_frame_to_match_observation_map_frame():
    with pytest.raises(ShelfSceneError, match="map_frame"):
        build_visual_grasp_plan_request(
            visual_input(), planning_frame="odom",
            shelf_scene={"frame_id": "odom", "collision_objects": [{"id": "a",
                "shape": "box", "dimensions": [1, 1, 1], "position": [0, 0, 0],
                "orientation_xyzw": [0, 0, 0, 1]}]},
            tf_parent_frame="odom", tf_child_frame=CAMERA_FRAME,
            tf_translation_xyz=(0.0, 0.0, 0.0), tf_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            candidates=(candidate(),), joint_names=("openarmx_right_joint1",),
            joint_positions=(0.0,))


def test_build_rejects_object_id_string_instead_of_visual_input():
    with pytest.raises(ShelfSceneError, match="VisualGraspPointCloudInput"):
        build_visual_grasp_plan_request(
            "thesis_item_01", planning_frame="map", shelf_scene=SIM_SHELF_SCENE,
            tf_parent_frame="map", tf_child_frame=CAMERA_FRAME,
            tf_translation_xyz=(0.0, 0.0, 0.0), tf_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            candidates=(candidate(),), joint_names=("openarmx_right_joint1",),
            joint_positions=(0.0,))
