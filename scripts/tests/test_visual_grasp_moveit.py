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
)
from openflex_roboos_bridge.visual_grasp_moveit import (  # noqa: E402
    CandidateState,
    CollisionBox,
    MoveItTransportError,
    assert_plan_only_parameters,
    build_moveit_plan,
    collision_object_id,
    visual_grasp_moveit_payload,
)

PREFLIGHT_MOVE_GROUP_PARAMETERS = {
    "allow_trajectory_execution": False,
    "moveit_manage_controllers": False,
}


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


def visual_input():
    product = np.zeros((100, 6), dtype=np.float32)
    product[:, 0] = np.linspace(0.05, 0.15, 100)
    product[:, 1] = np.linspace(-0.10, 0.00, 100)
    product[:, 2] = np.linspace(0.95, 1.05, 100)
    scene = np.zeros((120, 3), dtype=np.float32)
    scene[:, 2] = 0.5
    return VisualGraspPointCloudInput(
        observation(), product, scene, "thesis_head_color_optical_frame", 12.5)


def plan_request(transform=None):
    return VisualGraspPlanRequest.from_visual_input(
        visual_input(),
        planning_frame="map",
        camera_to_planning=transform or FrameTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        static_obstacles_planning=[Aabb((-0.5, -0.5, 0.0), (0.5, 0.5, 1.0))],
        candidates=[GraspCandidatePose(
            candidate_id="red-g5", position_camera_xyz=(0.0, 0.0, 1.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0), score=0.9, required_width_m=0.05)],
        joint_names=["lift_joint", "openarmx_right_joint1"],
        joint_positions=[0.1, -0.2],
    )


def test_collision_object_id_is_identity_free():
    assert collision_object_id("track_red_001") == "visual_target_track_red_001"
    assert collision_object_id("track/red:001") == "visual_target_track_red_001"
    with pytest.raises(MoveItTransportError):
        collision_object_id("///")


def test_collision_box_from_aabb_centers_and_sizes():
    box = CollisionBox.from_aabb(Aabb((0.0, 1.0, 2.0), (2.0, 3.0, 5.0)), object_id="b", frame_id="map")
    assert box.position_xyz == (1.0, 2.0, 3.5)
    assert box.dimensions_xyz == (2.0, 2.0, 3.0)
    assert box.to_dict()["shape"] == "box"


def test_build_plan_carries_target_and_shelf_boxes():
    plan = build_moveit_plan(plan_request(), move_group_parameters=PREFLIGHT_MOVE_GROUP_PARAMETERS)
    identifiers = [box.object_id for box in plan.scene_diff]
    assert identifiers[0] == "visual_target_track_red_001"
    assert identifiers[1] == "visual_shelf_00"
    assert all(box.frame_id == "map" for box in plan.scene_diff)
    assert plan.plan_only is True and plan.trajectory_execution is False


def test_build_plan_candidate_states_use_measured_joints():
    plan = build_moveit_plan(plan_request())
    state = plan.candidate_states[0]
    assert state.candidate_id == "red-g5"
    assert state.joint_names == ("lift_joint", "openarmx_right_joint1")
    assert state.joint_positions == (0.1, -0.2)


def test_payload_is_plan_only_and_truth_free():
    payload = visual_grasp_moveit_payload(plan_request())
    assert payload["plan_only"] is True
    assert payload["trajectory_execution"] is False
    assert payload["robot_motion_executed"] is False
    assert payload["object_memory_used"] is False
    text = repr(payload)
    for forbidden in ("object_id", "mujoco", "oracle"):
        assert forbidden not in text


def test_plan_only_parameters_guard():
    assert_plan_only_parameters(PREFLIGHT_MOVE_GROUP_PARAMETERS)
    assert_plan_only_parameters({})
    with pytest.raises(MoveItTransportError):
        assert_plan_only_parameters({"allow_trajectory_execution": True})
    with pytest.raises(MoveItTransportError):
        assert_plan_only_parameters({"moveit_manage_controllers": True})


def test_build_plan_rejects_unsafe_parameters():
    with pytest.raises(MoveItTransportError):
        build_moveit_plan(plan_request(), move_group_parameters={"allow_trajectory_execution": True})


def test_candidate_state_rejects_non_unit_quaternion():
    with pytest.raises(MoveItTransportError):
        CandidateState(candidate_id="c", joint_names=("j",), joint_positions=(0.0,),
                       position_planning_xyz=(0.0, 0.0, 0.0),
                       quaternion_xyzw_planning=(0.0, 0.0, 0.0, 2.0))


def test_scene_boxes_must_share_the_planning_frame():
    plan = build_moveit_plan(plan_request())
    with pytest.raises(MoveItTransportError):
        type(plan)(planning_frame="odom", scene_diff=plan.scene_diff,
                   candidate_states=plan.candidate_states)
