import importlib.util
import json
from pathlib import Path
import sys

import mujoco
import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.scene_registry import load_scene_registry
from openflex_roboos_bridge.navigation_guard import NavigationGuard


SPEC = importlib.util.spec_from_file_location(
    "prepare_openflex_single_red_runtime",
    ROOT / "scripts/prepare_openflex_single_red_runtime.py",
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
SOURCE = ROOT / "local_dependencies/openflex_thesis/retail_fixture_single_red_20261113_v3"


def test_profile_maps_scene_frames_and_marks_motion_candidates_unvalidated(tmp_path):
    output = tmp_path / "runtime_profile"
    report = BUILDER.prepare_runtime_profile(SOURCE, output)
    config = json.loads((output / "profile_audit.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(output / "scene.xml"))
    data = mujoco.MjData(model)
    parameters = __import__("yaml").safe_load((output / "runtime.yaml").read_text())
    bridge = parameters["openflex_thesis_mujoco_bridge"]["ros__parameters"]
    mission = parameters["thesis_mujoco_mission_client"]["ros__parameters"]
    for name, value in zip(("base_x", "base_y", "base_yaw"), bridge["initial_base_pose"]):
        joint = model.joint(name)
        data.qpos[joint.qposadr[0]] = value
    mujoco.mj_forward(model, data)

    assert report["execution_enabled"] is False
    assert report["simulation_steps"] == 0
    assert data.xpos[model.body("openflex_robot_yaw_root").id].tolist() == pytest.approx(
        [0.0, 3.6, 0.17], abs=1e-6
    )
    assert model.body("thesis_shelf").id > 0
    assert model.body("thesis_delivery_table").id > 0
    assert model.geom("thesis_delivery_top").id > 0
    assert bridge["scene_model_path"] == str(output / "scene.xml")
    assert bridge["initial_base_pose"] == pytest.approx([0.0, 3.6, 4.71238898038469])
    assert bridge["object_nominal_pose"][:3] == pytest.approx(
        config["object_world_xyzyaw"][:3]
    )
    assert mission["pickup_navigation_pose"] == pytest.approx([0.2, 0.53, 0.0])
    assert report["pickup_navigation_candidate_xyyaw_unvalidated"] == pytest.approx(
        mission["pickup_navigation_pose"]
    )

    registry = load_scene_registry(output / "runtime.yaml", output / "scene.xml")
    station = registry["stations"]["delivery_zone"]
    affordance = registry["affordances"]["delivery_zone"]
    assert station["validation"] == "configured_only_path_not_validated"
    assert affordance["geom"] == "thesis_delivery_top"
    assert affordance["surface_center_xyz"] == pytest.approx(
        report["delivery_surface_world_center_xyz_m"]
    )
    assert affordance["upright_center_height_consistent"] is True

    guard = NavigationGuard(output / "scene.xml")
    joint_names, joint_positions = [], []
    for joint_id in range(guard.model.njnt):
        if guard.model.jnt_type[joint_id] not in (
                mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        name = mujoco.mj_id2name(guard.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name in {"base_x", "base_y", "base_yaw"}:
            continue
        joint_names.append(name)
        joint_positions.append(float(guard.model.qpos0[guard.model.jnt_qposadr[joint_id]]))
    joints = type("JointStateFixture", (), {"name": joint_names, "position": joint_positions})()
    pickup = mission["pickup_navigation_pose"]
    direct = guard.check(joints, pickup, station["approach_candidate_xyyaw"])
    routed = guard.check(joints, pickup, station["approach_candidate_xyyaw"],
                         via=station["route_waypoints"])
    assert direct["accepted"] is False
    assert routed["accepted"] is True
    assert routed["samples"] > direct["samples"]
    assert routed["scope"] == "empty_robot_static_scene_sampled_translation_then_rotation"


def test_profile_refuses_to_overwrite_existing_directory(tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    marker = output / "keep.txt"
    marker.write_text("preserve")
    with pytest.raises(FileExistsError):
        BUILDER.prepare_runtime_profile(SOURCE, output)
    assert marker.read_text() == "preserve"
