from types import SimpleNamespace

import mujoco
import pytest

from openflex_thesis_mujoco.bridge_node import MujocoBridgeNode
from openflex_thesis_mujoco.contact_audit import forbidden_environment_contact, forbidden_payload_contact
from openflex_thesis_mujoco.model_builder import build_thesis_model

from test_model_builder import upstream_model


def test_contact_filter_includes_inactive_robot_but_not_supported_object():
    assert forbidden_environment_contact((2, 10), -0.001, {1, 2, 3}, {10, 11})
    assert forbidden_environment_contact((10, 2), 0.0, {1, 2, 3}, {10, 11})
    assert not forbidden_environment_contact((20, 10), -0.001, {1, 2, 3}, {10, 11})
    assert not forbidden_environment_contact((2, 10), 0.001, {1, 2, 3}, {10, 11})


def test_held_payload_cannot_pass_through_shelf_table_or_floor():
    assert forbidden_payload_contact((20, 10), -0.001, 20, {10, 11})
    assert forbidden_payload_contact((11, 20), 0.0, 20, {10, 11})
    assert forbidden_payload_contact((20, 0), -0.001, 20, {10, 11})
    assert not forbidden_payload_contact((20, 10), 0.001, 20, {10, 11})
    assert not forbidden_payload_contact((20, 10), -0.001, None, {10, 11})


@pytest.fixture(scope="module")
def model(tmp_path_factory):
    path = tmp_path_factory.mktemp("contact-audit") / "scene.xml"
    return mujoco.MjModel.from_xml_path(str(build_thesis_model(str(upstream_model()), str(path))))


def test_real_mujoco_left_shelf_contact_latches_a_single_stop(model):
    data = mujoco.MjData(model)
    positions = {"base_x": 0.5, "base_y": 0.0, "lift_joint": 0.17}
    positions.update({f"openarmx_left_joint{index}": value
                      for index, value in enumerate([-0.7, -1.2, 0.4, 1.2, 0, 0, 0], 1)})
    for name, value in positions.items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_forward(model, data)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "openflex_robot_yaw_root")
    shelf = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "thesis_shelf")
    logs = []
    bridge = SimpleNamespace(model=model, data=data, safety_stop=None,
        robot_body_ids={index for index in range(model.nbody) if model.body_rootid[index] == root},
        environment_body_ids={shelf}, scalar_joint_names=[], base_pose=[0.5, 0.0, 0.0],
        actual_twist=[0.2, 0.0, 0.0], commanded_twist=[0.2, 0.0, 0.0],
        environment_contact_checks=0, last_contact_audit_time=0.0,
        get_logger=lambda: SimpleNamespace(error=logs.append))
    MujocoBridgeNode._audit_environment_contacts(bridge)
    MujocoBridgeNode._audit_environment_contacts(bridge)
    assert len(logs) == 1 and "MUJOCO_SAFETY_STOP" in logs[0]
    assert bridge.safety_stop["contacts"]
    assert bridge.actual_twist == [0.0, 0.0, 0.0]
    assert all("thesis_shelf" in contact["bodies"] for contact in bridge.safety_stop["contacts"])
