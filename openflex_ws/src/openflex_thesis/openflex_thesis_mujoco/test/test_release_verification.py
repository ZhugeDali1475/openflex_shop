from pathlib import Path
import threading
import time
from types import MethodType, SimpleNamespace

import mujoco
import numpy as np
from openflex_thesis_msgs.msg import ControlMode, FixedPlaceControl
import pytest

from openflex_thesis_mujoco.bridge_node import GRIPPER_JOINTS, MujocoBridgeNode
from openflex_thesis_mujoco.model_builder import build_thesis_model
from openflex_thesis_mujoco.release_verification import (
    box_world_extents, object_settled_on_surface, release_request_rejection,
)


HALF_SIZE = (0.035, 0.03, 0.08)
SURFACE = (-0.85, -0.8, 0.455)
SURFACE_HALF = (0.35, 0.35, 0.025)
TARGET = (-0.85, -0.8, 0.56)
UPRIGHT = (0.0, 0.0, 0.0, 1.0)


def settled(**changes):
    arguments = dict(position=TARGET, orientation=UPRIGHT, half_size=HALF_SIZE, target=TARGET,
                     surface_center=SURFACE, surface_half_size=SURFACE_HALF,
                     velocity=[0.0] * 6, supported=True, attached=False, gripper_open=True)
    arguments.update(changes)
    return object_settled_on_surface(**arguments)


def test_settled_requires_support_pose_open_gripper_and_zero_motion():
    assert settled()


@pytest.mark.parametrize('change', [dict(supported=False), dict(attached=True), dict(gripper_open=False),
    dict(velocity=[0.1, 0, 0, 0, 0, 0]), dict(velocity=[0, 0, 0, 0, 0.1, 0]),
    dict(velocity=[float('nan')] * 6), dict(position=(-0.85, -0.8, 0.59)),
    dict(position=(0.0, 0.0, 0.84)), dict(orientation=(0.70710678, 0, 0, 0.70710678))])
def test_unverified_states_never_become_success(change):
    assert not settled(**change)


def test_remote_object_request_is_rejected_instead_of_teleported():
    assert 'teleportation' in release_request_rejection((1.12, 0, 0.84), UPRIGHT, TARGET,
                                                        HALF_SIZE, SURFACE, SURFACE_HALF)


@pytest.mark.parametrize('target', [(-0.85, -0.8, 0.48), (-0.51, -0.8, 0.56),
                                    (float('nan'), -0.8, 0.56)])
def test_invalid_destination_geometry_is_rejected(target):
    assert release_request_rejection(target, UPRIGHT, target, HALF_SIZE, SURFACE, SURFACE_HALF)


def test_invalid_quaternion_is_not_silently_replaced():
    with pytest.raises(ValueError):
        box_world_extents(HALF_SIZE, (0, 0, 0, 0))


@pytest.fixture(scope='module')
def model(tmp_path_factory):
    upstream = Path(__file__).resolve().parents[3] / 'openflex_mujoco/openflex_mujoco.xml'
    path = build_thesis_model(str(upstream), str(tmp_path_factory.mktemp('release') / 'scene.xml'))
    return mujoco.MjModel.from_xml_path(str(path))


def bridge_fixture(model, position=TARGET):
    from builtin_interfaces.msg import Time

    data = mujoco.MjData(model)
    bridge = SimpleNamespace(model=model, data=data, lock=threading.RLock(),
        control_mode_lock=threading.RLock(),
        attached_arm='right', attachment_offset=((0, 0, 0), UPRIGHT),
        pending_place=None, place_status=None, place_status_until=0.0,
        safety_stop=None, trajectories={}, actual_twist=[0.0] * 3, commanded_twist=[0.0] * 3,
        gripper_targets={'left': 0.04, 'right': 0.0}, control_mode=ControlMode(mode=ControlMode.PLACE),
        control_mode_receipt=time.monotonic(), get_parameter=lambda name: SimpleNamespace(
            value={'delivery_surface_geom': 'thesis_delivery_top'}.get(name, 'thesis_item_01')),
        _simulation_stamp=lambda: Time(sec=int(data.time)),
        _mujoco_body_pose=MujocoBridgeNode._mujoco_body_pose)
    bridge.object_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'thesis_object')
    bridge.joint_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                        for name in ['thesis_object_free', *GRIPPER_JOINTS.values()]}
    for name in ['_id', '_joint_position', '_delivery_geometry', '_place_guard_valid', '_place_failure',
                 '_on_place_control', '_step_placement', '_step_attachment']:
        setattr(bridge, name, MethodType(getattr(MujocoBridgeNode, name), bridge))
    address = model.jnt_qposadr[bridge.joint_ids['thesis_object_free']]
    data.qpos[address:address + 7] = [*position, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    return bridge


def command():
    message = FixedPlaceControl(enabled=True, object_id='thesis_item_01', active_arm=2)
    message.delivery_pose.header.frame_id = 'map'
    message.delivery_pose.pose.position.x, message.delivery_pose.pose.position.y, message.delivery_pose.pose.position.z = TARGET
    message.delivery_pose.pose.orientation.w = 1.0
    return message


def test_bridge_rejects_legacy_remote_placement_without_qpos_changes(model):
    bridge = bridge_fixture(model, position=(1.12, 0, 0.84))
    before = bridge.data.qpos.copy()
    bridge._on_place_control(command())
    np.testing.assert_array_equal(before, bridge.data.qpos)
    assert bridge.attached_arm == 'right' and bridge.gripper_targets['right'] == 0.0
    assert bridge.place_status.failure_code == 95
    assert not bridge.place_status.verified and not bridge.place_status.released


def test_request_only_opens_gripper_and_does_not_claim_success(model):
    bridge = bridge_fixture(model, position=(-0.85, -0.8, 0.58))
    before = bridge.data.qpos.copy()
    bridge._on_place_control(command())
    np.testing.assert_array_equal(before, bridge.data.qpos)
    assert bridge.attached_arm == 'right' and bridge.gripper_targets['right'] == 0.04
    assert not bridge.place_status.verified and not bridge.place_status.released


def test_measured_open_detaches_at_actual_pose_without_qpos_or_velocity_rewrite(model):
    bridge = bridge_fixture(model)
    bridge._on_place_control(command())
    address = model.jnt_qposadr[bridge.joint_ids[GRIPPER_JOINTS['right']]]
    bridge.data.qpos[address] = 0.04
    before = bridge.data.qpos.copy()
    bridge._step_attachment()
    np.testing.assert_array_equal(before, bridge.data.qpos)
    assert bridge.attached_arm is None and bridge.attachment_offset is None


def test_actual_gravity_settling_is_required_for_verified_place(model):
    bridge = bridge_fixture(model, position=(-0.85, -0.8, 0.58))
    bridge._on_place_control(command())
    address = model.jnt_qposadr[bridge.joint_ids[GRIPPER_JOINTS['right']]]
    bridge.data.qpos[address] = 0.04
    bridge._step_attachment()
    bridge._step_placement()
    assert not bridge.place_status.verified
    for sample_index in range(600):
        mujoco.mj_step(model, bridge.data)
        bridge.data.qpos[address] = 0.04
        mujoco.mj_forward(model, bridge.data)
        bridge.control_mode_receipt = time.monotonic()
        bridge._step_placement()
        if bridge.place_status.verified:
            break
    assert bridge.place_status.verified and bridge.place_status.released
    assert bridge.data.xpos[bridge.object_body_id, 2] == pytest.approx(0.56, abs=0.002)
    assert bridge.data.time >= 0.5


def test_cancel_before_open_preserves_attachment_and_never_verifies(model):
    bridge = bridge_fixture(model)
    message = command()
    bridge._on_place_control(message)
    message.enabled = False
    bridge._on_place_control(message)
    assert bridge.pending_place is None and bridge.attached_arm == 'right'
    assert bridge.gripper_targets['right'] == 0.0
    assert bridge.place_status.failure_code == 93 and not bridge.place_status.verified


def test_cancel_after_detach_never_claims_that_release_was_undone(model):
    bridge = bridge_fixture(model)
    message = command()
    bridge._on_place_control(message)
    address = model.jnt_qposadr[bridge.joint_ids[GRIPPER_JOINTS['right']]]
    bridge.data.qpos[address] = 0.04
    bridge._step_attachment()
    message.enabled = False
    bridge._on_place_control(message)
    assert bridge.place_status.released and not bridge.place_status.verified
    assert bridge.attached_arm is None and bridge.gripper_targets['right'] == 0.04


def test_frozen_physics_never_accumulates_settling_time(model):
    bridge = bridge_fixture(model, position=(-0.85, -0.8, 0.55995))
    bridge._on_place_control(command())
    address = model.jnt_qposadr[bridge.joint_ids[GRIPPER_JOINTS['right']]]
    bridge.data.qpos[address] = 0.04
    bridge._step_attachment()
    mujoco.mj_forward(model, bridge.data)
    for sample_index in range(100):
        bridge._step_placement()
    assert not bridge.place_status.verified


def test_release_deadline_fails_and_keeps_attached_gripper_closed(model):
    bridge = bridge_fixture(model)
    bridge._on_place_control(command())
    bridge.pending_place['started'] -= 11
    bridge._step_placement()
    assert bridge.place_status.failure_code == 96 and not bridge.place_status.verified
    assert bridge.gripper_targets['right'] == 0.0


@pytest.mark.parametrize('change', ['stale', 'mode', 'motion', 'trajectory', 'safety_stop', 'identity'])
def test_place_ownership_and_identity_are_fail_closed(model, change):
    bridge = bridge_fixture(model)
    message = command()
    if change == 'stale':
        bridge.control_mode_receipt -= 2
    elif change == 'mode':
        bridge.control_mode.mode = ControlMode.NAV2
    elif change == 'motion':
        bridge.actual_twist[0] = 0.1
    elif change == 'trajectory':
        bridge.trajectories['right_arm'] = object()
    elif change == 'safety_stop':
        bridge.safety_stop = {'violation': True}
    elif change == 'identity':
        message.object_id = 'other'
    bridge._on_place_control(message)
    assert bridge.place_status.failure_code and not bridge.place_status.verified
    assert bridge.pending_place is None and bridge.gripper_targets['right'] == 0.0
