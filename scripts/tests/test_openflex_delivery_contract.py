from copy import deepcopy
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco'))
from openflex_delivery_contract import DeliveryContract, fresh_data


def state():
    diagnostic = {'source': 'mujoco_diagnostic_only_NOT_PHYSICAL', 'object_id': 'thesis_item_01', 'safety_stop': None, 'attached_arm': None,
        'object_pose': {'position_m': [-0.85, -0.8, 0.5609], 'orientation_xyzw': [0, 0, 0, 1]},
        'delivery_support_contact': True, 'object_velocity_linear_angular': [0.0] * 6}
    values = {'simulation_diagnostic': diagnostic,
        'odometry': {'twist': {'twist': {'linear': {'x': 0.0, 'y': 0.0}, 'angular': {'z': 0.0}}}},
        'joint_states': {'name': ['openarmx_right_finger_joint1'], 'position': [0.04], 'velocity': [0.0]}}
    return {'environment': 'mujoco_simulation', 'physical_verification': False,
        'sources': {name: {'status': 'fresh', 'data': value} for name, value in values.items()},
        'scene_registry': {'objects': {'thesis_item_01': {'nominal_pose_xyzyaw': [1.12, 0, 0.84, 3.14]}},
            'affordances': {'delivery_zone': {'surface_center_xyz': [-0.85, -0.8, 0.48],
                                                            'surface_dimensions_xy': [0.7, 0.7]}}}}


def initialized():
    contract = DeliveryContract()
    initial = state()
    initial['sources']['simulation_diagnostic']['data']['object_pose']['position_m'] = [1.12, 0, 0.84]
    contract.record('get_robot_state', {}, initial)
    contract.record('observe_scene', {}, {'sources': {'camera': {'status': 'fresh'}}}, image_count=1)
    return contract


def terminal(**updates):
    return dict({'status': 'succeeded', 'success': True, 'stopped_confirmed': True,
                 'grasp_verified': True, 'transport_safe': True, 'place_verified': True}, **updates)


def placed():
    contract = initialized()
    for name, arguments in [('navigate_to_target', {'target_id': 'pickup_zone'}),
        ('grasp_object', {'object_id': 'thesis_item_01'}),
        ('navigate_to_target', {'target_id': 'delivery_zone'}),
        ('place_to_affordance', {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone'})]:
        contract.validate(name, arguments)
        contract.record(name, arguments, terminal())
    return contract


def test_successful_skills_do_not_complete_without_independent_final_state():
    contract = placed()
    assert contract.place and not contract.completed
    contract.record('get_robot_state', {}, state())
    assert contract.completed


@pytest.mark.parametrize('phase', ['missing', 'stale', 'future_timestamp', 'invalid'])
def test_initial_stale_diagnostics_never_authorize_motion(phase):
    sample = state()
    sample['sources']['simulation_diagnostic']['status'] = phase
    with pytest.raises(RuntimeError):
        DeliveryContract().record('get_robot_state', {}, sample)


def test_camera_text_without_image_is_not_observation():
    with pytest.raises(RuntimeError):
        DeliveryContract().record('observe_scene', {}, {'sources': {'camera': {'status': 'fresh'}}})


def test_already_delivered_object_cannot_be_used_as_new_delivery_evidence():
    with pytest.raises(RuntimeError, match='shelf origin'):
        DeliveryContract().record('get_robot_state', {}, state())


@pytest.mark.parametrize('updates', [{'status': 'unknown'}, {'status': 'running'}, {'success': False},
                                  {'stopped_confirmed': False}, {'status': 'timed_out'}])
def test_uncertain_or_failed_results_abort_loop(updates):
    with pytest.raises(RuntimeError):
        initialized().record('navigate_to_target', {'target_id': 'pickup_zone'}, terminal(**updates))


def test_model_cannot_supply_own_request_id_or_unreviewed_coordinates():
    with pytest.raises(ValueError):
        initialized().validate('navigate_to_target', {'target_id': 'pickup_zone', 'request_id': 'spoof'})


def test_model_cannot_skip_pickup_and_grasp_prerequisites():
    with pytest.raises(ValueError):
        initialized().validate('place_to_affordance', {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone'})


def test_repeated_motion_is_not_automatic_recovery():
    contract = initialized()
    arguments = {'target_id': 'pickup_zone'}
    contract.record('navigate_to_target', arguments, terminal())
    with pytest.raises(ValueError):
        contract.validate('navigate_to_target', arguments)


@pytest.mark.parametrize('field,value', [('delivery_support_contact', False), ('attached_arm', 'right'),
    ('object_velocity_linear_angular', [1, 0, 0, 0, 0, 0]),
    ('object_pose', {'position_m': [0, 0, 0.56], 'orientation_xyzw': [0, 0, 0, 1]})])
def test_final_live_release_evidence_is_required(field, value):
    contract = placed()
    sample = state()
    sample['sources']['simulation_diagnostic']['data'][field] = value
    with pytest.raises(RuntimeError):
        contract.record('get_robot_state', {}, sample)
    assert not contract.completed


def test_final_robot_motion_is_not_stopped_delivery():
    contract = placed()
    sample = state()
    sample['sources']['joint_states']['data']['velocity'] = [0.5]
    with pytest.raises(RuntimeError):
        contract.record('get_robot_state', {}, sample)
    assert not contract.completed
