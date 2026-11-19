from copy import deepcopy
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco'))
from openflex_delivery_contract import (BudgetExhausted, DeliveryContract, LoopBudget,
    MODEL_LOOP_BUDGET, MODEL_LOOP_DEADLINE_SEC, fresh_data)
from openflex_thesis_mujoco.release_verification import RELEASE_REACH_M


def state():
    diagnostic = {'source': 'mujoco_diagnostic_only_NOT_PHYSICAL', 'object_id': 'thesis_item_01', 'safety_stop': None, 'attached_arm': None,
        'object_pose': {'position_m': [-0.85, -0.8, 0.5609], 'orientation_xyzw': [0, 0, 0, 1]},
        'delivery_support_contact': True, 'object_velocity_linear_angular': [0.0] * 6}
    values = {'simulation_diagnostic': diagnostic,
        # Base placed exactly one measured reach from the surface centre so the
        # reachable release centre coincides with the surface centre.
        'odometry': {'pose': {'pose': {'position': {'x': -0.85 - RELEASE_REACH_M, 'y': -0.8, 'z': 0.0}}},
                     'twist': {'twist': {'linear': {'x': 0.0, 'y': 0.0}, 'angular': {'z': 0.0}}}},
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


def test_reachable_release_center_matches_the_contract_geometry():
    from openflex_thesis_mujoco.release_verification import RELEASE_REACH_M, reachable_release_center

    surface = [-0.85, -0.8, 0.48]
    half_size = [0.35, 0.35]
    reached = reachable_release_center([surface[0] - RELEASE_REACH_M, surface[1]], surface, half_size)
    assert reached[:2] == pytest.approx(surface[:2])
    assert reachable_release_center([surface[0], surface[1]], surface, half_size) is None
    assert reachable_release_center([5.0, 5.0], surface, half_size) is None


def composite_terminal(**updates):
    search = {'outcome': 'authorized', 'attachment_verified': True,
              'grasp': {'status': 'succeeded', 'success': True, 'stopped_confirmed': True,
                        'grasp_verified': True, 'transport_safe': True}}
    return dict({'status': 'succeeded', 'success': True, 'stopped_confirmed': True,
                 'search': search}, **updates)


def test_composite_search_and_pick_is_a_registered_model_choice_with_host_owned_ids():
    contract = initialized()
    for arguments in ({'query': '红色的快递盒', 'color': 'red', 'request_id': 'injected'},
                      {'query': '红色的快递盒', 'color': 'red', 'timeout_sec': 600},
                      {'query': '红色的快递盒', 'color': 'red', 'x': 1.0}):
        with pytest.raises(ValueError, match='may not inject'):
            contract.validate('search_and_pick', arguments)
    for arguments in ({'query': '   ', 'color': 'red'}, {'query': 'box', 'color': 'chartreuse'}):
        with pytest.raises(ValueError, match='registered colour|non-empty natural-language'):
            contract.validate('search_and_pick', arguments)


def test_composite_pickup_must_be_image_authorized_and_independently_verified():
    contract = initialized()
    arguments = {'query': '红色的快递盒', 'color': 'red'}
    contract.validate('search_and_pick', arguments)
    broken = [
        composite_terminal(search={'outcome': 'wrong_class', 'attachment_verified': True}),
        composite_terminal(search={'outcome': 'authorized', 'attachment_verified': False}),
        composite_terminal(search={'outcome': 'authorized', 'attachment_verified': True,
                                   'grasp': {'grasp_verified': False, 'transport_safe': True}}),
        composite_terminal(search={'outcome': 'authorized', 'attachment_verified': True,
                                   'grasp': {'grasp_verified': True, 'transport_safe': False}}),
        composite_terminal(success=False),
        composite_terminal(stopped_confirmed=False),
    ]
    for result in broken:
        with pytest.raises(RuntimeError):
            contract.record('search_and_pick', arguments, result)
    assert not contract.pickup and not contract.grasp


def test_composite_pickup_then_delivery_still_requires_independent_final_state():
    contract = initialized()
    pick = {'query': '红色的快递盒', 'color': 'red'}
    contract.validate('search_and_pick', pick)
    contract.record('search_and_pick', pick, composite_terminal())
    assert contract.pickup and contract.grasp and not contract.delivery and not contract.completed
    contract.validate('navigate_to_target', {'target_id': 'delivery_zone'})
    contract.record('navigate_to_target', {'target_id': 'delivery_zone'}, terminal())
    place = {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone'}
    contract.validate('place_to_affordance', place)
    contract.record('place_to_affordance', place, terminal())
    assert contract.place and not contract.completed
    contract.record('get_robot_state', {}, state())
    assert contract.completed


def test_composite_cannot_be_replayed_as_recovery():
    contract = initialized()
    pick = {'query': '红色的快递盒', 'color': 'red'}
    contract.validate('search_and_pick', pick)
    contract.record('search_and_pick', pick, composite_terminal())
    with pytest.raises(ValueError, match='Repeated motion is not recovery'):
        contract.validate('search_and_pick', pick)


@pytest.mark.parametrize('name,limit', sorted(MODEL_LOOP_BUDGET.items()))
def test_model_loop_budget_halts_on_every_declared_dimension(name, limit):
    budget = LoopBudget(started_at=0.0)
    for _ in range(limit):
        budget.spend(name, now=1.0)
    with pytest.raises(BudgetExhausted, match=f'{name} budget exhausted'):
        budget.spend(name, now=1.0)
    assert budget.spent[name] == limit + 1
    assert budget.as_dict()['exhausted'] == [name]


def test_model_loop_deadline_halts_even_before_any_step_is_spent():
    budget = LoopBudget(started_at=0.0)
    with pytest.raises(BudgetExhausted, match='total time budget exhausted'):
        budget.spend('model_steps', now=MODEL_LOOP_DEADLINE_SEC + 1.0)
    assert budget.as_dict()['spent']['model_steps'] == 0
    assert budget.as_dict()['limits'] == MODEL_LOOP_BUDGET


def test_model_loop_budget_rejects_unknown_dimensions_and_nan_time():
    with pytest.raises(KeyError):
        LoopBudget().spend('unbounded_retries', now=1.0)
    with pytest.raises(ValueError, match='finite'):
        LoopBudget().spend('model_steps', now=float('nan'))
