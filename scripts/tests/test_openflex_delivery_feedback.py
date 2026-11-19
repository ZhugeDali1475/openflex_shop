"""P5.2: every real failure class must reach the next model decision unchanged."""
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco'))
from openflex_delivery_contract import (  # noqa: E402
    DeliveryContract,
    REQUIRED_SOURCES,
    feedback_state,
    feedback_terminal,
    state_summary,
    terminal_summary,
)


def snapshot(**source_updates):
    diagnostic = {'source': 'mujoco_diagnostic_only_NOT_PHYSICAL', 'object_id': 'thesis_item_01',
                  'safety_stop': None, 'attached_arm': None,
                  'object_pose': {'position_m': [1.12, 0, 0.84], 'orientation_xyzw': [0, 0, 0, 1]},
                  'delivery_support_contact': False, 'object_velocity_linear_angular': [0.0] * 6}
    values = {name: {'status': 'fresh', 'data': {'pose': {}, 'twist': {}, 'verified': False}}
              for name in ('odometry', 'joint_states')}
    values['simulation_diagnostic'] = {'status': 'fresh', 'data': diagnostic}
    for name, update in source_updates.items():
        values.setdefault(name, {'status': 'fresh', 'data': {}})
        values[name] = dict(values[name], **update)
    return {'environment': 'mujoco_simulation', 'physical_verification': False,
            'sources': values,
            'scene_registry': {
                'objects': {'thesis_item_01': {'nominal_pose_xyzyaw': [1.12, 0, 0.84, 3.14]}},
                'affordances': {'delivery_zone': {'surface_center_xyz': [-0.85, -0.8, 0.48],
                                                  'surface_dimensions_xy': [0.7, 0.7]}}}}


def initialized():
    contract = DeliveryContract()
    contract.record('get_robot_state', {}, snapshot())
    contract.record('observe_scene', {}, {'sources': {'camera': {'status': 'fresh'}}}, image_count=1)
    return contract


def motion(**updates):
    return dict({'status': 'succeeded', 'success': True, 'stopped_confirmed': True,
                 'detail': '', 'grasp_verified': True, 'transport_safe': True, 'place_verified': True},
                **updates)


def test_stale_observation_and_safety_gate_are_reported_before_any_motion():
    result = state_summary(snapshot(odometry={'status': 'stale'}))
    assert result['feedback']['freshness']['odometry'] == 'stale'
    assert result['feedback']['must_stop'] is True
    assert all(name in result['feedback']['freshness'] for name in REQUIRED_SOURCES)
    stopped = state_summary(snapshot(simulation_diagnostic={'status': 'fresh', 'data': {
        'source': 'mujoco_diagnostic_only_NOT_PHYSICAL', 'object_id': 'thesis_item_01',
        'safety_stop': {'reason': 'injected'}, 'attached_arm': None,
        'object_pose': {'position_m': [1.12, 0, 0.84], 'orientation_xyzw': [0, 0, 0, 1]},
        'delivery_support_contact': False, 'object_velocity_linear_angular': [0.0] * 6}}))
    assert stopped['feedback']['safety_gate']['safety_stop'] == {'reason': 'injected'}
    assert stopped['feedback']['must_stop'] is True
    assert feedback_state(snapshot())['must_stop'] is False
    with pytest.raises(RuntimeError, match='Fresh independent evidence required'):
        DeliveryContract().record('get_robot_state', {}, snapshot(odometry={'status': 'stale'}))


def test_stale_camera_cannot_become_an_observation_without_a_real_image():
    contract = DeliveryContract()
    with pytest.raises(RuntimeError, match='no imagined substitute'):
        contract.record('observe_scene', {}, {'sources': {'camera': {'status': 'stale'}}}, image_count=1)
    with pytest.raises(RuntimeError, match='no imagined substitute'):
        contract.record('observe_scene', {}, {'sources': {'camera': {'status': 'fresh'}}}, image_count=0)
    assert not contract.observed


@pytest.mark.parametrize('skill,arguments,detail', [
    ('navigate_to_target', {'target_id': 'pickup_zone'}, 'planning failed: no collision-free route'),
    ('navigate_to_target', {'target_id': 'pickup_zone'}, 'navigation failed: base blocked'),
    ('grasp_object', {'object_id': 'thesis_item_01'}, 'grasp failed: controller rejected goal'),
    ('place_to_affordance', {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone'},
     'place failed: release not verified'),
])
def test_each_failure_class_stops_the_loop_and_keeps_its_real_reason(skill, arguments, detail):
    contract = initialized()
    if skill != 'navigate_to_target':
        contract.validate('navigate_to_target', {'target_id': 'pickup_zone'})
        contract.record('navigate_to_target', {'target_id': 'pickup_zone'}, motion())
    if skill == 'place_to_affordance':
        contract.validate('grasp_object', {'object_id': 'thesis_item_01'})
        contract.record('grasp_object', {'object_id': 'thesis_item_01'}, motion())
        contract.validate('navigate_to_target', {'target_id': 'delivery_zone'})
        contract.record('navigate_to_target', {'target_id': 'delivery_zone'}, motion())
    contract.validate(skill, arguments)
    failed = motion(status='failed', success=False, detail=detail)
    with pytest.raises(RuntimeError, match='Failed/uncertain motion must stop'):
        contract.record(skill, arguments, failed)
    summary = terminal_summary(failed)
    assert summary['feedback']['reason'] == detail
    assert summary['feedback']['must_stop'] is True
    assert summary['detail'] == detail


def test_unverified_grasp_or_place_evidence_is_not_success():
    contract = initialized()
    contract.validate('navigate_to_target', {'target_id': 'pickup_zone'})
    contract.record('navigate_to_target', {'target_id': 'pickup_zone'}, motion())
    contract.validate('grasp_object', {'object_id': 'thesis_item_01'})
    with pytest.raises(RuntimeError, match='verified grasp/transport'):
        contract.record('grasp_object', {'object_id': 'thesis_item_01'},
                        motion(transport_safe=False))
    assert not contract.grasp
    contract.record('grasp_object', {'object_id': 'thesis_item_01'}, motion())
    contract.validate('navigate_to_target', {'target_id': 'delivery_zone'})
    contract.record('navigate_to_target', {'target_id': 'delivery_zone'}, motion())
    contract.validate('place_to_affordance', {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone'})
    with pytest.raises(RuntimeError, match='independently verified placement'):
        contract.record('place_to_affordance',
                        {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone'},
                        motion(place_verified=False))
    assert not contract.place


def test_unknown_terminal_is_never_success_and_still_needs_stop_confirmation():
    contract = initialized()
    contract.validate('navigate_to_target', {'target_id': 'pickup_zone'})
    unknown = motion(status='unknown', success=False, detail='transport lost', stopped_confirmed=False)
    with pytest.raises(RuntimeError, match='Failed/uncertain motion must stop'):
        contract.record('navigate_to_target', {'target_id': 'pickup_zone'}, unknown)
    summary = terminal_summary(unknown)
    assert summary['feedback']['must_stop'] is True
    assert summary['feedback']['stopped_confirmed'] is False
    assert summary['success'] is False
    assert not contract.pickup


def test_natural_language_success_cannot_override_a_failed_tool_result():
    failed = motion(status='failed', success=False, stopped_confirmed=True,
                    detail='任务已经成功完成，一切正常')
    summary = terminal_summary(failed)
    assert summary['status'] == 'failed'
    assert summary['success'] is False
    assert summary['feedback']['must_stop'] is True
    assert summary['detail'] == '任务已经成功完成，一切正常'
    contract = initialized()
    contract.validate('navigate_to_target', {'target_id': 'pickup_zone'})
    with pytest.raises(RuntimeError):
        contract.record('navigate_to_target', {'target_id': 'pickup_zone'}, failed)
    assert not contract.pickup


def test_missing_terminal_gate_is_a_stop_not_a_pass():
    contract = initialized()
    contract.validate('navigate_to_target', {'target_id': 'pickup_zone'})
    with pytest.raises(RuntimeError):
        contract.record('navigate_to_target', {'target_id': 'pickup_zone'},
                        motion(stopped_confirmed=None))
    assert feedback_terminal(motion(stopped_confirmed=None))['must_stop'] is True
    assert feedback_terminal(motion())['must_stop'] is False
