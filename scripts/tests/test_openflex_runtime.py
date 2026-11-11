from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import openflex_runtime as runtime
from record_openflex_live_sim import frame_state


def snapshot():
    return {'environment': 'mujoco_simulation', 'physical_verification': False, 'sources': {
        'simulation_diagnostic': {'status': 'fresh', 'data': {
            'source': 'mujoco_diagnostic_only_NOT_PHYSICAL',
            'control_ack': {'mode': 0, 'fresh': True, 'inputs_healthy': True}}},
        'odometry': {'status': 'fresh', 'data': {'twist': {'twist': {
            'linear': dict(x=0.0, y=0.0, z=0.0), 'angular': dict(x=0.0, y=0.0, z=0.0)}}}},
        'joint_states': {'status': 'fresh', 'data': {'velocity': [0.0, 0.0]}}}}


def test_hold_requires_fresh_measured_ack_and_stopped_joints():
    healthy = snapshot()
    assert runtime.measured_hold(healthy)
    for name in healthy['sources']:
        stale = deepcopy(healthy)
        stale['sources'][name]['status'] = 'stale'
        assert not runtime.measured_hold(stale)
    for name, value in [('mode', 1), ('fresh', False), ('inputs_healthy', False)]:
        invalid = deepcopy(healthy)
        invalid['sources']['simulation_diagnostic']['data']['control_ack'][name] = value
        assert not runtime.measured_hold(invalid)


@pytest.mark.parametrize('velocity', [[0.04], [float('nan')], [float('inf')], []])
def test_moving_missing_or_nonfinite_joints_never_prove_stopped(velocity):
    value = snapshot()
    value['sources']['joint_states']['data']['velocity'] = velocity
    assert not runtime.measured_hold(value)


def test_wrong_environment_or_incomplete_state_is_not_safe_hold():
    value = snapshot()
    value['environment'] = 'physical_robot'
    assert not runtime.measured_hold(value)
    assert not runtime.measured_hold({})


def test_journal_uses_latest_records_and_retains_unknown_lock(tmp_path):
    path = tmp_path / 'operations.jsonl'
    records = [{'request_id': 'done', 'status': 'running'}, {'request_id': 'done', 'status': 'canceled'},
               {'request_id': 'uncertain', 'status': 'unknown'}]
    path.write_text('\n'.join(json.dumps(record) for record in records))
    assert runtime.pending_operations(path) == [records[-1]]


def test_start_requires_motion_opt_in_before_any_service_call(monkeypatch):
    monkeypatch.setattr(runtime, 'start_unit', lambda *args: pytest.fail('Must not start a service'))
    with pytest.raises(RuntimeError, match='requires --enable-simulation-motion'):
        runtime.start(SimpleNamespace(enable_simulation_motion=False))


def test_start_rejects_unresolved_operation(monkeypatch):
    monkeypatch.setattr(runtime, 'pending_operations', lambda: [{'status': 'unknown'}])
    monkeypatch.setattr(runtime, 'start_unit', lambda *args: pytest.fail('Must not start a service'))
    with pytest.raises(RuntimeError, match='Pending operation'):
        runtime.start(SimpleNamespace(enable_simulation_motion=True))


def test_run_preserves_delivered_scene_without_reset(monkeypatch):
    monkeypatch.setattr(runtime, 'scheduler_process', lambda: None)
    monkeypatch.setattr(runtime, 'pending_operations', lambda: [])
    monkeypatch.setattr(runtime, 'active', lambda unit: unit != runtime.UNITS['scheduler'])
    value = snapshot()
    value['scene_registry'] = {'objects': {'thesis_item_01': {'nominal_pose_xyzyaw': [1.12, 0, 0.84, 0]}}}
    value['sources']['simulation_diagnostic']['data'].update(attached_arm=None,
        object_pose={'position_m': [-0.85, -0.8, 0.56]})
    monkeypatch.setattr(runtime, 'request_json', lambda *args: value)
    monkeypatch.setattr(runtime, 'start_unit', lambda *args: pytest.fail('Must not start delivery'))
    with pytest.raises(RuntimeError, match='no automatic reset'):
        runtime.run_delivery(SimpleNamespace(execute_isolated_simulation=True))


@pytest.mark.parametrize('sample,sim,now,previous,expected', [
    (10.0, 2.0, 10.1, 1.0, 'fresh'),
    (10.0, 2.0, 10.6, 1.0, 'stale'),
    (0.0, 2.0, 10.1, None, 'stale'),
    (10.0, 2.0, 10.1, 2.0, 'not_advanced'),
    (10.0, 1.0, 10.1, 2.0, 'clock_rollback'),
    (10.0, float('nan'), 10.1, None, 'stale')])
def test_recorder_never_labels_frozen_or_reset_clock_as_live(sample, sim, now, previous, expected):
    assert frame_state(sample, sim, now, previous) == expected
