from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from openflex_roboos_bridge.navigation_node import NavigationNode
from openflex_roboos_bridge.telemetry_wait import TelemetryUnavailable
from openflex_thesis_msgs.msg import ControlMode


def acknowledgement(**updates):
    return dict({'mode': ControlMode.MOVEIT, 'epoch': 7, 'fresh': True,
        'inputs_healthy': True, 'arm_control_valid': True}, **updates)


def test_service_acceptance_waits_for_simulator_applied_epoch(monkeypatch):
    responses = iter([acknowledgement(mode=ControlMode.HOLD, epoch=6),
        acknowledgement(epoch=6), acknowledgement()])
    received = []

    def sample(name):
        received.append(name)
        return {'control_ack': next(responses)}

    monkeypatch.setattr('openflex_roboos_bridge.navigation_node.time.sleep', lambda seconds: None)
    NavigationNode.wait_control_ack(SimpleNamespace(sample=sample), ControlMode.MOVEIT, 7)
    assert received == ['diagnostic'] * 3


@pytest.mark.parametrize('updates', [{'epoch': 6}, {'fresh': False}, {'inputs_healthy': False},
    {'arm_control_valid': False}, {'mode': ControlMode.HOLD}])
def test_unconfirmed_arm_ownership_times_out_without_commanding_motion(updates):
    node = SimpleNamespace(sample=lambda name: {'control_ack': acknowledgement(**updates)})
    with pytest.raises(TimeoutError, match='has not acknowledged'):
        NavigationNode.wait_control_ack(node, ControlMode.MOVEIT, 7, timeout=0.001)


def test_place_requires_actual_simulator_place_guard():
    node = SimpleNamespace(sample=lambda name: {'control_ack': acknowledgement(mode=ControlMode.PLACE)})
    with pytest.raises(TimeoutError):
        NavigationNode.wait_control_ack(node, ControlMode.PLACE, 7, timeout=0.001)


def test_continuous_guard_uses_applied_epoch_not_independent_subscriber_receipt():
    node = SimpleNamespace(sample=lambda name: {'control_ack': acknowledgement()})
    assert NavigationNode.require_control_ack(node, ControlMode.MOVEIT, 7)['epoch'] == 7


@pytest.mark.parametrize('updates', [{'epoch': 8}, {'fresh': False}, {'inputs_healthy': False},
    {'arm_control_valid': False}, {'mode': ControlMode.HOLD}, {'epoch': None}])
def test_continuous_guard_never_reuses_invalid_executor_ack(updates):
    node = SimpleNamespace(sample=lambda name: {'control_ack': acknowledgement(**updates)})
    with pytest.raises(RuntimeError, match='revoked'):
        NavigationNode.require_control_ack(node, ControlMode.MOVEIT, 7)


def test_continuous_guard_propagates_stale_diagnostic():
    def stale(name):
        raise RuntimeError('Missing/stale telemetry: diagnostic')
    with pytest.raises(RuntimeError, match='stale'):
        NavigationNode.require_control_ack(SimpleNamespace(sample=stale), ControlMode.MOVEIT, 7)


def test_place_continuous_guard_requires_actual_place_validity():
    node = SimpleNamespace(sample=lambda name: {'control_ack': acknowledgement(mode=ControlMode.PLACE)})
    with pytest.raises(RuntimeError, match='revoked'):
        NavigationNode.require_control_ack(node, ControlMode.PLACE, 7)


def test_handoff_can_wait_for_new_fresh_sample_without_reusing_stale_ack(monkeypatch):
    responses = iter([None, acknowledgement()])

    def sample(name):
        value = next(responses)
        if value is None:
            raise TelemetryUnavailable('Missing/stale telemetry: diagnostic')
        return {'control_ack': value}

    monkeypatch.setattr('openflex_roboos_bridge.navigation_node.time.sleep', lambda seconds: None)
    NavigationNode.wait_control_ack(SimpleNamespace(sample=sample), ControlMode.MOVEIT, 7)


def test_continuous_motion_guard_does_not_wait_out_telemetry_loss():
    def sample(name):
        raise TelemetryUnavailable('Missing/stale telemetry: diagnostic')

    with pytest.raises(TelemetryUnavailable):
        NavigationNode.require_control_ack(SimpleNamespace(sample=sample), ControlMode.MOVEIT, 7)


def test_revoked_ack_error_carries_the_failing_gate():
    """A bare "revoked" cannot tell a stale receipt from a moving base."""
    node = SimpleNamespace(sample=lambda name: {'control_ack': acknowledgement(
        fresh=False, mode_receipt_age_sec=0.613, actual_twist=[0.0, 0.0, 0.0])})
    with pytest.raises(RuntimeError) as error:
        NavigationNode.require_control_ack(node, ControlMode.MOVEIT, 7)
    message = str(error.value)
    assert 'revoked' in message
    assert '0.613' in message
    assert 'actual_twist' in message


def test_timed_out_ack_wait_reports_the_observed_gate(monkeypatch):
    node = SimpleNamespace(sample=lambda name: {'control_ack': acknowledgement(
        arm_control_valid=False, actual_twist=[0.0, 0.0, 0.025])})
    monkeypatch.setattr('openflex_roboos_bridge.navigation_node.time.sleep', lambda seconds: None)
    with pytest.raises(TimeoutError) as error:
        NavigationNode.wait_control_ack(node, ControlMode.MOVEIT, 7, timeout=0.001)
    message = str(error.value)
    assert 'has not acknowledged' in message
    assert 'actual_twist' in message
