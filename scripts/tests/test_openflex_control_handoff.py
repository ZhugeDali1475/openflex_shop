from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from openflex_roboos_bridge.navigation_node import NavigationNode
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
