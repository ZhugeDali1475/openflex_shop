from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest


pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from openflex_roboos_bridge.manipulation_skill import ManipulationSkill
from openflex_roboos_bridge.operation_store import OperationStore


@pytest.fixture
def skill(tmp_path):
    instance = object.__new__(ManipulationSkill)
    instance.node = SimpleNamespace(store=OperationStore(tmp_path / 'operations.jsonl'))
    return instance


def test_navigation_and_grasp_share_one_operation_mutex(skill):
    skill.node.store.begin('navigation', 'navigate_to_target', {})
    result = skill.submit({'object_id': 'thesis_item_01', 'request_id': 'grasp', 'timeout_sec': 60})
    assert result['status'] == 'rejected_busy' and not result['success']


def test_retreat_stops_immediately_if_released_payload_loses_stability(skill):
    skill.deadline = time.monotonic() + 10
    skill.released_target = [-0.85, -0.8, 0.56]
    skill.node.guard = object()
    skill.node.healthy = lambda **arguments: None
    skill.node.placement = SimpleNamespace(placement_still_verified=lambda target: False)
    with pytest.raises(RuntimeError, match='stop immediately'):
        skill.gate(attached=False)


def test_repeated_grasp_does_not_start_a_second_thread(skill):
    arguments = {'object_id': 'thesis_item_01', 'request_id': 'grasp', 'timeout_sec': 60}
    with patch('openflex_roboos_bridge.manipulation_skill.threading.Thread') as thread:
        first = skill.submit(arguments)
        repeated = skill.submit(arguments)
    assert thread.call_count == 1 and first == repeated and not first['success']


@pytest.mark.parametrize('timeout', [0, -1, 601, float('nan'), float('inf'), True, '60'])
def test_invalid_timeout_is_rejected(skill, timeout):
    with pytest.raises(ValueError):
        skill.submit({'object_id': 'thesis_item_01', 'request_id': 'grasp', 'timeout_sec': timeout})


def test_unregistered_object_is_rejected_without_claiming_control(skill):
    with pytest.raises(ValueError):
        skill.submit({'object_id': 'invented', 'request_id': 'grasp', 'timeout_sec': 60})
    assert skill.node.store.active_id is None


def test_equivalent_integer_and_float_deadline_is_idempotent(skill):
    with patch('openflex_roboos_bridge.manipulation_skill.threading.Thread') as thread:
        first = skill.submit({'object_id': 'thesis_item_01', 'request_id': 'grasp', 'timeout_sec': 60})
        repeated = skill.submit({'object_id': 'thesis_item_01', 'request_id': 'grasp', 'timeout_sec': 60.0})
    assert thread.call_count == 1 and first == repeated
