from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest


pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[2]
for package in ['openflex_roboos_bridge', 'openflex_thesis_mujoco']:
    sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis' / package))
from openflex_roboos_bridge.operation_store import OperationStore
from openflex_roboos_bridge.place_skill import PlaceSkill
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


@pytest.fixture
def skill(tmp_path):
    instance = object.__new__(PlaceSkill)
    instance.node = SimpleNamespace(store=OperationStore(tmp_path / 'operations.jsonl'))
    return instance


def arguments(**updates):
    return dict({'request_id': 'place_test', 'object_id': 'thesis_item_01',
                 'affordance_id': 'delivery_zone', 'timeout_sec': 600}, **updates)


def test_placement_shares_navigation_and_grasp_mutex(skill):
    skill.node.store.begin('grasp_running', 'grasp_object', {})
    assert skill.submit(arguments())['status'] == 'rejected_busy'


def test_release_publication_and_status_topic_match_existing_simulator(monkeypatch):
    topics = {}
    class NodeFixture:
        manipulation = object()
        group = object()

        def create_publisher(self, message, topic, qos):
            topics['publisher'] = (topic, qos)

        def create_subscription(self, message, topic, callback, qos, **arguments):
            topics['status'] = topic

    monkeypatch.setenv('OPENFLEX_ENABLE_SIM_PLACE', '1')
    PlaceSkill(NodeFixture())
    assert topics['status'] == '/thesis/place/status'
    assert topics['publisher'][0] == '/thesis/place/control'
    qos = topics['publisher'][1]
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE


def test_placement_is_idempotent_with_normalized_timeout(skill):
    with patch('openflex_roboos_bridge.place_skill.threading.Thread') as thread:
        started = skill.submit(arguments())
        repeated = skill.submit(arguments(timeout_sec=600.0))
    assert thread.call_count == 1 and started == repeated and not started['success']


@pytest.mark.parametrize('updates', [{'timeout_sec': True}, {'timeout_sec': '60'}, {'timeout_sec': float('nan')},
    {'timeout_sec': float('inf')}, {'timeout_sec': 0}, {'timeout_sec': 601}, {'object_id': 'other'},
    {'affordance_id': 'shelf'}, {'base_xyyaw': [0, 0, 0]}])
def test_placement_rejects_unreviewed_inputs(skill, updates):
    with pytest.raises(ValueError):
        skill.submit(arguments(**updates))
    assert skill.node.store.active_id is None
