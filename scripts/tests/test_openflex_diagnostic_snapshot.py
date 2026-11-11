import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from openflex_roboos_bridge.ros_snapshot import SnapshotNode
from openflex_roboos_bridge.snapshot import SnapshotCache


def diagnostic(**updates):
    return dict({'source': 'mujoco_diagnostic_only_NOT_PHYSICAL', 'stamp': {'sec': 10, 'nanosec': 0},
        'attached_arm': None, 'delivery_support_contact': True, 'safety_stop': None}, **updates)


def test_readonly_diagnostic_keeps_original_source_timestamp_and_provenance():
    cache = SnapshotCache(lambda: 10_000_000_000)
    node = SimpleNamespace(cache=cache)
    SnapshotNode.receive(node, 'simulation_diagnostic', SimpleNamespace(data=json.dumps(diagnostic())))
    sample = cache.snapshot()['sources']['simulation_diagnostic']
    assert sample['status'] == 'fresh'
    assert sample['source_stamp_ns'] == 10_000_000_000
    assert sample['data']['delivery_support_contact'] is True
    assert sample['data']['source'] == 'mujoco_diagnostic_only_NOT_PHYSICAL'


@pytest.mark.parametrize('value', [diagnostic(source='physical_robot'), {'source': 'mujoco_diagnostic_only_NOT_PHYSICAL'}, []])
def test_invalid_diagnostic_replaces_old_valid_evidence(value):
    cache = SnapshotCache(lambda: 10_000_000_000)
    node = SimpleNamespace(cache=cache)
    SnapshotNode.receive(node, 'simulation_diagnostic', SimpleNamespace(data=json.dumps(diagnostic())))
    SnapshotNode.receive(node, 'simulation_diagnostic', SimpleNamespace(data=json.dumps(value)))
    sample = cache.snapshot()['sources']['simulation_diagnostic']
    assert sample['status'] == 'invalid' and sample['data'] is None
