import asyncio
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from openflex_delivery_experiment import FIXED_SEQUENCE, reject_stale_frame, run_fixed


def test_controlled_fault_is_explicit_and_does_not_mutate_real_camera_evidence():
    raw = {'sources': {'camera': {'status': 'fresh'}}}
    original = deepcopy(raw)
    feedback = reject_stale_frame(raw, 1)
    assert raw == original
    assert feedback['no_motion_sent'] and not feedback['success']
    assert feedback['fault'] == 'experiment_injected_stale_observation_once'


@pytest.mark.parametrize('raw,count', [({}, 0), ({'sources': {'camera': {'status': 'stale'}}}, 1),
                                      ({'sources': {'camera': {'status': 'fresh'}}}, 0)])
def test_fault_cannot_claim_a_fresh_preinjection_camera_when_missing(raw, count):
    with pytest.raises(RuntimeError):
        reject_stale_frame(raw, count)


def test_fixed_dispatch_uses_seven_same_skill_calls_and_independent_completion():
    calls = []
    contract = SimpleNamespace(checked_state=True, observed=True, pickup=True, grasp=True,
                               delivery=True, place=True, completed=True)

    async def execute(name, arguments):
        calls.append((name, arguments))
        return SimpleNamespace(isError=False)

    result = asyncio.run(run_fixed(execute, contract))
    assert calls == list(FIXED_SEQUENCE)
    assert result['success'] and result['completion_verified']


def test_fixed_dispatch_stops_on_unusable_observation_without_motion():
    calls = []
    contract = SimpleNamespace(checked_state=True, observed=False)

    async def execute(name, arguments):
        calls.append(name)
        return SimpleNamespace(isError=False)

    with pytest.raises(RuntimeError, match='unusable observation'):
        asyncio.run(run_fixed(execute, contract))
    assert calls == ['get_robot_state', 'observe_scene']


def test_fixed_dispatch_never_retries_failed_skill():
    calls = []
    contract = SimpleNamespace(checked_state=True, observed=True)

    async def execute(name, arguments):
        calls.append(name)
        if name == 'navigate_to_target':
            raise RuntimeError('Actual navigation failure')
        return SimpleNamespace(isError=False)

    with pytest.raises(RuntimeError, match='Actual navigation failure'):
        asyncio.run(run_fixed(execute, contract))
    assert calls == ['get_robot_state', 'observe_scene', 'navigate_to_target']
