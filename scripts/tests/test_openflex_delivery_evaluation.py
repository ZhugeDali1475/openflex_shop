from copy import deepcopy
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from evaluate_openflex_delivery import digest, independent_completion, initial_sources_ready, load_frozen, validate_protocol


def protocol():
    return json.loads((ROOT / 'scripts/openflex_delivery_protocol.json').read_text())


def test_frozen_protocol_has_explicit_fault_and_fixed_denominators():
    value = protocol()
    validate_protocol(value)
    assert value['denominators']['all_scheduled'] == 3
    assert [trial['policy'] for trial in value['trials']] == ['model', 'model', 'fixed']
    assert sum(trial['inject_stale_once'] for trial in value['trials']) == 1


@pytest.mark.parametrize('mutation', ['hardware', 'retry', 'duplicate', 'denominator', 'unbounded', 'fixed_fault'])
def test_unsafe_or_changed_experiment_contract_is_rejected(mutation):
    value = deepcopy(protocol())
    if mutation == 'hardware':
        value['scope'] = 'hardware'
    elif mutation == 'retry':
        value['automatic_retry'] = True
    elif mutation == 'duplicate':
        value['trials'][1]['id'] = value['trials'][0]['id']
    elif mutation == 'denominator':
        value['denominators']['all_scheduled'] = 4
    elif mutation == 'unbounded':
        value['max_trial_seconds'] = 10000
    else:
        value['trials'][2]['inject_stale_once'] = True
    with pytest.raises(ValueError):
        validate_protocol(value)


def test_edited_protocol_seal_is_rejected(tmp_path):
    path = tmp_path / 'frozen_protocol.json'
    path.write_text('{}')
    (tmp_path / 'freeze_seal.json').write_text(json.dumps({'frozen_protocol_sha256': digest(path)}))
    path.write_text('{"modified":true}')
    with pytest.raises(RuntimeError, match='Frozen protocol has changed'):
        load_frozen(tmp_path)


def test_independent_completion_rejects_missing_or_false_evidence():
    assert not independent_completion({})
    assert not independent_completion({'sources': {'simulation_diagnostic': {'status': 'stale'}}})


def test_tc_wam_has_no_published_result_before_first_query(monkeypatch):
    import evaluate_openflex_delivery as evaluation

    monkeypatch.setattr(evaluation, 'measured_hold', lambda value: True)
    snapshot = {'sources': {'object_memory': {'status': 'fresh'}, 'affordance': {'status': 'missing'}}}
    assert initial_sources_ready(snapshot)
    snapshot['sources']['object_memory']['status'] = 'stale'
    assert not initial_sources_ready(snapshot)
