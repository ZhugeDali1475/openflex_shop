from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from openflex_roboos_bridge.action_journal import (
    GoalAcceptanceUnknown, cancel_to_hold_and_audit, goal_receipt_summary,
    record_terminal, send_audited_goal)
from openflex_roboos_bridge.operation_store import OperationStore


@pytest.fixture
def node(tmp_path):
    store = OperationStore(tmp_path / 'journal.jsonl')
    store.begin('original', 'grasp_object', {})
    return SimpleNamespace(store=store, await_future=lambda future, timeout: future.result())


def handle(identifier, accepted=True):
    result = Future()
    result.set_result(SimpleNamespace(status=6, result=SimpleNamespace(error_code=-5, error_string='revoked')))
    cancels = []
    return SimpleNamespace(goal_id=identifier, accepted=accepted, cancel_goal_async=lambda: cancels.append(True),
                           get_result_async=lambda: result, cancels=cancels)


def client_for(node, accepted=True):
    sent = Future()

    def send(goal, *, goal_uuid):
        assert node.store.read('original')['events'][-1]['kind'] == 'controller_goal_intent'
        sent.set_result(handle(goal_uuid, accepted))
        return sent

    return SimpleNamespace(send_goal_async=send)


def send(node, client):
    return send_audited_goal(node, 'original', client, object(), '/arm/follow_joint_trajectory', 'transport')


def test_intent_is_durable_before_send_and_real_result_is_correlated(node):
    accepted = send(node, client_for(node))
    assert not goal_receipt_summary(node.store.read('original'))['all_goal_terminals_audited']
    record_terminal(node.store, 'original', accepted, accepted.get_result_async().result())
    evidence = goal_receipt_summary(node.store.read('original'))
    assert evidence['all_goal_terminals_audited']
    assert list(evidence['goals'].values())[0]['terminal_status'] == 6
    assert node.store.read('original')['status'] == 'running'
    assert node.store.begin('new', 'grasp_object', {})[0]['status'] == 'rejected_busy'


def test_intent_failure_prevents_send(node, monkeypatch):
    def fail(*arguments):
        raise OSError('disk full')

    monkeypatch.setattr(node.store, 'event', fail)
    client = SimpleNamespace(send_goal_async=lambda *args, **kwargs: pytest.fail('No command allowed'))
    with pytest.raises(OSError):
        send(node, client)


def test_transport_send_error_preserves_uncertainty(node):
    def fail(*args, **kwargs):
        raise RuntimeError('transport lost')

    with pytest.raises(GoalAcceptanceUnknown):
        send(node, SimpleNamespace(send_goal_async=fail))
    assert goal_receipt_summary(node.store.read('original'))['unresolved_goal_uuids']


def test_explicit_rejection_is_audited_without_claiming_task_success(node):
    with pytest.raises(RuntimeError, match='rejected'):
        send(node, client_for(node, accepted=False))
    assert goal_receipt_summary(node.store.read('original'))['all_goal_terminals_audited']
    assert node.store.read('original')['success'] is False


def test_late_acceptance_is_canceled_and_audited_but_never_unlocks(node):
    sent = Future()
    identifiers = []

    def request(goal, *, goal_uuid):
        identifiers.append(goal_uuid)
        return sent

    def timeout(future, limit):
        raise TimeoutError('deadline')

    node.await_future = timeout
    with pytest.raises(GoalAcceptanceUnknown):
        send(node, SimpleNamespace(send_goal_async=request))
    accepted = handle(identifiers[0])
    node.store.finish('original', 'unknown', success=False)
    sent.set_result(accepted)
    assert accepted.cancels == [True]
    assert goal_receipt_summary(node.store.read('original'))['all_goal_terminals_audited']
    assert node.store.read('original')['status'] == 'unknown'
    assert node.store.begin('new', 'grasp_object', {})[0]['status'] == 'rejected_busy'


@pytest.mark.parametrize('status', [0, 1, 2, 3])
def test_nonterminal_response_never_proves_goal_stopped(node, status):
    accepted = send(node, client_for(node))
    with pytest.raises(RuntimeError, match='unknown'):
        record_terminal(node.store, 'original', accepted, SimpleNamespace(status=status, result=object()))
    assert not goal_receipt_summary(node.store.read('original'))['all_goal_terminals_audited']


def test_old_records_without_goal_uuids_cannot_be_reconstructed_as_audited():
    assert not goal_receipt_summary({'events': [{'kind': 'trajectory_start', 'data': {'phase': 'transport'}}]})['all_goal_terminals_audited']


@pytest.mark.parametrize('mutation', ['orphan', 'duplicate', 'bad_uuid', 'conflict', 'boolean_status'])
def test_invalid_or_conflicting_receipts_fail_closed(node, mutation):
    accepted = send(node, client_for(node))
    record_terminal(node.store, 'original', accepted, accepted.get_result_async().result())
    record = node.store.read('original')
    if mutation == 'orphan':
        record['events'].pop(0)
    elif mutation == 'duplicate':
        record['events'].insert(0, record['events'][0])
    elif mutation == 'bad_uuid':
        record['events'][0]['data']['goal_uuid'] = 'z' * 32
    elif mutation == 'conflict':
        record['events'].append({'kind': 'controller_result', 'data': {
            'goal_uuid': bytes(accepted.goal_id.uuid).hex(), 'status': 4}})
    else:
        record['events'][-1]['data']['status'] = True
    assert not goal_receipt_summary(record)['all_goal_terminals_audited']


def test_cleanup_cancels_all_then_holds_even_when_one_result_is_unknown(node):
    order = []
    accepted = [send(node, client_for(node)), send(node, client_for(node))]
    pairs = []
    for index, item in enumerate(accepted):
        item.cancel_goal_async = lambda index=index: order.append(f'cancel_{index}')
        future = Future()
        future.set_result(SimpleNamespace(status=0 if index == 0 else 6, result=object()))
        pairs.append((item, future))
    node.set_mode = lambda mode: order.append('HOLD')
    with pytest.raises(RuntimeError, match='unknown'):
        cancel_to_hold_and_audit(node, 'original', pairs, 0)
    assert order == ['cancel_0', 'cancel_1', 'HOLD']
    results = [event for event in node.store.read('original')['events'] if event['kind'] == 'controller_result']
    assert [event['data']['status'] for event in results] == [0, 6]


def test_cleanup_still_checks_results_when_hold_fails(node):
    accepted = send(node, client_for(node))

    def fail(mode):
        raise RuntimeError('ack missing')

    node.set_mode = fail
    with pytest.raises(RuntimeError, match='ack missing'):
        cancel_to_hold_and_audit(node, 'original', [(accepted, accepted.get_result_async())], 0)
    assert goal_receipt_summary(node.store.read('original'))['all_goal_terminals_audited']
