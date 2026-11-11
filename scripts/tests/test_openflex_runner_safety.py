import asyncio
import json
from pathlib import Path
import sys

import httpx
from jsonschema.exceptions import ValidationError
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from openflex_runner_safety import cancel_and_audit, check_stop, validate_selection


TOOLS = [{'function': {'name': 'get_robot_state', 'parameters': {
    'type': 'object', 'properties': {}, 'additionalProperties': False}}}]


def test_unknown_or_extra_model_arguments_fail_before_execution():
    with pytest.raises(ValueError, match='Unregistered'):
        validate_selection(TOOLS, 'unreviewed_motion', {})
    with pytest.raises(ValidationError):
        validate_selection(TOOLS, 'get_robot_state', {'request_id': 'spoof'})
    validate_selection(TOOLS, 'get_robot_state', {})


def test_operator_stop_is_checked_before_dispatch(tmp_path):
    check_stop(tmp_path)
    (tmp_path / 'STOP').write_text('operator stop')
    with pytest.raises(asyncio.CancelledError):
        check_stop(tmp_path)


def run_cleanup(tmp_path, handler, attempts=2):
    async def run():
        async with httpx.AsyncClient(transport=httpx.MockTransport(handler), trust_env=False) as client:
            return await cancel_and_audit(client, 'original_request', tmp_path, attempts=attempts, interval=0)
    return asyncio.run(run())


def test_disconnected_cancel_is_unknown_not_false_stopped(tmp_path):
    def handler(request):
        raise httpx.ConnectError('injected disconnect', request=request)
    audit = run_cleanup(tmp_path, handler)
    assert not audit['stopped_confirmed']
    assert 'ConnectError' in audit['error']
    assert json.loads((tmp_path / 'exit_cleanup_result.json').read_text()) == audit


@pytest.mark.parametrize('terminal', [
    {'status': 'unknown', 'stopped_confirmed': True},
    {'status': 'running', 'stopped_confirmed': False},
    {'status': 'canceled', 'stopped_confirmed': False},
    {'status': 'not_found', 'stopped_confirmed': True}])
def test_cancel_ack_or_uncertain_terminal_never_proves_stopped(tmp_path, terminal):
    requests = []
    def handler(request):
        requests.append((request.url.path, json.loads(request.content)))
        return httpx.Response(200, json={'status': 'cancel_requested'} if request.url.path == '/cancel' else terminal)
    audit = run_cleanup(tmp_path, handler)
    assert not audit['stopped_confirmed']
    assert all(arguments == {'request_id': 'original_request'} for _, arguments in requests)
    assert [route for route, _ in requests].count('/cancel') == 1


def test_actual_terminal_and_stopped_confirmation_are_both_required(tmp_path):
    def handler(request):
        return httpx.Response(200, json={'status': 'cancel_requested'} if request.url.path == '/cancel'
            else {'status': 'canceled', 'stopped_confirmed': True})
    assert run_cleanup(tmp_path, handler)['stopped_confirmed']
