from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys

import pytest


pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_openflex_manipulation_safety import SafetyProbe


class FakeFuture:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result


class FakeClient:
    def __init__(self, srv_name, outcomes=(), available=True):
        self.srv_name = srv_name
        self.outcomes = list(outcomes)
        self.available = available
        self.requests = []
        self.cancelled = []

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return self.outcomes.pop(0) if self.outcomes else FakeFuture(done=False)

    def remove_pending_request(self, future):
        self.cancelled.append(future)


@pytest.fixture
def probe():
    instance = object.__new__(SafetyProbe)
    instance.get_logger = lambda: SimpleNamespace(warn=lambda message: None)
    instance.pause = lambda seconds: None
    return instance


def invoke(probe, client, request, **arguments):
    with patch('check_openflex_manipulation_safety.rclpy.spin_until_future_complete'):
        return probe.call(client, request, **arguments)


def test_transient_no_response_is_retried_before_succeeding(probe):
    client = FakeClient('/get_planning_scene', [FakeFuture(done=False), FakeFuture('scene')])
    assert invoke(probe, client, {'kind': 'query'}) == 'scene'
    assert len(client.requests) == 2 and len(client.cancelled) == 1


def test_exhausted_responses_raise_after_bounded_attempts(probe):
    client = FakeClient('/get_planning_scene', [FakeFuture(done=False)], available=True)
    with pytest.raises(RuntimeError, match=r'No planning response: /get_planning_scene \(after 3 attempts\)'):
        invoke(probe, client, {'kind': 'query'})
    assert len(client.requests) == 3 and len(client.cancelled) == 3


def test_unavailable_service_is_retried_then_raises(probe):
    client = FakeClient('/check_state_validity', available=False)
    with pytest.raises(RuntimeError, match=r'Unavailable planning service: /check_state_validity \(after 3 attempts\)'):
        invoke(probe, client, {'kind': 'query'})
    assert client.requests == [] and client.cancelled == []


def test_first_response_returns_without_extra_attempts(probe):
    client = FakeClient('/apply_planning_scene', [FakeFuture('applied')])
    assert invoke(probe, client, {'kind': 'diff'}) == 'applied'
    assert len(client.requests) == 1 and client.cancelled == []


def test_each_attempt_sends_an_independent_request_copy(probe):
    payload = {'kind': 'query', 'world': ['shelf']}
    client = FakeClient('/apply_planning_scene', [FakeFuture('applied')])
    assert invoke(probe, client, payload) == 'applied'
    assert client.requests[0] == payload and client.requests[0] is not payload
    assert client.requests[0]['world'] is not payload['world']
