from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from openflex_roboos_bridge.telemetry_wait import TelemetryUnavailable, wait_for_fresh_stop


@pytest.fixture
def clock(monkeypatch):
    current = [0.0]
    monkeypatch.setattr('openflex_roboos_bridge.telemetry_wait.time.monotonic', lambda: current[0])

    def advance(seconds):
        current[0] += seconds

    monkeypatch.setattr('openflex_roboos_bridge.telemetry_wait.time.sleep', advance)
    return current


def test_stop_requires_full_fresh_half_second(clock):
    assert wait_for_fresh_stop(lambda: True, 1)
    assert 0.5 <= clock[0] < 0.55


@pytest.mark.parametrize('invalid', ['stale', 'moving'])
def test_invalid_sample_resets_continuous_window(clock, invalid):
    def stationary():
        if 0.24 <= clock[0] < 0.3:
            if invalid == 'stale':
                raise TelemetryUnavailable('Missing/stale telemetry: diagnostic')
            return False
        return True

    assert wait_for_fresh_stop(stationary, 1)
    assert clock[0] >= 0.8


def test_timeout_cannot_borrow_earlier_fresh_samples(clock):
    def stationary():
        if 0.24 <= clock[0] < 0.3:
            raise TelemetryUnavailable('Telemetry clock mismatch: diagnostic')
        return True

    assert not wait_for_fresh_stop(stationary, 0.7)


def test_permanent_telemetry_loss_is_not_stopped(clock):
    def stationary():
        raise TelemetryUnavailable('Missing/stale telemetry: odometry')

    assert not wait_for_fresh_stop(stationary, 1)
    assert 1 <= clock[0] < 1.05


def test_other_failures_still_propagate(clock):
    def stationary():
        raise RuntimeError('Unsafe control ownership')

    with pytest.raises(RuntimeError, match='ownership'):
        wait_for_fresh_stop(stationary, 1)


def test_slow_sample_cannot_confirm_after_deadline(clock):
    def stationary():
        clock[0] += 0.6
        return True

    assert not wait_for_fresh_stop(stationary, 1)
