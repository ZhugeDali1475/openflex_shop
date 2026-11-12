from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from openflex_motion_profile import FASTER_PHASES, KNOWN_PHASES, scaling_for_phase, selected_profile


@pytest.mark.parametrize('phase', sorted(KNOWN_PHASES))
def test_baseline_exactly_preserves_original_scaling(phase):
    assert scaling_for_phase(phase, 'baseline') == (0.05, 0.05)


@pytest.mark.parametrize('phase', sorted(KNOWN_PHASES))
def test_balanced_keeps_acceleration_and_contact_phases_conservative(monkeypatch, phase):
    monkeypatch.setenv('ROS_DOMAIN_ID', '83')
    monkeypatch.setenv('OPENFLEX_ENABLE_SIM_MANIPULATION', '1')
    velocity, acceleration = scaling_for_phase(phase, 'balanced_v1')
    assert acceleration == 0.05
    assert velocity == (0.10 if phase in FASTER_PHASES else 0.05)


@pytest.mark.parametrize('domain,opt_in', [('0', '1'), ('83', '0'), ('', '')])
def test_faster_profile_never_works_without_isolation(monkeypatch, domain, opt_in):
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    monkeypatch.setenv('OPENFLEX_ENABLE_SIM_MANIPULATION', opt_in)
    with pytest.raises(RuntimeError):
        scaling_for_phase('transport', 'balanced_v1')


def test_unknown_profile_or_phase_fails_closed(monkeypatch):
    monkeypatch.setenv('OPENFLEX_MOTION_PROFILE', 'turbo')
    with pytest.raises(ValueError):
        selected_profile()
    with pytest.raises(ValueError):
        scaling_for_phase('unknown', 'baseline')
