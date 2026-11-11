import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[2] / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge/openflex_roboos_bridge/trajectory_guard.py'
SPEC = importlib.util.spec_from_file_location('trajectory_guard', MODULE_PATH)
GUARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GUARD)


def test_sampling_preserves_endpoints_and_units():
    samples = GUARD.sample_joint_path(['lift_joint', 'arm_joint'], [[0.0, -0.2], [-0.1, 0.4]])
    assert samples[0] == [0.0, -0.2]
    assert samples[-1] == pytest.approx([-0.1, 0.4])
    for previous, following in zip(samples, samples[1:]):
        assert abs(following[0] - previous[0]) <= 0.00500001
        assert abs(following[1] - previous[1]) <= 0.03000001


@pytest.mark.parametrize('names,points', [([], []), (['joint', 'joint'], [[0, 0]]),
    (['joint'], []), (['joint'], [[0, 1]]), (['joint'], [[float('nan')]]),
    (['joint'], [[float('inf')]])])
def test_invalid_paths_are_rejected(names, points):
    with pytest.raises(ValueError):
        GUARD.sample_joint_path(names, points)


def test_excessive_paths_are_rejected():
    with pytest.raises(ValueError, match='budget'):
        GUARD.sample_joint_path(['lift_joint'], [[0], [10]], maximum_samples=100)


def test_single_state_still_requires_a_validity_check():
    assert GUARD.sample_joint_path(['joint'], [[0.25]]) == [[0.25]]
