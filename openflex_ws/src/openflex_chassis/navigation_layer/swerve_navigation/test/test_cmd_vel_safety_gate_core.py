from pathlib import Path
import sys

import pytest

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from cmd_vel_safety_gate_core import (  # noqa: E402
    authorization_is_fresh,
    reverse_policy_violation,
)


DEFAULTS = {
    "allow_reverse": False,
    "reverse_authorized": False,
    "reverse_epsilon": 1.0e-3,
    "authorized_reverse_max_speed": 0.06,
    "authorized_reverse_lateral_tolerance": 1.0e-3,
    "authorized_reverse_angular_tolerance": 1.0e-3,
}


def violation(linear_x, linear_y=0.0, angular_z=0.0, **overrides):
    parameters = {**DEFAULTS, **overrides}
    return reverse_policy_violation(
        linear_x,
        linear_y,
        angular_z,
        **parameters,
    )


def test_forward_and_stationary_commands_do_not_need_authorization():
    assert violation(0.04) is None
    assert violation(0.0) is None


def test_reverse_without_authorization_is_rejected():
    assert "authorization" in violation(-0.04)


def test_short_straight_authorized_reverse_is_allowed():
    assert violation(-0.04, reverse_authorized=True) is None


@pytest.mark.parametrize(
    ("linear_x", "linear_y", "angular_z", "reason"),
    [
        (-0.061, 0.0, 0.0, "speed"),
        (-0.04, 0.002, 0.0, "lateral"),
        (-0.04, 0.0, 0.002, "rotation"),
    ],
)
def test_authorization_does_not_allow_unsafe_reverse_shapes(
    linear_x, linear_y, angular_z, reason
):
    assert reason in violation(
        linear_x,
        linear_y,
        angular_z,
        reverse_authorized=True,
    )


def test_global_reverse_setting_retains_legacy_behavior():
    assert violation(-0.20, 0.10, 0.30, allow_reverse=True) is None


def test_reverse_authorization_requires_a_fresh_heartbeat():
    assert not authorization_is_fresh(None, now=5.0, timeout=0.25)
    assert authorization_is_fresh(4.75, now=5.0, timeout=0.25)
    assert not authorization_is_fresh(4.74, now=5.0, timeout=0.25)
    assert not authorization_is_fresh(5.01, now=5.0, timeout=0.25)
