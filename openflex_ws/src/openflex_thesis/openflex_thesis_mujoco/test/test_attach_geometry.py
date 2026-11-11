"""Tests for the jaw-bracketed attachment gate in the MuJoCo bridge.

The G4 acceptance recorded `verified=true` from a pure proximity check:
`grasp_attach_distance_m` was 0.13-0.16 m, so the object attached to the TCP
whenever the wrist came near it, whether or not it was between the fingers.
`attach_rejection` is the replacement gate. It is pure geometry, so it is
tested here without starting a ROS node or a simulation.
"""

from types import SimpleNamespace

import pytest

from openflex_thesis_mujoco.bridge_node import MujocoBridgeNode

# Measured in the TCP frame from the MuJoCo hand meshes with the finger joints
# closed; see the bridge's `_build_attach_geometry`.
GEOMETRY = (-0.069, 0.016, 0.5 * 0.0855)
LATERAL_TOLERANCE_M = 0.020


def attach_rejection(offset, geometry=GEOMETRY, lateral=LATERAL_TOLERANCE_M):
    bridge = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=lateral)
    )
    return MujocoBridgeNode.attach_rejection(bridge, offset, geometry)


def test_a_real_grasp_is_accepted():
    assert attach_rejection((0.0, 0.0, -0.03)) == ""


def test_object_behind_the_fingertips_is_rejected():
    # A wrist-level hit, e.g. the 0.13 m proximity attach, lands near z = -0.18.
    assert "behind the fingertips" in attach_rejection((0.0, 0.0, -0.18))


def test_object_past_the_pads_is_rejected():
    assert "beyond the finger pads" in attach_rejection((0.0, 0.0, 0.08))


def test_object_outside_the_jaw_span_is_rejected():
    assert "outside the jaw span" in attach_rejection((0.0, 0.06, -0.03))
    assert "outside the jaw span" in attach_rejection((0.0, -0.06, -0.03))


def test_object_laterally_off_the_jaw_axis_is_rejected():
    assert "laterally off the jaw axis" in attach_rejection((0.05, 0.0, -0.03))


@pytest.mark.parametrize("y", [0.0, 0.02, -0.02, 0.040])
def test_jaw_span_boundary_accepts_inside_the_opening(y):
    assert attach_rejection((0.0, y, -0.03)) == ""


def test_the_old_proximity_threshold_would_have_accepted_a_miss():
    """The recorded 0.16 m attach distance accepted these poses; the gate does not."""
    distance = 0.16
    # Object directly below the TCP, 0.16 m away: outside the pad region.
    assert "behind the fingertips" in attach_rejection((0.0, 0.0, -distance))
