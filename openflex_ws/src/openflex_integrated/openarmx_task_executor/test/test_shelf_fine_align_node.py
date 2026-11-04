import math

import pytest

from openarmx_task_executor.shelf_fine_align_node import (
    LOCALIZATION_GUARD_COMPLETED,
    LOCALIZATION_GUARD_INACTIVE,
    alignment_control_yaw_error,
    alignment_target_clearance,
    fine_alignment_guard_terminal_state,
    filtered_measurement_is_stale,
    first_measurement_timed_out,
    localization_state_allows_alignment,
)


def test_latched_localization_true_does_not_expire_without_a_heartbeat():
    assert localization_state_allows_alignment(True, True)


def test_localization_must_be_seen_and_true():
    assert not localization_state_allows_alignment(False, False)
    assert not localization_state_allows_alignment(False, True)
    assert not localization_state_allows_alignment(True, False)


def test_alignment_motion_uses_map_target_not_unstable_shelf_yaw():
    map_error = math.radians(2.3)
    measured_shelf_error = math.radians(-6.9)

    assert alignment_control_yaw_error(
        map_error, measured_shelf_error
    ) == pytest.approx(map_error)


def test_yaw_only_alignment_preserves_the_measured_clearance():
    assert alignment_target_clearance(0.10, 0.126, True) == pytest.approx(0.126)
    assert alignment_target_clearance(0.10, 0.126, False) == pytest.approx(0.10)


def test_initial_measurement_uses_independent_acquisition_timeout():
    assert not first_measurement_timed_out(10.50, 10.0, 0.0, 1.50)
    assert first_measurement_timed_out(11.51, 10.0, 0.0, 1.50)
    assert not first_measurement_timed_out(20.0, 10.0, 10.20, 1.50)


def test_sparse_measurement_gap_has_its_own_timeout():
    assert not filtered_measurement_is_stale(10.79, 10.0, 0.80)
    assert filtered_measurement_is_stale(10.81, 10.0, 0.80)
    assert not filtered_measurement_is_stale(100.0, 0.0, 0.80)


def test_motion_alignment_publishes_completed_or_inactive_guard_state():
    assert fine_alignment_guard_terminal_state(True, True) == (
        LOCALIZATION_GUARD_COMPLETED
    )
    assert fine_alignment_guard_terminal_state(True, False) == (
        LOCALIZATION_GUARD_INACTIVE
    )


def test_measurement_only_does_not_change_guard_lease():
    assert fine_alignment_guard_terminal_state(False, True) is None
    assert fine_alignment_guard_terminal_state(False, False) is None
