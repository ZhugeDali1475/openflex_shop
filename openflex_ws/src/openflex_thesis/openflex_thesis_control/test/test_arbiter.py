from openflex_thesis_control.arbiter import (
    ACT,
    HOLD,
    MOVEIT,
    NAV2,
    ArbiterConfig,
    Velocity,
    VelocityArbiter,
)


def test_act_requires_stop_and_valid_physical_map():
    arbiter = VelocityArbiter(ArbiterConfig(stop_confirm_sec=0.5))
    arbiter.update_safety(True, True, True, True)
    accepted, _ = arbiter.request_mode(ACT, 0.0)
    assert not accepted
    arbiter.update_measured_velocity(Velocity(), 0.0)
    arbiter.update_measured_velocity(Velocity(), 0.6)
    accepted, _ = arbiter.request_mode(ACT, 0.6)
    assert accepted


def test_stale_source_outputs_zero():
    arbiter = VelocityArbiter(ArbiterConfig(input_timeout_sec=0.2))
    assert arbiter.request_mode(NAV2, 0.0)[0]
    arbiter.update_source(NAV2, Velocity(0.1, 0.0, 0.0), 0.0)
    assert arbiter.selected_velocity(0.1).vx == 0.1
    assert arbiter.selected_velocity(0.3) == Velocity()


def test_lost_target_forces_hold_during_act():
    arbiter = VelocityArbiter(ArbiterConfig(stop_confirm_sec=0.0))
    arbiter.update_safety(True, True, True, True)
    arbiter.update_measured_velocity(Velocity(), 0.0)
    assert arbiter.request_mode(ACT, 0.0)[0]
    arbiter.update_safety(True, True, False, False)
    assert arbiter.mode == HOLD


def test_stale_measured_zero_does_not_authorize_act():
    arbiter = VelocityArbiter(
        ArbiterConfig(stop_confirm_sec=0.1, measured_timeout_sec=0.2)
    )
    arbiter.update_safety(True, True, True, True)
    arbiter.update_measured_velocity(Velocity(), 0.0)
    assert not arbiter.request_mode(ACT, 0.5)[0]


def test_moveit_mode_locks_velocity_and_requires_measured_stop():
    arbiter = VelocityArbiter(ArbiterConfig(stop_confirm_sec=0.2))
    assert not arbiter.request_mode(MOVEIT, 0.0)[0]
    arbiter.update_measured_velocity(Velocity(), 0.0)
    arbiter.update_measured_velocity(Velocity(), 0.3)
    assert arbiter.request_mode(MOVEIT, 0.3)[0]
    assert arbiter.selected_velocity(0.3) == Velocity()
