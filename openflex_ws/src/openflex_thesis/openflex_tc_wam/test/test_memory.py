from openflex_tc_wam.memory import (
    EVENT_AFFORDANCE_INVALID,
    EVENT_LOST,
    EVENT_MOVED,
    MemoryConfig,
    ObjectMemoryFilter,
)


def observe(memory, position, stamp, sequence):
    return memory.observe(
        "item", "map", position, 0.0, (0.001, 0.001, 0.001), 0.01,
        0.9, stamp, sequence, "test",
    )


def test_confirmed_motion_invalidates_map_once():
    memory = ObjectMemoryFilter(MemoryConfig(move_confirm_frames=2))
    initial = observe(memory, (1.0, 0.0, 0.8), 0.0, 1)
    assert initial.map_version == 1
    observe(memory, (1.1, 0.0, 0.8), 0.1, 2)
    moved = observe(memory, (1.1, 0.0, 0.8), 0.2, 3)
    assert moved.event_flags & EVENT_MOVED
    assert moved.event_flags & EVENT_AFFORDANCE_INVALID
    assert moved.map_version == 2


def test_occluded_memory_never_authorizes_grasp():
    memory = ObjectMemoryFilter(MemoryConfig(lost_timeout_sec=0.5))
    observe(memory, (1.0, 0.0, 0.8), 0.0, 1)
    lost = memory.tick(0.6)
    assert lost.event_flags & EVENT_LOST
    assert not memory.grasp_authorized(0.6)


def test_confirmed_move_event_survives_timer_until_next_observation():
    memory = ObjectMemoryFilter(MemoryConfig(move_confirm_frames=1))
    observe(memory, (1.0, 0.0, 0.8), 0.0, 1)
    moved = observe(memory, (1.1, 0.0, 0.8), 0.1, 2)
    assert moved.event_flags & EVENT_MOVED
    assert memory.tick(0.11).event_flags & EVENT_MOVED


def test_out_of_order_observation_is_rejected_without_mutating_memory():
    memory = ObjectMemoryFilter(MemoryConfig())
    state = observe(memory, (1.0, 0.0, 0.8), 1.0, 10)
    try:
        observe(memory, (2.0, 0.0, 0.8), 0.9, 9)
    except ValueError as error:
        assert "timestamp" in str(error)
    else:
        raise AssertionError("out-of-order observation was accepted")
    assert state.position == (1.0, 0.0, 0.8)
    assert state.sequence == 10


def test_yaw_covariance_is_fused_and_preserved():
    memory = ObjectMemoryFilter(MemoryConfig())
    initial = memory.observe(
        "item", "map", (1.0, 0.0, 0.8), 0.0,
        (0.001, 0.001, 0.001), 0.04, 0.9, 1.0, 1, "test",
    )
    initial_yaw_variance = initial.yaw_variance
    updated = memory.observe(
        "item", "map", (1.0, 0.0, 0.8), 0.1,
        (0.001, 0.001, 0.001), 0.01, 0.9, 1.1, 2, "test",
    )
    assert 0.0 < updated.yaw < 0.1
    assert updated.yaw_variance < initial_yaw_variance
