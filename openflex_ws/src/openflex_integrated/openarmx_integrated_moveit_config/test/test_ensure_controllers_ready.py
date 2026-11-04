from types import SimpleNamespace
from unittest.mock import patch


def test_ensure_returns_without_mutating_ready_controllers():
    import ensure_controllers_ready as module

    node = object.__new__(module.ControllerReadiness)
    node.controller_states = lambda: {
        "arm": "active",
        "lift": "inactive",
    }
    node.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
        warn=lambda _message: None,
        error=lambda _message: None,
    )
    node._load = lambda _name: (_ for _ in ()).throw(AssertionError("unexpected load"))
    node._configure = lambda _name: (_ for _ in ()).throw(
        AssertionError("unexpected configure")
    )
    node._activate = lambda _names: (_ for _ in ()).throw(
        AssertionError("unexpected activation")
    )

    with patch.object(module.rclpy, "ok", return_value=True):
        assert node.ensure({"arm"}, {"lift"}, 1.0, 0.0)


def test_ensure_repairs_missing_unconfigured_and_inactive_controllers():
    import ensure_controllers_ready as module

    states = {
        "arm": "inactive",
        "lift": "unconfigured",
    }
    node = object.__new__(module.ControllerReadiness)
    node.controller_states = lambda: dict(states)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
        warn=lambda _message: None,
        error=lambda _message: None,
    )
    node._load = lambda name: states.__setitem__(name, "unconfigured") or True
    node._configure = lambda name: states.__setitem__(name, "inactive") or True

    def activate(names):
        for name in names:
            states[name] = "active"
        return True

    node._activate = activate

    with patch.object(module.rclpy, "ok", return_value=True), patch.object(
        module.time, "sleep", return_value=None
    ):
        assert node.ensure({"arm", "head"}, {"lift"}, 1.0, 0.0)

    assert states == {
        "arm": "active",
        "head": "active",
        "lift": "inactive",
    }


def test_ensure_activates_runtime_controllers_one_at_a_time_in_safe_order():
    import ensure_controllers_ready as module

    states = {
        "joint_state_broadcaster": "inactive",
        "swerve_drive_controller": "inactive",
        "left_forward_position_controller": "inactive",
    }
    activation_calls = []
    node = object.__new__(module.ControllerReadiness)
    node.controller_states = lambda: dict(states)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
        warn=lambda _message: None,
        error=lambda _message: None,
    )
    node._load = lambda _name: True
    node._configure = lambda _name: True

    def activate(names):
        activation_calls.append(list(names))
        for name in names:
            states[name] = "active"
        return True

    node._activate = activate

    with patch.object(module.rclpy, "ok", return_value=True), patch.object(
        module.time, "sleep", return_value=None
    ):
        assert node.ensure(set(states), set(), 1.0, 0.0)

    assert activation_calls == [
        ["joint_state_broadcaster"],
        ["swerve_drive_controller"],
        ["left_forward_position_controller"],
    ]
