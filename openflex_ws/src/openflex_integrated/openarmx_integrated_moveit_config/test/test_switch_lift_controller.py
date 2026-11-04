import sys
from types import SimpleNamespace
from unittest.mock import patch

from controller_manager_msgs.srv import SwitchController


def test_main_accepts_launch_ros_arguments(monkeypatch):
    import switch_lift_controller as module

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "switch_lift_controller.py",
            "--mode",
            "moveit",
            "--ros-args",
            "-r",
            "__node:=switch_lift_controller_for_moveit",
        ],
    )
    monkeypatch.setattr(module.rclpy, "init", lambda: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)

    fake_node = SimpleNamespace(
        switch=lambda mode, timeout: mode == "moveit" and timeout == 20.0,
        destroy_node=lambda: None,
    )
    with patch.object(module, "LiftControllerSwitch", return_value=fake_node):
        assert module.main() == 0


def test_switch_is_idempotent_when_target_is_already_active():
    import switch_lift_controller as module

    node = object.__new__(module.LiftControllerSwitch)
    node.wait_until_loaded = lambda _timeout: {
        "lift_position_controller": "active",
        "lift_manual_position_controller": "inactive",
    }
    node.switch_client = SimpleNamespace(wait_for_service=lambda timeout_sec: True)
    messages = []
    node.get_logger = lambda: SimpleNamespace(info=messages.append, error=messages.append)

    assert node.switch("moveit", 1.0)
    assert messages == ["Lift controller ownership is already moveit"]


def test_switch_uses_one_atomic_request():
    import switch_lift_controller as module

    captured = []

    class Future:
        @staticmethod
        def done():
            return True

        @staticmethod
        def result():
            return SimpleNamespace(ok=True)

    node = object.__new__(module.LiftControllerSwitch)
    node.wait_until_loaded = lambda _timeout: {
        "lift_position_controller": "inactive",
        "lift_manual_position_controller": "active",
    }
    node.switch_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: True,
        call_async=lambda request: captured.append(request) or Future(),
    )
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None, error=lambda _message: None)

    with patch.object(module.rclpy, "spin_until_future_complete"):
        assert node.switch("moveit", 1.0)

    assert len(captured) == 1
    request = captured[0]
    assert request.activate_controllers == ["lift_position_controller"]
    assert request.deactivate_controllers == ["lift_manual_position_controller"]
    assert request.strictness == SwitchController.Request.STRICT
