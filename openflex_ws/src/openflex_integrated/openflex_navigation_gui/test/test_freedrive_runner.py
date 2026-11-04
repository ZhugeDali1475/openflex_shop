from types import SimpleNamespace
from unittest.mock import MagicMock

from rcl_interfaces.msg import ParameterType

from openflex_navigation_gui.freedrive_node import FreeDriveNode
from openflex_navigation_gui.freedrive_runner import ConsoleFreeDriveNode


def _make_freedrive_node(events):
    node = object.__new__(FreeDriveNode)
    node.sides = ["right"]
    node._original_kp = {}
    node._original_modes = {}
    node._changed_sides = []
    node._switched_sides = []
    node._mirror_enabled = True
    node._moveit_disabled_by_us = True
    node.active = False
    node.restore_settle_sec = 0.0
    node.wait_until_ready = lambda: True
    node._get_arm_control_mode = lambda side: "csp" if not node.active else "mit"
    node._get_kp = lambda side: {
        f"kp_joint{index}": 10.0 for index in range(1, 8)
    }
    node._set_arm_control_mode = lambda side, mode: events.append(
        ("mode", side, mode)
    )
    node._set_kp = lambda side, values: events.append(
        ("kp", side, tuple(values.values()))
    )
    node._spin_for = lambda duration: events.append(("settle", duration))
    node._enable_moveit = lambda: events.append(("moveit", "enabled"))
    node.get_logger = MagicMock(return_value=MagicMock())
    return node


def test_local_freedrive_switches_to_mit_before_zeroing_kp():
    events = []
    node = _make_freedrive_node(events)

    assert FreeDriveNode.activate(node)

    mode_index = events.index(("mode", "right", "mit"))
    zero_kp_index = next(
        index
        for index, event in enumerate(events)
        if event[0:2] == ("kp", "right")
        and all(value == 0.0 for value in event[2])
    )
    assert mode_index < zero_kp_index


def test_local_freedrive_restores_kp_before_csp_and_moveit():
    events = []
    node = _make_freedrive_node(events)
    assert FreeDriveNode.activate(node)
    events.clear()

    FreeDriveNode.restore_original_kp(node)

    restore_kp_index = next(
        index
        for index, event in enumerate(events)
        if event[0:2] == ("kp", "right")
        and all(value == 10.0 for value in event[2])
    )
    csp_index = events.index(("mode", "right", "csp"))
    moveit_index = events.index(("moveit", "enabled"))
    assert restore_kp_index < csp_index < moveit_index


def test_console_freedrive_snapshots_kp_and_kd_for_restore():
    requests = []

    class Client:
        @staticmethod
        def call_async(request):
            requests.append(request)
            return SimpleNamespace(response=True)

    node = object.__new__(ConsoleFreeDriveNode)
    node._get_clients = {"right": Client()}
    values = [
        SimpleNamespace(type=ParameterType.PARAMETER_DOUBLE, double_value=float(index))
        for index in range(14)
    ]
    node._wait_for_future = lambda _future: SimpleNamespace(values=values)

    gains = ConsoleFreeDriveNode._get_kp(node, "right")

    assert requests[0].names[:4] == [
        "kp_joint1",
        "kd_joint1",
        "kp_joint2",
        "kd_joint2",
    ]
    assert gains["kp_joint1"] == 0.0
    assert gains["kd_joint7"] == 13.0


def test_freedrive_home_temporarily_changes_gains_without_switching_mode():
    node = object.__new__(ConsoleFreeDriveNode)
    node.sides = ["left", "right"]
    node._home_state = "STARTING"
    node._home_requested = True
    node._home_cancel_requested = False
    node._home_targets = {}
    node._home_saved_gains = {}
    node._home_stable_samples = 0
    node._home_required_stable_samples = 1
    node._home_tolerance = 0.03
    node._original_kp = {
        side: {"kp_joint1": 80.0, "kd_joint1": 4.0}
        for side in node.sides
    }
    node._joint_positions = {
        name: 0.2
        for side in node.sides
        for name in ConsoleFreeDriveNode._position_joint_names(side)
    }
    drag_gains = {
        side: {"kp_joint1": 0.0, "kd_joint1": 1.5}
        for side in node.sides
    }
    gain_updates = []
    mode_updates = []
    statuses = []
    node._get_kp = lambda side: dict(drag_gains[side])
    node._set_kp = lambda side, gains: gain_updates.append((side, dict(gains)))
    node._set_arm_control_mode = lambda side, mode: mode_updates.append((side, mode))

    def publish_status(state, detail):
        node._home_state = state
        statuses.append((state, detail))

    node._publish_home_status = publish_status

    ConsoleFreeDriveNode.step_home(node)

    assert node._home_state == "RUNNING"
    assert [gains for _side, gains in gain_updates] == [
        node._original_kp["left"],
        node._original_kp["right"],
    ]
    assert mode_updates == []

    for name in node._joint_positions:
        if "finger" not in name:
            node._joint_positions[name] = 0.0
    ConsoleFreeDriveNode.step_home(node)

    assert node._home_state == "SUCCEEDED"
    assert [gains for _side, gains in gain_updates[-2:]] == [
        drag_gains["left"],
        drag_gains["right"],
    ]
    assert not node._home_targets
    assert mode_updates == []
    assert statuses[-1][0] == "SUCCEEDED"
