import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from PyQt5.QtWidgets import QApplication, QMessageBox

from openflex_navigation_gui.joint_control_page import ALL_SPECS, JointControlPage, step_towards
from openflex_navigation_gui.main_window import NavigationRosNode


def _telemetry():
    return SimpleNamespace(
        joint_positions={spec.name: 0.0 for spec in ALL_SPECS},
        joint_velocities={},
        joint_efforts={},
        last_seen={"/joint_states": time.monotonic()},
        lift_homing_state="IDLE",
        lift_homing_detail="",
        lift_homing_progress_seen=False,
        freedrive_home_state="IDLE",
        freedrive_home_detail="",
    )


def test_step_towards_limits_each_command_increment():
    assert step_towards(0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert step_towards(0.95, 1.0, 0.1) == pytest.approx(1.0)
    assert step_towards(0.0, -1.0, 0.1) == pytest.approx(-0.1)


def test_joint_page_syncs_all_controls_from_robot_state():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()
    telemetry.joint_positions["openarmx_right_joint1"] = 0.5
    page = JointControlPage(None, telemetry, lambda: (True, "在线"))

    assert page.sync_from_robot(quiet=True)
    assert page.target_positions()["openarmx_right_joint1"] == pytest.approx(0.5)
    assert len(page.rows) == 19
    assert page.rows["lift_joint"][1].minimumWidth() >= 240
    assert page.rows["openarmx_head_yaw_joint"][1].minimumWidth() >= 240
    assert page.rows["openarmx_head_pitch_joint"][1].minimumWidth() >= 240
    assert page.lift_zero_button.text() == "立柱调零"
    assert len(page.gain_rows) == 14
    assert page.freedrive_button.text() == "进入拖动模式"

    page.close()
    app.processEvents()


def test_joint_page_refuses_incomplete_joint_state():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()
    telemetry.joint_positions.pop("lift_joint")
    page = JointControlPage(None, telemetry, lambda: (True, "在线"))

    assert not page.sync_from_robot(quiet=True)

    page.close()
    app.processEvents()


def test_lift_zero_button_dispatches_driver_homing_service():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()

    class FakeFuture:
        @staticmethod
        def done():
            return False

    class FakeNode:
        requests = 0

        @staticmethod
        def lift_homing_ready():
            return True

        def start_lift_homing(self):
            self.requests += 1
            return FakeFuture()

    node = FakeNode()
    page = JointControlPage(node, telemetry, lambda: (True, "在线"))

    with patch("PyQt5.QtWidgets.QMessageBox.question", return_value=QMessageBox.Yes):
        page.start_lift_homing()

    assert node.requests == 1
    assert page._lift_homing_requested
    assert "请求已发出" in page.lift_homing_status.text()

    page.close()
    app.processEvents()


def test_fullbody_publish_uses_active_manual_lift_step_controller():
    published = []

    class Publisher:
        @staticmethod
        def publish(message):
            published.append(list(message.data))

    node = object.__new__(NavigationRosNode)
    node.telemetry = SimpleNamespace(joint_positions={"lift_joint": 0.10})
    node.left_joint_command_pub = Publisher()
    node.right_joint_command_pub = Publisher()
    node.head_joint_command_pub = Publisher()
    node.lift_step_command_pub = Publisher()
    node._last_lift_step_target = None

    node.publish_fullbody_joint_positions(
        [0.0] * 7,
        0.0,
        [0.0] * 7,
        0.0,
        0.14,
        [0.0, 0.0],
    )

    assert published[2] == pytest.approx([0.04, 0.05])

    node.publish_fullbody_joint_positions(
        [0.0] * 7,
        0.0,
        [0.0] * 7,
        0.0,
        0.14,
        [0.0, 0.0],
    )
    lift_commands = [values for values in published if len(values) == 2 and values[1] == 0.05]
    assert len(lift_commands) == 1


def test_auxiliary_publish_does_not_publish_either_arm():
    published = []

    class Publisher:
        def __init__(self, name):
            self.name = name

        def publish(self, message):
            published.append((self.name, list(message.data)))

    node = object.__new__(NavigationRosNode)
    node.telemetry = SimpleNamespace(joint_positions={"lift_joint": 0.10})
    node._manual_joint_control_state = "acquired"
    node.left_joint_command_pub = None
    node.right_joint_command_pub = None
    node.head_joint_command_pub = Publisher("head")
    node.lift_step_command_pub = Publisher("lift")
    node._last_lift_step_target = None

    node.publish_auxiliary_joint_positions(0.14, [0.2, -0.1])

    assert published == [
        ("lift", pytest.approx([0.04, 0.05])),
        ("head", pytest.approx([0.2, -0.1])),
    ]


def test_manual_auxiliary_control_omits_arm_publishers():
    created = []
    destroyed = []

    class Publisher:
        pass

    node = object.__new__(NavigationRosNode)
    node._manual_joint_control_state = "acquired"
    node._manual_joint_control_requested = True
    node.left_joint_command_pub = Publisher()
    node.right_joint_command_pub = Publisher()
    node.head_joint_command_pub = None
    node.lift_step_command_pub = None
    node.create_publisher = lambda _type, topic, _depth: created.append(topic) or Publisher()
    node.destroy_publisher = destroyed.append

    accepted, _detail = node.request_manual_joint_control(include_arms=False)

    assert accepted
    assert node.left_joint_command_pub is None
    assert node.right_joint_command_pub is None
    assert node.head_joint_command_pub is not None
    assert node.lift_step_command_pub is not None
    assert len(destroyed) == 2
    assert created == [
        "/head_forward_position_controller/commands",
        "/lift_manual_position_controller/step_command",
    ]


def test_record_pose_uses_current_robot_state_instead_of_stale_target():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()
    page = JointControlPage(None, telemetry, lambda: (True, "在线"))
    page.rows["openarmx_right_joint1"][1].setValue(0)
    telemetry.joint_positions["openarmx_right_joint1"] = 0.42
    page.preset_name.setText("drag_pose")

    page.save_robot_preset()

    assert page.presets["drag_pose"]["openarmx_right_joint1"] == pytest.approx(0.42)
    page.close()
    app.processEvents()


def test_gain_apply_only_updates_selected_freedrive_arm():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()
    updates = []

    class FakeFuture:
        @staticmethod
        def done():
            return False

    class FakeNode:
        @staticmethod
        def set_arm_gains(side, gains):
            updates.append((side, gains))
            return FakeFuture()

    page = JointControlPage(FakeNode(), telemetry, lambda: (True, "在线"))
    page._freedrive_active = True
    page.freedrive_arm.setCurrentIndex(page.freedrive_arm.findData("right"))
    page.gain_rows[("right", 3)][0].setValue(123.0)
    page.gain_rows[("right", 3)][1].setValue(4.5)

    page.apply_arm_gains()

    assert [side for side, _gains in updates] == ["right"]
    assert updates[0][1]["kp_joint3"] == pytest.approx(123.0)
    assert updates[0][1]["kd_joint3"] == pytest.approx(4.5)
    page.close()
    app.processEvents()


def test_freedrive_joint_tick_only_commands_lift_and_head():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()
    auxiliary_commands = []

    class FakeNode:
        @staticmethod
        def lift_homing_ready():
            return False

        @staticmethod
        def manual_joint_control_ready(include_arms=True):
            return not include_arms

        @staticmethod
        def publish_auxiliary_joint_positions(lift, head):
            auxiliary_commands.append((lift, list(head)))

        @staticmethod
        def publish_fullbody_joint_positions(*_args):
            raise AssertionError("freedrive must not publish arm targets")

    page = JointControlPage(FakeNode(), telemetry, lambda: (True, "在线"))
    page.freedrive_running = lambda: True
    page.unlock.blockSignals(True)
    page.unlock.setChecked(True)
    page.unlock.blockSignals(False)
    page.sync_from_robot(quiet=True)
    page.rows["lift_joint"][1].setValue(100)
    page._target_dirty = True

    page._command_tick()

    assert auxiliary_commands
    assert auxiliary_commands[-1][0] == pytest.approx(0.1)
    page.refresh_status()
    assert not page.rows["openarmx_left_joint1"][1].isEnabled()
    assert page.rows["lift_joint"][1].isEnabled()
    assert page.unlock.text() == "解锁立柱/头部运动"
    page.close()
    app.processEvents()


def test_freedrive_home_stays_in_mit_and_uses_node_service():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()

    class FakeFuture:
        @staticmethod
        def done():
            return False

    class FakeNode:
        home_requests = 0
        manual_requests = 0

        @staticmethod
        def freedrive_home_ready():
            return True

        def start_freedrive_home(self):
            self.home_requests += 1
            return FakeFuture()

        def request_manual_joint_control(self, include_arms=True):
            self.manual_requests += 1
            return True, "unexpected"

        @staticmethod
        def reset_lift_command_target():
            pass

    node = FakeNode()
    page = JointControlPage(node, telemetry, lambda: (True, "在线"))
    page.status_timer.stop()
    page.command_timer.stop()
    page.freedrive_running = lambda: True

    with patch("PyQt5.QtWidgets.QMessageBox.question", return_value=QMessageBox.Yes):
        page.move_home()

    assert page._home_phase == "freedrive_arms"
    assert node.home_requests == 1
    assert node.manual_requests == 0
    assert "保持 MIT" in page.home_status.text()
    page._home_phase = "idle"
    page.close()
    app.processEvents()


def test_home_keeps_lift_still_until_arm_feedback_reaches_zero():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()
    for spec in ALL_SPECS:
        telemetry.joint_positions[spec.name] = 0.05
    telemetry.joint_positions["lift_joint"] = 0.20
    published_lifts = []

    class FakeNode:
        @staticmethod
        def manual_joint_control_ready(include_arms=True):
            return True

        @staticmethod
        def publish_fullbody_joint_positions(
            _left, _left_gripper, _right, _right_gripper, lift, _head
        ):
            published_lifts.append(lift)

    page = JointControlPage(FakeNode(), telemetry, lambda: (True, "在线"))
    page.status_timer.stop()
    page.command_timer.stop()
    page.unlock.blockSignals(True)
    page.unlock.setChecked(True)
    page.unlock.blockSignals(False)
    page.sync_from_robot(quiet=True)
    targets = dict(telemetry.joint_positions)
    targets.update(
        {
            spec.name: 0.0
            for spec in ALL_SPECS
            if spec.name != "lift_joint"
            and not spec.name.startswith("openarmx_head_")
        }
    )
    page._set_targets(targets)
    page._home_phase = "arms"
    page.angle_step.setValue(0.05)

    page._command_tick()
    assert page._home_phase == "arms"
    assert published_lifts[-1] == pytest.approx(0.20)

    for name in telemetry.joint_positions:
        if "joint" in name and (
            name.startswith("openarmx_left_joint")
            or name.startswith("openarmx_right_joint")
        ):
            telemetry.joint_positions[name] = 0.0
    for _ in range(60):
        page._command_tick()
        if page._home_phase == "auxiliary":
            break
    assert page._home_phase == "auxiliary"
    assert all(lift == pytest.approx(0.20) for lift in published_lifts)

    page.close()
    app.processEvents()


def test_freedrive_home_success_starts_lift_only_after_both_arms_zero():
    app = QApplication.instance() or QApplication([])
    telemetry = _telemetry()
    telemetry.joint_positions["lift_joint"] = 0.20
    auxiliary_commands = []

    class FakeNode:
        @staticmethod
        def request_manual_joint_control(include_arms=True):
            assert not include_arms
            return True, "辅助控制已获取"

        @staticmethod
        def manual_joint_control_ready(include_arms=True):
            return not include_arms

        @staticmethod
        def publish_auxiliary_joint_positions(lift, head):
            auxiliary_commands.append((lift, list(head)))

    page = JointControlPage(FakeNode(), telemetry, lambda: (True, "在线"))
    page.status_timer.stop()
    page.command_timer.stop()
    page.freedrive_running = lambda: True
    page._home_phase = "freedrive_arms"
    telemetry.freedrive_home_state = "SUCCEEDED"
    telemetry.freedrive_home_detail = "MIT 拖动模式保持启用"

    page._poll_freedrive_home()
    assert page._home_phase == "auxiliary"
    assert not auxiliary_commands

    page._command_tick()
    assert auxiliary_commands[-1][0] == pytest.approx(0.0)
    page._home_phase = "idle"
    page.close()
    app.processEvents()


def test_navigation_node_arm_gain_requests_use_per_joint_parameter_names():
    calls = []

    class FakeClient:
        @staticmethod
        def service_is_ready():
            return True

        @staticmethod
        def call_async(request):
            calls.append(request)
            return object()

    node = object.__new__(NavigationRosNode)
    node._arm_parameter_get_clients = {"left": FakeClient(), "right": FakeClient()}
    node._arm_parameter_set_clients = {"left": FakeClient(), "right": FakeClient()}

    node.get_arm_gains("right")
    node.set_arm_gains("right", {"kp_joint1": 12.0, "kd_joint1": 0.8})

    assert calls[0].names[:4] == [
        "kp_joint1",
        "kd_joint1",
        "kp_joint2",
        "kd_joint2",
    ]
    sent = {parameter.name: parameter.value.double_value for parameter in calls[1].parameters}
    assert sent == {"kp_joint1": pytest.approx(12.0), "kd_joint1": pytest.approx(0.8)}
