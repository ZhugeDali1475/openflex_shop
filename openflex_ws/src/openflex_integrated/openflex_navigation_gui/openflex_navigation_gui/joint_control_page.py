"""Full-body joint controls embedded in the supermarket competition console."""

import json
import math
import os
import signal
import time
from dataclasses import dataclass
from typing import Callable, Optional

from PyQt5.QtCore import QProcess, QSettings, Qt, QTimer
from rcl_interfaces.msg import ParameterType
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)


@dataclass(frozen=True)
class JointControlSpec:
    name: str
    label: str
    minimum: float
    maximum: float
    kind: str = "angle"


LEFT_ARM_SPECS = tuple(
    JointControlSpec(f"openarmx_left_joint{index}", f"左臂 J{index}", lower, upper)
    for index, (lower, upper) in enumerate(
        (
            (-3.344396, 0.905604),
            (-3.2707963268, 0.1292036732),
            (-1.57, 1.57),
            (0.0, 1.8),
            (-1.5, 1.5),
            (-0.75, 0.75),
            (-1.4, 1.4),
        ),
        start=1,
    )
)
RIGHT_ARM_SPECS = tuple(
    JointControlSpec(f"openarmx_right_joint{index}", f"右臂 J{index}", lower, upper)
    for index, (lower, upper) in enumerate(
        (
            (-1.25, 3.0),
            (0.0, 3.2),
            (-1.57, 1.57),
            (0.0, 1.8),
            (-1.5, 1.5),
            (-0.75, 0.75),
            (-1.4, 1.4),
        ),
        start=1,
    )
)
LEFT_GRIPPER_SPEC = JointControlSpec(
    "openarmx_left_finger_joint1", "左夹爪", 0.0, 0.044, "gripper"
)
RIGHT_GRIPPER_SPEC = JointControlSpec(
    "openarmx_right_finger_joint1", "右夹爪", 0.0, 0.044, "gripper"
)
LIFT_SPEC = JointControlSpec("lift_joint", "升降柱", -0.650, 0.300, "linear")
HEAD_SPECS = (
    JointControlSpec("openarmx_head_yaw_joint", "头部 Yaw", -1.5708, 1.5708),
    JointControlSpec("openarmx_head_pitch_joint", "头部 Pitch", -1.5708, 1.5708),
)
ALL_SPECS = (
    *LEFT_ARM_SPECS,
    LEFT_GRIPPER_SPEC,
    *RIGHT_ARM_SPECS,
    RIGHT_GRIPPER_SPEC,
    LIFT_SPEC,
    *HEAD_SPECS,
)
AUXILIARY_SPECS = (LIFT_SPEC, *HEAD_SPECS)
HOME_ARM_SPECS = (
    *LEFT_ARM_SPECS,
    *RIGHT_ARM_SPECS,
)
HOME_ARM_JOINT_SPECS = HOME_ARM_SPECS
HOME_AUXILIARY_SPECS = (LIFT_SPEC, *HEAD_SPECS)
ARM_CONTROL_NAMES = {
    spec.name
    for spec in (
        *LEFT_ARM_SPECS,
        LEFT_GRIPPER_SPEC,
        *RIGHT_ARM_SPECS,
        RIGHT_GRIPPER_SPEC,
    )
}
ARM_SIDES = ("left", "right")
ARM_SIDE_LABELS = {"left": "左臂", "right": "右臂"}


def step_towards(current: float, target: float, maximum_step: float) -> float:
    delta = target - current
    if abs(delta) <= maximum_step:
        return target
    return current + math.copysign(maximum_step, delta)


class JointControlPage(QWidget):
    """RViz-style joint sliders using the existing ros2_control owners."""

    def __init__(
        self,
        ros_node,
        telemetry,
        motion_allowed: Callable[[], tuple[bool, str]],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.ros_node = ros_node
        self.telemetry = telemetry
        self.motion_allowed = motion_allowed
        self.settings = QSettings("OpenArmX", "OpenFlexCompetitionJointControl")
        self.rows = {}
        self._suppress_changes = False
        self._command_positions: dict[str, float] = {}
        self._target_dirty = False
        self._last_publish_time = 0.0
        self._home_phase = "idle"
        self._freedrive_home_future = None
        self._lift_homing_future = None
        self._lift_homing_requested = False
        self._lift_homing_completion_synced = False
        self._freedrive_process = QProcess(self)
        self._freedrive_process.setProcessChannelMode(QProcess.MergedChannels)
        self._freedrive_process.readyReadStandardOutput.connect(
            self._read_freedrive_output
        )
        self._freedrive_process.finished.connect(self._freedrive_finished)
        self._freedrive_active = False
        self._freedrive_stop_requested = False
        self._freedrive_process_group_id = None
        self._freedrive_output_tail: list[str] = []
        self._gain_read_futures = {}
        self._gain_write_futures = {}
        self.gain_rows = {}
        self._build_ui()
        self._load_presets()

        self.command_timer = QTimer(self)
        self.command_timer.timeout.connect(self._command_tick)
        self.command_timer.start(20)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(200)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        status_group = QGroupBox("整机关节控制")
        status_layout = QGridLayout(status_group)
        self.connection_status = QLabel("等待整机控制和 /joint_states")
        self.connection_status.setWordWrap(True)
        self.sync_button = QPushButton("从机器人同步")
        self.sync_button.clicked.connect(self.sync_from_robot)
        self.unlock = QCheckBox("解锁关节运动")
        self.unlock.toggled.connect(self._unlock_changed)
        self.home_button = QPushButton("Home（双臂优先）")
        self.home_button.clicked.connect(self.move_home)
        self.home_status = QLabel("Home：待命")
        self.home_status.setWordWrap(True)
        self.stop_button = QPushButton("停止关节指令")
        self.stop_button.setObjectName("dangerButton")
        self.stop_button.clicked.connect(self.stop_commands)
        status_layout.addWidget(self.connection_status, 0, 0, 1, 3)
        status_layout.addWidget(self.sync_button, 1, 0)
        status_layout.addWidget(self.unlock, 1, 1)
        status_layout.addWidget(self.stop_button, 1, 2)
        status_layout.addWidget(self.home_button, 2, 0)
        status_layout.addWidget(self.home_status, 2, 1, 1, 2)
        status_layout.addWidget(QLabel("角度步长"), 3, 1)
        self.angle_step = self._spin(0.001, 0.05, 0.01, 0.001, " rad/周期", 3)
        status_layout.addWidget(self.angle_step, 3, 2)
        status_layout.addWidget(QLabel("直线步长"), 4, 1)
        self.linear_step = self._spin(0.0001, 0.005, 0.001, 0.0001, " m/周期", 4)
        status_layout.addWidget(self.linear_step, 4, 2)
        root.addWidget(status_group)

        freedrive_group = QGroupBox("拖动示教")
        freedrive_layout = QGridLayout(freedrive_group)
        freedrive_layout.addWidget(QLabel("拖动手臂"), 0, 0)
        self.freedrive_arm = QComboBox()
        self.freedrive_arm.addItem("双臂", "both")
        self.freedrive_arm.addItem("左臂", "left")
        self.freedrive_arm.addItem("右臂", "right")
        freedrive_layout.addWidget(self.freedrive_arm, 0, 1)
        self.freedrive_button = QPushButton("进入拖动模式")
        self.freedrive_button.clicked.connect(self.toggle_freedrive)
        freedrive_layout.addWidget(self.freedrive_button, 0, 2)
        self.freedrive_status = QLabel("拖动模式：未启用")
        self.freedrive_status.setWordWrap(True)
        freedrive_layout.addWidget(self.freedrive_status, 1, 0, 1, 3)
        root.addWidget(freedrive_group)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        content_layout = QGridLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setColumnStretch(0, 1)
        content_layout.setColumnStretch(1, 1)
        content_layout.addWidget(
            self._joint_group("左臂与夹爪", (*LEFT_ARM_SPECS, LEFT_GRIPPER_SPEC)),
            0,
            0,
        )
        content_layout.addWidget(
            self._joint_group("右臂与夹爪", (*RIGHT_ARM_SPECS, RIGHT_GRIPPER_SPEC)),
            0,
            1,
        )
        lift_group = self._joint_group("升降柱", (LIFT_SPEC,))
        self.lift_zero_button = QPushButton("立柱调零")
        self.lift_zero_button.setToolTip("调用升降驱动的零点标定流程")
        self.lift_zero_button.clicked.connect(self.start_lift_homing)
        lift_layout = lift_group.layout()
        lift_layout.addWidget(self.lift_zero_button, 2, 0)
        self.lift_homing_status = QLabel("调零状态：等待驱动")
        self.lift_homing_status.setWordWrap(True)
        lift_layout.addWidget(self.lift_homing_status, 2, 1, 1, 3)
        content_layout.addWidget(lift_group, 1, 0, 1, 2)
        content_layout.addWidget(self._joint_group("头部", HEAD_SPECS), 2, 0, 1, 2)
        content_layout.addWidget(self._gain_group(), 3, 0, 1, 2)
        scroll.setWidget(content)
        root.addWidget(scroll, 1)

        preset_group = QGroupBox("姿态记录")
        preset_layout = QHBoxLayout(preset_group)
        self.preset_name = QLineEdit()
        self.preset_name.setPlaceholderText("姿态名称")
        save_button = QPushButton("记录当前机器人位姿")
        save_button.clicked.connect(self.save_robot_preset)
        self.preset_buttons = QWidget()
        self.preset_buttons_layout = QHBoxLayout(self.preset_buttons)
        self.preset_buttons_layout.setContentsMargins(0, 0, 0, 0)
        preset_layout.addWidget(self.preset_name)
        preset_layout.addWidget(save_button)
        preset_layout.addWidget(self.preset_buttons, 1)
        root.addWidget(preset_group)

    def _gain_group(self) -> QGroupBox:
        group = QGroupBox("拖动模式关节刚度 / 阻尼")
        layout = QGridLayout(group)
        layout.addWidget(QLabel("手臂"), 0, 0)
        layout.addWidget(QLabel("关节"), 0, 1)
        layout.addWidget(QLabel("Kp"), 0, 2)
        layout.addWidget(QLabel("Kd"), 0, 3)
        row = 1
        for side in ARM_SIDES:
            for joint in range(1, 8):
                kp_maximum = 5000.0 if joint <= 4 else 500.0
                kd_maximum = 100.0 if joint <= 4 else 5.0
                kp_default = 50.0 if joint <= 4 else 10.0
                kd_default = 2.5 if joint <= 4 else 0.5
                side_label = QLabel(ARM_SIDE_LABELS[side] if joint == 1 else "")
                joint_label = QLabel(f"J{joint}")
                kp = self._spin(0.0, kp_maximum, kp_default, 1.0, "", 2)
                kd = self._spin(0.0, kd_maximum, kd_default, 0.1, "", 2)
                layout.addWidget(side_label, row, 0)
                layout.addWidget(joint_label, row, 1)
                layout.addWidget(kp, row, 2)
                layout.addWidget(kd, row, 3)
                self.gain_rows[(side, joint)] = (kp, kd)
                row += 1
        self.read_gains_button = QPushButton("读取当前 Kp/Kd")
        self.read_gains_button.clicked.connect(self.read_arm_gains)
        self.apply_gains_button = QPushButton("应用所选手臂 Kp/Kd")
        self.apply_gains_button.clicked.connect(self.apply_arm_gains)
        self.gain_status = QLabel("等待硬件参数服务")
        self.gain_status.setWordWrap(True)
        layout.addWidget(self.read_gains_button, row, 0, 1, 2)
        layout.addWidget(self.apply_gains_button, row, 2, 1, 2)
        layout.addWidget(self.gain_status, row + 1, 0, 1, 4)
        return group

    @staticmethod
    def _spin(minimum, maximum, value, step, suffix, decimals) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setSuffix(suffix)
        return spin

    def _joint_group(self, title: str, specs) -> QGroupBox:
        group = QGroupBox(title)
        layout = QGridLayout(group)
        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("目标"), 0, 1, 1, 2)
        layout.addWidget(QLabel("当前位置 / 速度 / 力矩"), 0, 3)
        for row_index, spec in enumerate(specs, start=1):
            label = QLabel(spec.label)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimumWidth(240)
            slider.setRange(round(spec.minimum * 1000), round(spec.maximum * 1000))
            slider.setSingleStep(1)
            spin = QDoubleSpinBox()
            if spec.kind == "angle":
                spin.setRange(math.degrees(spec.minimum), math.degrees(spec.maximum))
                spin.setSingleStep(0.5)
                spin.setDecimals(1)
                spin.setSuffix(" deg")
            elif spec.kind == "gripper":
                spin.setRange(spec.minimum * 1000.0, spec.maximum * 1000.0)
                spin.setSingleStep(0.5)
                spin.setDecimals(1)
                spin.setSuffix(" mm")
            else:
                spin.setRange(spec.minimum, spec.maximum)
                spin.setSingleStep(0.005)
                spin.setDecimals(3)
                spin.setSuffix(" m")
            current = QLabel("--")
            current.setMinimumWidth(170)
            slider.valueChanged.connect(
                lambda raw, item=spec: self._slider_changed(item, raw)
            )
            spin.valueChanged.connect(
                lambda value, item=spec: self._spin_changed(item, value)
            )
            layout.addWidget(label, row_index, 0)
            layout.addWidget(slider, row_index, 1)
            layout.addWidget(spin, row_index, 2)
            layout.addWidget(current, row_index, 3)
            layout.setColumnStretch(1, 1)
            self.rows[spec.name] = (spec, slider, spin, current)
        return group

    @staticmethod
    def _display_value(spec: JointControlSpec, value: float) -> float:
        if spec.kind == "angle":
            return math.degrees(value)
        if spec.kind == "gripper":
            return value * 1000.0
        return value

    @staticmethod
    def _internal_value(spec: JointControlSpec, value: float) -> float:
        if spec.kind == "angle":
            return math.radians(value)
        if spec.kind == "gripper":
            return value / 1000.0
        return value

    def _slider_changed(self, spec: JointControlSpec, raw: int) -> None:
        if self._suppress_changes:
            return
        internal = raw / 1000.0
        spin = self.rows[spec.name][2]
        self._suppress_changes = True
        spin.setValue(self._display_value(spec, internal))
        self._suppress_changes = False
        self._target_changed()

    def _spin_changed(self, spec: JointControlSpec, value: float) -> None:
        if self._suppress_changes:
            return
        internal = self._internal_value(spec, value)
        slider = self.rows[spec.name][1]
        self._suppress_changes = True
        slider.setValue(round(internal * 1000.0))
        self._suppress_changes = False
        self._target_changed()

    def _target_changed(self) -> None:
        if self.unlock.isChecked():
            self._target_dirty = True

    def target_positions(self) -> dict[str, float]:
        return {
            name: slider.value() / 1000.0
            for name, (_spec, slider, _spin, _current) in self.rows.items()
        }

    def _has_complete_state(self) -> bool:
        return all(name in self.telemetry.joint_positions for name in self.rows)

    def sync_from_robot(self, quiet: bool = False) -> bool:
        if not self._has_complete_state():
            if not quiet:
                QMessageBox.warning(self, "关节状态不完整", "等待整机发布全部受控关节状态。")
            return False
        self._suppress_changes = True
        for name, (_spec, slider, _spin, _current) in self.rows.items():
            slider.setValue(round(self.telemetry.joint_positions[name] * 1000.0))
        self._suppress_changes = False
        self._command_positions = self.target_positions()
        self._target_dirty = False
        return True

    def _unlock_changed(self, checked: bool) -> None:
        if not checked:
            self.stop_commands()
            if self.ros_node is not None and hasattr(
                self.ros_node, "release_manual_joint_control"
            ):
                self.ros_node.release_manual_joint_control()
            return
        allowed, reason = self.motion_allowed()
        if not allowed or not self.sync_from_robot(quiet=True):
            self.unlock.blockSignals(True)
            self.unlock.setChecked(False)
            self.unlock.blockSignals(False)
            QMessageBox.warning(
                self,
                "无法解锁关节控制",
                reason if not allowed else "等待完整且新鲜的 /joint_states。",
            )
            return
        if self.ros_node is None:
            self.unlock.setChecked(False)
            return
        accepted, detail = self.ros_node.request_manual_joint_control(
            include_arms=not self.freedrive_running()
        )
        if not accepted:
            self.unlock.blockSignals(True)
            self.unlock.setChecked(False)
            self.unlock.blockSignals(False)
            QMessageBox.warning(self, "无法解锁关节控制", detail)

    def selected_freedrive_sides(self) -> tuple[str, ...]:
        selection = str(self.freedrive_arm.currentData())
        return ARM_SIDES if selection == "both" else (selection,)

    def freedrive_running(self) -> bool:
        return self._freedrive_process.state() != QProcess.NotRunning

    def toggle_freedrive(self) -> None:
        if self.freedrive_running():
            self.stop_freedrive()
        else:
            self.start_freedrive()

    def start_freedrive(self) -> None:
        allowed, reason = self.motion_allowed()
        if not allowed:
            QMessageBox.warning(self, "拖动模式不可用", reason)
            return
        if self.ros_node is None:
            QMessageBox.warning(self, "拖动模式不可用", "GUI ROS 节点未连接。")
            return
        sides = self.selected_freedrive_sides()
        if (
            hasattr(self.ros_node, "arm_gain_services_ready")
            and not self.ros_node.arm_gain_services_ready(sides)
        ):
            QMessageBox.warning(
                self,
                "拖动模式不可用",
                "机械臂硬件参数服务未就绪，请确认真实硬件已经启动。",
            )
            return
        if QMessageBox.question(
            self,
            "进入拖动模式",
            "机械臂将切换到 MIT 并把所选手臂 Kp 置零。请先托住机械臂，"
            "确认急停可用后继续。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        ) != QMessageBox.Yes:
            return
        if self.unlock.isChecked():
            self.stop_commands()
            self.ros_node.request_manual_joint_control(include_arms=False)
        elif hasattr(self.ros_node, "release_manual_joint_control"):
            self.ros_node.release_manual_joint_control()
        self.stop_commands()
        self._freedrive_active = False
        self._freedrive_stop_requested = False
        self._freedrive_output_tail.clear()
        selection = str(self.freedrive_arm.currentData())
        args = [
            "run",
            "openflex_navigation_gui",
            "openflex_freedrive",
            "--ros-args",
            "-p",
            f"arm_selection:={selection}",
        ]
        self.freedrive_status.setText("拖动模式：正在切换控制模式")
        self.freedrive_status.setStyleSheet("color:#f1c75b;")
        self.freedrive_button.setText("停止切换")
        self.freedrive_arm.setEnabled(False)
        self._freedrive_process.start("setsid", ["ros2", *args])
        if not self._freedrive_process.waitForStarted(3000):
            self.freedrive_status.setText(
                f"拖动模式：启动失败：{self._freedrive_process.errorString()}"
            )
            self.freedrive_status.setStyleSheet("color:#ff6b6b;")
            self.freedrive_button.setText("进入拖动模式")
            self.freedrive_arm.setEnabled(True)
            self._freedrive_process_group_id = None
            return
        self._freedrive_process_group_id = int(self._freedrive_process.processId())

    def stop_freedrive(self) -> None:
        if not self.freedrive_running():
            return
        self._freedrive_stop_requested = True
        self.freedrive_status.setText("拖动模式：正在恢复 Kp、控制模式和 MoveIt")
        self.freedrive_status.setStyleSheet("color:#f1c75b;")
        self.freedrive_button.setEnabled(False)
        try:
            if self._freedrive_process_group_id is None:
                raise ProcessLookupError
            os.killpg(self._freedrive_process_group_id, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            self._freedrive_process.terminate()

    def shutdown_freedrive(self, timeout_ms: int = 30000) -> bool:
        if not self.freedrive_running():
            return True
        self.stop_freedrive()
        return self._freedrive_process.waitForFinished(timeout_ms)

    def _read_freedrive_output(self) -> None:
        data = bytes(self._freedrive_process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        for line in data.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            self._freedrive_output_tail.append(stripped)
            del self._freedrive_output_tail[:-12]
            if "FREE-DRIVE ACTIVE" in stripped:
                self._freedrive_active = True
                self.freedrive_status.setText(
                    "拖动模式：双臂已释放；立柱、头部和底盘仍可控制"
                )
                self.freedrive_status.setStyleSheet("color:#5fd38d;")
                self.freedrive_button.setText("退出拖动模式")
                self.read_arm_gains(silent=True)

    def _freedrive_finished(self, code: int, _status) -> None:
        was_active = self._freedrive_active
        requested = self._freedrive_stop_requested
        self._freedrive_active = False
        self._freedrive_stop_requested = False
        self._freedrive_process_group_id = None
        self.freedrive_button.setEnabled(True)
        self.freedrive_button.setText("进入拖动模式")
        self.freedrive_arm.setEnabled(True)
        if self._home_phase == "freedrive_arms":
            self._home_phase = "idle"
            self._freedrive_home_future = None
            self.home_status.setText("Home：拖动进程退出，双臂回零已中止")
            self.home_status.setStyleSheet("color:#ff6b6b;")
        if code == 0 and (requested or was_active):
            self.freedrive_status.setText("拖动模式：已退出，原控制参数已恢复")
            self.freedrive_status.setStyleSheet("color:#5fd38d;")
            self.sync_from_robot(quiet=True)
            if self.unlock.isChecked() and hasattr(
                self.ros_node, "request_manual_joint_control"
            ):
                self.ros_node.request_manual_joint_control(include_arms=True)
            self.read_arm_gains(silent=True)
            return
        detail = "\n".join(self._freedrive_output_tail[-5:])
        self.freedrive_status.setText(
            f"拖动模式：异常退出（代码 {code}）"
            + (f"：{self._freedrive_output_tail[-1]}" if detail else "")
        )
        self.freedrive_status.setStyleSheet("color:#ff6b6b;")
        if self.unlock.isChecked():
            self.unlock.setChecked(False)
        if not requested:
            QMessageBox.critical(
                self,
                "拖动模式异常退出",
                f"freedrive_node 已退出，代码 {code}。\n\n{detail}",
            )

    def read_arm_gains(self, silent: bool = False) -> None:
        if self.ros_node is None or not hasattr(self.ros_node, "get_arm_gains"):
            if not silent:
                QMessageBox.warning(self, "无法读取 Kp/Kd", "硬件参数接口不可用。")
            return
        pending = {}
        for side in ARM_SIDES:
            future = self.ros_node.get_arm_gains(side)
            if future is None:
                if not silent:
                    QMessageBox.warning(
                        self,
                        "无法读取 Kp/Kd",
                        f"{ARM_SIDE_LABELS[side]}硬件参数服务未就绪。",
                    )
                return
            pending[side] = future
        self._gain_read_futures = pending
        self.gain_status.setText("正在读取双臂 Kp/Kd")
        self.gain_status.setStyleSheet("color:#f1c75b;")

    def apply_arm_gains(self) -> None:
        if self.ros_node is None or not hasattr(self.ros_node, "set_arm_gains"):
            QMessageBox.warning(self, "无法应用 Kp/Kd", "硬件参数接口不可用。")
            return
        if not self._freedrive_active:
            QMessageBox.warning(
                self,
                "Kp/Kd 调节不可用",
                "请先进入拖动模式。退出拖动模式时会恢复进入前的原始 Kp/Kd。",
            )
            return
        pending = {}
        for side in self.selected_freedrive_sides():
            gains = {}
            for joint in range(1, 8):
                kp, kd = self.gain_rows[(side, joint)]
                gains[f"kp_joint{joint}"] = kp.value()
                gains[f"kd_joint{joint}"] = kd.value()
            future = self.ros_node.set_arm_gains(side, gains)
            if future is None:
                QMessageBox.warning(
                    self,
                    "无法应用 Kp/Kd",
                    f"{ARM_SIDE_LABELS[side]}硬件参数服务未就绪。",
                )
                return
            pending[side] = future
        self._gain_write_futures = pending
        side_text = "、".join(ARM_SIDE_LABELS[side] for side in pending)
        self.gain_status.setText(f"正在应用{side_text} Kp/Kd")
        self.gain_status.setStyleSheet("color:#f1c75b;")

    def move_home(self) -> None:
        if self._home_active():
            QMessageBox.information(self, "整机 Home", "当前 Home 序列正在执行。")
            return
        allowed, reason = self.motion_allowed()
        if not allowed:
            QMessageBox.warning(self, "整机 Home 不可用", reason)
            return
        if self.ros_node is None or not self.sync_from_robot(quiet=True):
            QMessageBox.warning(
                self, "整机 Home 不可用", "等待完整且新鲜的 /joint_states。"
            )
            return
        dragging = self.freedrive_running()
        if QMessageBox.question(
            self,
            "整机 Home",
            "确认执行顺序回零？\n\n"
            "1. 双臂先回零并等待实际位置确认\n"
            "2. 确认双臂到位后，升降柱和头部才开始回零"
            + (
                "\n\n当前将保持 MIT 拖动模式，仅临时调整 Kp/Kd 完成双臂回零。"
                if dragging
                else ""
            ),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        ) != QMessageBox.Yes:
            return
        if dragging:
            if self.selected_freedrive_sides() != ARM_SIDES:
                QMessageBox.warning(
                    self,
                    "整机 Home 不可用",
                    "拖动模式下的顺序 Home 需要以“双臂”进入拖动模式。",
                )
                return
            if not hasattr(self.ros_node, "freedrive_home_ready") or not (
                self.ros_node.freedrive_home_ready()
            ):
                QMessageBox.warning(
                    self,
                    "整机 Home 不可用",
                    "/openarmx_freedrive/home_arms 服务未就绪。",
                )
                return
            self._target_dirty = False
            if hasattr(self.ros_node, "reset_lift_command_target"):
                self.ros_node.reset_lift_command_target()
            self.telemetry.freedrive_home_state = "REQUESTED"
            self.telemetry.freedrive_home_detail = "等待 freedrive_node 接收"
            self._freedrive_home_future = self.ros_node.start_freedrive_home()
            if self._freedrive_home_future is None:
                QMessageBox.warning(
                    self, "整机 Home 失败", "双臂回零服务在请求前离线。"
                )
                return
            self._home_phase = "freedrive_arms"
            self.home_status.setText(
                "Home：保持 MIT，正在临时调整 Kp/Kd 并让双臂回零；立柱保持不动"
            )
            self.home_status.setStyleSheet("color:#f1c75b;")
            return
        accepted, detail = self.ros_node.request_manual_joint_control(
            include_arms=True
        )
        if not accepted:
            self.home_status.setText(f"Home：无法获取双臂控制：{detail}")
            self.home_status.setStyleSheet("color:#ff6b6b;")
            return
        self.unlock.blockSignals(True)
        self.unlock.setChecked(True)
        self.unlock.blockSignals(False)
        current_targets = dict(self.telemetry.joint_positions)
        current_targets.update({spec.name: 0.0 for spec in HOME_ARM_SPECS})
        self._set_targets(current_targets)
        self._home_phase = "arms"
        self.home_status.setText("Home：双臂回零中，立柱保持当前位置")
        self.home_status.setStyleSheet("color:#f1c75b;")

    def _home_active(self) -> bool:
        return self._home_phase != "idle"

    def _home_positions_reached(self, specs) -> bool:
        for spec in specs:
            position = self.telemetry.joint_positions.get(spec.name)
            if position is None:
                return False
            tolerance = 0.003 if spec.kind in ("linear", "gripper") else 0.03
            if abs(float(position)) > tolerance:
                return False
        return True

    def _start_home_auxiliary_phase(self) -> None:
        self._set_targets({spec.name: 0.0 for spec in HOME_AUXILIARY_SPECS})
        self._home_phase = "auxiliary"
        self.home_status.setText("Home：双臂已到零位，升降柱和头部开始回零")
        self.home_status.setStyleSheet("color:#f1c75b;")

    def _poll_freedrive_home(self) -> None:
        future = self._freedrive_home_future
        if future is not None and future.done():
            self._freedrive_home_future = None
            try:
                response = future.result()
            except Exception as exc:
                self._home_phase = "idle"
                self.home_status.setText(f"Home：双臂回零请求异常：{exc}")
                self.home_status.setStyleSheet("color:#ff6b6b;")
                return
            if not response.success:
                self._home_phase = "idle"
                self.home_status.setText(f"Home：双臂回零被拒绝：{response.message}")
                self.home_status.setStyleSheet("color:#ff6b6b;")
                return
        if self._home_phase != "freedrive_arms":
            return
        state = self.telemetry.freedrive_home_state
        detail = self.telemetry.freedrive_home_detail
        if state in ("STARTING", "RUNNING", "RESTORING", "REQUESTED"):
            self.home_status.setText(
                f"Home：{detail or '拖动模式双臂回零进行中'}；立柱保持不动"
            )
            self.home_status.setStyleSheet("color:#f1c75b;")
            return
        if state == "SUCCEEDED":
            if not self._home_positions_reached(HOME_ARM_JOINT_SPECS):
                self.home_status.setText("Home：等待双臂零位反馈稳定，立柱保持不动")
                return
            accepted, control_detail = self.ros_node.request_manual_joint_control(
                include_arms=False
            )
            if not accepted:
                self._home_phase = "idle"
                self.home_status.setText(
                    f"Home：双臂已回零，但立柱控制获取失败：{control_detail}"
                )
                self.home_status.setStyleSheet("color:#ff6b6b;")
                return
            self.unlock.blockSignals(True)
            self.unlock.setChecked(True)
            self.unlock.blockSignals(False)
            self._command_positions = dict(self.telemetry.joint_positions)
            self._start_home_auxiliary_phase()
            return
        if state in ("FAILED", "CANCELED"):
            self._home_phase = "idle"
            self._target_dirty = False
            self.home_status.setText(f"Home：{detail or state}")
            self.home_status.setStyleSheet("color:#ff6b6b;")

    def stop_commands(self) -> None:
        if self._home_active():
            if self._home_phase == "freedrive_arms" and hasattr(
                self.ros_node, "cancel_freedrive_home"
            ):
                self.ros_node.cancel_freedrive_home()
            self._home_phase = "idle"
            self.home_status.setText("Home：已停止")
            self.home_status.setStyleSheet("color:#f1c75b;")
        self._target_dirty = False
        if self.ros_node is not None and hasattr(
            self.ros_node, "reset_lift_command_target"
        ):
            self.ros_node.reset_lift_command_target()
        if self._has_complete_state():
            self._command_positions = dict(self.telemetry.joint_positions)

    def start_lift_homing(self) -> None:
        allowed, reason = self.motion_allowed()
        if not allowed:
            QMessageBox.warning(self, "立柱调零不可用", reason)
            return
        if self.ros_node is None or not self.ros_node.lift_homing_ready():
            QMessageBox.warning(
                self,
                "立柱调零不可用",
                "/lift_slide_driver/start_homing 服务未就绪。",
            )
            return
        if self._lift_homing_active():
            QMessageBox.information(self, "立柱调零", "当前已有立柱调零操作正在执行。")
            return
        if QMessageBox.question(
            self,
            "确认立柱调零",
            "立柱将执行零点标定并自动运动，确认周围安全后继续。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        ) != QMessageBox.Yes:
            return
        self.stop_commands()
        self.unlock.setChecked(False)
        self._lift_homing_future = self.ros_node.start_lift_homing()
        if self._lift_homing_future is None:
            QMessageBox.warning(self, "立柱调零失败", "调零服务在请求前离线。")
            return
        self._lift_homing_requested = True
        self._lift_homing_completion_synced = False
        self.telemetry.lift_homing_progress_seen = False
        self.lift_homing_status.setText("调零状态：请求已发出")
        self.lift_homing_status.setStyleSheet("color:#f1c75b;")

    def _lift_homing_active(self) -> bool:
        return (
            self._lift_homing_requested
            or self.telemetry.lift_homing_state == "IN_PROGRESS"
            or bool(self._lift_homing_future and not self._lift_homing_future.done())
        )

    def _poll_lift_homing_future(self) -> None:
        future = self._lift_homing_future
        if future is None or not future.done():
            return
        self._lift_homing_future = None
        try:
            response = future.result()
        except Exception as exc:
            self._lift_homing_requested = False
            self.lift_homing_status.setText(f"调零状态：请求异常：{exc}")
            self.lift_homing_status.setStyleSheet("color:#ff6b6b;")
            return
        if not response.success:
            self._lift_homing_requested = False
            self.lift_homing_status.setText(f"调零状态：驱动拒绝：{response.message}")
            self.lift_homing_status.setStyleSheet("color:#ff6b6b;")

    def _poll_gain_futures(self) -> None:
        if self._gain_read_futures and all(
            future.done() for future in self._gain_read_futures.values()
        ):
            futures = self._gain_read_futures
            self._gain_read_futures = {}
            try:
                for side, future in futures.items():
                    response = future.result()
                    if len(response.values) != 14:
                        raise RuntimeError(
                            f"{ARM_SIDE_LABELS[side]}返回了不完整的参数"
                        )
                    for joint in range(1, 8):
                        kp_parameter = response.values[(joint - 1) * 2]
                        kd_parameter = response.values[(joint - 1) * 2 + 1]
                        if (
                            kp_parameter.type != ParameterType.PARAMETER_DOUBLE
                            or kd_parameter.type != ParameterType.PARAMETER_DOUBLE
                        ):
                            raise RuntimeError(
                                f"{ARM_SIDE_LABELS[side]} J{joint} Kp/Kd 类型错误"
                            )
                        kp, kd = self.gain_rows[(side, joint)]
                        kp.setValue(float(kp_parameter.double_value))
                        kd.setValue(float(kd_parameter.double_value))
            except Exception as exc:
                self.gain_status.setText(f"读取 Kp/Kd 失败：{exc}")
                self.gain_status.setStyleSheet("color:#ff6b6b;")
            else:
                self.gain_status.setText("双臂 Kp/Kd 已同步")
                self.gain_status.setStyleSheet("color:#5fd38d;")

        if self._gain_write_futures and all(
            future.done() for future in self._gain_write_futures.values()
        ):
            futures = self._gain_write_futures
            self._gain_write_futures = {}
            try:
                for side, future in futures.items():
                    response = future.result()
                    failures = [
                        result.reason
                        for result in response.results
                        if not result.successful
                    ]
                    if failures:
                        raise RuntimeError(
                            f"{ARM_SIDE_LABELS[side]}：{'；'.join(failures)}"
                        )
            except Exception as exc:
                self.gain_status.setText(f"应用 Kp/Kd 失败：{exc}")
                self.gain_status.setStyleSheet("color:#ff6b6b;")
            else:
                self.gain_status.setText("所选手臂 Kp/Kd 已应用")
                self.gain_status.setStyleSheet("color:#5fd38d;")

    def _set_targets(self, positions: dict[str, float]) -> None:
        self._suppress_changes = True
        for name, value in positions.items():
            if name in self.rows:
                self.rows[name][1].setValue(round(float(value) * 1000.0))
        self._suppress_changes = False
        self._target_dirty = True

    def _command_tick(self) -> None:
        if self._lift_homing_active():
            return
        if self._home_phase == "freedrive_arms":
            return
        if (
            not self.unlock.isChecked()
            or (not self._target_dirty and not self._home_active())
            or self.ros_node is None
        ):
            return
        auxiliary_only = self.freedrive_running()
        if (
            hasattr(self.ros_node, "manual_joint_control_ready")
            and not self.ros_node.manual_joint_control_ready(
                include_arms=not auxiliary_only
            )
        ):
            return
        allowed, _reason = self.motion_allowed()
        if not allowed:
            self.unlock.setChecked(False)
            return
        desired = self.target_positions()
        if not self._command_positions:
            if not self.sync_from_robot(quiet=True):
                return
        reached = True
        if self._home_phase == "arms":
            command_specs = HOME_ARM_SPECS
        elif self._home_phase == "auxiliary":
            command_specs = HOME_AUXILIARY_SPECS
        else:
            command_specs = AUXILIARY_SPECS if auxiliary_only else ALL_SPECS
        for spec in command_specs:
            current = self._command_positions[spec.name]
            if spec == LIFT_SPEC:
                updated = desired[spec.name]
            else:
                step = (
                    self.angle_step.value()
                    if spec.kind == "angle"
                    else self.linear_step.value()
                )
                updated = step_towards(current, desired[spec.name], step)
            self._command_positions[spec.name] = updated
            reached = reached and math.isclose(updated, desired[spec.name], abs_tol=1e-9)
        if auxiliary_only:
            self.ros_node.publish_auxiliary_joint_positions(
                self._command_positions[LIFT_SPEC.name],
                [self._command_positions[spec.name] for spec in HEAD_SPECS],
            )
        else:
            self.ros_node.publish_fullbody_joint_positions(
                [self._command_positions[spec.name] for spec in LEFT_ARM_SPECS],
                self._command_positions[LEFT_GRIPPER_SPEC.name],
                [self._command_positions[spec.name] for spec in RIGHT_ARM_SPECS],
                self._command_positions[RIGHT_GRIPPER_SPEC.name],
                self._command_positions[LIFT_SPEC.name],
                [self._command_positions[spec.name] for spec in HEAD_SPECS],
            )
        self._last_publish_time = time.monotonic()
        if self._home_phase == "arms":
            if reached and self._home_positions_reached(HOME_ARM_JOINT_SPECS):
                self._start_home_auxiliary_phase()
            else:
                self._target_dirty = True
        elif self._home_phase == "auxiliary":
            if reached and self._home_positions_reached(HOME_AUXILIARY_SPECS):
                self._home_phase = "idle"
                self._target_dirty = False
                self.home_status.setText("Home：完成，双臂先于立柱回零")
                self.home_status.setStyleSheet("color:#5fd38d;")
            else:
                self._target_dirty = True
        else:
            self._target_dirty = not reached

    def refresh_status(self) -> None:
        self._poll_freedrive_home()
        self._poll_lift_homing_future()
        self._poll_gain_futures()
        if self.ros_node is not None and hasattr(
            self.ros_node, "poll_manual_joint_control"
        ):
            self.ros_node.poll_manual_joint_control()
        now = time.monotonic()
        age_stamp = self.telemetry.last_seen.get("/joint_states")
        age = None if age_stamp is None else now - age_stamp
        allowed, reason = self.motion_allowed()
        complete = self._has_complete_state()
        if complete and age is not None and age < 1.0:
            state_text = f"关节状态在线（{age:.1f}s）"
            color = "#5fd38d" if allowed else "#f1c75b"
        else:
            state_text = "关节状态不完整或已超时"
            color = "#ff6b6b"
        manual_state = "released"
        manual_detail = ""
        if self.ros_node is not None and hasattr(
            self.ros_node, "manual_joint_control_status"
        ):
            manual_state, manual_detail = self.ros_node.manual_joint_control_status()
        if self.unlock.isChecked() and manual_state == "error":
            self.unlock.blockSignals(True)
            self.unlock.setChecked(False)
            self.unlock.blockSignals(False)
        suffix = f"；{manual_detail}" if manual_detail else ""
        self.connection_status.setText(f"{state_text}；{reason}{suffix}")
        self.connection_status.setStyleSheet(f"color:{color};")
        if self.unlock.isChecked() and not allowed:
            self.unlock.setChecked(False)
        self.unlock.setEnabled(allowed and complete)
        auxiliary_only = self.freedrive_running()
        self.unlock.setText(
            "解锁立柱/头部运动" if auxiliary_only else "解锁关节运动"
        )
        manual_ready = bool(
            self.ros_node
            and hasattr(self.ros_node, "manual_joint_control_ready")
            and self.ros_node.manual_joint_control_ready(
                include_arms=not auxiliary_only
            )
        )
        self.home_button.setEnabled(
            allowed
            and complete
            and not self._lift_homing_active()
            and not self._home_active()
        )
        homing_state = self.telemetry.lift_homing_state
        homing_detail = self.telemetry.lift_homing_detail
        if homing_state == "IN_PROGRESS":
            homing_text = "调零进行中"
            color = "#f1c75b"
        elif homing_state == "COMPLETED" and self.telemetry.lift_homing_progress_seen:
            self._lift_homing_requested = False
            homing_text = "调零完成"
            color = "#5fd38d"
            if complete and not self._lift_homing_completion_synced:
                self.sync_from_robot(quiet=True)
                self._lift_homing_completion_synced = True
        elif homing_state == "ERROR" and self.telemetry.lift_homing_progress_seen:
            self._lift_homing_requested = False
            homing_text = "调零错误"
            color = "#ff6b6b"
        else:
            homing_text = "调零待命"
            color = "#9a9a9a"
        if homing_detail:
            homing_text += f"：{homing_detail}"
        self.lift_homing_status.setText(f"调零状态：{homing_text}")
        self.lift_homing_status.setStyleSheet(f"color:{color};")
        homing_ready = bool(self.ros_node and self.ros_node.lift_homing_ready())
        self.lift_zero_button.setEnabled(
            allowed
            and homing_ready
            and not self._lift_homing_active()
        )
        gain_ready = bool(
            self.ros_node
            and hasattr(self.ros_node, "arm_gain_services_ready")
            and self.ros_node.arm_gain_services_ready(ARM_SIDES)
        )
        gains_busy = bool(self._gain_read_futures or self._gain_write_futures)
        self.read_gains_button.setEnabled(gain_ready and not gains_busy)
        self.apply_gains_button.setEnabled(
            gain_ready and self._freedrive_active and not gains_busy
        )
        for name, (_spec, slider, spin, _current_label) in self.rows.items():
            enabled = not auxiliary_only or name not in ARM_CONTROL_NAMES
            slider.setEnabled(enabled)
            spin.setEnabled(enabled)
        for name, (spec, _slider, _spin, current_label) in self.rows.items():
            if name not in self.telemetry.joint_positions:
                current_label.setText("--")
                continue
            position = self.telemetry.joint_positions[name]
            velocity = self.telemetry.joint_velocities.get(name, 0.0)
            effort = self.telemetry.joint_efforts.get(name, 0.0)
            if spec.kind == "angle":
                current_label.setText(
                    f"{math.degrees(position):+.1f} deg / "
                    f"{math.degrees(velocity):+.1f} deg/s / {effort:+.2f}"
                )
            elif spec.kind == "gripper":
                current_label.setText(
                    f"{position * 1000:+.1f} mm / {velocity * 1000:+.1f} mm/s / {effort:+.2f}"
                )
            else:
                current_label.setText(f"{position:+.3f} m / {velocity:+.3f} m/s / {effort:+.2f}")

    def _load_presets(self) -> None:
        try:
            presets = json.loads(str(self.settings.value("presets", "{}")))
        except (TypeError, ValueError):
            presets = {}
        self.presets = presets if isinstance(presets, dict) else {}
        self._rebuild_preset_buttons()

    def _store_presets(self) -> None:
        self.settings.setValue("presets", json.dumps(self.presets, sort_keys=True))

    def save_preset(self) -> None:
        name = self.preset_name.text().strip()
        if not name:
            QMessageBox.warning(self, "缺少名称", "请输入姿态名称。")
            return
        self.presets[name] = self.target_positions()
        self._store_presets()
        self._rebuild_preset_buttons()
        self.preset_name.clear()

    def save_robot_preset(self) -> None:
        stamp = self.telemetry.last_seen.get("/joint_states")
        fresh = stamp is not None and time.monotonic() - stamp < 1.0
        if not fresh or not self.sync_from_robot(quiet=True):
            QMessageBox.warning(
                self,
                "无法记录位姿",
                "等待完整且新鲜的 /joint_states 后再记录。",
            )
            return
        self.save_preset()

    def _rebuild_preset_buttons(self) -> None:
        while self.preset_buttons_layout.count():
            item = self.preset_buttons_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        for name in sorted(self.presets):
            move = QPushButton(name)
            move.setToolTip("调用姿态")
            move.clicked.connect(lambda _checked=False, key=name: self._recall_preset(key))
            delete = QPushButton("x")
            delete.setFixedWidth(28)
            delete.setToolTip(f"删除姿态 {name}")
            delete.clicked.connect(lambda _checked=False, key=name: self._delete_preset(key))
            self.preset_buttons_layout.addWidget(move)
            self.preset_buttons_layout.addWidget(delete)
        self.preset_buttons_layout.addStretch(1)

    def _recall_preset(self, name: str) -> None:
        if not self.unlock.isChecked():
            QMessageBox.warning(self, "关节控制未解锁", "解锁后才能调用姿态。")
            return
        positions = self.presets.get(name, {})
        if isinstance(positions, dict):
            self._set_targets(positions)

    def _delete_preset(self, name: str) -> None:
        self.presets.pop(name, None)
        self._store_presets()
        self._rebuild_preset_buttons()
