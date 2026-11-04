"""Competition workflow tab with ROS scene-driven product selection."""

from __future__ import annotations

import math
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Optional

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSizePolicy,
    QStyle,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

import yaml

from .competition_flow_core import (
    CompetitionOrderItem,
    WristCameraCalibration,
    camera_aware_waypoint_from_anchor,
    index_shelf_face_anchors,
    measure_start_clearance,
    parse_competition_order,
)
from .scene_client import SceneRosClient
from .scene_registration_core import fingerprint_map_files
from .shelf_anchor_calibration import (
    ShelfAnchorCalibration,
    calibration_path,
    load_calibrations,
    save_calibration,
)

if TYPE_CHECKING:
    from .competition_window import CompetitionMainWindow


COMPETITION_SHELF_IDS = frozenset((1, 2))
SELECT_SKU_TEXT = "（请选择商品）"
TEST9_MAP_NAME = "supermarket_test_09"
TEST9_PRE_START_STATION = "PRE_START"
START_CLEARANCE_DISTANCE_M = 0.30
START_CLEARANCE_MAX_SPEED_MPS = 0.05
START_CLEARANCE_MIN_SPEED_MPS = 0.02
START_CLEARANCE_TIMEOUT_SEC = 12.0
START_CLEARANCE_READY_TIMEOUT_SEC = 20.0
START_CLEARANCE_LOCALIZATION_STABLE_SEC = 1.5
START_CLEARANCE_ODOM_MAX_AGE_SEC = 0.50
START_CLEARANCE_LATERAL_TOLERANCE_M = 0.03
START_CLEARANCE_YAW_TOLERANCE_RAD = math.radians(3.0)
START_CLEARANCE_SETTLE_SEC = 0.30
START_CLEARANCE_REUSE_MAX_AGE_SEC = 300.0
START_CLEARANCE_REUSE_WHEEL_DISTANCE_M = 0.10
START_CLEARANCE_REUSE_MAP_DISTANCE_M = 0.15
START_CLEARANCE_START_MAX_DISTANCE_M = 0.65
START_CLEARANCE_START_MAX_YAW_ERROR_RAD = math.radians(20.0)
TICKET_DELIVERY_CLEARANCE_M = 0.20
START_CLEARANCE_STAGES = frozenset(
    (
        "waiting_start_buffer_ready",
        "start_buffer",
        "start_buffer_settling",
        "waiting_start_buffer_navigation_ready",
    )
)


class CompetitionFlowTab(QWidget):
    """Phase 1: ROS scene-driven pick -> delivery using measured waypoints."""

    def __init__(self, parent: CompetitionMainWindow):
        super().__init__()
        self.window = parent
        self._phase1_stage = "idle"
        self._test9_workflow = parent.profile.map_name == TEST9_MAP_NAME
        self._selected_sku: Optional[str] = None
        self._selected_slot_info: Optional[dict] = None
        self._real_grasp_stage = "idle"
        self._delivery_station = "jiaofu2" if self._test9_workflow else "delivery_counter"
        self._start_station = (
            TEST9_PRE_START_STATION if self._test9_workflow else "PRE_START"
        )
        self._anchors: dict[tuple[int, int], dict] = {}
        self._selected_anchor_key: Optional[tuple[int, int]] = None
        self._anchor_problem = "等待场景数据"
        self._order_lines: tuple[CompetitionOrderItem, ...] = ()
        self._order_queue: list[dict] = []
        self._order_completed = 0
        self._recognized_order_loaded = False
        self._applying_recognized_order = False
        self._start_buffer_origin: Optional[tuple[float, float, float]] = None
        self._start_buffer_started_at = 0.0
        self._start_buffer_wait_deadline = 0.0
        self._start_buffer_settle_since: Optional[float] = None
        self._start_buffer_destination = None
        self._start_buffer_completed_wheel_pose: Optional[tuple[float, float]] = None
        self._start_buffer_completed_map_pose: Optional[tuple[float, float]] = None
        self._start_buffer_completed_at = 0.0
        self._camera_calibrations = self._load_camera_calibrations()
        self.scene_client = SceneRosClient(parent.ros_node) if parent.ros_node else None
        self._scene_snapshot: Optional[dict] = None
        self._scene_future = None
        self._scene_request_identity: Optional[tuple[str, str]] = None
        self._scene_identity: Optional[tuple[str, str]] = None
        self._scene_reload_needed = True
        self._next_scene_retry_at = 0.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        title = QLabel("商超比赛流程 - 第一阶段：拣货与交付")
        title.setStyleSheet("font-size:16px; font-weight:bold; color:#5fd38d;")
        layout.addWidget(title)

        phase1_group = self._build_phase1_group()
        layout.addWidget(phase1_group)

        # The manual harness owns the widgets the phase-1 automation drives,
        # so it must live on this page and be created before any status read.
        layout.addWidget(parent._build_pick_chain_group())

        phase2_placeholder = QGroupBox("第二阶段：补货")
        phase2_placeholder.setEnabled(False)
        phase2_label = QLabel("待实现")
        phase2_label.setAlignment(Qt.AlignCenter)
        phase2_layout = QVBoxLayout(phase2_placeholder)
        phase2_layout.addWidget(phase2_label)
        layout.addWidget(phase2_placeholder)

        phase3_placeholder = QGroupBox("第三阶段：归位错放商品")
        phase3_placeholder.setEnabled(False)
        phase3_label = QLabel("待实现")
        phase3_label.setAlignment(Qt.AlignCenter)
        phase3_layout = QVBoxLayout(phase3_placeholder)
        phase3_layout.addWidget(phase3_label)
        layout.addWidget(phase3_placeholder)

        layout.addStretch(1)

        self._populate_sku_combo([])
        self.refresh_scene()

    @staticmethod
    def _load_camera_calibrations() -> dict[int, WristCameraCalibration]:
        installed = (
            Path(get_package_share_directory("openflex_navigation_gui"))
            / "config/right_wrist_shelf_camera.yaml"
        )
        source = Path(__file__).resolve().parents[1] / "config/right_wrist_shelf_camera.yaml"
        path = installed if installed.is_file() else source
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict) or int(document.get("schema_version", 0)) != 1:
            raise ValueError("不支持的右腕货架相机标定格式")
        base_height = float(document["base_height_m"])
        default_safe_roi = float(document["safe_roi_fraction"])
        result = {}
        for raw_level, value in document.get("levels", {}).items():
            level = int(raw_level)
            image = value["image"]
            arm_level = int(value["arm_level"])
            safe_roi = float(value.get("safe_roi_fraction", default_safe_roi))
            if arm_level < 0:
                raise ValueError("右腕货架相机标定的示教层级不能为负数")
            if not 0.0 < safe_roi < 0.5:
                raise ValueError("右腕货架相机标定的安全画面边界必须在 0 到 0.5 之间")
            result[level] = WristCameraCalibration(
                level=level,
                translation_xyz_m=tuple(float(v) for v in value["translation_xyz_m"]),
                quaternion_xyzw=tuple(float(v) for v in value["quaternion_xyzw"]),
                image_width=int(image["width"]),
                image_height=int(image["height"]),
                fx=float(image["fx"]),
                fy=float(image["fy"]),
                cx=float(image["cx"]),
                cy=float(image["cy"]),
                base_height_m=base_height,
                safe_roi_fraction=safe_roi,
                arm_level=arm_level,
            )
        return result

    def _build_phase1_group(self) -> QGroupBox:
        group = QGroupBox("第一阶段：拣货与交付")
        layout = QGridLayout(group)

        info = QLabel(
            "购物票订单 → 对应货架面锚点导航与角度对齐 → 右臂观察姿态 → "
            "沿货架横移到相机点 → 手动 Qwen 识别与右臂抓取 → move 姿态 → "
            "从商品点直退 30cm → C 点放置 → 返回起点。"
        )
        info.setWordWrap(True)
        info.setStyleSheet("color:#a0a0a0; font-size:11px;")

        layout.addWidget(QLabel("场景数据"), 0, 0)
        self.scene_status_label = QLabel("等待 ROS 场景服务")
        self.scene_status_label.setWordWrap(True)
        layout.addWidget(self.scene_status_label, 0, 1, 1, 2)
        self.refresh_scene_button = QToolButton()
        self.refresh_scene_button.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserReload)
        )
        self.refresh_scene_button.setToolTip("刷新场景数据")
        self.refresh_scene_button.clicked.connect(self.refresh_scene)
        layout.addWidget(self.refresh_scene_button, 0, 3)

        layout.addWidget(QLabel("货架面基准"), 1, 0)
        self.anchor_status_label = QLabel("等待场景数据")
        self.anchor_status_label.setWordWrap(True)
        layout.addWidget(self.anchor_status_label, 1, 1, 1, 2)
        self.capture_anchor_button = QPushButton("保存当前基准")
        self.capture_anchor_button.clicked.connect(self._capture_anchor_calibration)
        layout.addWidget(self.capture_anchor_button, 1, 3)

        layout.addWidget(QLabel("商品 1"), 2, 0)
        self.sku_combo = QComboBox()
        # Scene SKU names are external data and can be arbitrarily long.  Do not
        # let one value raise the page's minimum width and push it under the map.
        self.sku_combo.setMinimumContentsLength(8)
        self.sku_combo.setSizeAdjustPolicy(
            QComboBox.AdjustToMinimumContentsLengthWithIcon
        )
        self.sku_combo.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.sku_combo.currentTextChanged.connect(self._on_sku_selected)
        layout.addWidget(self.sku_combo, 2, 1, 1, 2)
        self.quantity_spin = QSpinBox()
        self.quantity_spin.setRange(1, 9)
        self.quantity_spin.setValue(1)
        self.quantity_spin.setSuffix(" 件")
        self.quantity_spin.valueChanged.connect(self._on_order_input_edited)
        layout.addWidget(self.quantity_spin, 2, 3)

        layout.addWidget(QLabel("商品 2"), 3, 0)
        self.sku_combo_2 = QComboBox()
        self.sku_combo_2.setMinimumContentsLength(8)
        self.sku_combo_2.setSizeAdjustPolicy(
            QComboBox.AdjustToMinimumContentsLengthWithIcon
        )
        self.sku_combo_2.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.sku_combo_2.currentTextChanged.connect(self._on_second_sku_selected)
        layout.addWidget(self.sku_combo_2, 3, 1, 1, 2)
        self.quantity_spin_2 = QSpinBox()
        self.quantity_spin_2.setRange(1, 9)
        self.quantity_spin_2.setValue(1)
        self.quantity_spin_2.setSuffix(" 件")
        self.quantity_spin_2.valueChanged.connect(self._on_order_input_edited)
        layout.addWidget(self.quantity_spin_2, 3, 3)

        self.slot_info_label = QLabel("位置信息：请选择商品")
        self.slot_info_label.setWordWrap(True)
        layout.addWidget(self.slot_info_label, 4, 0, 1, 4)

        self.grasp_success_checkbox = QCheckBox("确认已在抓取调试页完成抓取")
        self.grasp_success_checkbox.setChecked(False)
        self.grasp_success_checkbox.setEnabled(False)
        layout.addWidget(self.grasp_success_checkbox, 5, 0, 1, 4)

        self.phase1_status_label = QLabel("状态：等待")
        self.phase1_status_label.setWordWrap(True)
        self.phase1_status_label.setSizePolicy(
            QSizePolicy.Ignored, QSizePolicy.Preferred
        )
        self.phase1_status_label.setStyleSheet("font-weight:bold;")
        layout.addWidget(self.phase1_status_label, 6, 0, 1, 4)

        button_row = QHBoxLayout()
        self.start_buffer_button = QPushButton("起点清障单项测试")
        self.start_buffer_button.clicked.connect(self._start_buffer_move)
        button_row.addWidget(self.start_buffer_button)
        self.start_phase1_button = QPushButton("开始第一阶段流程")
        self.start_phase1_button.setObjectName("primaryButton")
        self.start_phase1_button.clicked.connect(self._start_phase1)
        button_row.addWidget(self.start_phase1_button)

        self.cancel_phase1_button = QPushButton("停止当前流程")
        self.cancel_phase1_button.clicked.connect(self._cancel_phase1)
        self.cancel_phase1_button.setEnabled(False)
        button_row.addWidget(self.cancel_phase1_button)
        layout.addLayout(button_row, 7, 0, 1, 4)

        layout.addWidget(info, 8, 0, 1, 4)
        return group

    def _publish_start_buffer_stop(self) -> None:
        ros = self.window.ros_node
        if ros is not None and hasattr(ros, "publish_start_clearance"):
            ros.publish_start_clearance(0.0)

    def _start_buffer_move(self) -> None:
        if not self.window._process_running("navigation"):
            QMessageBox.warning(
                self,
                "定位导航未启动",
                "请先启动定位导航并等待初定位稳定，再执行起点清障测试。",
            )
            return
        self._queue_start_buffer(None)

    def _queue_start_buffer(self, destination) -> None:
        self._publish_start_buffer_stop()
        self._start_buffer_origin = None
        self._start_buffer_started_at = 0.0
        self._start_buffer_settle_since = None
        self._start_buffer_destination = destination
        self._start_buffer_wait_deadline = (
            time.monotonic() + START_CLEARANCE_READY_TIMEOUT_SEC
        )
        self._phase1_stage = "waiting_start_buffer_ready"
        self.phase1_status_label.setText(
            "状态：等待定位、Nav2 与安全感知稳定，随后相对当前位置直行 30cm"
        )
        self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")
        self.window._append_log(
            "[起点清障] 等待完整导航安全链就绪；目标是相对当前位置前进 "
            "0.30m，不假定到达 PRE_START"
        )

    def _start_buffer_preflight_problems(self) -> list[str]:
        problems = [
            problem
            for problem in self.window._benchmark_preflight_problems()
            if not problem.startswith("起步位置不安全：")
        ]
        ros = self.window.ros_node
        state = self.window.telemetry
        if ros is None or not hasattr(ros, "publish_start_clearance"):
            problems.append("起点清障速度发布器未就绪")
        localization_since = state.localization_true_since
        localization_age = (
            0.0
            if localization_since is None
            else time.monotonic() - localization_since
        )
        if localization_age < START_CLEARANCE_LOCALIZATION_STABLE_SEC:
            problems.append(
                "初定位持续稳定不足 "
                f"{START_CLEARANCE_LOCALIZATION_STABLE_SEC:.1f} 秒"
            )
        odom_stamp = state.last_seen.get("/odom")
        if (
            odom_stamp is None
            or time.monotonic() - odom_stamp > START_CLEARANCE_ODOM_MAX_AGE_SEC
        ):
            problems.append("轮式里程计未就绪：/odom 超过 0.5 秒无新数据")
        acceptance = self.window.profile.acceptance
        if (
            state.base_linear_speed > acceptance.stopped_linear_mps
            or state.base_angular_speed > acceptance.stopped_angular_radps
        ):
            problems.append("底盘尚未停止")
        teleop_enabled = getattr(self.window, "teleop_enabled", None)
        if teleop_enabled is not None and teleop_enabled.isChecked():
            problems.append("手动底盘遥控仍启用")
        physical_start = self.window._mission_waypoints.get("START")
        if physical_start is None:
            physical_start = getattr(self.window, "_start_waypoint", None)
        if physical_start is None:
            problems.append("当前地图缺少物理 START 点")
        elif state.has_map_pose:
            start_distance = math.hypot(
                state.map_x - physical_start.x,
                state.map_y - physical_start.y,
            )
            if start_distance > START_CLEARANCE_START_MAX_DISTANCE_M:
                problems.append(
                    "当前定位不在物理 START 附近："
                    f"偏差 {start_distance:.2f}m"
                )
            yaw_error = math.atan2(
                math.sin(state.map_yaw - physical_start.yaw),
                math.cos(state.map_yaw - physical_start.yaw),
            )
            if abs(yaw_error) > START_CLEARANCE_START_MAX_YAW_ERROR_RAD:
                problems.append(
                    "当前朝向与物理 START 相差过大："
                    f"{math.degrees(yaw_error):.1f}°"
                )
        return list(dict.fromkeys(problems))

    def _begin_start_buffer_motion(self) -> None:
        state = self.window.telemetry
        self._start_buffer_origin = (
            state.wheel_x,
            state.wheel_y,
            state.wheel_yaw,
        )
        self._start_buffer_started_at = time.monotonic()
        self._start_buffer_settle_since = None
        self._start_buffer_completed_wheel_pose = None
        self._start_buffer_completed_map_pose = None
        self._start_buffer_completed_at = 0.0
        self._phase1_stage = "start_buffer"
        self.phase1_status_label.setText(
            "状态：从当前初定位位置低速直行清障 0.30m"
        )
        self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")
        map_pose = (
            f"，地图位姿 ({state.map_x:.3f}, {state.map_y:.3f})"
            if state.has_map_pose
            else ""
        )
        self.window._append_log(
            "[起点清障] 捕获当前轮式里程计与朝向，开始相对前进 0.30m"
            + map_pose
        )

    def _start_buffer_completion_is_current(self) -> bool:
        if (
            self._start_buffer_completed_wheel_pose is None
            or self._start_buffer_completed_at <= 0.0
            or time.monotonic() - self._start_buffer_completed_at
            > START_CLEARANCE_REUSE_MAX_AGE_SEC
            or not self.window.telemetry.localization_ok
        ):
            return False
        state = self.window.telemetry
        wheel_distance = math.hypot(
            state.wheel_x - self._start_buffer_completed_wheel_pose[0],
            state.wheel_y - self._start_buffer_completed_wheel_pose[1],
        )
        if wheel_distance > START_CLEARANCE_REUSE_WHEEL_DISTANCE_M:
            return False
        if self._start_buffer_completed_map_pose is not None and state.has_map_pose:
            map_distance = math.hypot(
                state.map_x - self._start_buffer_completed_map_pose[0],
                state.map_y - self._start_buffer_completed_map_pose[1],
            )
            if map_distance > START_CLEARANCE_REUSE_MAP_DISTANCE_M:
                return False
        return True

    def _mark_start_buffer_completed(self) -> None:
        state = self.window.telemetry
        self._start_buffer_completed_wheel_pose = (state.wheel_x, state.wheel_y)
        self._start_buffer_completed_map_pose = (
            (state.map_x, state.map_y) if state.has_map_pose else None
        )
        self._start_buffer_completed_at = time.monotonic()

    def _dispatch_ticket_delivery(self, delivery) -> bool:
        ticket_delivery = replace(
            delivery,
            target_clearance_m=max(
                float(delivery.target_clearance_m),
                TICKET_DELIVERY_CLEARANCE_M,
            ),
        )
        if not self.window._dispatch_waypoint(ticket_delivery):
            return False
        self._start_buffer_destination = None
        self._phase1_stage = "navigating_ticket"
        self.phase1_status_label.setText(
            f"状态：正在从清障后的实际位置导航到 {self._delivery_station} 识别购物票"
        )
        self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")
        self.window._append_log(
            f"[比赛流程] 从清障后的实时定位位置前往 {self._delivery_station}，"
            f"购物票观察净距 {ticket_delivery.target_clearance_m:.2f}m；"
            "不假定机器人位于 PRE_START"
        )
        return True

    def _update_start_buffer_move(self) -> None:
        stage = self._phase1_stage
        if stage not in START_CLEARANCE_STAGES:
            return
        ros = self.window.ros_node
        state = self.window.telemetry
        now = time.monotonic()

        if stage == "waiting_start_buffer_ready":
            self._publish_start_buffer_stop()
            problems = self._start_buffer_preflight_problems()
            if not problems:
                self._begin_start_buffer_motion()
            elif now >= self._start_buffer_wait_deadline:
                self._fail_phase1("起点清障等待超时：" + "；".join(problems))
            else:
                self.phase1_status_label.setText(
                    "状态：等待起点清障条件（" + problems[0] + "）"
                )
            return

        if stage == "waiting_start_buffer_navigation_ready":
            self._publish_start_buffer_stop()
            problems = self.window._benchmark_preflight_problems()
            if not problems:
                destination = self._start_buffer_destination
                if destination is None or not self._dispatch_ticket_delivery(destination):
                    self._fail_phase1("起点清障完成，但交付台导航目标下发失败")
            elif now >= self._start_buffer_wait_deadline:
                self._fail_phase1(
                    "起点清障后导航等待超时：" + "；".join(problems)
                )
            else:
                self.phase1_status_label.setText(
                    "状态：清障完成，等待导航重新稳定（" + problems[0] + "）"
                )
            return

        origin = self._start_buffer_origin
        if ros is None or origin is None or not hasattr(ros, "publish_start_clearance"):
            self._publish_start_buffer_stop()
            self._fail_phase1("起点清障控制上下文丢失")
            return
        odom_stamp = state.last_seen.get("/odom")
        if (
            not state.localization_ok
            or odom_stamp is None
            or now - odom_stamp > START_CLEARANCE_ODOM_MAX_AGE_SEC
        ):
            self._publish_start_buffer_stop()
            self._fail_phase1("起点清障期间定位或轮式里程计失效")
            return

        try:
            progress = measure_start_clearance(
                origin,
                (state.wheel_x, state.wheel_y, state.wheel_yaw),
            )
        except ValueError as exc:
            self._publish_start_buffer_stop()
            self._fail_phase1(str(exc))
            return
        if abs(progress.lateral_drift_m) > START_CLEARANCE_LATERAL_TOLERANCE_M:
            self._publish_start_buffer_stop()
            self._fail_phase1(
                "起点清障横向漂移超限："
                f"{progress.lateral_drift_m:.3f}m"
            )
            return
        if abs(progress.yaw_error_rad) > START_CLEARANCE_YAW_TOLERANCE_RAD:
            self._publish_start_buffer_stop()
            self._fail_phase1(
                "起点清障偏航超限："
                f"{math.degrees(progress.yaw_error_rad):.1f}°"
            )
            return

        if stage == "start_buffer_settling":
            self._publish_start_buffer_stop()
            acceptance = self.window.profile.acceptance
            stopped = (
                state.base_linear_speed <= acceptance.stopped_linear_mps
                and state.base_angular_speed <= acceptance.stopped_angular_radps
            )
            if stopped:
                if self._start_buffer_settle_since is None:
                    self._start_buffer_settle_since = now
                elif now - self._start_buffer_settle_since >= START_CLEARANCE_SETTLE_SEC:
                    self._mark_start_buffer_completed()
                    if self._start_buffer_destination is None:
                        self._phase1_stage = "idle"
                        self.phase1_status_label.setText(
                            "状态：起点清障测试完成；当前位置由实时定位确定，"
                            "不要求等于 PRE_START"
                        )
                        self.phase1_status_label.setStyleSheet(
                            "font-weight:bold; color:#5fd38d;"
                        )
                        self.window._append_log(
                            "[起点清障] 单项测试完成；已记录当前实际位姿，"
                            "后续流程不会重复前进"
                        )
                    else:
                        self._phase1_stage = "waiting_start_buffer_navigation_ready"
                        self._start_buffer_wait_deadline = (
                            now + START_CLEARANCE_READY_TIMEOUT_SEC
                        )
                        self.phase1_status_label.setText(
                            "状态：相对前进 30cm 完成，等待代价地图与导航稳定"
                        )
            else:
                self._start_buffer_settle_since = None
            return

        if now - self._start_buffer_started_at > START_CLEARANCE_TIMEOUT_SEC:
            self._publish_start_buffer_stop()
            self._fail_phase1(
                "起点清障超时，仅前进 "
                f"{max(0.0, progress.forward_distance_m):.3f}m"
            )
            return
        if progress.forward_distance_m >= START_CLEARANCE_DISTANCE_M:
            self._publish_start_buffer_stop()
            self._phase1_stage = "start_buffer_settling"
            self._start_buffer_settle_since = None
            self.phase1_status_label.setText(
                "状态：相对当前位置已前进 "
                f"{progress.forward_distance_m:.3f}m，等待底盘停稳"
            )
            return

        remaining = START_CLEARANCE_DISTANCE_M - progress.forward_distance_m
        speed = min(
            START_CLEARANCE_MAX_SPEED_MPS,
            max(START_CLEARANCE_MIN_SPEED_MPS, remaining * 0.5),
        )
        ros.publish_start_clearance(speed)
        self.phase1_status_label.setText(
            "状态：相对初定位位置直行清障，前进 "
            f"{max(0.0, progress.forward_distance_m):.3f}/"
            f"{START_CLEARANCE_DISTANCE_M:.3f}m，"
            f"横漂 {progress.lateral_drift_m:.3f}m"
        )

    def _current_map_identity(self) -> tuple[str, str]:
        paths = self.window._map_paths()
        return paths.name, fingerprint_map_files(paths.yaml, paths.pgm)

    def _configure_anchor(self, snapshot: dict) -> None:
        self._anchors = {}
        self._selected_anchor_key = None
        try:
            indexed = index_shelf_face_anchors(
                self.window._mission_waypoints.values(), snapshot["shelves"]
            )
            if not indexed:
                raise RuntimeError("当前地图没有 shelf_approach 类型的货架面锚点")
            calibrations = load_calibrations(
                calibration_path(self.window._map_paths().map_2d_dir)
            )
            shelves = {int(value["id"]): value for value in snapshot["shelves"]}
            identity = self._current_map_identity()
            problems = []
            for key, (anchor, match) in sorted(indexed.items()):
                shelf = shelves[match.shelf_id]
                calibration = calibrations.get(anchor.waypoint_id)
                problem = ""
                if calibration is None:
                    problem = "待现场精调并保存基准"
                else:
                    problem = calibration.compatibility_problem(
                        map_name=identity[0],
                        map_fingerprint=identity[1],
                        anchor=anchor,
                        shelf=shelf,
                    )
                    if calibration.face != match.face:
                        problem = problem or "registered shelf face changed"
                self._anchors[key] = {
                    "anchor": anchor,
                    "match": match,
                    "shelf": shelf,
                    "calibration": None if problem else calibration,
                    "problem": problem,
                }
                if problem:
                    problems.append(
                        f"{anchor.waypoint_id}(货架{key[0]}/face={key[1]}): {problem}"
                    )
            ready = sum(
                value["calibration"] is not None for value in self._anchors.values()
            )
            self._anchor_problem = "；".join(problems)
            details = "，".join(
                f"{value['anchor'].waypoint_id}=货架{key[0]}/face={key[1]}"
                for key, value in sorted(self._anchors.items())
            )
            self.anchor_status_label.setText(
                f"已标定 {ready}/{len(self._anchors)} 个货架面；{details}"
                + (f"\n待处理：{self._anchor_problem}" if problems else "")
            )
            self.anchor_status_label.setStyleSheet(
                "color:#5fd38d;"
                if ready == len(self._anchors)
                else "color:#f1c75b;"
            )
        except Exception as exc:
            self._anchor_problem = str(exc)
            self.anchor_status_label.setText(self._anchor_problem)
            self.anchor_status_label.setStyleSheet("color:#f1c75b;")

    def _anchor_for_station(self, station_id: str) -> Optional[dict]:
        return next(
            (
                value
                for value in self._anchors.values()
                if value["anchor"].waypoint_id == station_id
            ),
            None,
        )

    def _capture_anchor_calibration(self) -> None:
        snapshot = self._scene_snapshot
        outcome = self.window._last_waypoint_outcome
        entry = None if outcome is None else self._anchor_for_station(outcome[0])
        state = self.window.telemetry
        if snapshot is None or entry is None:
            QMessageBox.warning(
                self,
                "货架面基准",
                "请先在比赛导航页执行一个货架面锚点并完成精调。",
            )
            return
        anchor = entry["anchor"]
        match = entry["match"]
        shelf = entry["shelf"]
        if (
            state.shelf_align_status != "succeeded"
            or not state.localization_ok
            or not state.has_map_pose
        ):
            QMessageBox.warning(
                self,
                "货架面基准",
                f"请重新执行 {anchor.waypoint_id}，并等待激光前沿精调成功。",
            )
            return
        clearance_error = abs(state.shelf_clearance - anchor.target_clearance_m)
        if clearance_error > anchor.clearance_tolerance_m + 0.005:
            QMessageBox.warning(
                self,
                "货架面基准",
                f"当前净距 {state.shelf_clearance:.3f}m 未满足该点精调容差。",
            )
            return
        if abs(math.atan2(
            math.sin(state.map_yaw - anchor.yaw),
            math.cos(state.map_yaw - anchor.yaw),
        )) > math.radians(2.0):
            QMessageBox.warning(self, "货架面基准", "当前朝向与锚点相差超过 2°。")
            return
        identity = self._current_map_identity()
        calibration = ShelfAnchorCalibration(
            map_name=identity[0],
            map_fingerprint=identity[1],
            station_id=anchor.waypoint_id,
            shelf_id=match.shelf_id,
            face=match.face,
            coarse_x=anchor.x,
            coarse_y=anchor.y,
            coarse_yaw=anchor.yaw,
            aligned_x=state.map_x,
            aligned_y=state.map_y,
            aligned_yaw=state.map_yaw,
            shelf_world_x=float(shelf["world_x"]),
            shelf_world_y=float(shelf["world_y"]),
            shelf_yaw=float(shelf["yaw"]),
            captured_at=datetime.now().astimezone().isoformat(timespec="seconds"),
        )
        save_calibration(
            calibration_path(self.window._map_paths().map_2d_dir), calibration
        )
        self._configure_anchor(snapshot)
        if self._selected_sku:
            self._select_sku(self._selected_sku)
        self.window._append_log(
            f"保存 {anchor.waypoint_id} 精调基准："
            f"({state.map_x:.3f}, {state.map_y:.3f}, "
            f"{math.degrees(state.map_yaw):.1f}°)"
        )

    def invalidate_scene(self) -> None:
        """Discard data derived from a previous map and request a new snapshot."""
        self._scene_snapshot = None
        self._scene_identity = None
        self._anchors = {}
        self._selected_anchor_key = None
        self._anchor_problem = "等待场景数据"
        self.anchor_status_label.setText(self._anchor_problem)
        self._scene_reload_needed = True
        self._selected_sku = None
        self._selected_slot_info = None
        self._populate_sku_combo([])
        self.slot_info_label.setText("位置信息：等待当前地图的场景数据")
        self.slot_info_label.setStyleSheet("color:#f1c75b;")
        self.refresh_scene()

    def refresh_scene(self) -> None:
        """Request a revision-consistent scene snapshot through ROS."""
        self._scene_reload_needed = True
        self._next_scene_retry_at = 0.0
        self._maybe_start_scene_request()

    def _maybe_start_scene_request(self) -> None:
        if self._scene_future is not None or not self._scene_reload_needed:
            return
        if time.monotonic() < self._next_scene_retry_at:
            return
        if self.scene_client is None:
            self.scene_status_label.setText("未连接 ROS")
            self.scene_status_label.setStyleSheet("color:#ff6b6b;")
            return
        if not self.scene_client.snapshot_service_ready:
            self.scene_status_label.setText("等待 get_scene_snapshot 服务")
            self.scene_status_label.setStyleSheet("color:#f1c75b;")
            self._next_scene_retry_at = time.monotonic() + 1.0
            return
        try:
            identity = self._current_map_identity()
            self._scene_future = self.scene_client.request_snapshot(*identity)
            self._scene_request_identity = identity
            self.scene_status_label.setText(f"正在读取 {identity[0]}")
            self.scene_status_label.setStyleSheet("color:#f1c75b;")
            self.refresh_scene_button.setEnabled(False)
        except Exception as exc:
            self._scene_request_failed(exc)

    def _poll_scene_future(self) -> None:
        if self._scene_future is None or not self._scene_future.done():
            return
        future = self._scene_future
        requested_identity = self._scene_request_identity
        self._scene_future = None
        self._scene_request_identity = None
        self.refresh_scene_button.setEnabled(True)
        try:
            snapshot = self.scene_client.snapshot_from_response(future.result())
            if requested_identity != self._current_map_identity():
                self._scene_reload_needed = True
                self._next_scene_retry_at = 0.0
                self._maybe_start_scene_request()
                return
            if not snapshot.get("requested_map_matches", False):
                binding = snapshot.get("map_binding")
                bound_map = "未绑定" if binding is None else binding.get("map_name", "")
                raise RuntimeError(
                    f"场景配准属于 {bound_map}，与当前地图 {requested_identity[0]} 不匹配"
                )
            shelf_ids = {int(value["id"]) for value in snapshot["shelves"]}
            if shelf_ids != COMPETITION_SHELF_IDS:
                raise RuntimeError(
                    "比赛场景必须且只能包含货架 ID 1、2；"
                    f"当前为 {sorted(shelf_ids)}"
                )
            self._scene_snapshot = snapshot
            self._scene_identity = requested_identity
            self._scene_reload_needed = False
            self.scene_status_label.setText(
                f"{requested_identity[0]} · 修订 {snapshot['scene_revision']}"
            )
            self.scene_status_label.setStyleSheet("color:#5fd38d;")
            self._configure_anchor(snapshot)
            self._populate_sku_combo(self._available_skus(snapshot))
        except Exception as exc:
            self._scene_request_failed(exc)

    def _scene_request_failed(self, error: Exception) -> None:
        self._scene_snapshot = None
        self._scene_identity = None
        self._scene_reload_needed = True
        self._next_scene_retry_at = time.monotonic() + 2.0
        self._selected_slot_info = None
        self._anchors = {}
        self._selected_anchor_key = None
        self._anchor_problem = "场景不可用"
        self.anchor_status_label.setText(self._anchor_problem)
        self.anchor_status_label.setStyleSheet("color:#ff6b6b;")
        self._populate_sku_combo([])
        self.scene_status_label.setText(f"场景不可用：{error}")
        self.scene_status_label.setStyleSheet("color:#ff6b6b;")
        self.slot_info_label.setText("位置信息：场景数据不可用")
        self.slot_info_label.setStyleSheet("color:#ff6b6b;")
        self.refresh_scene_button.setEnabled(self.scene_client is not None)

    def _available_skus(self, snapshot: dict) -> list[str]:
        return sorted({
            str(slot["actual_sku"])
            for slot in snapshot["slots"]
            if slot.get("actual_sku")
            and str(slot["actual_sku"]).strip().lower() != "unknown"
            and (int(slot["shelf_id"]), int(slot["face"])) in self._anchors
            and int(slot["level"]) in self._camera_calibrations
        })

    def _populate_sku_combo(self, skus: list[str]) -> None:
        current = self._selected_sku
        second = self.sku_combo_2.currentData() if self.sku_combo_2.count() else None
        for combo in (self.sku_combo, self.sku_combo_2):
            combo.blockSignals(True)
            combo.clear()
            combo.addItem(SELECT_SKU_TEXT, None)
            for sku in skus:
                combo.addItem(sku, sku)
        index = self.sku_combo.findData(current) if current is not None else -1
        second_index = self.sku_combo_2.findData(second) if second is not None else -1
        self.sku_combo.setCurrentIndex(index if index >= 0 else 0)
        self.sku_combo_2.setCurrentIndex(second_index if second_index >= 0 else 0)
        self.sku_combo.blockSignals(False)
        self.sku_combo_2.blockSignals(False)
        if index >= 0:
            self._select_sku(str(current))
        else:
            self._selected_sku = None
            self._selected_slot_info = None

    def _on_sku_selected(self, _text: str) -> None:
        self._on_order_input_edited()
        sku = self.sku_combo.currentData()
        if not sku:
            self.slot_info_label.setText("位置信息：请选择商品")
            self.slot_info_label.setStyleSheet("")
            self._selected_sku = None
            self._selected_slot_info = None
            return
        self._select_sku(str(sku))

    def _on_second_sku_selected(self, _text: str) -> None:
        self._on_order_input_edited()
        sku = self.sku_combo_2.currentData()
        if sku:
            self._select_sku(str(sku))

    def _on_order_input_edited(self, *_args) -> None:
        if not self._applying_recognized_order:
            self._recognized_order_loaded = False

    def _clear_previous_recognized_order(self) -> None:
        if not self._recognized_order_loaded:
            return
        self._applying_recognized_order = True
        try:
            self.sku_combo.setCurrentIndex(0)
            self.sku_combo_2.setCurrentIndex(0)
            self.quantity_spin.setValue(1)
            self.quantity_spin_2.setValue(1)
        finally:
            self._applying_recognized_order = False
            self._recognized_order_loaded = False

    def _select_sku(self, sku: str) -> None:
        """Derive one camera-centred goal using the SKU's shelf-face anchor."""
        self._selected_sku = sku
        self._selected_slot_info = None
        snapshot = self._scene_snapshot
        if snapshot is None or self._scene_identity is None:
            self.slot_info_label.setText("位置信息：等待当前地图的场景数据")
            self.slot_info_label.setStyleSheet("color:#f1c75b;")
            return
        try:
            if self._scene_identity != self._current_map_identity():
                self.invalidate_scene()
                return
            slots = [
                value for value in snapshot["slots"]
                if value.get("actual_sku") == sku
                and (int(value["shelf_id"]), int(value["face"])) in self._anchors
                and int(value["level"]) in self._camera_calibrations
            ]
            if not slots:
                raise RuntimeError(
                    f"有锚点的货架面没有已示教层级上的实际商品 {sku}"
                )

            sku_info = next(
                (value for value in snapshot["skus"] if value["sku"] == sku),
                None,
            )
            if sku_info is None:
                raise RuntimeError(f"SKU 表中没有 {sku}")
            qwen_prompt = str(sku_info.get("qwen_grounding_prompt", "")).strip()
            grasp_method = str(sku_info.get("grasp_method", "夹爪")).strip()
            if grasp_method != "夹爪":
                raise RuntimeError(f"抓取方式 {grasp_method} 尚未接入正式比赛链")
            if not qwen_prompt:
                raise RuntimeError("该 SKU 没有配置 Qwen 专用提示词")

            choices = []
            rejected = []
            for slot in slots:
                db_level = int(slot["level"])
                camera_calibration = self._camera_calibrations[db_level]
                arm_level = camera_calibration.arm_level
                if arm_level is None:
                    continue
                try:
                    key = (int(slot["shelf_id"]), int(slot["face"]))
                    entry = self._anchors[key]
                    anchor = entry["anchor"]
                    calibration = entry["calibration"]
                    if calibration is None:
                        raise RuntimeError(
                            f"{anchor.waypoint_id} 未完成现场精调标定："
                            f"{entry['problem']}"
                        )
                    target = camera_aware_waypoint_from_anchor(
                        slot,
                        entry["shelf"],
                        anchor,
                        calibration.aligned_pose,
                        camera_calibration,
                    )
                    safety_error = self.window._goal_rejection_reason(
                        target.waypoint.x,
                        target.waypoint.y,
                        target.waypoint.yaw,
                    )
                    choices.append((
                        bool(safety_error),
                        abs(target.lateral_shift_m),
                        str(slot["slot_id"]),
                        slot,
                        target,
                        arm_level,
                        safety_error,
                        key,
                        calibration,
                    ))
                except Exception as exc:
                    rejected.append(f"{slot['slot_id']}: {exc}")

            if not choices:
                detail = rejected[0] if rejected else "没有可用货位"
                raise RuntimeError(detail)

            (
                _, _, _, slot, target, arm_level, safety_error, key, calibration
            ) = min(choices)
            db_level = int(slot["level"])
            waypoint = target.waypoint
            self._selected_anchor_key = key
            self._selected_slot_info = {
                "sku": sku,
                "slot_id": slot["slot_id"],
                "shelf_id": int(slot["shelf_id"]),
                "face": int(slot["face"]),
                "level": db_level,
                "arm_level": arm_level,
                "y_cm": float(slot["y_cm"]),
                "station": waypoint.waypoint_id,
                "world_x": float(slot["world_x"]),
                "world_y": float(slot["world_y"]),
                "world_z": float(slot["world_z"]),
                "instances": len(slots),
                "compatible_instances": len(choices),
                "waypoint": waypoint,
                "aligned_target_pose": (
                    target.aligned_x,
                    target.aligned_y,
                    target.aligned_yaw,
                ),
                "anchor_return_pose": calibration.aligned_pose,
                "lateral_shift_m": target.lateral_shift_m,
                "camera_pixel_u": target.pixel_u,
                "camera_pixel_v": target.pixel_v,
                "camera_depth_m": target.camera_depth_m,
                "qwen_grounding_prompt": qwen_prompt,
                "grasp_method": grasp_method,
                "safety_error": safety_error,
                "scene_revision": int(snapshot["scene_revision"]),
            }
            text = (
                f"位置信息：{sku} / {slot['slot_id']} → 货架 {slot['shelf_id']} "
                f"face={slot['face']} 数据库 level={db_level}"
                f"（物理第 {db_level + 1} 层）y={slot['y_cm']:g}cm "
                f"→ {waypoint.waypoint_id} 精调后相机横移点\n"
                f"商品 map 坐标 ({slot['world_x']:.3f}, {slot['world_y']:.3f}, "
                f"{slot['world_z']:.3f})；相对货架面锚点横移 "
                f"{target.lateral_shift_m:+.3f}m；"
                f"横移目标 ({target.aligned_x:.3f}, {target.aligned_y:.3f})\n"
                f"右腕画面 ({target.pixel_u:.0f}, {target.pixel_v:.0f}) / "
                f"深度 {target.camera_depth_m:.3f}m；抓取方式 {grasp_method}\n"
                f"实际同款 {len(slots)} 个，可执行组合 {len(choices)} 个；"
                f"场景修订 {snapshot['scene_revision']}"
            )
            if safety_error:
                text += f"\n目标安全检查失败：{safety_error}"
                self.slot_info_label.setStyleSheet("color:#ff6b6b;")
            else:
                pose_name = (
                    "第四层货架右侧拍照"
                    if self._test9_workflow and arm_level == 4
                    else f"第 {arm_level} 层示教姿态"
                )
                text += f"\n货架观察使用 {pose_name}，目标安全检查通过"
                self.slot_info_label.setStyleSheet("color:#5fd38d;")
            self.slot_info_label.setText(text)
        except Exception as exc:
            self.slot_info_label.setText(f"选择失败：{exc}")
            self.slot_info_label.setStyleSheet("color:#ff6b6b;")
            self._selected_slot_info = None

    def _start_phase1(self) -> None:
        """Navigate to the delivery table, recognize the order, then pick it."""
        # An automatic result belongs to one run only. Keeping it visible after
        # completion must not silently turn it into the next run's manual order.
        self._clear_previous_recognized_order()
        manual_order = bool(
            self.sku_combo.currentData() or self.sku_combo_2.currentData()
        )
        if manual_order:
            if not self._build_order_queue():
                return
        else:
            ros = self.window.ros_node
            recognizer_ready = bool(
                ros
                and getattr(ros, "read_order_action_client", None)
                and ros.read_order_action_client.server_is_ready()
            )
            if not recognizer_ready:
                QMessageBox.warning(
                    self,
                    "购物票识别未就绪",
                    "未检测到 /read_delivery_order；请先启动购物票识别节点，"
                    "或在两个商品下拉框中输入联调订单。",
                )
                return
            self._order_lines = ()
            self._order_queue = []
            self._order_completed = 0
        delivery = self.window._mission_waypoints.get(self._delivery_station)
        if delivery is None:
            QMessageBox.warning(
                self, "交付台缺失", f"当前地图缺少 {self._delivery_station}。"
            )
            return
        if self._start_station not in self.window._mission_waypoints:
            QMessageBox.warning(
                self, "起点缓冲点缺失", f"当前地图缺少 {self._start_station}。"
            )
            return
        if self._start_buffer_completion_is_current():
            self._start_buffer_destination = delivery
            if not self._dispatch_ticket_delivery(delivery):
                self._fail_phase1("已完成起点清障，但交付台导航目标下发失败")
            return
        self._queue_start_buffer(delivery)

    def _build_order_queue(self) -> bool:
        """Build an executable per-item queue from the recognized/manual order."""
        raw_items = []
        for combo, quantity in (
            (self.sku_combo, self.quantity_spin),
            (self.sku_combo_2, self.quantity_spin_2),
        ):
            sku = combo.currentData()
            if sku:
                raw_items.append({"sku": str(sku), "quantity": quantity.value()})
        try:
            known_skus = self._available_skus(self._scene_snapshot or {})
            self._order_lines = parse_competition_order(
                {"items": raw_items}, known_skus=known_skus
            )
        except ValueError as exc:
            QMessageBox.warning(self, "购物票订单无效", str(exc))
            return False
        queue = []
        for item in self._order_lines:
            self._select_sku(item.sku)
            selected = self._selected_slot_info
            if selected is None:
                QMessageBox.warning(
                    self,
                    "商品不可执行",
                    f"{item.sku} 无法生成抓取路线：\n{self.slot_info_label.text()}",
                )
                return False
            if item.quantity > selected["instances"]:
                QMessageBox.warning(
                    self,
                    "库存数量不足",
                    f"订单需要 {item.sku} × {item.quantity}，"
                    f"当前可见库存只有 {selected['instances']}。",
                )
                return False
            queue.extend(dict(selected) for _ in range(item.quantity))
        self._order_queue = queue
        self._order_completed = 0
        return True

    def _begin_ticket_retreat(self) -> bool:
        delivery = self.window._mission_waypoints.get(self._delivery_station)
        ros = self.window.ros_node
        if delivery is None or ros is None:
            return False
        distance = (
            delivery.retreat_distance_m
            if delivery.should_retreat_after_place
            else 0.15
        )
        speed = delivery.retreat_speed_mps if delivery.retreat_speed_mps > 0 else 0.04
        if not ros.send_station_retreat_goal(
            delivery.waypoint_id,
            distance=distance,
            max_speed=speed,
            timeout_sec=8.0,
        ):
            return False
        self._phase1_stage = "ticket_retreating"
        self.phase1_status_label.setText("状态：购物票已确认，正在后退离开交付台")
        return True

    def _start_current_order_item(self) -> None:
        """Dispatch one queue entry through the existing proven pick chain."""
        if self._order_completed >= len(self._order_queue):
            self._fail_phase1("订单队列为空或索引越界")
            return
        self._selected_slot_info = self._order_queue[self._order_completed]
        self._selected_sku = self._selected_slot_info["sku"]
        if self._test9_workflow and (
            self.window.ros_node is None
            or not self.window.ros_node.right_gripper_action_client.server_is_ready()
        ):
            QMessageBox.warning(
                self,
                "右夹爪服务未就绪",
                "没有发现 /right_gripper_controller/gripper_cmd，"
                "无法保证 C 点放置后自动松开右夹爪。",
            )
            return

        station_id = self._selected_slot_info["station"]
        level = self._selected_slot_info["arm_level"]
        sku = self._selected_slot_info["sku"]

        safety_error = self._selected_slot_info.get("safety_error", "")
        if safety_error:
            QMessageBox.warning(
                self,
                "目标位姿不安全",
                f"{station_id} 未通过当前地图安全检查：\n{safety_error}",
            )
            return

        if level is None:
            QMessageBox.warning(
                self,
                "该层未示教",
                f"{sku} 位于场景第 {self._selected_slot_info['level']} 层，"
                "当前相机标定没有绑定可用的升降与手臂示教姿态，无法进入货架观察姿态。\n"
                "请先示教该层，或改选其他商品。",
            )
            return

        # Validate station exists in mission
        if station_id not in self.window._mission_waypoints:
            QMessageBox.warning(
                self,
                "导航点缺失",
                f"站点 {station_id} 不在当前地图的 competition_mission.yaml 中，"
                "请先在「比赛导航」页保存该站点。",
            )
            return

        if self._delivery_station not in self.window._mission_waypoints:
            QMessageBox.warning(
                self,
                "交付台缺失",
                f"交付台站点 {self._delivery_station} 不在当前地图中，"
                "请检查 competition_mission.yaml 是否包含 delivery_counter。",
            )
            return

        if self._start_station not in self.window._mission_waypoints:
            QMessageBox.warning(
                self,
                "起点缺失",
                f"起点 {self._start_station} 不在当前地图中。",
            )
            return

        window = self.window
        # Later stages of the shared chain re-read these widgets, so drive them
        # instead of writing the private state fields behind their back.
        source_index = window.pick_source_combo.findData(station_id)
        destination_index = window.pick_destination_combo.findData(self._delivery_station)
        if source_index < 0 or destination_index < 0:
            QMessageBox.warning(
                self,
                "站点未加载",
                f"手动联调下拉框中没有 {station_id} 或 {self._delivery_station}，"
                "请确认站点已保存并重新载入地图。",
            )
            return
        window.pick_source_combo.setCurrentIndex(source_index)
        window.pick_destination_combo.setCurrentIndex(destination_index)

        level_index = window.pick_shelf_level.findData(level)
        if level_index < 0:
            QMessageBox.warning(
                self, "层号不可用", f"手动联调下拉框中没有第 {level} 层。"
            )
            return
        window.pick_shelf_level.setCurrentIndex(level_index)

        arm_index = window.pick_arm_combo.findText("right")
        if arm_index >= 0:
            window.pick_arm_combo.setCurrentIndex(arm_index)
        window.pick_object_id.setText(sku)
        # Nav2 always approaches the proven A point.  The selected product pose
        # is reached later with the arm in its observation posture and the
        # shelf-clearance traverse controller active.
        window._pick_navigation_test_source_override = None
        window._pick_navigation_test_anchor_return_pose = self._selected_slot_info[
            "anchor_return_pose"
        ]
        if window.ros_node is not None:
            window.ros_node.grasp_debug_result = None
            invalidate_plan = getattr(
                window.ros_node, "invalidate_grasp_plan", None
            )
            if callable(invalidate_plan):
                invalidate_plan()
            else:
                window.ros_node.grasp_debug_plan_result = None
            window.ros_node.grasp_debug_status = "idle"
            window.ros_node.grasp_debug_message = ""

        self._real_grasp_stage = "waiting_observation"
        window._pick_navigation_test_return_destination = window._mission_waypoints[
            self._start_station
        ]
        self.grasp_success_checkbox.setChecked(False)
        self.grasp_success_checkbox.setText("确认已在抓取调试页完成抓取")
        self.grasp_success_checkbox.setEnabled(False)
        self.phase1_status_label.setText(
            f"状态：订单 {self._order_completed + 1}/{len(self._order_queue)}，"
            f"正在导航到 {station_id} 并准备抓取 {sku}"
        )
        self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")

        # _start_pick_preparation runs the full preflight and owns the state
        # machine; it shows its own dialog and leaves the stage alone on failure.
        window._start_pick_preparation(
            pre_navigation_level=0 if self._test9_workflow else None,
            transport_level=0 if self._test9_workflow else 2,
            item_target_pose=self._selected_slot_info["aligned_target_pose"],
        )
        if window._pick_navigation_test_stage not in (
            "releasing_manual_control_before_transport",
            "moving_to_transport_before_shelf",
            "navigating_to_shelf",
        ):
            self._fail_phase1("货架观察准备未启动，请查看上方联调状态")
            return

        window._append_log(
            f"[比赛流程] 订单 {self._order_completed + 1}/{len(self._order_queue)}："
            f"{sku} → {station_id} 第 {level} 层 → {self._delivery_station}"
        )

    def _cancel_phase1(self) -> None:
        """Stop the ongoing phase 1 workflow."""
        if self._phase1_stage not in (
            *START_CLEARANCE_STAGES,
            "navigating_ticket",
            "recognizing_ticket",
            "ticket_retreating",
            "executing",
            "returning_physical_start",
        ):
            return
        if self._phase1_stage in START_CLEARANCE_STAGES:
            self._publish_start_buffer_stop()
        elif self._phase1_stage == "executing":
            self.window._cancel_mock_pick_navigation_test("用户取消比赛流程")
        else:
            self.window._stop_competition_actions_for_manual_recovery()
        self._phase1_stage = "idle"
        self.phase1_status_label.setText("状态：已取消")
        self.phase1_status_label.setStyleSheet("font-weight:bold; color:#a0a0a0;")

    def continue_order_after_delivery_retreat(self) -> bool:
        """Start the next item, or let the shared chain return to PRE_START."""
        if self._phase1_stage != "executing" or not self._order_queue:
            return False
        self._order_completed += 1
        if self._order_completed >= len(self._order_queue):
            self.window._append_log(
                f"[比赛流程] {self._order_completed} 件商品全部交付，"
                f"开始返回 {self._start_station}"
            )
            return False
        self.window._pick_navigation_test_stage = "complete"
        self.window._append_log(
            f"[比赛流程] 已交付 {self._order_completed}/{len(self._order_queue)}，"
            "从交付台退避位置开始下一件"
        )
        self._start_current_order_item()
        return True

    def _fail_phase1(self, message: str) -> None:
        """Mark phase 1 as failed."""
        if self._phase1_stage in START_CLEARANCE_STAGES:
            self._publish_start_buffer_stop()
            self.window._stop_competition_actions_for_manual_recovery()
        if self.window._pick_navigation_test_stage != "failed":
            self.window._fail_mock_pick_navigation_test(message)
        self._phase1_stage = "failed"
        self.phase1_status_label.setText(
            f"状态：失败（{message}）；保持整机控制，可到关节控制页执行 Home"
        )
        self.phase1_status_label.setStyleSheet("font-weight:bold; color:#ff6b6b;")
        self.window._append_log(
            f"[比赛流程] 第一阶段失败：{message}；自动流程已停止，"
            "等待开放关节控制"
        )

    def _update_real_grasp(self) -> None:
        ros = self.window.ros_node
        selected = self._selected_slot_info
        if ros is None or selected is None:
            self._fail_phase1("手动抓取上下文不可用")
            return
        if self._real_grasp_stage == "waiting_observation":
            debug_page = getattr(self.window, "grasp_debug_page", None)
            sku_loaded = bool(
                debug_page
                and debug_page.select_manual_pick(selected["sku"], "right")
            )
            self._real_grasp_stage = "waiting_manual_grasp"
            self.grasp_success_checkbox.setText(
                "确认已在抓取调试页完成抓取（完成后勾选）"
            )
            self.grasp_success_checkbox.setChecked(False)
            self.grasp_success_checkbox.setEnabled(True)
            database_hint = "" if sku_loaded else "；若 SKU 未出现请先在该页刷新"
            self.phase1_status_label.setText(
                "状态：已到达手动抓取位置；请切到“抓取调试”页，"
                f"对右手 {selected['sku']} 手动执行仅检测、仅规划和执行抓取"
                f"{database_hint}。完成后返回本页勾选确认"
            )
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#5fd38d;"
            )
            self.window._append_log(
                f"[比赛流程] 已停在商品抓取位置；等待操作员在抓取调试页手动抓取 "
                f"{selected['sku']}，正式流程未下发 Qwen/规划/执行 Action"
            )
            return
        if self._real_grasp_stage == "waiting_manual_grasp":
            if not self.grasp_success_checkbox.isChecked():
                return
            self.grasp_success_checkbox.setEnabled(False)
            if not self.window._confirm_manual_pick_success():
                self.grasp_success_checkbox.setChecked(False)
                self.grasp_success_checkbox.setEnabled(True)
                self.phase1_status_label.setText(
                    "状态：尚未检测到右夹爪有效夹持；请在抓取调试页完成抓取后重试确认"
                )
                self.phase1_status_label.setStyleSheet(
                    "font-weight:bold; color:#ff6b6b;"
                )
                return
            self._real_grasp_stage = "complete"
            self.phase1_status_label.setText(
                "状态：手动抓取已确认，正在回 move 并从商品点直退"
            )
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#f1c75b;"
            )

    def update_phase1_from_workflow_state(self) -> None:
        """Called by the main window's periodic update to sync phase 1 UI."""
        if self._phase1_stage == "returning_physical_start":
            state = self.window.telemetry
            if state.retreat_test_status == "succeeded":
                self._start_buffer_completed_wheel_pose = None
                self._start_buffer_completed_map_pose = None
                self._start_buffer_completed_at = 0.0
                self._phase1_stage = "complete"
                self.phase1_status_label.setText(
                    "状态：第一阶段完成，已从 PRE_START 回到物理 START"
                )
                self.phase1_status_label.setStyleSheet(
                    "font-weight:bold; color:#5fd38d;"
                )
                self.window._append_log(
                    "[比赛流程] 第一阶段完成，机器人已回到物理 START"
                )
            elif state.retreat_test_status in ("failed", "rejected", "error"):
                self._fail_phase1(
                    state.retreat_test_message or "PRE_START 返回物理 START 失败"
                )
            return
        if self._phase1_stage == "navigating_ticket":
            state = self.window.telemetry
            if state.action_status in ("failed", "rejected", "error"):
                self._fail_phase1(f"购物票交付台导航失败：{state.action_status}")
                return
            if self.window._active_target is not None or not self.window._goal_terminal_handled:
                return
            outcome = self.window._last_waypoint_outcome
            if outcome is None or outcome[0] != self._delivery_station:
                return
            if not outcome[1] and state.shelf_align_status != "succeeded":
                self._fail_phase1(f"购物票交付台到达失败：{outcome[2]}")
                return
            ros = self.window.ros_node
            if ros is None:
                self._fail_phase1("购物票识别请求无法下发：ROS 未连接")
                return
            if self._order_queue:
                self.window._append_log(
                    "[购物票] 使用操作员主动确认的手动联调订单，不启动自动识别"
                )
                if not self._begin_ticket_retreat():
                    self._fail_phase1("购物票手动确认后退避 Action 下发失败")
                return
            if not ros.request_shopping_ticket(
                self._delivery_station,
                expected_distinct_skus=2,
                min_confidence=0.8,
            ):
                self._fail_phase1("ReadOrder Action 下发失败或已有识别请求正在运行")
                return
            self._phase1_stage = "recognizing_ticket"
            self.phase1_status_label.setText(
                "状态：交付台精对齐完成，已请求低头识别购物票"
            )
            self.window._append_log(
                "[购物票] 已发送 /read_delivery_order，等待真实识别结果"
            )
            return
        if self._phase1_stage == "recognizing_ticket":
            ros = self.window.ros_node
            if ros is None:
                self._fail_phase1("购物票识别期间 ROS 连接丢失")
                return
            status = str(getattr(ros, "shopping_ticket_status", "error"))
            if status == "succeeded":
                valid, message = self._apply_shopping_ticket_lines(
                    getattr(ros, "shopping_ticket_lines", ())
                )
                if not valid:
                    self._fail_phase1(message)
                    return
                if not self._build_order_queue():
                    self._fail_phase1("购物票识别结果无法生成可执行订单")
                    return
                if not self._begin_ticket_retreat():
                    self._fail_phase1("购物票识别后退避 Action 下发失败")
                return
            if status in (
                "failed",
                "rejected",
                "error",
                "canceled",
                "cancel_error",
                "cancel_rejected",
            ):
                detail = str(getattr(ros, "shopping_ticket_message", "")).strip()
                self._fail_phase1(detail or f"购物票识别失败：{status}")
                return
            phase = str(getattr(ros, "shopping_ticket_phase", "")).strip()
            phase_text = {
                "LOWERING_HEAD": "正在低头到 -70 度",
                "WAITING_FRESH_FRAME": "低头到位，等待 1280x720 新图像",
                "SUBMITTING_QWEN": "正在提交共享 Qwen 请求",
                "WAITING_FOR_QWEN": "共享 Qwen 正忙，正在串行等待",
                "QWEN_INFERENCE": "共享 Qwen 正在识别购物票",
                "RESTORING_HEAD": "图像已冻结，正在恢复头部角度",
                "VALIDATING_ORDER": "正在校验购物票订单",
            }.get(phase, phase or "等待 ReadOrder 结果")
            self.phase1_status_label.setText(f"状态：购物票识别中：{phase_text}")
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#f1c75b;"
            )
            return
        if self._phase1_stage == "ticket_retreating":
            state = self.window.telemetry
            if state.retreat_test_status == "succeeded":
                self._phase1_stage = "executing"
                self._start_current_order_item()
            elif state.retreat_test_status in ("failed", "rejected", "error"):
                self._fail_phase1(
                    state.retreat_test_message or "购物票识别后退避失败"
                )
            return
        if self._phase1_stage != "executing":
            return

        stage = self.window._pick_navigation_test_stage
        if stage == "ready":
            self._update_real_grasp()
        elif stage == "traversing_to_item":
            state = self.window.telemetry
            self.phase1_status_label.setText(
                "状态：右臂观察姿态保持，沿货架前往商品相机点 / "
                f"剩余 {state.shelf_traverse_remaining:.3f}m / "
                f"净距 {state.shelf_traverse_clearance:.3f}m"
            )
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#f1c75b;"
            )
        elif stage == "aligning_to_item_grasp_clearance":
            state = self.window.telemetry
            self.phase1_status_label.setText(
                "状态：商品横移点到位，向前精调至 5cm 抓取净距 / "
                f"当前 {state.shelf_clearance:.3f}m"
            )
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#f1c75b;"
            )
        elif stage == "retreating":
            state = self.window.telemetry
            self.phase1_status_label.setText(
                "状态：抓取完成，正在从商品点直退 30cm / "
                f"已退 {state.retreat_test_distance:.3f}m"
            )
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#f1c75b;"
            )
        elif stage == "ready_to_place":
            self.phase1_status_label.setText("状态：交付台精调完成，请放置并确认")
            self.phase1_status_label.setStyleSheet("font-weight:bold; color:#5fd38d;")
            self.grasp_success_checkbox.setText("确认精调放置完成（完成后勾选）")
            self.grasp_success_checkbox.setChecked(False)
            self.grasp_success_checkbox.setEnabled(True)
            self.window._pick_navigation_test_stage = "waiting_for_place_confirmation"
        elif stage == "waiting_for_place_confirmation":
            if self.grasp_success_checkbox.isChecked():
                self.grasp_success_checkbox.setEnabled(False)
                self.window._pick_navigation_test_stage = "ready_to_place"
                if not self.window._continue_after_place_confirmation():
                    self._fail_phase1("交付台退避 Action 下发失败")
                    return
                self.phase1_status_label.setText("状态：放置已确认，退避并返回起点")
        elif stage == "moving_to_place_transition":
            self.phase1_status_label.setText("状态：C 点精调完成，正在进入放置过渡2（夹爪保持）")
            self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")
        elif stage == "moving_to_place":
            self.phase1_status_label.setText("状态：正在进入放置2（夹爪保持）")
            self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")
        elif stage == "releasing_after_place":
            self.phase1_status_label.setText("状态：放置2到位，正在松开右夹爪")
            self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")
        elif stage == "returning_move_after_place":
            self.phase1_status_label.setText("状态：放置成功，正在返回 move 姿态")
            self.phase1_status_label.setStyleSheet("font-weight:bold; color:#f1c75b;")
        elif stage == "complete":
            ros = self.window.ros_node
            if self._test9_workflow and ros is not None:
                sent = ros.send_station_retreat_goal(
                    self._start_station,
                    distance=0.30,
                    max_speed=0.04,
                    timeout_sec=12.0,
                )
                if not sent:
                    self._fail_phase1("PRE_START 返回物理 START Action 下发失败")
                    return
                self._phase1_stage = "returning_physical_start"
                self.phase1_status_label.setText(
                    "状态：已回 PRE_START，正在直线后退 30cm 到物理 START"
                )
                self.phase1_status_label.setStyleSheet(
                    "font-weight:bold; color:#f1c75b;"
                )
                return
            self._phase1_stage = "complete"
            self.phase1_status_label.setText("状态：第一阶段完成")
            self.phase1_status_label.setStyleSheet("font-weight:bold; color:#5fd38d;")
            self.window._append_log("[比赛流程] 第一阶段完成")
        elif stage == "failed":
            self._fail_phase1(self.window.pick_chain_status.text())

    def _apply_shopping_ticket_lines(self, lines) -> tuple[bool, str]:
        if self._scene_snapshot is None:
            return False, "购物票已识别，但场景数据不可用"
        raw_items = [
            {"sku": str(line.sku).strip(), "quantity": int(line.quantity)}
            for line in lines
        ]
        try:
            order = parse_competition_order(
                {"items": raw_items},
                known_skus=self._available_skus(self._scene_snapshot),
            )
            if len(order) != 2:
                raise ValueError(f"期望 2 种商品，实际识别到 {len(order)} 种")
            self._applying_recognized_order = True
            try:
                for index, (combo, spin) in enumerate((
                    (self.sku_combo, self.quantity_spin),
                    (self.sku_combo_2, self.quantity_spin_2),
                )):
                    item = order[index] if index < len(order) else None
                    combo_index = (
                        combo.findData(item.sku) if item is not None else 0
                    )
                    if item is not None and combo_index < 0:
                        raise ValueError(f"场景中没有可执行 SKU：{item.sku}")
                    combo.setCurrentIndex(combo_index)
                    if item is not None:
                        spin.setValue(item.quantity)
            finally:
                self._applying_recognized_order = False
            self._recognized_order_loaded = True
            summary = "，".join(
                f"{item.sku} × {item.quantity}" for item in order
            )
            self.phase1_status_label.setText(f"状态：购物票已识别：{summary}")
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#5fd38d;"
            )
            self.window._append_log(f"[购物票] 已接收订单：{summary}")
            return True, ""
        except ValueError as exc:
            self.phase1_status_label.setText(f"状态：购物票结果无效（{exc}）")
            self.phase1_status_label.setStyleSheet(
                "font-weight:bold; color:#ff6b6b;"
            )
            self.window._append_log(f"[购物票] 结果被拒绝：{exc}")
            return False, f"购物票结果无效：{exc}"

    def _update_controls_enabled(self) -> None:
        """Enable/disable controls based on workflow state."""
        self._poll_scene_future()
        self._maybe_start_scene_request()
        self._update_start_buffer_move()
        workflow_idle = self._phase1_stage in ("idle", "complete", "failed")
        navigation_running = self.window._process_running("navigation")
        localization_ok = self.window.telemetry.localization_ok
        scene_ready = self._scene_snapshot is not None and not self._scene_reload_needed
        ros = self.window.ros_node
        backend_ready = bool(
            ros
            and ros.right_gripper_action_client.server_is_ready()
            and ros.detect_object_action_client.server_is_ready()
            and ros.plan_pick_action_client.server_is_ready()
            and ros.execute_pick_action_client.server_is_ready()
            and ros.plan_pregrasp_action_client.server_is_ready()
            and ros.cartesian_approach_action_client.server_is_ready()
            and ros.shelf_traverse_action_client.server_is_ready()
        )
        selected_skus = tuple(
            str(value)
            for value in (self.sku_combo.currentData(), self.sku_combo_2.currentData())
            if value
        )
        recognizer_ready = bool(
            ros
            and getattr(ros, "read_order_action_client", None)
            and ros.read_order_action_client.server_is_ready()
        )
        selection_safe = (
            bool(selected_skus)
            and len(selected_skus) == len(set(selected_skus))
        ) or (not selected_skus and recognizer_ready)

        self.sku_combo.setEnabled(workflow_idle and scene_ready)
        self.sku_combo_2.setEnabled(workflow_idle and scene_ready)
        self.quantity_spin.setEnabled(workflow_idle and scene_ready)
        self.quantity_spin_2.setEnabled(workflow_idle and scene_ready)
        self.refresh_scene_button.setEnabled(
            workflow_idle
            and self.scene_client is not None
            and self._scene_future is None
        )
        self.capture_anchor_button.setEnabled(
            workflow_idle and scene_ready and bool(self._anchors)
        )
        self.start_phase1_button.setEnabled(
            workflow_idle
            and navigation_running
            and localization_ok
            and scene_ready
            and backend_ready
            and selection_safe
        )
        self.start_buffer_button.setEnabled(
            workflow_idle
            and navigation_running
            and localization_ok
            and self.window.ros_node is not None
        )
        self.cancel_phase1_button.setEnabled(
            self._phase1_stage in (
                *START_CLEARANCE_STAGES,
                "navigating_ticket",
                "recognizing_ticket",
                "ticket_retreating",
                "executing",
                "returning_physical_start",
            )
        )
