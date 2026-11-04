#!/usr/bin/env python3
"""Competition-focused GUI for supermarket chassis benchmarks."""

import math
import os
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QSettings, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QStyle,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .benchmark import (
    BenchmarkProfile,
    BenchmarkWaypoint,
    evaluate_goal,
    find_waypoint_by_role,
    load_benchmark_profile,
    waypoint_landmarks,
)
from .competition_flow_tab import CompetitionFlowTab
from .competition_mission import (
    ROLE_LABELS,
    STATION_ROLES,
    load_mission,
    mission_path,
    save_mission,
    validate_station_id,
)
from .main_window import MainWindow, NavigationRosNode, RosExecutorThread
from .goal_safety import (
    GoalSafetyGeometry,
    GoalSafetyMap,
    costmap_pose_problem,
)
from .map_editor_core import parse_map_yaml, read_pgm
from .model import (
    discover_map_names,
    nearest_path_error,
    normalize_angle,
    resolve_map_paths,
)
from .waypoint_picker import WaypointPickerDialog
from .scene_registration import SceneRegistrationPage
from .web_scene import create_scene_management_page
from .grasp_debug_tab import GraspDebugTab
from .joint_control_page import (
    HEAD_SPECS,
    LEFT_ARM_SPECS,
    LIFT_SPEC,
    RIGHT_ARM_SPECS,
    JointControlPage,
)


APP_TITLE = "OpenFlex Supermarket Competition Navigation Benchmark"
SAFE_TRANSPORT_SHELF_LEVEL = 2
TEST9_TAUGHT_POSTURE_MAP = "supermarket_test_09"
TEST9_DELIVERY_STATION = "jiaofu2"
TEST9_MOVE_POSTURE_LEVEL = 0
TEST9_PLACE_TRANSITION_POSTURE_LEVEL = 10
TEST9_PLACE_POSTURE_LEVEL = 11
TEST9_GRASP_CLEARANCE_M = 0.05
TEST9_POST_PICK_RETREAT_DISTANCE_M = 0.30
# Reuse the old, field-proven near-range band. The deployed base cannot command
# a useful forward speed below 0.03 m/s, so a tighter band can re-trigger a
# minimum-speed approach on lidar-filter noise while the chassis is settling.
TEST9_GRASP_CLEARANCE_TOLERANCE_M = 0.010
RIGHT_GRIPPER_RELEASE_POSITION_M = 0.044
RIGHT_GRIPPER_JOINT_NAME = "openarmx_right_finger_joint1"
RIGHT_GRIPPER_VALID_MIN_M = -0.002
RIGHT_GRIPPER_VALID_MAX_M = 0.050
RIGHT_GRIPPER_HOLD_TOLERANCE_M = 0.008
# Require measurable closure beyond controller/feedback tolerance before a
# manual Test-9 confirmation can be treated as a physical grasp.
RIGHT_GRIPPER_MIN_GRASP_CLOSURE_M = 0.003
RIGHT_GRIPPER_GRASP_MIN_POSITION_M = 0.002
RIGHT_GRIPPER_GRASP_MAX_POSITION_M = 0.041
GLOBAL_COSTMAP_MAX_AGE_SEC = 1.0
NAV2_REQUIRED_NODES = {
    "/behavior_server": "Nav2 behavior_server",
    "/bt_navigator": "Nav2 bt_navigator",
    "/controller_server": "Nav2 controller_server",
    "/planner_server": "Nav2 planner_server",
    "/smoother_server": "Nav2 smoother_server",
    "/velocity_smoother": "Nav2 velocity_smoother",
}
NAV2_REQUIRED_ACTION_SERVERS = {
    "/compute_path_to_pose": "全局规划",
    "/follow_path": "路径跟踪",
    "/navigate_to_pose": "NavigateToPose",
    "/smooth_path": "路径平滑",
    "/wait": "安全等待恢复",
}
NAVIGATION_RUNTIME_NODES = {
    "/bt_navigator",
    "/collision_monitor",
    "/controller_server",
    "/fastlio2/fast_livo_core",
    "/fastlio2/lio_bridge",
    "/icp_registration",
    "/livox_lidar_publisher",
    "/map_server",
    "/planner_server",
    "/smoother_server",
    "/velocity_smoother",
    "/waypoint_follower",
}


def nav2_stack_readiness_problems(
    node_names: set[str], action_servers: set[str]
) -> list[str]:
    problems = []
    for node_name, label in NAV2_REQUIRED_NODES.items():
        if node_name not in node_names:
            problems.append(f"缺少 {label} 节点 {node_name}")
    for action_name, label in NAV2_REQUIRED_ACTION_SERVERS.items():
        if action_name not in action_servers:
            problems.append(f"{label} Action 未就绪：{action_name}")
    return problems


def _find_workspace_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "install" / "setup.bash").is_file() and (parent / "src").is_dir():
            return parent
    # Installed copies live under install/, so the walk above misses src/.
    # The launch script exports this; keep the CWD fallback for bare `ros2 run`.
    override = os.environ.get("OPENFLEX_WORKSPACE_ROOT", "").strip()
    if override:
        return Path(override).expanduser()
    return Path.cwd()


WORKSPACE_ROOT = _find_workspace_root()
CAN_ENABLE_HELPER = (
    WORKSPACE_ROOT
    / "src/openflex_integrated/openflex_gui/openflex_gui/enable_can_helper.sh"
)
CAN_DISABLE_HELPER = (
    WORKSPACE_ROOT
    / "src/openflex_integrated/openflex_gui/openflex_gui/disable_can_helper.sh"
)
MOTOR_CHECK_SCRIPT = (
    WORKSPACE_ROOT
    / "src/openflex_integrated/openflex_manager/scripts/check_motor_status.py"
)


def default_profile_path() -> Path:
    override = os.environ.get("OPENFLEX_COMPETITION_PROFILE", "").strip()
    if override:
        return Path(override).expanduser()
    return (
        Path(get_package_share_directory("swerve_navigation"))
        / "config"
        / "supermarket_test_05_competition.yaml"
    )


class CompetitionMainWindow(MainWindow):
    """Specialized competition console built on the proven telemetry GUI."""

    def __init__(
        self,
        ros_node: Optional[NavigationRosNode],
        profile: BenchmarkProfile,
        profile_path: Path,
    ):
        self.profile = profile
        self.profile_path = profile_path
        self.fullbody_mode = (
            os.environ.get("OPENFLEX_FULLBODY_COMPETITION", "").strip().lower()
            in ("1", "true", "yes")
        )
        self.competition_launch_package = os.environ.get(
            "OPENFLEX_COMPETITION_LAUNCH_PACKAGE", "swerve_navigation"
        ).strip()
        self.competition_launch_file = os.environ.get(
            "OPENFLEX_COMPETITION_LAUNCH_FILE", profile.launch_file
        ).strip()
        self.runtime_launch_package = os.environ.get(
            "OPENFLEX_COMPETITION_RUNTIME_LAUNCH_PACKAGE", "openarmx_task_executor"
        ).strip()
        self.runtime_launch_file = os.environ.get(
            "OPENFLEX_COMPETITION_RUNTIME_LAUNCH_FILE",
            "supermarket_fullbody_runtime.launch.py",
        ).strip()
        self.navigation_launch_package = os.environ.get(
            "OPENFLEX_COMPETITION_NAV_LAUNCH_PACKAGE", "swerve_navigation"
        ).strip()
        self.navigation_launch_file = os.environ.get(
            "OPENFLEX_COMPETITION_NAV_LAUNCH_FILE", profile.launch_file
        ).strip()
        self._mission_waypoints: dict[str, BenchmarkWaypoint] = dict(profile.waypoints)
        self._start_waypoint = find_waypoint_by_role(profile, "start_position")
        self._editing_station_id: Optional[str] = None
        self._loading_mission = False
        self._active_settle_sec = 1.0
        self._waypoint_rows: dict[str, int] = {}
        self._tour_queue: list[BenchmarkWaypoint] = []
        self._tour_total = 0
        self._tour_completed = 0
        self._tour_active = False
        self._tour_waiting = False
        self._active_target: Optional[BenchmarkWaypoint] = None
        self._pick_navigation_test_stage = "idle"
        self._pick_navigation_test_source_id = ""
        self._pick_navigation_test_source_override: Optional[BenchmarkWaypoint] = None
        self._pick_navigation_test_anchor_return_pose: Optional[
            tuple[float, float, float]
        ] = None
        self._pick_navigation_test_item_target_pose: Optional[
            tuple[float, float, float]
        ] = None
        self._pick_navigation_test_at_grasp_clearance = False
        self._pick_navigation_test_level = SAFE_TRANSPORT_SHELF_LEVEL
        self._pick_navigation_test_transport_level = SAFE_TRANSPORT_SHELF_LEVEL
        self._pick_navigation_test_pre_navigation_level: Optional[int] = None
        self._pick_navigation_test_handoff_deadline = 0.0
        self._pick_navigation_test_navigation_ready_deadline = 0.0
        self._pick_navigation_test_retreat_distance = 0.15
        self._pick_navigation_test_retreat_speed = 0.04
        self._pick_navigation_test_destination: Optional[BenchmarkWaypoint] = None
        self._pick_navigation_test_return_destination: Optional[BenchmarkWaypoint] = None
        self._pick_navigation_test_grasp_position: Optional[float] = None
        self._last_waypoint_outcome: Optional[tuple[str, bool, str]] = None
        self._active_target_row: Optional[int] = None
        self._active_is_map_goal = False
        self._last_map_goal_pose: Optional[tuple[float, float, float]] = None
        self._active_started_at = 0.0
        self._stop_stable_since: Optional[float] = None
        self._goal_terminal_handled = True
        self._active_motion_phase = "idle"
        self._fine_align_preview_active = False
        self._fine_align_preview_station_id = ""
        self._active_max_cte = 0.0
        self._active_max_icp_step = 0.0
        self._active_max_icp_yaw = 0.0
        self._active_safety_interventions = 0
        self._safety_intervention_active = False
        self._pass_count = 0
        self._fail_count = 0
        # base_link is at the centre of the configured transport footprint.
        # Padding belongs to Nav2 planning, while this GUI policy requires a
        # physical-body-to-obstacle clearance configured by the profile.
        self._goal_safety_geometry = GoalSafetyGeometry(
            half_length_m=profile.transport_length_m * 0.5,
            half_width_m=profile.transport_width_m * 0.5,
            clearance_m=profile.goal_obstacle_clearance_m,
        )
        super().__init__(ros_node)

        self.settings = QSettings("OpenArmX", "OpenFlexCompetitionNavigationGUI")
        self.setWindowTitle(
            "OpenFlex 商超比赛整机控制台"
            if self.fullbody_mode
            else "OpenFlex 商超比赛导航基准台"
        )
        self.setMinimumSize(1280, 800)
        self.resize(1540, 940)
        for label in self.findChildren(QLabel):
            if label.text() == "OpenFlex 建图定位导航工作台":
                label.setText(
                    "OpenFlex 商超比赛整机控制台"
                    if self.fullbody_mode
                    else "OpenFlex 商超比赛导航基准台"
                )
                break

        # Keep mapping first, replace generic navigation with the competition
        # station workflow, and retain the system preflight as the third page.
        self.control_tabs.removeTab(1)
        self.control_tabs.insertTab(1, self._build_benchmark_tab(), "比赛导航")
        self.benchmark_tab_index = 1
        # The station policy editor contains several parameter rows and a
        # wider results table. Give the competition tab enough room so those
        # controls do not get clipped against the map splitter pane.
        self.control_tabs.setMinimumWidth(570)
        self.control_tabs.setMaximumWidth(720)
        self.control_tabs.setTabText(2, "系统检查")
        if self.fullbody_mode:
            self.control_tabs.insertTab(1, self._build_fullbody_control_tab(), "整机控制")
            self.competition_flow_tab = CompetitionFlowTab(self)
            self.control_tabs.insertTab(
                2, self._scroll_page(self.competition_flow_tab), "比赛流程"
            )
            self.benchmark_tab_index = 3
            self.control_tabs.setTabToolTip(0, "可复用已启动的全身硬件；开始导航前必须先停止建图")
            self.nav_rviz.setText("启动导航 RViz（比赛模式必启）")
            self.nav_rviz.setChecked(True)
            self.nav_rviz.setEnabled(False)
            self.control_tabs.setCurrentIndex(1)
        else:
            self.control_tabs.setCurrentIndex(0)
        self._load_competition_settings()
        self.map_name_combo.currentTextChanged.connect(self._map_selection_changed)
        self.map_root_edit.currentTextChanged.connect(self._map_selection_changed)
        self._load_mission_for_current_map()
        self._append_log(f"比赛基准配置: {self.profile_path}")
        self._build_workspace_tabs()

    def _build_workspace_tabs(self) -> None:
        robot_workspace = self.takeCentralWidget()
        self.workspace_tabs = QTabWidget()
        self.workspace_tabs.addTab(robot_workspace, "机器人控制")
        if self.fullbody_mode:
            self.joint_control_page = JointControlPage(
                self.ros_node,
                self.telemetry,
                self._joint_motion_allowed,
                self,
            )
            self.workspace_tabs.addTab(self.joint_control_page, "关节控制")
        self.scene_registration_page = SceneRegistrationPage(
            self.ros_node,
            self,
            map_root=self.map_root_edit.currentText(),
            map_name=self.map_name_combo.currentText(),
        )
        self.workspace_tabs.addTab(self.scene_registration_page, "2D 场景配准")
        self.scene_management_page = create_scene_management_page(self.ros_node, self)
        self.workspace_tabs.addTab(self.scene_management_page, "场景管理")
        self.grasp_debug_page = GraspDebugTab(self.ros_node, self)
        self.workspace_tabs.addTab(self.grasp_debug_page, "抓取调试")
        self.setCentralWidget(self.workspace_tabs)
        self.map_root_edit.currentTextChanged.connect(
            self._sync_scene_registration_map
        )
        self.map_name_combo.currentTextChanged.connect(
            self._sync_scene_registration_map
        )

    def _sync_scene_registration_map(self, *_args) -> None:
        self.scene_registration_page.set_map_selection(
            self.map_root_edit.currentText(),
            self.map_name_combo.currentText(),
            silent=True,
        )

    def _load_settings(self) -> None:
        """Called by MainWindow while all generic widgets already exist."""
        self.map_root_edit.setCurrentText(str(self.profile.map_root))
        self.map_name_combo.setEditText(self.profile.map_name)
        self.initial_x.setValue(self.profile.initial_x)
        self.initial_y.setValue(self.profile.initial_y)
        self.initial_yaw.setValue(self.profile.initial_yaw)
        self.nav_speed.setValue(0.60)
        self.nav_accel.setValue(0.50)
        self.auto_record.setChecked(True)

    def _load_competition_settings(self) -> None:
        selection_key = f"profiles/{self.profile.profile_id}"
        saved_root = str(
            self.settings.value(f"{selection_key}/map_root", self.profile.map_root)
        )
        saved_name = str(
            self.settings.value(f"{selection_key}/map_name", self.profile.map_name)
        )
        available_names = discover_map_names(saved_root)
        if saved_name not in available_names:
            saved_name = (
                self.profile.map_name
                if self.profile.map_name in available_names
                else (available_names[0] if available_names else self.profile.map_name)
            )
        self.map_root_edit.setCurrentText(saved_root)
        self.map_name_combo.setEditText(saved_name)
        self._refresh_competition_map_selector()

        repeats = int(self.settings.value("tour_repeats", 1))
        self.tour_repeats.setValue(max(1, min(20, repeats)))
        auto_record = str(self.settings.value("auto_record", "true")).lower() == "true"
        self.auto_record.setChecked(auto_record)
        if self.fullbody_mode:
            vr_base = str(self.settings.value("vr_base_control", "false")).lower() == "true"
            self.vr_base_control.setChecked(vr_base)
            mode = str(self.settings.value("competition_control_mode", "autonomous"))
            self.control_mode.setCurrentIndex(1 if mode == "vr" else 0)
            moveit_rviz = str(self.settings.value("moveit_rviz", "false")).lower() == "true"
            self.moveit_rviz.setChecked(moveit_rviz)

    def _save_settings(self) -> None:
        MainWindow._save_settings(self)
        selection_key = f"profiles/{self.profile.profile_id}"
        self.settings.setValue(
            f"{selection_key}/map_root", self.map_root_edit.currentText()
        )
        self.settings.setValue(
            f"{selection_key}/map_name", self.map_name_combo.currentText()
        )
        self.settings.setValue("tour_repeats", self.tour_repeats.value())
        self.settings.setValue("auto_record", self.auto_record.isChecked())
        if self.fullbody_mode:
            self.settings.setValue("vr_base_control", self.vr_base_control.isChecked())
            self.settings.setValue(
                "competition_control_mode",
                "vr" if self.control_mode.currentIndex() == 1 else "autonomous",
            )
            self.settings.setValue("moveit_rviz", self.moveit_rviz.isChecked())

    def _map_paths(self):
        return resolve_map_paths(
            self.map_root_edit.currentText(), self.map_name_combo.currentText()
        )

    def _build_fullbody_control_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        hardware_group = QGroupBox("1. 硬件准备")
        hardware_layout = QGridLayout(hardware_group)
        self.can_status = QLabel("CAN：等待检查")
        self.motor_status = QLabel("电机：尚未检查")
        enable_can = QPushButton("启用全部 CAN")
        enable_can.clicked.connect(self._enable_all_can)
        disable_can = QPushButton("禁用全部 CAN")
        disable_can.setObjectName("dangerButton")
        disable_can.clicked.connect(self._disable_all_can)
        check_motors = QPushButton("检查全部电机")
        check_motors.clicked.connect(self._check_all_motors)
        self.disable_can_button = disable_can
        hardware_layout.addWidget(self.can_status, 0, 0, 1, 2)
        hardware_layout.addWidget(self.motor_status, 1, 0, 1, 2)
        hardware_layout.addWidget(enable_can, 2, 0)
        hardware_layout.addWidget(disable_can, 2, 1)
        hardware_layout.addWidget(check_motors, 3, 0, 1, 2)
        layout.addWidget(hardware_group)

        system_group = QGroupBox("2. 全身控制与比赛基础系统")
        system_layout = QGridLayout(system_group)
        self.fullbody_status = QLabel("整机：未启动")
        self.control_mode = QComboBox()
        self.control_mode.addItems(("全自主比赛", "VR 调试 / 遥控"))
        self.control_mode.currentIndexChanged.connect(self._control_mode_changed)
        self.moveit_rviz = QCheckBox("启动 MoveIt RViz")
        start_system = QPushButton("启动全身控制")
        start_system.setObjectName("primaryButton")
        start_system.clicked.connect(self._start_fullbody)
        stop_system = QPushButton("停止全身控制")
        stop_system.setObjectName("dangerButton")
        stop_system.clicked.connect(self._stop_fullbody)
        self.fullbody_start_button = start_system
        self.fullbody_stop_button = stop_system
        system_layout.addWidget(self.fullbody_status, 0, 0, 1, 2)
        system_layout.addWidget(QLabel("比赛控制方式"), 1, 0)
        system_layout.addWidget(self.control_mode, 1, 1)
        system_layout.addWidget(self.moveit_rviz, 2, 0, 1, 2)
        system_layout.addWidget(start_system, 3, 0)
        system_layout.addWidget(stop_system, 3, 1)
        layout.addWidget(system_group)

        state_group = QGroupBox("3. 机器人实时状态")
        state_layout = QGridLayout(state_group)
        self.joint_state_status = QLabel("关节状态：等待 /joint_states")
        self.position_controller_status = QLabel("位置控制器：等待")
        self.left_arm_state = QLabel("左臂：--")
        self.right_arm_state = QLabel("右臂：--")
        self.lift_state = QLabel("升降柱：--")
        self.head_state = QLabel("头部：--")
        self.base_state = QLabel("底盘：--")
        for label in (
            self.joint_state_status,
            self.position_controller_status,
            self.left_arm_state,
            self.right_arm_state,
            self.lift_state,
            self.head_state,
            self.base_state,
        ):
            label.setWordWrap(True)
        state_layout.addWidget(self.joint_state_status, 0, 0, 1, 2)
        state_layout.addWidget(self.position_controller_status, 1, 0, 1, 2)
        state_layout.addWidget(self.left_arm_state, 2, 0)
        state_layout.addWidget(self.right_arm_state, 2, 1)
        state_layout.addWidget(self.lift_state, 3, 0)
        state_layout.addWidget(self.head_state, 3, 1)
        state_layout.addWidget(self.base_state, 4, 0, 1, 2)
        layout.addWidget(state_group)

        vr_group = QGroupBox("4. VR 遥操作")
        vr_layout = QGridLayout(vr_group)
        self.vr_status = QLabel("VR：未启动")
        self.vr_base_control = QCheckBox("允许 VR 控制底盘")
        self.vr_base_control.setToolTip(
            "自主比赛保持关闭；开启后 VR 摇杆速度仍经过碰撞监控和定位安全门"
        )
        start_vr = QPushButton("启动 VR 遥操作")
        start_vr.clicked.connect(self._start_vr)
        stop_vr = QPushButton("停止 VR")
        stop_vr.clicked.connect(self._stop_vr)
        self.vr_start_button = start_vr
        self.vr_stop_button = stop_vr
        vr_layout.addWidget(self.vr_status, 0, 0, 1, 2)
        vr_layout.addWidget(self.vr_base_control, 1, 0, 1, 2)
        vr_layout.addWidget(start_vr, 2, 0)
        vr_layout.addWidget(stop_vr, 2, 1)
        layout.addWidget(vr_group)

        layers_group = QGroupBox("5. 分层接口状态")
        layers_layout = QGridLayout(layers_group)
        layers_layout.addWidget(QLabel("层"), 0, 0)
        layers_layout.addWidget(QLabel("运行接口"), 0, 1)
        layers_layout.addWidget(QLabel("状态"), 0, 2)
        layer_rows = (
            ("hardware", "全身硬件", "/controller_manager"),
            ("moveit", "B / MoveIt", "/move_group"),
            ("navigation", "底盘导航", "/navigate_to_pose"),
            ("detection", "C / 局部检测", "/detect_object"),
            ("pick", "A+B / 抓取", "/pick_object"),
            ("shelf_view", "货架观察姿态", "/competition/move_to_shelf_view"),
            ("mock_pick", "联调模拟抓取", "/competition/mock_pick_object"),
            ("shelf_traverse", "货架横移", "/return_to_shelf_anchor"),
            ("retreat", "底盘安全退避", "/retreat_from_station"),
            ("place", "B / 放置", "/place_object"),
            ("task", "任务编排", "/execute_transport"),
            ("vr", "VR 遥操作", "/openarmx_teleop_vr_node"),
        )
        self.layer_status_labels: dict[str, QLabel] = {}
        for row, (key, owner, endpoint) in enumerate(layer_rows, start=1):
            status = QLabel("等待")
            self.layer_status_labels[key] = status
            layers_layout.addWidget(QLabel(owner), row, 0)
            layers_layout.addWidget(QLabel(endpoint), row, 1)
            layers_layout.addWidget(status, row, 2)
        layout.addWidget(layers_group)

        layout.addStretch(1)
        scroll = self._scroll_page(tab)
        # Keep a fallback for narrow screens; the normal competition console
        # is widened above, but horizontal clipping is worse than a scrollbar.
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        return scroll

    def _build_pick_chain_group(self) -> QGroupBox:
        """Manual pick-chain harness, shown on the competition flow page."""
        pick_group = QGroupBox("手动联调")
        pick_layout = QGridLayout(pick_group)
        workflow = QLabel(
            "导航精停 → 货架观察姿态 → 模拟抓取 → 运输姿态 → 退避导航"
        )
        workflow.setWordWrap(True)
        workflow.setStyleSheet("color:#a0a0a0;")
        warning = QLabel(
            "货架观察和运输姿态使用已示教的关节轨迹，经 MoveIt 命令桥接端点驱动真实升降台与右臂；"
            "真正抓取的预规划、直线进给和回撤由 MoveIt 规划器完成。"
            "模拟抓取仅假定物体附着，不会启动真实检测、GraspNet 或夹爪。"
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#f1c75b;")
        self.pick_source_combo = QComboBox()
        self.pick_destination_combo = QComboBox()
        for station_combo in (self.pick_source_combo, self.pick_destination_combo):
            station_combo.setMinimumContentsLength(6)
            station_combo.setSizeAdjustPolicy(
                QComboBox.AdjustToMinimumContentsLengthWithIcon
            )
            station_combo.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.pick_shelf_level = QComboBox()
        self.pick_shelf_level.addItem("第二层", 2)
        self.pick_shelf_level.addItem("第三层", 3)
        self.pick_shelf_level.addItem("第四层", 4)
        self.pick_object_id = QLineEdit("mock_item")
        self.pick_arm_combo = QComboBox()
        self.pick_arm_combo.addItems(("right", "left"))
        self.pick_retreat_distance = QDoubleSpinBox()
        self.pick_retreat_distance.setRange(0.05, 0.30)
        self.pick_retreat_distance.setDecimals(2)
        self.pick_retreat_distance.setSingleStep(0.01)
        self.pick_retreat_distance.setValue(0.15)
        self.pick_retreat_distance.setSuffix(" m")
        self.pick_retreat_speed = QDoubleSpinBox()
        self.pick_retreat_speed.setRange(0.01, 0.06)
        self.pick_retreat_speed.setDecimals(2)
        self.pick_retreat_speed.setSingleStep(0.01)
        self.pick_retreat_speed.setValue(0.04)
        self.pick_retreat_speed.setSuffix(" m/s")
        self.pick_chain_status = QLabel("联调状态：等待")
        self.pregrasp_result_status = QLabel("观察姿态结果：尚未执行")
        self.pick_result_status = QLabel("抓取结果：尚未执行")
        self.retreat_result_status = QLabel("退避结果：尚未执行")
        for status_label in (
            self.pick_chain_status,
            self.pregrasp_result_status,
            self.pick_result_status,
            self.retreat_result_status,
        ):
            status_label.setWordWrap(True)
            status_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        prepare_pick = QPushButton("导航、精停并进入观察姿态")
        prepare_pick.setObjectName("primaryButton")
        prepare_pick.clicked.connect(self._start_pick_preparation)
        self.pick_prepare_button = prepare_pick
        run_pick_chain = QPushButton("模拟抓取成功并执行后续链路")
        run_pick_chain.setObjectName("primaryButton")
        run_pick_chain.clicked.connect(self._start_mock_pick_navigation_test)
        self.pick_chain_button = run_pick_chain
        pick_layout.addWidget(workflow, 0, 0, 1, 4)
        pick_layout.addWidget(warning, 1, 0, 1, 4)
        pick_layout.addWidget(QLabel("当前货架站"), 2, 0)
        pick_layout.addWidget(self.pick_source_combo, 2, 1)
        pick_layout.addWidget(QLabel("下一导航站"), 2, 2)
        pick_layout.addWidget(self.pick_destination_combo, 2, 3)
        pick_layout.addWidget(QLabel("货架层"), 3, 0)
        pick_layout.addWidget(self.pick_shelf_level, 3, 1)
        pick_layout.addWidget(QLabel("手臂标签"), 3, 2)
        pick_layout.addWidget(self.pick_arm_combo, 3, 3)
        pick_layout.addWidget(QLabel("模拟物品 ID"), 4, 0)
        pick_layout.addWidget(self.pick_object_id, 4, 1)
        pick_layout.addWidget(QLabel("后退距离"), 4, 2)
        pick_layout.addWidget(self.pick_retreat_distance, 4, 3)
        pick_layout.addWidget(QLabel("后退速度"), 5, 0)
        pick_layout.addWidget(self.pick_retreat_speed, 5, 1)
        pick_layout.addWidget(self.pregrasp_result_status, 6, 0, 1, 2)
        pick_layout.addWidget(self.pick_result_status, 6, 2, 1, 2)
        pick_layout.addWidget(self.retreat_result_status, 7, 0, 1, 2)
        pick_layout.addWidget(self.pick_chain_status, 7, 2, 1, 2)
        # These labels are the widest controls in the narrow left panel.  A
        # vertical command row keeps both labels intact at the minimum window
        # width and leaves the map in its own splitter pane.
        pick_layout.addWidget(prepare_pick, 8, 0, 1, 4)
        pick_layout.addWidget(run_pick_chain, 9, 0, 1, 4)
        return pick_group

    def _build_benchmark_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        profile_group = QGroupBox("当前比赛场景")
        profile_layout = QGridLayout(profile_group)
        profile_layout.setColumnStretch(1, 1)
        profile_layout.addWidget(QLabel("地图"), 0, 0)
        self.competition_map_combo = QComboBox()
        self.competition_map_combo.currentTextChanged.connect(
            self._competition_map_selected
        )
        refresh_maps = QPushButton()
        refresh_maps.setFixedWidth(44)
        refresh_maps.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserReload)
        )
        refresh_maps.setToolTip("刷新地图列表")
        refresh_maps.clicked.connect(self._refresh_maps)
        self.competition_map_refresh_button = refresh_maps
        profile_layout.addWidget(self.competition_map_combo, 0, 1)
        profile_layout.addWidget(refresh_maps, 0, 2)
        profile_layout.addWidget(QLabel("配置"), 1, 0)
        self.mission_path_label = QLabel("站点尚未保存")
        self.mission_path_label.setWordWrap(True)
        profile_layout.addWidget(self.mission_path_label, 1, 1)
        profile_layout.addWidget(QLabel("目标"), 2, 0)
        profile_layout.addWidget(QLabel(self.profile.purpose), 2, 1)
        layout.addWidget(profile_group)

        launch_group = QGroupBox("定位导航运行参数" if self.fullbody_mode else "比赛导航系统")
        launch_layout = QGridLayout(launch_group)
        launch_layout.addWidget(QLabel("轮速上限"), 0, 0)
        launch_layout.addWidget(self.nav_speed, 0, 1)
        launch_layout.addWidget(QLabel("加速度"), 1, 0)
        launch_layout.addWidget(self.nav_accel, 1, 1)
        launch_layout.addWidget(self.nav_rviz, 2, 0)
        launch_layout.addWidget(self.auto_record, 2, 1)
        if not self.fullbody_mode:
            start_navigation = QPushButton("启动比赛基准导航")
            start_navigation.setObjectName("primaryButton")
            start_navigation.clicked.connect(self._start_navigation)
            stop_navigation = QPushButton("停止导航系统")
            stop_navigation.clicked.connect(self._stop_navigation)
            launch_layout.addWidget(start_navigation, 3, 0)
            launch_layout.addWidget(stop_navigation, 3, 1)
        else:
            start_navigation = QPushButton("启动定位导航")
            start_navigation.setObjectName("primaryButton")
            start_navigation.clicked.connect(self._start_navigation)
            stop_navigation = QPushButton("停止定位导航")
            stop_navigation.setObjectName("dangerButton")
            stop_navigation.clicked.connect(self._stop_navigation)
            self.navigation_start_button = start_navigation
            self.navigation_stop_button = stop_navigation
            launch_layout.addWidget(start_navigation, 3, 0)
            launch_layout.addWidget(stop_navigation, 3, 1)
        layout.addWidget(launch_group)

        self.relocalization_group = QGroupBox("定位恢复")
        relocalization_layout = QGridLayout(self.relocalization_group)
        relocalization_layout.addWidget(QLabel("X"), 0, 0)
        relocalization_layout.addWidget(self.initial_x, 0, 1)
        relocalization_layout.addWidget(QLabel("Y"), 1, 0)
        relocalization_layout.addWidget(self.initial_y, 1, 1)
        relocalization_layout.addWidget(QLabel("Yaw"), 2, 0)
        relocalization_layout.addWidget(self.initial_yaw, 2, 1)
        pick_initial_pose = QPushButton("地图选择初始位姿")
        pick_initial_pose.clicked.connect(self._pick_and_publish_initial_pose)
        self.publish_initial_button.setText("发送输入的初始位姿")
        relocalization_layout.addWidget(pick_initial_pose, 3, 0)
        relocalization_layout.addWidget(self.publish_initial_button, 3, 1)
        save_field_start = QPushButton("保存为任务返回起点")
        save_field_start.setToolTip(
            "可选：将当前 X/Y/Yaw 保存为本地图唯一 start_position，"
            "仅用于返回起点和下次启动的默认重定位参考"
        )
        save_field_start.clicked.connect(self._save_task_start_pose)
        self.save_field_start_button = save_field_start
        relocalization_layout.addWidget(save_field_start, 4, 0, 1, 2)
        layout.addWidget(self.relocalization_group)

        acceptance = self.profile.acceptance
        acceptance_group = QGroupBox("比赛精度门槛")
        acceptance_layout = QGridLayout(acceptance_group)
        acceptance_layout.addWidget(
            QLabel(
                f"终点 ≤ {acceptance.goal_position_m * 100:.1f} cm / "
                f"{math.degrees(acceptance.goal_yaw_rad):.1f}°"
            ),
            0,
            0,
        )
        acceptance_layout.addWidget(
            QLabel(f"路径横差 ≤ {acceptance.path_cross_track_m * 100:.1f} cm"),
            0,
            1,
        )
        self.safety_status = QLabel("安全链：等待速度数据")
        self.localization_status = QLabel("定位：等待 /localization_ok")
        self.start_readiness_status = QLabel("任务起点参考：等待定位")
        acceptance_layout.addWidget(self.localization_status, 1, 0)
        acceptance_layout.addWidget(self.safety_status, 1, 1)
        acceptance_layout.addWidget(self.start_readiness_status, 2, 0, 1, 2)
        layout.addWidget(acceptance_group)

        map_goal_group = QGroupBox("地图任意点精度测试")
        map_goal_layout = QHBoxLayout(map_goal_group)
        send_map_goal = QPushButton("地图选点并立即导航")
        send_map_goal.setObjectName("primaryButton")
        send_map_goal.clicked.connect(self._pick_and_send_map_goal)
        cancel_map_goal = QPushButton("取消当前目标")
        cancel_map_goal.clicked.connect(self._stop_benchmark)
        self.map_goal_status = QLabel("尚未发送临时测点")
        self.map_goal_status.setWordWrap(True)
        map_goal_layout.addWidget(send_map_goal)
        map_goal_layout.addWidget(cancel_map_goal)
        map_goal_layout.addWidget(self.map_goal_status, 1)
        layout.addWidget(map_goal_group)

        waypoint_group = QGroupBox("比赛语义站点")
        waypoint_layout = QVBoxLayout(waypoint_group)
        station_form = QFormLayout()
        self.station_id = QLineEdit()
        self.station_id.setPlaceholderText("例如 START、DELIVERY、SHELF_A")
        self.station_label = QLineEdit()
        self.station_role = QComboBox()
        for label, role in STATION_ROLES:
            self.station_role.addItem(label, role)
        self.station_role.currentIndexChanged.connect(
            self._station_role_policy_defaults
        )
        self.station_x = self._double_spin(-100.0, 100.0, 0.0, 0.05, " m", 3)
        self.station_y = self._double_spin(-100.0, 100.0, 0.0, 0.05, " m", 3)
        self.station_yaw_deg = QDoubleSpinBox()
        self.station_yaw_deg.setRange(-180.0, 180.0)
        self.station_yaw_deg.setDecimals(1)
        self.station_yaw_deg.setSingleStep(5.0)
        self.station_yaw_deg.setSuffix(" deg")
        station_form.addRow("ID", self.station_id)
        station_form.addRow("名称", self.station_label)
        station_form.addRow("类型", self.station_role)
        coordinate_row = QHBoxLayout()
        coordinate_row.addWidget(self.station_x)
        coordinate_row.addWidget(self.station_y)
        coordinate_row.addWidget(self.station_yaw_deg)
        station_form.addRow("X / Y / Yaw", coordinate_row)

        self.station_fine_align = QCheckBox("到点后执行激光前沿精调")
        self.station_fine_align.toggled.connect(
            self._update_station_policy_controls
        )
        self.station_fine_align_mode = QComboBox()
        self.station_fine_align_mode.addItem(
            "激光前沿（货架/交付台）", "front_surface_scan"
        )
        fine_align_row = QHBoxLayout()
        fine_align_row.addWidget(self.station_fine_align)
        fine_align_row.addWidget(self.station_fine_align_mode)
        station_form.addRow("精调", fine_align_row)

        self.station_target_clearance = self._double_spin(
            0.04, 0.45, 0.05, 0.01, " m", 3
        )
        self.station_clearance_tolerance = self._double_spin(
            0.005, 0.05, 0.01, 0.005, " m", 3
        )
        self.station_fine_yaw_tolerance_deg = QDoubleSpinBox()
        self.station_fine_yaw_tolerance_deg.setRange(0.5, 5.0)
        self.station_fine_yaw_tolerance_deg.setDecimals(1)
        self.station_fine_yaw_tolerance_deg.setSingleStep(0.5)
        self.station_fine_yaw_tolerance_deg.setValue(1.0)
        self.station_fine_yaw_tolerance_deg.setSuffix(" deg")
        self.station_fine_timeout = QDoubleSpinBox()
        self.station_fine_timeout.setRange(5.0, 60.0)
        self.station_fine_timeout.setDecimals(1)
        self.station_fine_timeout.setSingleStep(5.0)
        self.station_fine_timeout.setValue(20.0)
        self.station_fine_timeout.setSuffix(" s")
        fine_parameters = QHBoxLayout()
        fine_parameters.addWidget(QLabel("净距"))
        fine_parameters.addWidget(self.station_target_clearance)
        fine_parameters.addWidget(QLabel("距离容差"))
        fine_parameters.addWidget(self.station_clearance_tolerance)
        fine_parameters.addWidget(QLabel("角容差"))
        fine_parameters.addWidget(self.station_fine_yaw_tolerance_deg)
        fine_parameters.addWidget(QLabel("超时"))
        fine_parameters.addWidget(self.station_fine_timeout)
        station_form.addRow("精调参数", fine_parameters)

        self.station_retreat_after_pick = QCheckBox("取货后")
        self.station_retreat_after_place = QCheckBox("放置后")
        self.station_retreat_distance = self._double_spin(
            0.05, 0.30, 0.15, 0.01, " m", 2
        )
        self.station_retreat_speed = self._double_spin(
            0.01, 0.06, 0.04, 0.01, " m/s", 2
        )
        retreat_row = QHBoxLayout()
        retreat_row.addWidget(self.station_retreat_after_pick)
        retreat_row.addWidget(self.station_retreat_after_place)
        retreat_row.addWidget(QLabel("距离"))
        retreat_row.addWidget(self.station_retreat_distance)
        retreat_row.addWidget(QLabel("速度"))
        retreat_row.addWidget(self.station_retreat_speed)
        station_form.addRow("操作后退避", retreat_row)
        waypoint_layout.addLayout(station_form)

        station_capture = QHBoxLayout()
        new_station = QPushButton("新建站点")
        new_station.clicked.connect(self._new_station)
        pick_on_map = QPushButton("地图选点")
        pick_on_map.clicked.connect(self._pick_station_on_map)
        use_current_pose = QPushButton("使用当前位姿")
        use_current_pose.clicked.connect(self._capture_current_pose)
        save_station = QPushButton("新增/更新站点")
        save_station.setObjectName("primaryButton")
        save_station.clicked.connect(self._upsert_station)
        preview_alignment = QPushButton("只观测精调回波")
        preview_alignment.setToolTip(
            "读取并滤波左右前沿回波，不发送底盘运动命令"
        )
        preview_alignment.clicked.connect(self._preview_station_alignment)
        self.preview_alignment_button = preview_alignment
        station_capture.addWidget(new_station)
        station_capture.addWidget(pick_on_map)
        station_capture.addWidget(use_current_pose)
        station_capture.addWidget(save_station)
        station_capture.addWidget(preview_alignment)
        waypoint_layout.addLayout(station_capture)
        self.preview_alignment_status = QLabel("精调观测：尚未执行")
        self.preview_alignment_status.setWordWrap(True)
        waypoint_layout.addWidget(self.preview_alignment_status)

        self.waypoint_table = QTableWidget(len(self.profile.waypoints), 12)
        self.waypoint_table.setHorizontalHeaderLabels(
            [
                "ID",
                "类型",
                "名称",
                "X",
                "Y",
                "Yaw",
                "状态",
                "最终误差",
                "最大横差",
                "最大ICP",
                "精调策略",
                "退避策略",
            ]
        )
        self.waypoint_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.waypoint_table.setSelectionMode(QTableWidget.SingleSelection)
        self.waypoint_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.waypoint_table.itemSelectionChanged.connect(self._station_selected)
        self._refresh_waypoint_table()
        self.waypoint_table.resizeColumnsToContents()
        if self.waypoint_table.rowCount() > 0:
            self.waypoint_table.selectRow(0)
        waypoint_layout.addWidget(self.waypoint_table)
        waypoint_buttons = QHBoxLayout()
        return_to_start = QPushButton("返回起点并对正")
        return_to_start.setObjectName("primaryButton")
        return_to_start.setEnabled(self._start_waypoint is not None)
        return_to_start.clicked.connect(self._return_to_start)
        send_selected = QPushButton("发送选中航点")
        send_selected.clicked.connect(self._send_selected_waypoint)
        cancel = QPushButton("取消当前目标")
        cancel.clicked.connect(self._stop_benchmark)
        delete_station = QPushButton("删除站点")
        delete_station.clicked.connect(self._delete_selected_station)
        move_up = QPushButton("上移")
        move_up.clicked.connect(lambda: self._move_selected_station(-1))
        move_down = QPushButton("下移")
        move_down.clicked.connect(lambda: self._move_selected_station(1))
        self.return_to_start_button = return_to_start
        waypoint_buttons.addWidget(delete_station)
        waypoint_buttons.addWidget(move_up)
        waypoint_buttons.addWidget(move_down)
        waypoint_layout.addLayout(waypoint_buttons)
        navigation_buttons = QHBoxLayout()
        navigation_buttons.addWidget(return_to_start)
        navigation_buttons.addWidget(send_selected)
        navigation_buttons.addWidget(cancel)
        waypoint_layout.addLayout(navigation_buttons)
        layout.addWidget(waypoint_group)

        tour_group = QGroupBox("自动验收巡游")
        tour_layout = QGridLayout(tour_group)
        self.tour_combo = QComboBox()
        self._refresh_tour_combo()
        self.tour_repeats = QSpinBox()
        self.tour_repeats.setRange(1, 20)
        self.tour_repeats.setValue(1)
        start_tour = QPushButton("开始巡游")
        start_tour.setObjectName("primaryButton")
        start_tour.clicked.connect(self._start_benchmark_tour)
        stop_tour = QPushButton("停止巡游")
        stop_tour.setObjectName("dangerButton")
        stop_tour.clicked.connect(self._stop_benchmark)
        reset_results = QPushButton("清空结果")
        reset_results.clicked.connect(self._reset_results)
        self.tour_status = QLabel("未运行")
        self.result_status = QLabel("通过 0 / 失败 0")
        tour_layout.addWidget(QLabel("项目"), 0, 0)
        tour_layout.addWidget(self.tour_combo, 0, 1)
        tour_layout.addWidget(QLabel("重复"), 1, 0)
        tour_layout.addWidget(self.tour_repeats, 1, 1)
        tour_layout.addWidget(start_tour, 2, 0)
        tour_layout.addWidget(stop_tour, 2, 1)
        tour_layout.addWidget(reset_results, 3, 0, 1, 2)
        tour_layout.addWidget(self.tour_status, 4, 0, 1, 2)
        tour_layout.addWidget(self.result_status, 5, 0, 1, 2)
        layout.addWidget(tour_group)
        layout.addStretch(1)
        scroll = self._scroll_page(tab)
        # The station table is intentionally wide; if the window is narrower
        # than the widened competition pane, expose a scrollbar instead of
        # allowing the right side to be clipped by the map splitter.
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        return scroll

    def _map_selection_changed(self, *_args) -> None:
        if self._loading_mission or not hasattr(self, "waypoint_table"):
            return
        self._refresh_competition_map_selector()
        selection_key = f"profiles/{self.profile.profile_id}"
        self.settings.setValue(
            f"{selection_key}/map_root", self.map_root_edit.currentText()
        )
        self.settings.setValue(
            f"{selection_key}/map_name", self.map_name_combo.currentText()
        )
        self._load_mission_for_current_map()

    def _refresh_maps(self) -> None:
        super()._refresh_maps()
        if hasattr(self, "competition_map_combo"):
            self._refresh_competition_map_selector()

    def _refresh_competition_map_selector(self) -> None:
        if not hasattr(self, "competition_map_combo"):
            return
        current = self.map_name_combo.currentText().strip()
        names = discover_map_names(self.map_root_edit.currentText())
        if current and current not in names:
            names.append(current)
            names.sort()
        self.competition_map_combo.blockSignals(True)
        self.competition_map_combo.clear()
        self.competition_map_combo.addItems(names)
        if current:
            self.competition_map_combo.setCurrentText(current)
        self.competition_map_combo.blockSignals(False)

    def _competition_map_selected(self, map_name: str) -> None:
        map_name = map_name.strip()
        if not map_name or map_name == self.map_name_combo.currentText().strip():
            return
        if self._process_running("mapping") or self._process_running("navigation"):
            self._refresh_competition_map_selector()
            QMessageBox.warning(
                self,
                "工作流正在运行",
                "请先停止建图或定位导航，再切换地图。",
            )
            return
        self.map_name_combo.blockSignals(True)
        self.map_name_combo.setEditText(map_name)
        self.map_name_combo.blockSignals(False)
        self._update_map_file_status()
        self._map_selection_changed()

    def _load_mission_for_current_map(self) -> None:
        try:
            paths = self._map_paths()
        except ValueError:
            self.path_plot.clear_map()
            return
        self._loading_mission = True
        try:
            self.path_plot.set_map(
                paths.pgm, paths.yaml, self._goal_safety_geometry
            )
            path = mission_path(paths)
            if path.is_file():
                stations = load_mission(path, paths.name)
                self._mission_waypoints = {
                    station.waypoint_id: station for station in stations
                }
                self._append_log(f"已加载比赛站点: {path}")
            elif paths.name == self.profile.map_name:
                self._mission_waypoints = dict(self.profile.waypoints)
            else:
                self._mission_waypoints = {}
            self._refresh_competition_map_selector()
            self.mission_path_label.setText(str(path))
            self._editing_station_id = None
            self._update_start_waypoint()
            initialization_waypoint = self._start_waypoint
            if initialization_waypoint is not None:
                self.initial_x.setValue(initialization_waypoint.x)
                self.initial_y.setValue(initialization_waypoint.y)
                self.initial_yaw.setValue(initialization_waypoint.yaw)
            self._refresh_waypoint_table()
            self._refresh_tour_combo()
        except Exception as exc:
            self.path_plot.clear_map()
            self._append_log(f"比赛站点加载失败: {exc}")
        finally:
            self._loading_mission = False
            flow_tab = getattr(self, "competition_flow_tab", None)
            if flow_tab is not None:
                flow_tab.invalidate_scene()

    def _persist_mission(self) -> bool:
        try:
            paths = self._map_paths()
            path = mission_path(paths)
            save_mission(
                path,
                paths.name,
                list(self._mission_waypoints.values()),
            )
        except Exception as exc:
            QMessageBox.critical(self, "保存站点失败", str(exc))
            return False
        self.mission_path_label.setText(str(path))
        self._append_log(f"已保存比赛站点: {path}")
        return True

    def _update_start_waypoint(self) -> None:
        matches = [
            waypoint
            for waypoint in self._mission_waypoints.values()
            if waypoint.role == "start_position"
        ]
        self._start_waypoint = matches[0] if len(matches) == 1 else None
        if hasattr(self, "return_to_start_button"):
            self.return_to_start_button.setEnabled(self._start_waypoint is not None)
        self.path_plot.reference_points = waypoint_landmarks(
            list(self._mission_waypoints.values())
        )
        self.path_plot.update()

    def _save_task_start_pose(self) -> None:
        """Optionally persist a task return point for the current map."""
        if self._process_running("mapping") or self._process_running("navigation"):
            QMessageBox.warning(
                self,
                "工作流正在运行",
                "请先停止建图和定位导航，再保存现场起点。",
            )
            return
        if self._start_waypoint is None:
            QMessageBox.warning(
                self,
                "未配置起点",
                "请先在比赛语义站点中创建唯一的 start_position。",
            )
            return
        x = self.initial_x.value()
        y = self.initial_y.value()
        yaw = self.initial_yaw.value()
        reason = self._goal_rejection_reason(x, y, yaw)
        if reason:
            QMessageBox.warning(self, "任务起点不安全", reason)
            return
        previous = self._mission_waypoints
        updated_start = replace(
            self._start_waypoint,
            x=x,
            y=y,
            yaw=yaw,
        )
        self._mission_waypoints = {
            station_id: updated_start
            if station_id == updated_start.waypoint_id
            else station
            for station_id, station in previous.items()
        }
        if not self._persist_mission():
            self._mission_waypoints = previous
            return
        self._update_start_waypoint()
        self._refresh_waypoint_table()
        self._append_log(
            "已保存任务返回起点："
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )
        QMessageBox.information(
            self,
            "任务起点已保存",
            "已写入当前地图 competition_mission.yaml。\n"
            "该点用于返回起点，不限制机器人当前实际位置。",
        )

    def _refresh_waypoint_table(self) -> None:
        if not hasattr(self, "waypoint_table"):
            return
        self.waypoint_table.blockSignals(True)
        self.waypoint_table.setRowCount(len(self._mission_waypoints))
        self.waypoint_table.clearContents()
        self._waypoint_rows.clear()
        for row, waypoint in enumerate(self._mission_waypoints.values()):
            self._waypoint_rows[waypoint.waypoint_id] = row
            values = (
                waypoint.waypoint_id,
                ROLE_LABELS.get(waypoint.role, waypoint.role),
                waypoint.label,
                f"{waypoint.x:.3f}",
                f"{waypoint.y:.3f}",
                f"{math.degrees(waypoint.yaw):.1f}°",
                "未测试",
                "--",
                "--",
                "--",
                (
                    f"激光前沿 / {waypoint.target_clearance_m * 100:.0f}cm"
                    if waypoint.requires_fine_alignment
                    else "无"
                ),
                " / ".join(
                    value
                    for enabled, value in (
                        (waypoint.should_retreat_after_pick, "取后"),
                        (waypoint.should_retreat_after_place, "放后"),
                    )
                    if enabled
                )
                or "无",
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.waypoint_table.setItem(row, column, item)
        self.waypoint_table.blockSignals(False)
        self.waypoint_table.resizeColumnsToContents()
        if self.waypoint_table.rowCount() > 0:
            self.waypoint_table.selectRow(0)
            self._station_selected()
        self._refresh_pick_navigation_targets()

    def _refresh_pick_navigation_targets(self) -> None:
        if not hasattr(self, "pick_source_combo"):
            return
        source_id = self.pick_source_combo.currentData()
        destination_id = self.pick_destination_combo.currentData()
        self.pick_source_combo.clear()
        self.pick_destination_combo.clear()
        for waypoint in self._mission_waypoints.values():
            label = f"{waypoint.label} ({waypoint.waypoint_id})"
            self.pick_source_combo.addItem(label, waypoint.waypoint_id)
            self.pick_destination_combo.addItem(label, waypoint.waypoint_id)
        source_index = self.pick_source_combo.findData(source_id)
        if source_index < 0:
            source_index = next(
                (
                    index
                    for index in range(self.pick_source_combo.count())
                    if self._mission_waypoints[
                        self.pick_source_combo.itemData(index)
                    ].role
                    == "shelf_approach"
                ),
                0,
            )
        self.pick_source_combo.setCurrentIndex(source_index)
        destination_index = self.pick_destination_combo.findData(destination_id)
        if destination_index < 0 and self.pick_destination_combo.count() > 1:
            destination_index = 1 if source_index == 0 else 0
        self.pick_destination_combo.setCurrentIndex(max(0, destination_index))

    def _refresh_tour_combo(self) -> None:
        if not hasattr(self, "tour_combo"):
            return
        selected = self.tour_combo.currentData()
        self.tour_combo.clear()
        self.tour_combo.addItem("当前站点顺序验证", "__station_order__")
        available = set(self._mission_waypoints)
        for tour in self.profile.tours.values():
            if set(tour.sequence).issubset(available):
                self.tour_combo.addItem(tour.label, tour.tour_id)
        index = self.tour_combo.findData(selected)
        self.tour_combo.setCurrentIndex(max(0, index))

    def _station_selected(self) -> None:
        waypoint = self._selected_waypoint()
        if waypoint is None:
            self._editing_station_id = None
            return
        self._editing_station_id = waypoint.waypoint_id
        self.station_id.setText(waypoint.waypoint_id)
        self.station_label.setText(waypoint.label)
        role_index = self.station_role.findData(waypoint.role)
        self.station_role.setCurrentIndex(max(0, role_index))
        self.station_x.setValue(waypoint.x)
        self.station_y.setValue(waypoint.y)
        self.station_yaw_deg.setValue(math.degrees(waypoint.yaw))
        self.station_fine_align.setChecked(waypoint.requires_fine_alignment)
        mode_index = self.station_fine_align_mode.findData(
            waypoint.fine_align_mode
        )
        self.station_fine_align_mode.setCurrentIndex(max(0, mode_index))
        self.station_target_clearance.setValue(waypoint.target_clearance_m)
        self.station_clearance_tolerance.setValue(
            waypoint.clearance_tolerance_m
        )
        self.station_fine_yaw_tolerance_deg.setValue(
            math.degrees(waypoint.fine_align_yaw_tolerance_rad)
        )
        self.station_fine_timeout.setValue(waypoint.fine_align_timeout_sec)
        self.station_retreat_after_pick.setChecked(
            waypoint.should_retreat_after_pick
        )
        self.station_retreat_after_place.setChecked(
            waypoint.should_retreat_after_place
        )
        self.station_retreat_distance.setValue(waypoint.retreat_distance_m)
        self.station_retreat_speed.setValue(waypoint.retreat_speed_mps)
        self._update_station_policy_controls()
        self.path_plot.set_selected_pose(waypoint.x, waypoint.y, waypoint.yaw)

    def _station_role_policy_defaults(self, _index: int = -1) -> None:
        if self._editing_station_id is not None:
            return
        role = str(self.station_role.currentData() or "waypoint")
        is_shelf = role == "shelf_approach"
        is_counter = role in ("delivery_table", "replenishment")
        self.station_fine_align.setChecked(is_shelf or is_counter)
        self.station_target_clearance.setValue(0.05 if is_shelf else 0.20)
        self.station_retreat_after_pick.setChecked(is_shelf)
        self.station_retreat_after_place.setChecked(is_counter)
        self._update_station_policy_controls()

    def _update_station_policy_controls(self, _checked: bool = False) -> None:
        enabled = self.station_fine_align.isChecked()
        for widget in (
            self.station_fine_align_mode,
            self.station_target_clearance,
            self.station_clearance_tolerance,
            self.station_fine_yaw_tolerance_deg,
            self.station_fine_timeout,
        ):
            widget.setEnabled(enabled)

    def _new_station(self) -> None:
        self.waypoint_table.clearSelection()
        self._editing_station_id = None
        self.station_id.clear()
        self.station_label.clear()
        role_index = self.station_role.findData("waypoint")
        self.station_role.setCurrentIndex(max(0, role_index))
        self.station_x.setValue(0.0)
        self.station_y.setValue(0.0)
        self.station_yaw_deg.setValue(0.0)
        self.station_fine_align.setChecked(False)
        self.station_target_clearance.setValue(0.05)
        self.station_clearance_tolerance.setValue(0.01)
        self.station_fine_yaw_tolerance_deg.setValue(1.0)
        self.station_fine_timeout.setValue(20.0)
        self.station_retreat_after_pick.setChecked(False)
        self.station_retreat_after_place.setChecked(False)
        self.station_retreat_distance.setValue(0.15)
        self.station_retreat_speed.setValue(0.04)
        self._update_station_policy_controls()
        self.path_plot.set_selected_pose(0.0, 0.0, 0.0)
        self.station_id.setFocus()

    def _pick_station_on_map(self) -> None:
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        missing = [path for path in (paths.pgm, paths.yaml) if not path.is_file()]
        if missing:
            QMessageBox.warning(self, "二维地图不完整", "\n".join(map(str, missing)))
            return
        initial = (
            self.station_x.value(),
            self.station_y.value(),
            math.radians(self.station_yaw_deg.value()),
        )
        try:
            dialog = WaypointPickerDialog(
                paths.pgm,
                paths.yaml,
                list(self._mission_waypoints.values()),
                initial,
                self,
                self._goal_safety_geometry,
            )
        except Exception as exc:
            QMessageBox.critical(self, "打开地图失败", str(exc))
            return
        if dialog.exec_() != QDialog.Accepted:
            return
        x, y, yaw = dialog.selected_pose
        self.station_x.setValue(x)
        self.station_y.setValue(y)
        self.station_yaw_deg.setValue(math.degrees(yaw))
        self.path_plot.set_selected_pose(x, y, yaw)

    def _capture_current_pose(self) -> None:
        state = self.telemetry
        if not state.localization_ok or not state.has_map_pose:
            QMessageBox.warning(self, "定位未就绪", "当前没有可信的地图位姿")
            return
        self.station_x.setValue(state.map_x)
        self.station_y.setValue(state.map_y)
        self.station_yaw_deg.setValue(math.degrees(state.map_yaw))

    def _preview_station_alignment(self) -> None:
        if not self.ros_node:
            QMessageBox.warning(self, "ROS 未就绪", "GUI ROS 节点尚未连接。")
            return
        if not self.station_fine_align.isChecked():
            QMessageBox.warning(self, "未启用精调", "请先为该站点启用激光前沿精调。")
            return
        try:
            station_id = validate_station_id(self.station_id.text())
        except ValueError as exc:
            QMessageBox.warning(self, "站点 ID", str(exc))
            return
        if self.telemetry.action_status in ("sending", "running", "canceling"):
            QMessageBox.warning(self, "导航正在运行", "请先停止 Nav2 目标再观测精调回波。")
            return
        if not self.telemetry.localization_ok:
            QMessageBox.warning(self, "定位未就绪", "需要 /localization_ok=true。")
            return
        if (
            self.telemetry.base_linear_speed
            > self.profile.acceptance.stopped_linear_mps
            or self.telemetry.base_angular_speed
            > self.profile.acceptance.stopped_angular_radps
        ):
            QMessageBox.warning(self, "底盘未停止", "底盘保持静止后才能进行只观测检查。")
            return
        scan_age = self._age(time.monotonic(), "/scan")
        if scan_age is None or scan_age > 0.50:
            QMessageBox.warning(self, "扫描未就绪", "/scan 超过 0.5 秒没有新数据。")
            return
        safety_cloud_age = self._age(time.monotonic(), "/livox/body_cloud_safety")
        if safety_cloud_age is None or safety_cloud_age > 0.50:
            QMessageBox.warning(
                self,
                "碰撞感知未就绪",
                "/livox/body_cloud_safety 超过 0.5 秒没有新数据。",
            )
            return
        if not self.ros_node.send_shelf_alignment_goal(
            station_id,
            math.radians(self.station_yaw_deg.value()),
            target_clearance=self.station_target_clearance.value(),
            clearance_tolerance=self.station_clearance_tolerance.value(),
            yaw_tolerance=math.radians(
                self.station_fine_yaw_tolerance_deg.value()
            ),
            timeout_sec=self.station_fine_timeout.value(),
            measure_only=True,
        ):
            QMessageBox.warning(
                self,
                "观测服务未就绪",
                "无法调用 /shelf_fine_align；请确认全身控制和精调服务已启动。",
            )
            return
        self._fine_align_preview_active = True
        self._fine_align_preview_station_id = station_id
        self.preview_alignment_status.setText(
            f"精调观测：正在采集 {station_id} 左右前沿回波（保证零速度）"
        )
        self.preview_alignment_status.setStyleSheet("color:#f1c75b;")
        self._append_log(f"开始精调只观测检查：{station_id}")

    def _pick_and_publish_initial_pose(self) -> None:
        if not self.ros_node:
            return
        navigation_running = self._process_running("navigation")
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        missing = [path for path in (paths.pgm, paths.yaml) if not path.is_file()]
        if missing:
            QMessageBox.warning(self, "二维地图不完整", "\n".join(map(str, missing)))
            return

        initial = (
            self.telemetry.map_x,
            self.telemetry.map_y,
            self.telemetry.map_yaw,
        ) if self.telemetry.has_map_pose else (
            self.initial_x.value(),
            self.initial_y.value(),
            self.initial_yaw.value(),
        )
        try:
            dialog = WaypointPickerDialog(
                paths.pgm,
                paths.yaml,
                list(self._mission_waypoints.values()),
                initial,
                self,
            )
        except Exception as exc:
            QMessageBox.critical(self, "打开地图失败", str(exc))
            return
        dialog.setWindowTitle(f"选择初始位姿 - {paths.name}")
        if dialog.exec_() != QDialog.Accepted:
            return

        if navigation_running:
            # Relocalization must never race an active base motion command.
            if self.telemetry.action_status in ("sending", "running", "canceling"):
                self._stop_benchmark()
            else:
                self._stop_motion()
        x, y, yaw = dialog.selected_pose
        self.initial_x.setValue(x)
        self.initial_y.setValue(y)
        self.initial_yaw.setValue(yaw)
        self.path_plot.set_selected_pose(x, y, yaw)
        if navigation_running:
            self._publish_initial_pose()
            self._append_log(
                f"地图重定位: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
            )
        else:
            self._append_log(
                f"已选择待保存现场起点: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
            )

    def _pick_and_send_map_goal(self) -> None:
        problems = self._benchmark_preflight_problems()
        if problems:
            QMessageBox.warning(self, "比赛导航未就绪", "\n".join(problems))
            return
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        missing = [path for path in (paths.pgm, paths.yaml) if not path.is_file()]
        if missing:
            QMessageBox.warning(self, "二维地图不完整", "\n".join(map(str, missing)))
            return

        if self._last_map_goal_pose is not None:
            initial = self._last_map_goal_pose
        elif self.telemetry.has_map_pose:
            initial = (
                self.telemetry.map_x,
                self.telemetry.map_y,
                self.telemetry.map_yaw,
            )
        elif self._start_waypoint is not None:
            initial = (
                self._start_waypoint.x,
                self._start_waypoint.y,
                self._start_waypoint.yaw,
            )
        else:
            initial = (0.0, 0.0, 0.0)

        try:
            dialog = WaypointPickerDialog(
                paths.pgm,
                paths.yaml,
                list(self._mission_waypoints.values()),
                initial,
                self,
                self._goal_safety_geometry,
            )
        except Exception as exc:
            QMessageBox.critical(self, "打开地图失败", str(exc))
            return
        dialog.setWindowTitle(f"选择临时导航目标 - {paths.name}")
        if dialog.exec_() != QDialog.Accepted:
            return

        x, y, yaw = dialog.selected_pose
        self._last_map_goal_pose = (x, y, yaw)
        waypoint = BenchmarkWaypoint(
            waypoint_id="MAP_GOAL",
            label="地图临时测点",
            x=x,
            y=y,
            yaw=yaw,
            clearance_m=0.0,
            role="waypoint",
        )
        self._tour_active = False
        self._tour_queue.clear()
        self.map_goal_status.setText(
            f"准备发送：X {x:.3f} m / Y {y:.3f} m / "
            f"Yaw {math.degrees(yaw):.1f} deg"
        )
        self._dispatch_waypoint(waypoint, track_table=False)

    def _upsert_station(self) -> None:
        reason = self._goal_rejection_reason(
            self.station_x.value(),
            self.station_y.value(),
            math.radians(self.station_yaw_deg.value()),
        )
        if reason:
            QMessageBox.warning(self, "站点位姿不安全", reason)
            return
        try:
            station_id = validate_station_id(self.station_id.text())
        except ValueError as exc:
            QMessageBox.warning(self, "站点 ID", str(exc))
            return
        role = str(self.station_role.currentData())
        if role == "start_position":
            other_start = next(
                (
                    waypoint
                    for waypoint in self._mission_waypoints.values()
                    if waypoint.role == role
                    and waypoint.waypoint_id != self._editing_station_id
                ),
                None,
            )
            if other_start is not None:
                QMessageBox.warning(
                    self,
                    "起点重复",
                    f"地图已经配置起点 {other_start.waypoint_id}",
                )
                return
        if station_id in self._mission_waypoints and station_id != self._editing_station_id:
            QMessageBox.warning(self, "站点重复", f"站点 {station_id} 已存在")
            return
        waypoint = BenchmarkWaypoint(
            waypoint_id=station_id,
            label=self.station_label.text().strip() or station_id,
            x=self.station_x.value(),
            y=self.station_y.value(),
            yaw=math.radians(self.station_yaw_deg.value()),
            clearance_m=0.0,
            role=role,
            fine_align_enabled=self.station_fine_align.isChecked(),
            fine_align_mode=str(self.station_fine_align_mode.currentData()),
            target_clearance_m=self.station_target_clearance.value(),
            clearance_tolerance_m=self.station_clearance_tolerance.value(),
            fine_align_yaw_tolerance_rad=math.radians(
                self.station_fine_yaw_tolerance_deg.value()
            ),
            fine_align_timeout_sec=self.station_fine_timeout.value(),
            retreat_after_pick=self.station_retreat_after_pick.isChecked(),
            retreat_after_place=self.station_retreat_after_place.isChecked(),
            retreat_distance_m=self.station_retreat_distance.value(),
            retreat_speed_mps=self.station_retreat_speed.value(),
        )
        items = list(self._mission_waypoints.items())
        if self._editing_station_id in self._mission_waypoints:
            index = [key for key, _ in items].index(self._editing_station_id)
            items[index] = (station_id, waypoint)
        else:
            items.append((station_id, waypoint))
        previous_waypoints = self._mission_waypoints
        self._mission_waypoints = dict(items)
        if not self._persist_mission():
            self._mission_waypoints = previous_waypoints
            return
        self._editing_station_id = station_id
        self._update_start_waypoint()
        self._refresh_waypoint_table()
        self._refresh_tour_combo()
        self.waypoint_table.selectRow(self._waypoint_rows[station_id])

    def _delete_selected_station(self) -> None:
        waypoint = self._selected_waypoint()
        if waypoint is None:
            return
        answer = QMessageBox.question(
            self,
            "删除比赛站点",
            f"确定删除 {waypoint.waypoint_id}？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        previous_waypoints = self._mission_waypoints
        self._mission_waypoints = dict(self._mission_waypoints)
        self._mission_waypoints.pop(waypoint.waypoint_id, None)
        if not self._persist_mission():
            self._mission_waypoints = previous_waypoints
            return
        self._editing_station_id = None
        self._update_start_waypoint()
        self._refresh_waypoint_table()
        self._refresh_tour_combo()

    def _move_selected_station(self, offset: int) -> None:
        waypoint = self._selected_waypoint()
        if waypoint is None:
            return
        items = list(self._mission_waypoints.items())
        index = [key for key, _ in items].index(waypoint.waypoint_id)
        target = index + offset
        if not 0 <= target < len(items):
            return
        items[index], items[target] = items[target], items[index]
        previous_waypoints = self._mission_waypoints
        self._mission_waypoints = dict(items)
        if not self._persist_mission():
            self._mission_waypoints = previous_waypoints
            return
        self._refresh_waypoint_table()
        self._refresh_tour_combo()
        self.waypoint_table.selectRow(target)

    @staticmethod
    def _can_interface_up(interface: str) -> bool:
        path = Path(f"/sys/class/net/{interface}/operstate")
        try:
            return path.read_text(encoding="utf-8").strip() in ("up", "unknown")
        except OSError:
            return False

    def _all_can_interfaces_up(self) -> bool:
        return all(self._can_interface_up(f"can{index}") for index in range(6))

    def _enable_all_can(self) -> None:
        if not CAN_ENABLE_HELPER.is_file():
            QMessageBox.critical(self, "CAN 工具缺失", str(CAN_ENABLE_HELPER))
            return
        self._start_process("can_enable", "启用全部 CAN", "bash", [str(CAN_ENABLE_HELPER)])

    def _disable_all_can(self) -> None:
        if self._process_running("fullbody") or "/controller_manager" in self.telemetry.node_names:
            QMessageBox.warning(self, "整机仍在运行", "请先停止全身控制，再禁用 CAN。")
            return
        if not CAN_DISABLE_HELPER.is_file():
            QMessageBox.critical(self, "CAN 工具缺失", str(CAN_DISABLE_HELPER))
            return
        self._start_process("can_disable", "禁用全部 CAN", "bash", [str(CAN_DISABLE_HELPER)])

    def _check_all_motors(self) -> None:
        if not MOTOR_CHECK_SCRIPT.is_file():
            QMessageBox.critical(self, "电机检查工具缺失", str(MOTOR_CHECK_SCRIPT))
            return
        self.motor_status.setText("电机：检查中")
        self.motor_status.setStyleSheet("color:#f1c75b;")
        self._start_process("motor_check", "检查全部电机", "python3", [str(MOTOR_CHECK_SCRIPT)])

    def _control_mode_changed(self, index: int) -> None:
        if index == 0 and self._process_running("vr"):
            self._append_log("切换到全自主比赛，正在停止 VR 遥操作")
            self._stop_vr()
        self._update_fullbody_controls()

    def _start_pick_preparation(
        self,
        *,
        pre_navigation_level: Optional[int] = None,
        transport_level: int = SAFE_TRANSPORT_SHELF_LEVEL,
        item_target_pose: Optional[tuple[float, float, float]] = None,
    ) -> None:
        if not self.ros_node:
            QMessageBox.warning(self, "ROS 未就绪", "GUI ROS 节点尚未连接。")
            return
        if self._pick_navigation_test_stage not in ("idle", "complete", "failed"):
            QMessageBox.warning(self, "联调正在运行", "请等待当前抓取导航链路结束。")
            return
        source_id = str(self.pick_source_combo.currentData() or "")
        destination_id = str(self.pick_destination_combo.currentData() or "")
        source = self._current_pick_source(source_id)
        destination = self._mission_waypoints.get(destination_id)
        if not source_id or not destination_id or source_id == destination_id:
            QMessageBox.warning(
                self,
                "站点不足",
                "请在比赛导航页至少保存两个不同站点，并选择当前货架站和下一导航站。",
            )
            return
        if source is None or destination is None:
            QMessageBox.warning(self, "目标不存在", "当前货架站或下一导航站不存在。")
            return
        if source.role != "shelf_approach":
            QMessageBox.warning(self, "货架站类型错误", "当前货架站必须是 shelf_approach 类型。")
            return
        if self.pick_arm_combo.currentText().strip().lower() != "right":
            QMessageBox.warning(self, "手臂未接入", "当前抓取执行流程仅接入右臂。")
            return
        if not self.ros_node.shelf_view_action_client.server_is_ready():
            QMessageBox.warning(
                self,
                "货架观察服务未就绪",
                "没有发现 /competition/move_to_shelf_view，请确认全身控制已启动。",
            )
            return
        if pre_navigation_level is None:
            problems = self._benchmark_preflight_problems()
            if problems:
                QMessageBox.warning(self, "比赛导航未就绪", "\n".join(problems))
                return

        level = int(self.pick_shelf_level.currentData() or SAFE_TRANSPORT_SHELF_LEVEL)
        self._pick_navigation_test_source_id = source_id
        self._pick_navigation_test_destination = destination
        self._pick_navigation_test_level = level
        self._pick_navigation_test_transport_level = int(transport_level)
        self._pick_navigation_test_item_target_pose = item_target_pose
        self._pick_navigation_test_at_grasp_clearance = False
        self._pick_navigation_test_pre_navigation_level = (
            None if pre_navigation_level is None else int(pre_navigation_level)
        )
        self._pick_navigation_test_grasp_position = None
        self._pick_navigation_test_retreat_distance = (
            source.retreat_distance_m
            if source.should_retreat_after_pick
            else self.pick_retreat_distance.value()
        )
        self._pick_navigation_test_retreat_speed = (
            source.retreat_speed_mps
            if source.should_retreat_after_pick
            else self.pick_retreat_speed.value()
        )
        self.pick_retreat_distance.setValue(self._pick_navigation_test_retreat_distance)
        self.pick_retreat_speed.setValue(self._pick_navigation_test_retreat_speed)
        self.pregrasp_result_status.setText("观察姿态结果：等待导航和精停")
        self.pregrasp_result_status.setStyleSheet("color:#f1c75b;")
        self.pick_result_status.setText("抓取结果：等待观察姿态完成")
        self.pick_result_status.setStyleSheet("color:#a0a0a0;")
        self.retreat_result_status.setText("退避结果：尚未执行")
        self.retreat_result_status.setStyleSheet("color:#a0a0a0;")
        self.pick_chain_status.setText(
            f"联调状态：正在导航到 {source_id} 并准备第 {level} 层"
        )
        self.pick_chain_status.setStyleSheet("color:#f1c75b;")

        self._tour_active = False
        self._tour_queue.clear()
        if pre_navigation_level is not None:
            manual_state = "released"
            if hasattr(self.ros_node, "manual_joint_control_status"):
                manual_state, _detail = self.ros_node.manual_joint_control_status()
            if manual_state != "released" and hasattr(
                self.ros_node, "release_manual_joint_control"
            ):
                self.ros_node.release_manual_joint_control()
                self._pick_navigation_test_handoff_deadline = time.monotonic() + 10.0
                self._pick_navigation_test_stage = (
                    "releasing_manual_control_before_transport"
                )
                self.pick_chain_status.setText(
                    "联调状态：正在交还手动关节控制，准备进入 move 姿态"
                )
                self._append_log("比赛流程正在交还手动关节控制权")
                return
            self._send_pre_navigation_posture()
            return
        self._dispatch_pick_source(source)

    def _send_pre_navigation_posture(self) -> bool:
        level = self._pick_navigation_test_pre_navigation_level
        if self.ros_node is None or level is None:
            self._fail_mock_pick_navigation_test("出发前 move 姿态配置缺失")
            return False
        if not self.ros_node.send_shelf_view_goal(
            self._pick_navigation_test_source_id,
            level,
            self._taught_posture_arm_name(level),
        ):
            self._fail_mock_pick_navigation_test("出发前 move 姿态 Action 下发失败")
            return False
        self._pick_navigation_test_stage = "moving_to_transport_before_shelf"
        self.pick_chain_status.setText("联调状态：出发前正在进入 move 姿态")
        self._append_log("出发前开始进入示教 move 姿态")
        return True

    def _taught_posture_arm_name(self, level: Optional[int] = None) -> str:
        """Use both arms only for the Test-9 poses that teach both arms."""
        if (
            self.profile.map_name == TEST9_TAUGHT_POSTURE_MAP
            and int(self._pick_navigation_test_level if level is None else level)
            in (0, 4, 10, 11)
        ):
            return "both"
        return self.pick_arm_combo.currentText()

    def _current_pick_source(
        self, station_id: Optional[str] = None
    ) -> Optional[BenchmarkWaypoint]:
        source_id = self._pick_navigation_test_source_id if station_id is None else station_id
        override = self._pick_navigation_test_source_override
        if override is not None and override.waypoint_id == source_id:
            return override
        return self._mission_waypoints.get(source_id)

    def _dispatch_pick_source(self, source: BenchmarkWaypoint) -> bool:
        self._pick_navigation_test_stage = "navigating_to_shelf"
        if not self._dispatch_waypoint(source):
            self._fail_mock_pick_navigation_test("货架导航目标下发失败")
            return False
        self._append_log(
            f"开始货架观察准备：{source.waypoint_id} -> 精停 -> "
            f"第 {self._pick_navigation_test_level} 层观察姿态"
        )
        return True

    def _confirm_manual_pick_success(self) -> bool:
        """Continue the route after the operator confirms a physical/demo pick."""
        if not self.ros_node or self._pick_navigation_test_stage != "ready":
            return False
        if self.profile.map_name == TEST9_TAUGHT_POSTURE_MAP:
            position = self.telemetry.joint_positions.get(RIGHT_GRIPPER_JOINT_NAME)
            stamp = self.telemetry.last_seen.get("/joint_states")
            if (
                position is None
                or not math.isfinite(position)
                or stamp is None
                or time.monotonic() - stamp >= 1.0
                or not RIGHT_GRIPPER_VALID_MIN_M
                <= position
                <= RIGHT_GRIPPER_VALID_MAX_M
            ):
                self.pick_result_status.setText("抓取结果：右夹爪反馈无效，禁止运输")
                self.pick_result_status.setStyleSheet("color:#ff6b6b;")
                return False
            if position < RIGHT_GRIPPER_GRASP_MIN_POSITION_M:
                self.pick_result_status.setText(
                    "抓取结果：右夹爪接近全闭"
                    f"（{position * 1000:.1f} mm），缺少物体夹持证据"
                )
                self.pick_result_status.setStyleSheet("color:#ff6b6b;")
                self._append_log(
                    "抓取确认被拒绝：右夹爪位置 "
                    f"{position * 1000:.1f} mm，低于有效夹持下限 "
                    f"{RIGHT_GRIPPER_GRASP_MIN_POSITION_M * 1000:.1f} mm"
                )
                return False
            if position > RIGHT_GRIPPER_GRASP_MAX_POSITION_M:
                self.pick_result_status.setText(
                    "抓取结果：右夹爪仍接近全开"
                    f"（{position * 1000:.1f} mm），禁止运输"
                )
                self.pick_result_status.setStyleSheet("color:#ff6b6b;")
                self._append_log(
                    "抓取确认被拒绝：右夹爪位置 "
                    f"{position * 1000:.1f} mm，至少需要闭合 "
                    f"{RIGHT_GRIPPER_MIN_GRASP_CLOSURE_M * 1000:.1f} mm"
                )
                return False
            self._pick_navigation_test_grasp_position = float(position)
            self._append_log(
                f"抓取确认时右夹爪位置 {position * 1000:.1f} mm；运输期间持续监控"
            )
        self.pick_result_status.setText("抓取结果：操作员已确认成功")
        self.pick_result_status.setStyleSheet("color:#5fd38d;")
        if self._pick_navigation_test_level != self._pick_navigation_test_transport_level:
            if not self.ros_node.send_shelf_view_goal(
                self._pick_navigation_test_source_id or "GUI_SHELF",
                self._pick_navigation_test_transport_level,
                self._taught_posture_arm_name(
                    self._pick_navigation_test_transport_level
                ),
            ):
                return False
            self._pick_navigation_test_stage = "returning_transport"
            self.pick_chain_status.setText("联调状态：抓取已确认，正在回 move 运输姿态")
            self._append_log("抓取成功已确认，开始回 move 运输姿态")
            return True
        return self._continue_after_pick()

    def _continue_after_place_confirmation(self) -> bool:
        """Retreat from the delivery station, then return to the demo start."""
        if not self.ros_node or self._pick_navigation_test_stage != "ready_to_place":
            return False
        destination = self._pick_navigation_test_destination
        return_target = self._pick_navigation_test_return_destination
        if destination is None or return_target is None:
            return False
        distance = (
            destination.retreat_distance_m
            if destination.should_retreat_after_place
            else self.pick_retreat_distance.value()
        )
        speed = (
            destination.retreat_speed_mps
            if destination.should_retreat_after_place
            else self.pick_retreat_speed.value()
        )
        if not self.ros_node.send_station_retreat_goal(
            destination.waypoint_id,
            distance=distance,
            max_speed=speed,
            timeout_sec=8.0,
        ):
            return False
        self._pick_navigation_test_stage = "retreating_after_place"
        self.retreat_result_status.setText("退避结果：交付台放置后退避中")
        self.retreat_result_status.setStyleSheet("color:#f1c75b;")
        self.pick_chain_status.setText("联调状态：放置已确认，正在退出交付台")
        self._append_log(
            f"交付完成已确认，从 {destination.waypoint_id} 退避后返回 "
            f"{return_target.waypoint_id}"
        )
        return True

    def _uses_test9_delivery_sequence(self, station_id: str) -> bool:
        return (
            self.profile.map_name == TEST9_TAUGHT_POSTURE_MAP
            and str(station_id) == TEST9_DELIVERY_STATION
        )

    def _start_test9_place_sequence(self) -> bool:
        """Enter the first C-point place posture without changing the gripper."""
        destination = self._pick_navigation_test_destination
        if self.ros_node is None or destination is None:
            return False
        if not self.ros_node.send_shelf_view_goal(
            destination.waypoint_id,
            TEST9_PLACE_TRANSITION_POSTURE_LEVEL,
            "both",
        ):
            return False
        self._pick_navigation_test_stage = "moving_to_place_transition"
        self.pick_chain_status.setText("联调状态：C 点精调完成，正在进入放置过渡2")
        self.pick_chain_status.setStyleSheet("color:#f1c75b;")
        self._append_log("C 点精调完成，开始进入放置过渡2；右夹爪继续保持抓取")
        return True

    def _right_gripper_hold_problem(self) -> str:
        baseline = self._pick_navigation_test_grasp_position
        if baseline is None:
            return ""
        stamp = self.telemetry.last_seen.get("/joint_states")
        if stamp is None or time.monotonic() - stamp >= 1.0:
            return "右夹爪反馈超时"
        position = self.telemetry.joint_positions.get(RIGHT_GRIPPER_JOINT_NAME)
        if position is None or not math.isfinite(position):
            return "右夹爪位置缺失或非有限值"
        if not RIGHT_GRIPPER_VALID_MIN_M <= position <= RIGHT_GRIPPER_VALID_MAX_M:
            return f"右夹爪位置越界：{position:.4f} m"
        delta = position - baseline
        if abs(delta) > RIGHT_GRIPPER_HOLD_TOLERANCE_M:
            return (
                f"右夹爪未保持抓取：确认时 {baseline:.4f} m，"
                f"当前 {position:.4f} m"
            )
        return ""

    def _start_mock_pick_navigation_test(self) -> None:
        if not self.ros_node:
            QMessageBox.warning(self, "ROS 未就绪", "GUI ROS 节点尚未连接。")
            return
        if self._pick_navigation_test_stage != "ready":
            QMessageBox.warning(
                self,
                "货架观察尚未完成",
                "请先点击“导航、精停并进入观察姿态”，等待姿态到位后再模拟抓取。",
            )
            return
        source_id = str(self.pick_source_combo.currentData() or "")
        destination_id = str(self.pick_destination_combo.currentData() or "")
        if not source_id or not destination_id or source_id == destination_id:
            QMessageBox.warning(
                self,
                "站点不足",
                "请在比赛导航页至少保存两个不同站点，并选择当前货架站和下一导航站。",
            )
            return
        destination = self._mission_waypoints.get(destination_id)
        if destination is None:
            QMessageBox.warning(self, "目标不存在", "下一导航站已不在当前地图配置中。")
            return
        if not self._process_running("navigation"):
            QMessageBox.warning(self, "定位导航未运行", "请先启动定位导航与底盘安全链。")
            return
        if not self.telemetry.localization_ok:
            QMessageBox.warning(self, "定位不安全", "当前禁止执行抓取后的底盘退避。")
            return
        if (
            self.telemetry.base_linear_speed > self.profile.acceptance.stopped_linear_mps
            or self.telemetry.base_angular_speed
            > self.profile.acceptance.stopped_angular_radps
        ):
            QMessageBox.warning(self, "底盘未停止", "请等待货架精停完成并保持静止。")
            return
        if not self.ros_node.mock_pick_action_client.server_is_ready():
            QMessageBox.warning(
                self,
                "模拟抓取未就绪",
                "没有发现 /competition/mock_pick_object，请确认全身控制已启动。",
            )
            return
        if not self.ros_node.retreat_action_client.server_is_ready():
            QMessageBox.warning(
                self,
                "安全退避未就绪",
                "没有发现 /retreat_from_station，请确认全身控制已启动。",
            )
            return
        if not self.ros_node.send_mock_pick_goal(
            self.pick_object_id.text(), self.pick_arm_combo.currentText()
        ):
            QMessageBox.warning(self, "模拟抓取下发失败", "模拟抓取 Action 当前不可用。")
            return
        self._pick_navigation_test_destination = destination
        self._pick_navigation_test_source_id = source_id
        source = self._current_pick_source()
        if source is not None and source.should_retreat_after_pick:
            self._pick_navigation_test_retreat_distance = source.retreat_distance_m
            self._pick_navigation_test_retreat_speed = source.retreat_speed_mps
        else:
            self._pick_navigation_test_retreat_distance = self.pick_retreat_distance.value()
            self._pick_navigation_test_retreat_speed = self.pick_retreat_speed.value()
        self._pick_navigation_test_stage = "picking"
        self.pick_result_status.setText("抓取结果：模拟执行中")
        self.pick_result_status.setStyleSheet("color:#f1c75b;")
        self.retreat_result_status.setText("退避结果：等待抓取成功")
        self.retreat_result_status.setStyleSheet("color:#a0a0a0;")
        self.pick_chain_status.setText("联调状态：正在模拟抓取")
        self._append_log(
            f"开始模拟抓取链路：{source_id} -> 退避 -> {destination_id}"
        )

    def _send_retreat_after_mock_pick(self) -> bool:
        if not self.ros_node:
            return False
        sent = self.ros_node.send_station_retreat_goal(
            self._pick_navigation_test_source_id or "GUI_SHELF",
            distance=self._pick_navigation_test_retreat_distance,
            max_speed=self._pick_navigation_test_retreat_speed,
            timeout_sec=8.0,
        )
        if sent:
            self._pick_navigation_test_stage = "retreating"
            self.pick_chain_status.setText("联调状态：运输姿态就绪，正在安全退避")
            self._append_log("开始调用 /retreat_from_station")
        return sent

    def _mark_pick_observation_ready(self) -> None:
        self._pick_navigation_test_stage = "ready"
        self.pregrasp_result_status.setText(
            f"观察姿态结果：第 {self._pick_navigation_test_level} 层到位"
        )
        self.pregrasp_result_status.setStyleSheet("color:#5fd38d;")
        confirmation = (
            "请到抓取调试页手动检测、规划并执行抓取"
            if self._pick_navigation_test_return_destination is not None
            else "可点击模拟抓取"
        )
        self.pick_chain_status.setText(f"联调状态：商品相机点到位，{confirmation}")
        self.pick_chain_status.setStyleSheet("color:#5fd38d;")
        self._append_log(f"货架商品相机点到位，{confirmation}")

    def _rebase_shelf_traverse_targets_to_current_anchor(self) -> None:
        """Keep the lateral plan, but anchor it to this run's fine-aligned pose."""
        anchor = self._pick_navigation_test_anchor_return_pose
        item = self._pick_navigation_test_item_target_pose
        if anchor is None or item is None or not self.telemetry.has_map_pose:
            return

        old_x, old_y, old_yaw = anchor
        delta_x = item[0] - old_x
        delta_y = item[1] - old_y
        local_forward = math.cos(old_yaw) * delta_x + math.sin(old_yaw) * delta_y
        local_lateral = -math.sin(old_yaw) * delta_x + math.cos(old_yaw) * delta_y

        current_x = float(self.telemetry.map_x)
        current_y = float(self.telemetry.map_y)
        current_yaw = float(self.telemetry.map_yaw)
        rebased_x = (
            current_x
            + math.cos(current_yaw) * local_forward
            - math.sin(current_yaw) * local_lateral
        )
        rebased_y = (
            current_y
            + math.sin(current_yaw) * local_forward
            + math.cos(current_yaw) * local_lateral
        )
        self._pick_navigation_test_anchor_return_pose = (
            current_x,
            current_y,
            current_yaw,
        )
        self._pick_navigation_test_item_target_pose = (
            rebased_x,
            rebased_y,
            current_yaw,
        )
        self._append_log(
            "按本次 A 精调实测位姿重绑定货架横移："
            f"A=({current_x:.3f}, {current_y:.3f}, "
            f"{math.degrees(current_yaw):.1f}°)，"
            f"商品点=({rebased_x:.3f}, {rebased_y:.3f})"
        )

    def _send_item_shelf_traverse_after_observation(self) -> bool:
        """Move laterally from the fine-aligned A pose to the selected item."""
        target = self._pick_navigation_test_item_target_pose
        if target is None:
            self._mark_pick_observation_ready()
            return True
        if not self.ros_node:
            return False
        source = self._mission_waypoints.get(self._pick_navigation_test_source_id)
        if source is None:
            return False
        target_x, target_y, target_yaw = target
        if (
            self.telemetry.has_map_pose
            and math.hypot(
                self.telemetry.map_x - target_x,
                self.telemetry.map_y - target_y,
            )
            <= 0.03
        ):
            self._append_log("A 精调位置已在商品相机点容差内，无需横移")
            self._mark_pick_observation_ready()
            return True
        sent = self.ros_node.send_shelf_traverse_goal(
            source.waypoint_id,
            target_x,
            target_y,
            target_yaw,
            target_clearance=source.target_clearance_m,
            clearance_tolerance=source.clearance_tolerance_m,
            lateral_tolerance=0.03,
            yaw_tolerance=source.fine_align_yaw_tolerance_rad,
            max_lateral_speed=min(0.04, source.retreat_speed_mps),
            timeout_sec=45.0,
            lateral_only=(self.profile.map_name == TEST9_TAUGHT_POSTURE_MAP),
        )
        if sent:
            self._pick_navigation_test_stage = "traversing_to_item"
            self.pick_chain_status.setText(
                "联调状态：观察姿态已保持，正在沿货架横移到商品相机点"
            )
            self._append_log(
                f"第 {self._pick_navigation_test_level} 层观察姿态到位，"
                "开始执行受净距保护的货架横移，前往商品相机点"
            )
        return sent

    def _send_item_grasp_clearance_alignment(self) -> bool:
        """Approach from the lateral safety band only after reaching the item."""
        source = self._current_pick_source()
        if source is None:
            return False
        staged_clearance = (
            self.profile.map_name == TEST9_TAUGHT_POSTURE_MAP
            and self._pick_navigation_test_anchor_return_pose is not None
            and source.target_clearance_m > TEST9_GRASP_CLEARANCE_M
        )
        if not staged_clearance:
            self._mark_pick_observation_ready()
            return True
        if not self.ros_node:
            return False
        target_yaw = (
            self._pick_navigation_test_item_target_pose[2]
            if self._pick_navigation_test_item_target_pose is not None
            else source.yaw
        )
        sent = self.ros_node.send_shelf_alignment_goal(
            source.waypoint_id,
            target_yaw,
            target_clearance=TEST9_GRASP_CLEARANCE_M,
            clearance_tolerance=TEST9_GRASP_CLEARANCE_TOLERANCE_M,
            yaw_tolerance=source.fine_align_yaw_tolerance_rad,
            timeout_sec=20.0,
        )
        if sent:
            self._pick_navigation_test_stage = "aligning_to_item_grasp_clearance"
            self.pick_chain_status.setText(
                "联调状态：商品横移点到位，正在向前精调至抓取净距"
            )
            self._append_log(
                f"商品横移点到位，开始从 {source.target_clearance_m:.3f}m "
                f"安全带向前精调至 {TEST9_GRASP_CLEARANCE_M:.3f}m 抓取净距"
            )
        return sent

    def _send_direct_retreat_after_pick(self) -> bool:
        """Retreat from the item far enough for Nav2 without returning to A."""
        source = self._current_pick_source()
        if not self.ros_node or source is None:
            return False
        distance = TEST9_POST_PICK_RETREAT_DISTANCE_M
        max_speed = min(0.03, self._pick_navigation_test_retreat_speed)
        timeout_sec = max(8.0, distance / max(max_speed, 0.01) + 3.0)
        sent = self.ros_node.send_station_retreat_goal(
            source.waypoint_id,
            distance=distance,
            max_speed=max_speed,
            timeout_sec=timeout_sec,
        )
        if sent:
            self._pick_navigation_test_stage = "retreating"
            self.pick_chain_status.setText(
                "联调状态：move 姿态就绪，正在从商品点直退至导航安全距离"
            )
            self._append_log(
                f"抓取后开始直退 {distance:.3f}m，"
                "不返回 A，退避完成后直接导航到交付台"
            )
        return sent

    def _continue_after_pick(self) -> bool:
        """Retreat when the source policy requests it, then navigate onward."""
        source = self._current_pick_source()
        if source is not None and not source.should_retreat_after_pick:
            destination = self._pick_navigation_test_destination
            if destination is None or not self._dispatch_waypoint(destination):
                self._fail_mock_pick_navigation_test("下一导航目标下发失败")
                return False
            self._pick_navigation_test_stage = "navigating"
            self.pick_chain_status.setText(
                f"联调状态：取货后无需退避，正在导航到 {destination.waypoint_id}"
            )
            self._append_log(
                f"取货后退避策略关闭，发送下一导航目标 {destination.waypoint_id}"
            )
            return True
        if self._pick_navigation_test_at_grasp_clearance:
            return self._send_direct_retreat_after_pick()
        return self._send_retreat_after_mock_pick()

    def _fail_mock_pick_navigation_test(self, message: str) -> None:
        self._stop_competition_actions_for_manual_recovery()
        self._pick_navigation_test_stage = "failed"
        self.pick_chain_status.setText(
            f"联调状态：失败（{message}）；自动流程已停止，可保持整机控制并执行 Home"
        )
        self.pick_chain_status.setStyleSheet("color:#ff6b6b;")
        self._append_log(
            f"模拟抓取导航链路失败：{message}；正在释放自动 Action，"
            "整机控制保持在线，关节控制将在机器人停止后开放"
        )

    def _stop_competition_actions_for_manual_recovery(self) -> None:
        """Cancel automatic work without stopping the holding controllers."""
        if self.ros_node:
            for method_name in (
                "cancel_navigation_goal",
                "cancel_shelf_alignment_goal",
                "cancel_shelf_traverse_goal",
                "cancel_shelf_view_goal",
                "cancel_grasp_debug_goal",
                "cancel_mock_pick_goal",
                "cancel_right_gripper_goal",
                "cancel_station_retreat_goal",
                "cancel_shopping_ticket",
            ):
                method = getattr(self.ros_node, method_name, None)
                if callable(method):
                    method()
        if self.ros_node is None or hasattr(self.ros_node, "publish_teleop"):
            self._stop_motion()
        else:
            self.teleop_command = (0.0, 0.0, 0.0)
        self._tour_active = False
        self._tour_waiting = False
        self._tour_queue.clear()
        self._active_target = None
        self._active_target_row = None
        self._active_is_map_goal = False
        self._goal_terminal_handled = True
        self._active_motion_phase = "idle"
        self._stop_stable_since = None
        joint_page = getattr(self, "joint_control_page", None)
        if joint_page is not None:
            joint_page.stop_commands()

    def _cancel_mock_pick_navigation_test(self, reason: str) -> None:
        stage = self._pick_navigation_test_stage
        if stage not in (
            "releasing_manual_control_before_transport",
            "moving_to_transport_before_shelf",
            "waiting_navigation_ready_before_shelf",
            "navigating_to_shelf",
            "moving_to_pregrasp",
            "traversing_to_item",
            "aligning_to_item_grasp_clearance",
            "ready",
            "executing_real_grasp",
            "picking",
            "returning_transport",
            "retreating",
            "navigating",
            "ready_to_place",
            "waiting_for_place_confirmation",
            "moving_to_place_transition",
            "moving_to_place",
            "releasing_after_place",
            "returning_move_after_place",
            "retreating_after_place",
            "returning_to_start",
        ):
            return
        self._stop_competition_actions_for_manual_recovery()
        self._pick_navigation_test_stage = "failed"
        self.pick_chain_status.setText(
            f"联调状态：已停止（{reason}）；可保持整机控制并执行 Home"
        )
        self.pick_chain_status.setStyleSheet("color:#f1c75b;")
        self._append_log(
            f"模拟抓取导航链路已停止：{reason}；"
            "关节控制将在自动 Action 退出且机器人停止后开放"
        )

    def _update_mock_pick_navigation_test(self) -> None:
        if not hasattr(self, "pick_chain_status") or not self.ros_node:
            return
        state = self.telemetry
        stage = self._pick_navigation_test_stage
        hold_required_stages = {
            "returning_transport",
            "retreating",
            "navigating",
            "moving_to_place_transition",
            "moving_to_place",
        }
        if stage in hold_required_stages:
            gripper_problem = self._right_gripper_hold_problem()
            if gripper_problem:
                self._cancel_mock_pick_navigation_test(gripper_problem)
                self.pick_result_status.setText(f"抓取结果：失败（{gripper_problem}）")
                self.pick_result_status.setStyleSheet("color:#ff6b6b;")
                return
        if stage == "releasing_manual_control_before_transport":
            if hasattr(self.ros_node, "poll_manual_joint_control"):
                self.ros_node.poll_manual_joint_control()
            manual_state, detail = ("released", "")
            if hasattr(self.ros_node, "manual_joint_control_status"):
                manual_state, detail = self.ros_node.manual_joint_control_status()
            self.pick_chain_status.setText(
                f"联调状态：正在交还手动关节控制（{detail or manual_state}）"
            )
            if manual_state == "released":
                self._send_pre_navigation_posture()
            elif manual_state == "error":
                self._fail_mock_pick_navigation_test(
                    f"手动关节控制交还失败：{detail or manual_state}"
                )
            elif time.monotonic() >= self._pick_navigation_test_handoff_deadline:
                self._fail_mock_pick_navigation_test(
                    f"手动关节控制交还超时：{detail or manual_state}"
                )
        elif stage == "moving_to_transport_before_shelf":
            self.pick_chain_status.setText(
                f"联调状态：出发前 move 姿态 {state.shelf_view_status} "
                f"{state.shelf_view_phase}"
            )
            if state.shelf_view_status == "succeeded":
                source = self._current_pick_source()
                if source is None:
                    self._fail_mock_pick_navigation_test("货架站点不存在")
                    return
                problems = self._benchmark_preflight_problems()
                if problems:
                    self._pick_navigation_test_stage = (
                        "waiting_navigation_ready_before_shelf"
                    )
                    self._pick_navigation_test_navigation_ready_deadline = (
                        time.monotonic() + 20.0
                    )
                    self.pick_chain_status.setText(
                        "联调状态：move 姿态到位，等待定位导航稳定"
                    )
                    self._append_log(
                        "move 姿态到位，等待定位导航稳定：" + "；".join(problems)
                    )
                else:
                    self._dispatch_pick_source(source)
            elif state.shelf_view_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.shelf_view_message or state.shelf_view_status
                )
        elif stage == "waiting_navigation_ready_before_shelf":
            problems = self._benchmark_preflight_problems()
            if not problems:
                source = self._current_pick_source()
                if source is None or not self._dispatch_pick_source(source):
                    return
                self._append_log("定位导航已稳定，开始前往货架台 A 点")
            elif time.monotonic() >= self._pick_navigation_test_navigation_ready_deadline:
                self._fail_mock_pick_navigation_test(
                    "move 姿态已到位，但定位导航等待超时：" + "；".join(problems)
                )
            else:
                self.pick_chain_status.setText(
                    "联调状态：move 姿态到位，等待定位导航稳定（"
                    + problems[0]
                    + "）"
                )
        elif stage == "moving_to_pregrasp":
            self.pregrasp_result_status.setText(
                f"观察姿态结果：{state.shelf_view_status} "
                f"{state.shelf_view_phase} / lift {state.shelf_view_lift_position:.4f}m"
            )
            if state.shelf_view_status == "succeeded":
                if not self._send_item_shelf_traverse_after_observation():
                    self._fail_mock_pick_navigation_test(
                        "观察姿态到位，但商品相机点横移 Action 下发失败"
                    )
            elif state.shelf_view_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.shelf_view_message or state.shelf_view_status
                )
        elif stage == "traversing_to_item":
            self.pick_chain_status.setText(
                "联调状态：沿货架前往商品相机点 / "
                f"{state.shelf_traverse_status} {state.shelf_traverse_phase} / "
                f"剩余 {state.shelf_traverse_remaining:.3f}m / "
                f"净距 {state.shelf_traverse_clearance:.3f}m"
            )
            if state.shelf_traverse_status == "succeeded":
                if not self._send_item_grasp_clearance_alignment():
                    self._fail_mock_pick_navigation_test(
                        "商品横移点到位，但抓取净距精调 Action 下发失败"
                    )
            elif state.shelf_traverse_status in (
                "failed",
                "rejected",
                "error",
            ):
                self._fail_mock_pick_navigation_test(
                    state.shelf_traverse_message or state.shelf_traverse_status
                )
        elif stage == "aligning_to_item_grasp_clearance":
            self.pick_chain_status.setText(
                "联调状态：商品点向前精调 / "
                f"{state.shelf_align_status} {state.shelf_align_phase} / "
                f"净距 {state.shelf_clearance:.3f}m"
            )
            if state.shelf_align_status == "succeeded":
                self._pick_navigation_test_at_grasp_clearance = True
                self._mark_pick_observation_ready()
            elif state.shelf_align_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.shelf_align_message or state.shelf_align_status
                )
        elif stage == "picking":
            self.pick_result_status.setText(
                f"抓取结果：{state.pick_test_status} "
                f"{state.pick_test_progress * 100:.0f}%"
            )
            if state.pick_test_status == "succeeded" and state.pick_test_attached:
                self.pick_result_status.setText("抓取结果：模拟成功 / 已假定附着")
                self.pick_result_status.setStyleSheet("color:#5fd38d;")
                if self._pick_navigation_test_level != self._pick_navigation_test_transport_level:
                    sent = self.ros_node.send_shelf_view_goal(
                        self._pick_navigation_test_source_id or "GUI_SHELF",
                        self._pick_navigation_test_transport_level,
                        self._taught_posture_arm_name(
                            self._pick_navigation_test_transport_level
                        ),
                    )
                    if not sent:
                        self._fail_mock_pick_navigation_test("运输姿态 Action 下发失败")
                        return
                    self._pick_navigation_test_stage = "returning_transport"
                    self.pick_chain_status.setText(
                        "联调状态：抓取成功，正在回安全运输姿态"
                    )
                    self._append_log("模拟抓取成功，开始回安全运输姿态")
                elif not self._continue_after_pick():
                    self._fail_mock_pick_navigation_test("安全退避 Action 下发失败")
                    return
            elif state.pick_test_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.pick_test_message or state.pick_test_status
                )
        elif stage == "returning_transport":
            self.pick_chain_status.setText(
                f"联调状态：回运输姿态 {state.shelf_view_status} "
                f"{state.shelf_view_phase} / lift {state.shelf_view_lift_position:.4f}m"
            )
            if state.shelf_view_status == "succeeded":
                if not self._continue_after_pick():
                    self._fail_mock_pick_navigation_test("安全退避 Action 下发失败")
                    return
            elif state.shelf_view_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.shelf_view_message or state.shelf_view_status
                )
        elif stage == "moving_to_place_transition":
            self.pick_chain_status.setText(
                f"联调状态：放置过渡2 {state.shelf_view_status} "
                f"{state.shelf_view_phase} / 右夹爪保持抓取"
            )
            if state.shelf_view_status == "succeeded":
                destination = self._pick_navigation_test_destination
                if destination is None or not self.ros_node.send_shelf_view_goal(
                    destination.waypoint_id,
                    TEST9_PLACE_POSTURE_LEVEL,
                    "both",
                ):
                    self._fail_mock_pick_navigation_test("放置2姿态 Action 下发失败")
                    return
                self._pick_navigation_test_stage = "moving_to_place"
                self._append_log("放置过渡2到位，开始进入放置2；右夹爪继续保持抓取")
            elif state.shelf_view_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.shelf_view_message or state.shelf_view_status
                )
        elif stage == "moving_to_place":
            self.pick_chain_status.setText(
                f"联调状态：放置2 {state.shelf_view_status} "
                f"{state.shelf_view_phase} / 右夹爪保持抓取"
            )
            if state.shelf_view_status == "succeeded":
                if not self.ros_node.send_right_gripper_goal(
                    RIGHT_GRIPPER_RELEASE_POSITION_M
                ):
                    self._fail_mock_pick_navigation_test("右夹爪释放 Action 下发失败")
                    return
                self._pick_navigation_test_stage = "releasing_after_place"
                self.pick_chain_status.setText("联调状态：放置2到位，正在松开右夹爪")
                self._append_log("放置2到位，开始松开右夹爪")
            elif state.shelf_view_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.shelf_view_message or state.shelf_view_status
                )
        elif stage == "releasing_after_place":
            self.pick_chain_status.setText(
                f"联调状态：正在松开右夹爪 {state.right_gripper_status} / "
                f"{state.right_gripper_position * 1000:.1f} mm"
            )
            if state.right_gripper_status == "succeeded":
                destination = self._pick_navigation_test_destination
                if destination is None or not self.ros_node.send_shelf_view_goal(
                    destination.waypoint_id,
                    TEST9_MOVE_POSTURE_LEVEL,
                    "both",
                ):
                    self._fail_mock_pick_navigation_test("放置后 move 姿态 Action 下发失败")
                    return
                self._pick_navigation_test_stage = "returning_move_after_place"
                self._append_log("右夹爪已松开，放置成功，开始返回 move 姿态")
            elif state.right_gripper_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.right_gripper_message or state.right_gripper_status
                )
        elif stage == "returning_move_after_place":
            self.pick_chain_status.setText(
                f"联调状态：放置成功，返回 move 姿态 {state.shelf_view_status} "
                f"{state.shelf_view_phase}"
            )
            if state.shelf_view_status == "succeeded":
                self._pick_navigation_test_stage = "ready_to_place"
                if not self._continue_after_place_confirmation():
                    self._fail_mock_pick_navigation_test("move 到位后的交付台退避失败")
                    return
                self._append_log("放置后 move 姿态到位，开始从 C 点后退")
            elif state.shelf_view_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.shelf_view_message or state.shelf_view_status
                )
        elif stage == "retreating":
            self.retreat_result_status.setText(
                f"退避结果：{state.retreat_test_status} / "
                f"{state.retreat_test_distance:.3f} m"
            )
            if state.retreat_test_status == "succeeded":
                self._pick_navigation_test_at_grasp_clearance = False
                self.retreat_result_status.setStyleSheet("color:#5fd38d;")
                destination = self._pick_navigation_test_destination
                if destination is None or not self._dispatch_waypoint(destination):
                    self._fail_mock_pick_navigation_test("下一导航目标下发失败")
                    return
                self._pick_navigation_test_stage = "navigating"
                self.pick_chain_status.setText(
                    f"联调状态：退避完成，正在导航到 {destination.waypoint_id}"
                )
                self._append_log(
                    f"商品点直退 30cm 完成，发送下一导航目标 "
                    f"{destination.waypoint_id}"
                )
            elif state.retreat_test_status in ("failed", "rejected", "error"):
                self.retreat_result_status.setStyleSheet("color:#ff6b6b;")
                self._fail_mock_pick_navigation_test(
                    state.retreat_test_message or state.retreat_test_status
                )
        elif stage == "retreating_after_place":
            self.retreat_result_status.setText(
                f"退避结果：交付台 {state.retreat_test_status} / "
                f"{state.retreat_test_distance:.3f} m"
            )
            if state.retreat_test_status == "succeeded":
                flow_tab = getattr(self, "competition_flow_tab", None)
                if (
                    flow_tab is not None
                    and flow_tab.continue_order_after_delivery_retreat()
                ):
                    return
                return_target = self._pick_navigation_test_return_destination
                if return_target is None or not self._dispatch_waypoint(return_target):
                    self._fail_mock_pick_navigation_test("返回起点目标下发失败")
                    return
                self._pick_navigation_test_stage = "returning_to_start"
                self.retreat_result_status.setStyleSheet("color:#5fd38d;")
                self.pick_chain_status.setText(
                    f"联调状态：交付台退避完成，正在返回 {return_target.waypoint_id}"
                )
            elif state.retreat_test_status in ("failed", "rejected", "error"):
                self._fail_mock_pick_navigation_test(
                    state.retreat_test_message or state.retreat_test_status
                )
        elif stage in ("navigating", "returning_to_start"):
            destination = (
                self._pick_navigation_test_return_destination
                if stage == "returning_to_start"
                else self._pick_navigation_test_destination
            )
            target_id = destination.waypoint_id if destination is not None else "?"
            self.pick_chain_status.setText(
                f"联调状态：导航到 {target_id} / {state.action_status}"
            )
            failed = state.action_status in (
                "failed",
                "rejected",
                "error",
                "cancel_error",
                "cancel_timeout",
            ) or state.action_status.startswith("finished:")
            if failed:
                self._fail_mock_pick_navigation_test(state.action_status)
            elif self._active_target is None and self._goal_terminal_handled:
                outcome = self._last_waypoint_outcome
                if outcome is None or outcome[0] != target_id:
                    return
                flow_alignment_accepted = (
                    stage == "navigating"
                    and destination is not None
                    and destination.requires_fine_alignment
                    and state.shelf_align_status == "succeeded"
                )
                if not outcome[1] and not flow_alignment_accepted:
                    self._fail_mock_pick_navigation_test(outcome[2])
                    return
                if not outcome[1]:
                    self._append_log(
                        f"{target_id} 精对齐成功；路径精度统计为{outcome[2]}，"
                        "比赛流程继续执行放置"
                    )
                if stage == "navigating" and self._pick_navigation_test_return_destination:
                    if self._uses_test9_delivery_sequence(target_id):
                        if not self._start_test9_place_sequence():
                            self._fail_mock_pick_navigation_test(
                                "C 点放置过渡2姿态 Action 下发失败"
                            )
                        return
                    self._pick_navigation_test_stage = "ready_to_place"
                    self.pick_chain_status.setText(
                        f"联调状态：已在 {target_id} 完成精调，等待放置完成确认"
                    )
                    self.pick_chain_status.setStyleSheet("color:#5fd38d;")
                    self._append_log(f"到达并精调 {target_id}，等待操作员确认放置")
                    return
                self._pick_navigation_test_stage = "complete"
                self.pick_chain_status.setText(
                    f"联调状态：完成，已到达 {target_id}"
                )
                self.pick_chain_status.setStyleSheet("color:#5fd38d;")
                self._append_log(f"抓取交付导航链路完成：{target_id}")

    def _start_vr(self) -> None:
        if self.control_mode.currentIndex() != 1:
            QMessageBox.warning(self, "当前为全自主比赛", "请先将比赛控制方式切换为“VR 调试 / 遥控”。")
            return
        fullbody_ready = (
            "/controller_manager" in self.telemetry.node_names
            and "/move_group" in self.telemetry.node_names
        )
        if not fullbody_ready:
            QMessageBox.warning(self, "全身控制未就绪", "请先启动全身控制，并等待硬件与 MoveIt 状态变为在线。")
            return
        base_enabled = self.vr_base_control.isChecked()
        if base_enabled and not self._process_running("navigation"):
            QMessageBox.warning(
                self,
                "底盘安全链未运行",
                "VR 控制底盘前必须先启动定位导航。",
            )
            return
        joint_page = getattr(self, "joint_control_page", None)
        arm_teleop_enabled = not (
            joint_page is not None and joint_page.freedrive_running()
        )
        args = [
            "launch",
            "openarmx_integrated_bringup",
            "integrated_vr_teleop.launch.py",
            f"enable_joystick_control:={'true' if base_enabled else 'false'}",
            f"vr_chassis:={'true' if base_enabled else 'false'}",
            "enable_waist_control:=false",
            f"enable_arm_teleop:={'true' if arm_teleop_enabled else 'false'}",
        ]
        if self._start_process("vr", "VR 遥操作", "ros2", args):
            self._vr_arm_teleop_enabled = arm_teleop_enabled
            if not arm_teleop_enabled:
                self._append_log("拖动模式运行中：VR 仅接管立柱、头部和底盘，双臂 IK 已禁用")

    def _stop_vr(self) -> None:
        self._stop_process("vr")

    def _start_mapping(self) -> None:
        if self.fullbody_mode and (
            self._process_running("navigation") or self._process_running("vr")
        ):
            QMessageBox.warning(self, "定位导航或 VR 正在运行", "请先停止定位导航和 VR，再启动建图。")
            return
        super()._start_mapping()

    def _mapping_starts_robot_hardware(self) -> bool:
        return not (
            self.fullbody_mode and self._process_running("fullbody")
        )

    def _mapping_uses_external_hardware(self) -> bool:
        # The full-body runtime owns the real chassis and listens on the final
        # safe command topic.  Manual driving may use it directly while no
        # mapping/navigation process is competing for that topic.
        return self.fullbody_mode and self._process_running("fullbody")

    def _mapping_available(self) -> bool:
        if not self.fullbody_mode:
            return super()._mapping_available()
        return (
            self._process_running("fullbody")
            and not self._process_running("navigation")
            and "/controller_manager" in self.telemetry.node_names
        )

    @staticmethod
    def _stale_runtime_processes() -> list[str]:
        patterns = (
            "controller_manager/ros2_control_node",
            "moveit_ros_move_group/move_group",
            "openflex_grasp.detect_object_server",
            "openflex_grasp.graspnet_worker",
            "openflex_grasp.pick_object_server",
            "openflex_grasp.pick_sequence_server",
            "openflex_grasp.grasp_web_visualizer",
            "openarmx_task_executor/lib/openarmx_task_executor/task_executor",
            "openarmx_task_executor/lib/openarmx_task_executor/station_docking",
            "openarmx_task_executor/lib/openarmx_task_executor/shelf_fine_align",
            "openarmx_task_executor/lib/openarmx_task_executor/station_retreat",
            "openarmx_task_executor/lib/openarmx_task_executor/shopping_ticket_reader",
            "openarmx_task_executor/lib/openarmx_task_executor/shelf_view",
            "openarmx_task_executor/lib/openarmx_task_executor/competition_coordinator",
            "openarmx_integrated_moveit_config/lib/openarmx_integrated_moveit_config/wait_for_robot_state.py",
            "openarmx_integrated_moveit_config/lib/openarmx_integrated_moveit_config/ensure_controllers_ready.py",
            "openarmx_integrated_moveit_config/lib/openarmx_integrated_moveit_config/moveit_command_bridge.py",
            "openarmx_integrated_moveit_config/lib/openarmx_integrated_moveit_config/openflex_pick_task_executor",
            "openarmx_integrated_moveit_config/lib/openarmx_integrated_moveit_config/openflex_joint_posture_executor",
            "openflex_grasp_moveit/lib/openflex_grasp_moveit/cartesian_approach_executor",
            "openarmx_gravity_comp/lib/openarmx_gravity_comp/gravity_comp_node",
        )
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            return []
        stale = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if stripped and any(pattern in stripped for pattern in patterns):
                stale.append(stripped)
        return stale

    @staticmethod
    def _stale_navigation_processes() -> list[str]:
        patterns = (
            "livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node",
            "fast_livo/lib/fast_livo/fastlivo_mapping",
            "fast_livo/lib/fast_livo/lio_bridge_node",
            "icp_registration/lib/icp_registration/icp_registration_node",
            "/nav2_controller/controller_server",
            "/nav2_planner/planner_server",
            "/nav2_bt_navigator/bt_navigator",
            "/nav2_behaviors/behavior_server",
            "/nav2_smoother/smoother_server",
            "/nav2_waypoint_follower/waypoint_follower",
            "/nav2_velocity_smoother/velocity_smoother",
        )
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            return []
        stale = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if stripped and any(pattern in stripped for pattern in patterns):
                stale.append(stripped)
        return stale

    def _start_fullbody(self) -> None:
        if self._process_running("mapping") or self._process_running("navigation"):
            QMessageBox.warning(
                self,
                "工作流正在运行",
                "请先停止建图或定位导航，再启动全身控制。",
            )
            return
        runtime_owned_nodes = {
            "/move_group",
            "/moveit_command_bridge",
            "/openflex_pick_task_executor",
            "/openflex_detect_object",
            "/openflex_pick_object",
            "/openflex_graspnet_worker",
            "/openflex_pick_sequence",
            "/openflex_grasp_readonly_web",
            "/cartesian_approach_executor",
            "/openflex_joint_posture_executor",
            "/openarmx_competition_coordinator",
            "/openarmx_task_executor",
            "/station_docking",
            "/shelf_fine_align",
            "/station_retreat",
            "/shopping_ticket_reader",
            "/shelf_view",
            "/gravity_comp_node",
        }
        conflicting_nodes = sorted(
            runtime_owned_nodes.intersection(self.telemetry.node_names)
        )
        if conflicting_nodes:
            QMessageBox.warning(
                self,
                "检测到旧抓取运行栈",
                "以下节点已由控制台之外的进程启动：\n"
                + "\n".join(conflicting_nodes)
                + "\n\n请先停止旧 MoveIt/抓取进程，再启动全身控制。"
                "禁止两套 Action 服务端同时控制真实机械臂和夹爪。",
            )
            return
        stale_processes = self._stale_runtime_processes()
        if stale_processes:
            QMessageBox.warning(
                self,
                "检测到遗留运行进程",
                "ROS 图未必能发现已孤立的旧进程。请先停止以下进程组：\n"
                + "\n".join(stale_processes[:8]),
            )
            return
        if self._has_external_chassis_stack():
            self._warn_external_stack()
            return
        if not self._all_can_interfaces_up():
            QMessageBox.warning(
                self,
                "CAN 未就绪",
                "can0 到 can5 尚未全部启用。请先点击“启用全部 CAN”。",
            )
            return

        args = [
            "launch",
            self.runtime_launch_package,
            self.runtime_launch_file,
            f"use_moveit_rviz:={'true' if self.moveit_rviz.isChecked() else 'false'}",
            "start_shelf_view_server:=true",
        ]
        station_config = os.environ.get("OPENFLEX_STATION_CONFIG", "").strip()
        if station_config:
            args.append(f"station_config:={station_config}")
        try:
            mission_file = mission_path(self._map_paths())
        except Exception:
            mission_file = None
        if mission_file is not None and mission_file.is_file():
            args.append(f"station_mission:={mission_file}")
        competition_config = os.environ.get(
            "OPENFLEX_COMPETITION_CONFIG", ""
        ).strip()
        if competition_config:
            args.append(f"competition_config:={competition_config}")
        self._start_process("fullbody", "全身控制", "ros2", args)

    def _stop_fullbody(self) -> None:
        if self._process_running("mapping") or self._process_running("navigation"):
            QMessageBox.warning(
                self,
                "工作流仍在运行",
                "请先停止建图或定位导航，再停止全身控制。",
            )
            return
        self._stop_vr()
        self._stop_motion()
        self._stop_process("fullbody")

    def _start_navigation(self) -> None:
        if self.fullbody_mode and self._process_running("mapping"):
            QMessageBox.warning(self, "建图正在运行", "请先保存并停止建图，再启动定位导航。")
            return
        if self.fullbody_mode and not self._process_running("fullbody"):
            QMessageBox.warning(self, "全身控制未运行", "请先在“整机控制”页启动全身控制。")
            return
        if self.fullbody_mode and not (
            "/controller_manager" in self.telemetry.node_names
            and "/move_group" in self.telemetry.node_names
        ):
            QMessageBox.warning(
                self,
                "全身控制未就绪",
                "请等待全身硬件与 MoveIt 状态变为在线。",
            )
            return
        stale_nodes = sorted(
            NAVIGATION_RUNTIME_NODES.intersection(self.telemetry.node_names)
        )
        if stale_nodes:
            QMessageBox.warning(
                self,
                "检测到遗留导航节点",
                "上一轮定位导航尚未完全退出，请等待清理完成后再启动：\n"
                + "\n".join(stale_nodes),
            )
            return
        stale_processes = self._stale_navigation_processes()
        if stale_processes:
            QMessageBox.warning(
                self,
                "检测到遗留导航进程",
                "Livox、FastLIO、ICP 或 Nav2 仍在退出。请等待约 10 秒后重试：\n"
                + "\n".join(stale_processes[:8]),
            )
            return
        if not self.fullbody_mode and self._has_external_chassis_stack():
            self._warn_external_stack()
            return
        paths = self._map_paths()
        missing = [
            str(path)
            for path in (paths.pcd, paths.yaml, paths.pgm)
            if not path.is_file()
        ]
        if missing:
            QMessageBox.warning(self, "基准地图不完整", "\n".join(missing))
            return
        if self.auto_record.isChecked():
            self._start_recording()
        pose = (
            f"[{self.initial_x.value():.6f},{self.initial_y.value():.6f},0.0,"
            f"0.0,0.0,{self.initial_yaw.value():.6f}]"
        )
        use_navigation_rviz = self.fullbody_mode or self.nav_rviz.isChecked()
        args = [
            "launch",
            self.navigation_launch_package
            if self.fullbody_mode
            else self.competition_launch_package,
            self.navigation_launch_file
            if self.fullbody_mode
            else self.competition_launch_file,
            f"pcd_path:={paths.pcd}",
            f"map_yaml:={paths.yaml}",
            f"initial_pose:={pose}",
            f"use_rviz:={'true' if use_navigation_rviz else 'false'}",
            f"nav_controller_max_wheel_speed:={self.nav_speed.value():.3f}",
            f"nav_controller_wheel_accel_limit:={self.nav_accel.value():.3f}",
        ]
        if self.fullbody_mode:
            args.extend(
                [
                    "start_robot_hardware:=false",
                    "start_safe_odom_mux:=false",
                ]
            )
        process_label = "比赛定位导航" if self.fullbody_mode else "比赛基准导航"
        if self._start_process("navigation", process_label, "ros2", args):
            QTimer.singleShot(8000, self._dump_metadata_async)

    def _stop_navigation(self) -> None:
        self._cancel_mock_pick_navigation_test("定位导航被停止")
        if self.fullbody_mode:
            self._stop_vr()
        super()._stop_navigation()

    def _process_finished(self, key: str, code: int, status) -> None:
        super()._process_finished(key, code, status)
        if key == "can_enable":
            self.can_status.setText("CAN：已启用" if code == 0 else "CAN：启用失败")
        elif key == "can_disable":
            self.can_status.setText("CAN：已禁用" if code == 0 else "CAN：禁用失败")
        elif key == "motor_check":
            self.motor_status.setText("电机：检查通过" if code == 0 else "电机：存在异常")
            self.motor_status.setStyleSheet(
                f"color:{'#5fd38d' if code == 0 else '#ff6b6b'};"
            )
        elif key == "navigation":
            self._cancel_mock_pick_navigation_test("定位导航进程已退出")
            if self._process_running("vr"):
                self._stop_vr()
        elif key == "fullbody":
            if self._process_running("mapping"):
                self._stop_process("mapping")
            if self._process_running("navigation"):
                self._stop_navigation()
            if self._process_running("vr"):
                self._stop_vr()
        if self.fullbody_mode:
            self._update_fullbody_controls()

    @staticmethod
    def _set_runtime_label(label: QLabel, state: str, text: str) -> None:
        colors = {
            "online": "#5fd38d",
            "waiting": "#f1c75b",
            "offline": "#ff6b6b",
            "disabled": "#9a9a9a",
        }
        label.setText(text)
        label.setStyleSheet(f"color:{colors[state]};")

    def _update_fullbody_controls(self) -> None:
        if not self.fullbody_mode:
            return
        nodes = self.telemetry.node_names
        actions = self.telemetry.action_servers
        fullbody_process = self._process_running("fullbody")
        navigation_process = self._process_running("navigation")
        mapping_process = self._process_running("mapping")
        vr_process = self._process_running("vr")
        hardware_online = "/controller_manager" in nodes
        moveit_online = "/move_group" in nodes

        up_count = sum(self._can_interface_up(f"can{index}") for index in range(6))
        self._set_runtime_label(
            self.can_status,
            "online" if up_count == 6 else "offline",
            f"CAN：{up_count}/6 已启用",
        )
        if hardware_online and moveit_online:
            self._set_runtime_label(self.fullbody_status, "online", "整机：全身硬件与 MoveIt 在线")
        elif fullbody_process:
            self._set_runtime_label(self.fullbody_status, "waiting", "整机：启动中，等待硬件与 MoveIt")
        else:
            self._set_runtime_label(self.fullbody_status, "offline", "整机：未启动")

        self._update_robot_state_summary()

        autonomous = self.control_mode.currentIndex() == 0
        vr_arm_teleop_enabled = getattr(self, "_vr_arm_teleop_enabled", True)
        vr_node_online = (
            "/openarmx_teleop_vr_node" in nodes
            if vr_arm_teleop_enabled
            else "/pico_pose_bridge" in nodes
        )
        if autonomous:
            self._set_runtime_label(self.vr_status, "disabled", "VR：全自主模式禁用")
        elif vr_node_online:
            if vr_arm_teleop_enabled:
                mode = (
                    "双臂 / 立柱 / 头部 / 底盘"
                    if self.vr_base_control.isChecked()
                    else "双臂 / 立柱 / 头部"
                )
            else:
                arm_state = (
                    "双臂拖动"
                    if getattr(self, "joint_control_page", None) is not None
                    and self.joint_control_page.freedrive_running()
                    else "双臂未接管"
                )
                mode = (
                    f"立柱 / 头部 / 底盘，{arm_state}"
                    if self.vr_base_control.isChecked()
                    else f"立柱 / 头部，{arm_state}"
                )
            self._set_runtime_label(self.vr_status, "online", f"VR：在线（{mode}）")
        elif vr_process:
            self._set_runtime_label(self.vr_status, "waiting", "VR：启动中")
        else:
            self._set_runtime_label(self.vr_status, "offline", "VR：未启动")

        # Use this GUI node's ActionClient readiness for manipulation servers.
        # A ROS graph can contain the action name from clients alone, which is
        # not evidence that a provider is available.
        detect_backend_ready = (
            self.ros_node.detect_object_action_client.server_is_ready()
            if self.ros_node
            else "/detect_object" in actions
        )
        pick_backend_ready = (
            self.ros_node.pick_object_action_client.server_is_ready()
            if self.ros_node
            else "/pick_object" in actions
        )
        layer_states = {
            "hardware": (hardware_online, "在线", "未启动"),
            "moveit": (moveit_online, "在线", "未启动"),
            "navigation": ("/navigate_to_pose" in actions, "在线", "等待 Nav2"),
            "detection": (detect_backend_ready, "在线", "等待检测服务"),
            "pick": (pick_backend_ready, "在线", "等待 A/B 接入"),
            "shelf_view": (
                "/competition/move_to_shelf_view" in actions,
                "货架观察可用",
                "等待货架姿态服务",
            ),
            "mock_pick": (
                "/competition/mock_pick_object" in actions,
                "模拟成功可用",
                "等待模拟服务器",
            ),
            "shelf_traverse": (
                "/return_to_shelf_anchor" in actions,
                "在线",
                "等待横移服务器",
            ),
            "retreat": (
                "/retreat_from_station" in actions,
                "在线",
                "等待退避服务器",
            ),
            "place": ("/place_object" in actions, "在线", "等待 B 接入"),
            "task": ("/execute_transport" in actions, "在线", "等待任务执行器"),
            "vr": (vr_node_online, "在线", "未启动"),
        }
        for key, (online, online_text, offline_text) in layer_states.items():
            if key == "vr" and autonomous:
                self._set_runtime_label(self.layer_status_labels[key], "disabled", "全自主模式禁用")
            else:
                self._set_runtime_label(
                    self.layer_status_labels[key],
                    "online" if online else "waiting",
                    online_text if online else offline_text,
                )

        self.fullbody_start_button.setEnabled(
            not fullbody_process and not navigation_process and not mapping_process
        )
        self.fullbody_stop_button.setEnabled(
            fullbody_process and not navigation_process and not mapping_process
        )
        self.navigation_start_button.setEnabled(
            fullbody_process
            and hardware_online
            and moveit_online
            and not navigation_process
            and not mapping_process
        )
        self.navigation_stop_button.setEnabled(navigation_process)
        manual_drive_available = (
            fullbody_process
            and hardware_online
            and not navigation_process
        )
        self.teleop_enabled.setEnabled(manual_drive_available)
        if manual_drive_available:
            self.teleop_enabled.setToolTip(
                "遥控独立于建图运行；速度命令发送到整机底盘控制器"
            )
        elif navigation_process:
            self.teleop_enabled.setToolTip("定位导航运行时由 Nav2 控制底盘")
        elif not fullbody_process:
            self.teleop_enabled.setToolTip("请先在“整机控制”页启动全身控制")
        else:
            self.teleop_enabled.setToolTip("等待 /controller_manager 上线")
        if not manual_drive_available and self.teleop_enabled.isChecked():
            self.teleop_enabled.setChecked(False)
        map_selection_enabled = not mapping_process and not navigation_process
        self.map_root_edit.setEnabled(map_selection_enabled)
        self.map_name_combo.setEnabled(map_selection_enabled)
        self.competition_map_combo.setEnabled(map_selection_enabled)
        self.competition_map_refresh_button.setEnabled(map_selection_enabled)
        self.disable_can_button.setEnabled(not fullbody_process and not hardware_online)
        self.vr_base_control.setEnabled(not autonomous and not vr_process)
        pick_workflow_idle = self._pick_navigation_test_stage in (
            "idle",
            "complete",
            "failed",
        )
        workflow_ready = (
            navigation_process
            and self.telemetry.localization_ok
            and "/competition/move_to_shelf_view" in actions
            and self.pick_source_combo.count() >= 2
        )
        self.pick_prepare_button.setEnabled(workflow_ready and pick_workflow_idle)
        self.pick_chain_button.setEnabled(
            navigation_process
            and self.telemetry.localization_ok
            and "/competition/mock_pick_object" in actions
            and "/retreat_from_station" in actions
            and self.pick_source_combo.count() >= 2
            and self._pick_navigation_test_stage == "ready"
        )
        parameters_enabled = pick_workflow_idle
        self.pick_source_combo.setEnabled(parameters_enabled)
        self.pick_destination_combo.setEnabled(parameters_enabled)
        self.pick_shelf_level.setEnabled(parameters_enabled)
        self.pick_arm_combo.setEnabled(parameters_enabled)
        self._update_mock_pick_navigation_test()
        flow_tab = getattr(self, "competition_flow_tab", None)
        if flow_tab is not None:
            flow_tab.update_phase1_from_workflow_state()
            flow_tab._update_controls_enabled()
        vr_base_ready = not self.vr_base_control.isChecked() or navigation_process
        self.vr_start_button.setEnabled(
            not autonomous
            and hardware_online
            and moveit_online
            and vr_base_ready
            and not vr_process
        )
        self.vr_stop_button.setEnabled(vr_process)

    def _joint_motion_allowed(self) -> tuple[bool, str]:
        """Gate manual joint commands independently from mapping/navigation startup."""
        if self.ros_node is None:
            return False, "GUI ROS 节点未连接"
        if "/controller_manager" not in self.telemetry.node_names:
            return False, "请先启动全身控制"
        if not self.ros_node.fullbody_position_controllers_ready():
            return False, "等待双臂、升降柱和头部位置控制器"
        joint_age = self._age(time.monotonic(), "/joint_states")
        if joint_age is None or joint_age >= 1.0:
            return False, "等待新鲜的 /joint_states"
        if self._process_running("vr") and getattr(
            self, "_vr_arm_teleop_enabled", True
        ):
            return False, "VR 遥操作运行中，手动关节控制已锁定"
        if hasattr(self.ros_node, "competition_actions_active") and (
            self.ros_node.competition_actions_active()
        ):
            return False, "正在停止比赛自动 Action，完成后开放关节控制"
        if self._active_motion_phase != "idle":
            return False, "比赛运动任务运行中，手动关节控制已锁定"
        if self._pick_navigation_test_stage not in ("idle", "complete", "failed"):
            return False, "抓取联调运行中，手动关节控制已锁定"
        if (
            self.telemetry.base_linear_speed
            > self.profile.acceptance.stopped_linear_mps
            or self.telemetry.base_angular_speed
            > self.profile.acceptance.stopped_angular_radps
        ):
            return False, "等待底盘完全停止后开放关节控制"
        return True, "整机控制在线，可独立于建图和导航调整关节"

    @staticmethod
    def _joint_summary(telemetry, specs) -> str:
        values = []
        for spec in specs:
            position = telemetry.joint_positions.get(spec.name)
            if position is None:
                return "--"
            values.append(f"{math.degrees(position):+.1f}°")
        return "  ".join(values)

    def _update_robot_state_summary(self) -> None:
        age = self._age(time.monotonic(), "/joint_states")
        joint_online = age is not None and age < 1.0
        self._set_runtime_label(
            self.joint_state_status,
            "online" if joint_online else "offline",
            f"关节状态：{'在线' if joint_online else '离线'}"
            + (f"（{age:.1f}s）" if age is not None else ""),
        )
        controller_ready = bool(
            self.ros_node and self.ros_node.fullbody_position_controllers_ready()
        )
        self._set_runtime_label(
            self.position_controller_status,
            "online" if controller_ready else "waiting",
            "位置控制器：双臂 / 升降柱 / 头部全部在线"
            if controller_ready
            else "位置控制器：等待双臂 / 升降柱 / 头部",
        )
        self.left_arm_state.setText(
            "左臂 J1-J7：" + self._joint_summary(self.telemetry, LEFT_ARM_SPECS)
        )
        self.right_arm_state.setText(
            "右臂 J1-J7：" + self._joint_summary(self.telemetry, RIGHT_ARM_SPECS)
        )
        lift = self.telemetry.joint_positions.get(LIFT_SPEC.name)
        lift_velocity = self.telemetry.joint_velocities.get(LIFT_SPEC.name, 0.0)
        self.lift_state.setText(
            "升降柱：--"
            if lift is None
            else f"升降柱：{lift:+.3f} m / {lift_velocity:+.3f} m/s"
        )
        self.head_state.setText(
            "头部 Yaw/Pitch：" + self._joint_summary(self.telemetry, HEAD_SPECS)
        )
        self.base_state.setText(
            f"底盘：线速度 {self.telemetry.base_linear_speed:.3f} m/s / "
            f"角速度 {self.telemetry.base_angular_speed:.3f} rad/s / "
            f"控制器 {'在线' if '/controller_manager' in self.telemetry.node_names else '离线'}"
        )

    def _plot_goal_selected(self, x: float, y: float) -> None:
        self.station_x.setValue(x)
        self.station_y.setValue(y)
        self.control_tabs.setCurrentIndex(self.benchmark_tab_index)

    def _plot_pose_selected(self, x: float, y: float, yaw: float) -> None:
        self.station_x.setValue(x)
        self.station_y.setValue(y)
        self.station_yaw_deg.setValue(math.degrees(yaw))
        self.path_plot.set_selected_pose(x, y, yaw)
        self.control_tabs.setCurrentIndex(self.benchmark_tab_index)

    def _selected_waypoint(self) -> Optional[BenchmarkWaypoint]:
        row = self.waypoint_table.currentRow()
        if row < 0:
            return None
        waypoint_id = self.waypoint_table.item(row, 0).text()
        return self._mission_waypoints.get(waypoint_id)

    def _send_selected_waypoint(self) -> None:
        waypoint = self._selected_waypoint()
        if waypoint is None:
            QMessageBox.information(self, "请选择航点", "请先选择一个航点")
            return
        problems = self._benchmark_preflight_problems()
        if problems:
            QMessageBox.warning(self, "比赛导航未就绪", "\n".join(problems))
            return
        self._tour_active = False
        self._tour_queue.clear()
        self._dispatch_waypoint(waypoint)

    def _return_to_start(self) -> None:
        waypoint = self._start_waypoint
        if waypoint is None:
            QMessageBox.warning(self, "未配置起点", "配置中缺少 start_position 航点")
            return
        problems = self._benchmark_preflight_problems()
        if problems:
            QMessageBox.warning(self, "比赛导航未就绪", "\n".join(problems))
            return
        self._tour_active = False
        self._tour_queue.clear()
        self.waypoint_table.selectRow(self._waypoint_rows[waypoint.waypoint_id])
        self._dispatch_waypoint(waypoint)

    def _start_benchmark_tour(self) -> None:
        problems = self._benchmark_preflight_problems()
        if problems:
            QMessageBox.warning(
                self,
                "比赛导航未就绪",
                "\n".join(problems),
            )
            return
        tour_id = str(self.tour_combo.currentData())
        try:
            if tour_id == "__station_order__":
                sequence = list(self._mission_waypoints.values())
                self._active_settle_sec = 1.0
            else:
                tour = self.profile.tours[tour_id]
                sequence = [
                    self._mission_waypoints[waypoint_id]
                    for waypoint_id in tour.sequence
                ]
                self._active_settle_sec = tour.settle_sec
            self._tour_queue = sequence * self.tour_repeats.value()
        except (KeyError, ValueError) as exc:
            QMessageBox.critical(self, "巡游配置错误", str(exc))
            return
        if not self._tour_queue:
            QMessageBox.warning(self, "站点为空", "请先配置比赛站点")
            return
        self._tour_total = len(self._tour_queue)
        self._tour_completed = 0
        self._tour_active = True
        self._tour_waiting = False
        self.tour_status.setText(f"准备执行 0 / {self._tour_total}")
        self._append_log(
            f"开始基准巡游 {tour_id}，目标数 {self._tour_total}"
        )
        self._dispatch_next_waypoint()

    def _benchmark_preflight_problems(self) -> list[str]:
        if not self.ros_node:
            return ["ROS 节点未创建"]
        now = time.monotonic()
        problems = []
        if self.telemetry.action_status in ("sending", "running", "canceling"):
            problems.append("已有 NavigateToPose 目标正在执行")
        if not self.telemetry.localization_state_seen or not self.telemetry.localization_ok:
            problems.append("定位未稳定：需要 /localization_ok=true")
        if not self.telemetry.has_map_pose:
            problems.append("当前 base_link 地图位姿不可用")
        else:
            map_pose_age = self._age(now, "map->base_link")
            if map_pose_age is None or map_pose_age > 0.50:
                problems.append("当前 base_link 地图位姿超过 0.5 秒未更新")
        costmap = self.telemetry.global_costmap
        if costmap is None:
            problems.append("全局代价地图未就绪：等待 /global_costmap/costmap_raw")
        elif now - costmap.received_at > GLOBAL_COSTMAP_MAX_AGE_SEC:
            problems.append(
                "全局代价地图过期："
                f"{now - costmap.received_at:.1f} 秒未更新"
            )
        elif self.telemetry.has_map_pose:
            pose_problem = costmap_pose_problem(
                costmap,
                self.telemetry.map_x,
                self.telemetry.map_y,
            )
            if pose_problem:
                problems.append(f"起步位置不安全：{pose_problem}")
        scan_age = self._age(now, "/scan")
        if scan_age is None or scan_age > 0.50:
            problems.append("障碍感知未就绪：/scan 超过 0.5 秒无新数据")
        safety_cloud_age = self._age(now, "/livox/body_cloud_safety")
        if safety_cloud_age is None or safety_cloud_age > 0.50:
            problems.append(
                "碰撞感知未就绪：/livox/body_cloud_safety 超过 0.5 秒无新数据"
            )
        odom_age = self._age(now, "/odom_safe")
        if odom_age is None or odom_age > 0.50:
            problems.append("安全里程计未就绪：/odom_safe 超过 0.5 秒无新数据")
        for node_name, label in (
            ("/collision_monitor", "碰撞监控"),
            ("/cmd_vel_safety_gate", "定位速度安全门"),
            ("/localization_costmap_guard", "定位恢复清图"),
        ):
            if node_name not in self.telemetry.node_names:
                problems.append(f"缺少 {label} 节点 {node_name}")
        problems.extend(
            nav2_stack_readiness_problems(
                self.telemetry.node_names, self.telemetry.action_servers
            )
        )
        if not self.ros_node.nav_action_client.server_is_ready() and not any(
            "/navigate_to_pose" in problem for problem in problems
        ):
            problems.append("NavigateToPose Action 客户端尚未连接")
        return problems

    def _dispatch_next_waypoint(self) -> None:
        if not self._tour_active or self._tour_waiting:
            return
        if not self._tour_queue:
            self._tour_active = False
            self.tour_status.setText(
                f"巡游完成 {self._tour_completed} / {self._tour_total}"
            )
            self._append_log("基准巡游完成")
            return
        waypoint = self._tour_queue.pop(0)
        if not self._dispatch_waypoint(waypoint):
            self._tour_active = False

    def _dispatch_waypoint(
        self, waypoint: BenchmarkWaypoint, *, track_table: bool = True
    ) -> bool:
        if not self.ros_node:
            return False
        problems = self._benchmark_preflight_problems()
        if problems:
            QMessageBox.warning(self, "比赛导航未就绪", "\n".join(problems))
            return False
        reason = self._goal_rejection_reason(waypoint.x, waypoint.y, waypoint.yaw)
        if reason:
            QMessageBox.warning(self, "目标位姿不安全", reason)
            return False
        if not self.telemetry.localization_ok:
            QMessageBox.warning(self, "定位失效", "已禁止下发航点")
            return False
        if (
            waypoint.requires_fine_alignment
            and not self.ros_node.shelf_align_action_client.server_is_ready()
        ):
            QMessageBox.warning(
                self,
                "前沿精调未就绪",
                "该站点要求精调，但没有发现 /shelf_fine_align Action；"
                "未下发 Nav2 目标。",
            )
            return False
        if not self.ros_node.send_navigation_goal(waypoint.x, waypoint.y, waypoint.yaw):
            QMessageBox.warning(self, "Nav2 未就绪", "没有发现 /navigate_to_pose Action")
            return False
        self._active_target = waypoint
        self._active_target_row = (
            self._waypoint_rows.get(waypoint.waypoint_id) if track_table else None
        )
        self._active_is_map_goal = not track_table
        self._active_started_at = time.monotonic()
        self._stop_stable_since = None
        self._active_max_cte = 0.0
        self._active_max_icp_step = 0.0
        self._active_max_icp_yaw = 0.0
        self._active_safety_interventions = 0
        self._safety_intervention_active = False
        self._goal_terminal_handled = False
        self._active_motion_phase = "navigation"
        self._last_waypoint_outcome = None
        if self._active_target_row is not None:
            self.waypoint_table.item(self._active_target_row, 6).setText("运行中")
        if self._active_is_map_goal:
            self.map_goal_status.setText(
                f"运行中：X {waypoint.x:.3f} m / Y {waypoint.y:.3f} m / "
                f"Yaw {math.degrees(waypoint.yaw):.1f} deg"
            )
        self.tour_status.setText(
            f"目标 {self._tour_completed + 1} / {max(self._tour_total, 1)}: "
            f"{waypoint.waypoint_id}"
        )
        self._append_log(
            f"基准航点 {waypoint.waypoint_id}: "
            f"x={waypoint.x:.3f}, y={waypoint.y:.3f}, yaw={waypoint.yaw:.3f}"
        )
        return True

    def _goal_rejection_reason(self, x: float, y: float, yaw: float) -> str:
        try:
            paths = self._map_paths()
            safety = GoalSafetyMap(
                read_pgm(paths.pgm),
                parse_map_yaml(paths.yaml),
                self._goal_safety_geometry,
            )
            return safety.rejection_reason(x, y, yaw)
        except Exception as exc:
            return f"无法验证目标安全边界: {exc}"

    def _stop_benchmark(self) -> None:
        self._cancel_mock_pick_navigation_test("运动任务被停止")
        stopped_map_goal = self._active_is_map_goal
        self._tour_active = False
        self._tour_waiting = False
        self._tour_queue.clear()
        self._active_target = None
        self._active_target_row = None
        self._active_is_map_goal = False
        self._goal_terminal_handled = True
        self._active_motion_phase = "idle"
        if self.ros_node:
            self.ros_node.cancel_shelf_alignment_goal()
            self.ros_node.cancel_shelf_view_goal()
        self._cancel_goal()
        self._stop_motion()
        self.tour_status.setText("已停止")
        if stopped_map_goal:
            self.map_goal_status.setText("临时测点已取消")
        self._append_log("已停止比赛基准巡游")

    def _reset_results(self) -> None:
        self._pass_count = 0
        self._fail_count = 0
        self.result_status.setText("通过 0 / 失败 0")
        self.map_goal_status.setText("尚未发送临时测点")
        for row in range(self.waypoint_table.rowCount()):
            self.waypoint_table.item(row, 6).setText("未测试")
            self.waypoint_table.item(row, 7).setText("--")
            self.waypoint_table.item(row, 8).setText("--")
            self.waypoint_table.item(row, 9).setText("--")

    def _update_telemetry(self) -> None:
        super()._update_telemetry()
        state = self.telemetry
        if state.localization_state_seen:
            self.localization_status.setText(
                "定位：重定位成功" if state.localization_ok else "定位：禁止运动"
            )
            self.localization_status.setStyleSheet(
                f"color:{'#5fd38d' if state.localization_ok else '#ff6b6b'};"
            )

        if self._start_waypoint is None:
            self.start_readiness_status.setText(
                "任务起点：未保存（不影响本次重定位和导航）"
            )
            self.start_readiness_status.setStyleSheet("color:#a0a0a0;")
        elif state.localization_ok and state.has_map_pose:
            start = self._start_waypoint
            position_error = math.hypot(state.map_x - start.x, state.map_y - start.y)
            yaw_error = abs(normalize_angle(state.map_yaw - start.yaw))
            stopped = (
                state.base_linear_speed <= self.profile.acceptance.stopped_linear_mps
                and state.base_angular_speed
                <= self.profile.acceptance.stopped_angular_radps
            )
            ready = (
                position_error <= self.profile.acceptance.goal_position_m
                and yaw_error <= self.profile.acceptance.goal_yaw_rad
                and stopped
                and state.global_costmap is not None
                and not costmap_pose_problem(
                    state.global_costmap,
                    state.map_x,
                    state.map_y,
                )
            )
            if ready:
                self.start_readiness_status.setText(
                    f"任务起点参考：当前已在起点（{position_error * 100:.1f}cm / "
                    f"{math.degrees(yaw_error):.1f}°）"
                )
                self.start_readiness_status.setStyleSheet("color:#5fd38d;")
            else:
                self.start_readiness_status.setText(
                    f"任务起点参考：距 {start.waypoint_id} {position_error:.2f}m / "
                    f"{math.degrees(yaw_error):.1f}°"
                )
                self.start_readiness_status.setStyleSheet("color:#f1c75b;")
        else:
            self.start_readiness_status.setText(
                "任务起点参考：等待定位（不影响选择重定位参考点）"
            )
            self.start_readiness_status.setStyleSheet("color:#a0a0a0;")

        raw_norm = math.sqrt(sum(value * value for value in state.cmd_raw))
        safe_norm = math.sqrt(sum(value * value for value in state.cmd_safe_in))
        if raw_norm > 0.02 and safe_norm < raw_norm * 0.75:
            self.safety_status.setText("安全链：碰撞预测正在限速/停车")
            self.safety_status.setStyleSheet("color:#f1c75b;")
            if self._active_target is not None and not self._safety_intervention_active:
                self._active_safety_interventions += 1
                self._safety_intervention_active = True
        elif self._age(time.monotonic(), "/cmd_vel_safe_in") is not None:
            self.safety_status.setText("安全链：正常")
            self.safety_status.setStyleSheet("color:#5fd38d;")
            self._safety_intervention_active = False

        if self._active_motion_phase == "shelf_alignment":
            self.tour_status.setText(
                f"前沿精调 {state.shelf_align_phase or state.shelf_align_status}："
                f"净距 {state.shelf_clearance * 100:.1f}cm / "
                f"角差 {math.degrees(state.shelf_yaw_error):.1f}° / "
                f"稳定 {state.shelf_stable_samples} 帧"
            )

        if self._fine_align_preview_active:
            preview_status = state.shelf_align_status
            self.preview_alignment_status.setText(
                f"精调观测 {self._fine_align_preview_station_id}："
                f"{state.shelf_align_phase or preview_status} / "
                f"净距 {state.shelf_clearance * 100:.1f}cm / "
                f"角差 {math.degrees(state.shelf_yaw_error):.1f}° / "
                f"滤波 {state.shelf_stable_samples} 帧"
            )
            if preview_status == "succeeded":
                self.preview_alignment_status.setStyleSheet("color:#5fd38d;")
                self._append_log(
                    f"精调观测通过：{self._fine_align_preview_station_id} / "
                    f"净距 {state.shelf_clearance:.3f}m / "
                    f"角差 {math.degrees(state.shelf_yaw_error):.1f}°"
                )
                self._fine_align_preview_active = False
            elif preview_status in ("failed", "rejected", "error"):
                self.preview_alignment_status.setStyleSheet("color:#ff6b6b;")
                self._append_log(
                    f"精调观测失败：{self._fine_align_preview_station_id} / "
                    f"{state.shelf_align_message or preview_status}"
                )
                self._fine_align_preview_active = False

        if (
            self._active_target is not None
            and self._active_motion_phase == "navigation"
        ):
            if (
                state.has_map_pose
                and state.path
                and state.last_seen.get("/plan", 0.0) >= self._active_started_at
            ):
                cte, _ = nearest_path_error(
                    state.map_x, state.map_y, state.map_yaw, state.path
                )
                self._active_max_cte = max(self._active_max_cte, cte)
            if state.last_seen.get("map->odom", 0.0) >= self._active_started_at:
                self._active_max_icp_step = max(
                    self._active_max_icp_step, state.icp_step_xy
                )
                self._active_max_icp_yaw = max(
                    self._active_max_icp_yaw, state.icp_step_yaw
                )

        self._handle_benchmark_result()
        if self.fullbody_mode:
            self._update_fullbody_controls()

    def _handle_benchmark_result(self) -> None:
        if self._active_target is None or self._goal_terminal_handled:
            return
        status = (
            self.telemetry.shelf_align_status
            if self._active_motion_phase == "shelf_alignment"
            else self.telemetry.action_status
        )
        if status not in (
            "succeeded",
            "failed",
            "rejected",
            "error",
            "fine_align_unavailable",
        ) and not status.startswith("finished:"):
            return
        if status == "succeeded":
            acceptance = self.profile.acceptance
            odom_age = self._age(time.monotonic(), "/odom_safe")
            stopped = (
                odom_age is not None
                and odom_age <= 0.50
                and self.telemetry.base_linear_speed <= acceptance.stopped_linear_mps
                and self.telemetry.base_angular_speed
                <= acceptance.stopped_angular_radps
            )
            if not stopped:
                self._stop_stable_since = None
                return
            if self._stop_stable_since is None:
                self._stop_stable_since = time.monotonic()
                return
            if time.monotonic() - self._stop_stable_since < acceptance.stopped_hold_sec:
                return
            if (
                self._active_motion_phase == "navigation"
                and self._active_target.requires_fine_alignment
            ):
                yaw_only = (
                    self.profile.map_name == TEST9_TAUGHT_POSTURE_MAP
                    and self._pick_navigation_test_stage == "navigating_to_shelf"
                    and self._active_target.waypoint_id
                    == self._pick_navigation_test_source_id
                )
                scan_age = self._age(time.monotonic(), "/scan")
                safety_cloud_age = self._age(
                    time.monotonic(), "/livox/body_cloud_safety"
                )
                if (
                    scan_age is None
                    or scan_age > 0.50
                    or safety_cloud_age is None
                    or safety_cloud_age > 0.50
                    or not self.telemetry.localization_ok
                    or not self.ros_node
                    or not self.ros_node.send_shelf_alignment_goal(
                        self._active_target.waypoint_id,
                        self._active_target.yaw,
                        target_clearance=self._active_target.target_clearance_m,
                        clearance_tolerance=(
                            self._active_target.clearance_tolerance_m
                        ),
                        yaw_tolerance=(
                            self._active_target.fine_align_yaw_tolerance_rad
                        ),
                        timeout_sec=self._active_target.fine_align_timeout_sec,
                        yaw_only=yaw_only,
                    )
                ):
                    status = "fine_align_unavailable"
                else:
                    self._active_motion_phase = "shelf_alignment"
                    self._stop_stable_since = None
                    if self._active_target_row is not None:
                        self.waypoint_table.item(
                            self._active_target_row, 6
                        ).setText("前沿精调中")
                    alignment_detail = (
                        "仅校正角度 / 保持当前净距"
                        if yaw_only
                        else (
                            "目标净距 "
                            f"{self._active_target.target_clearance_m * 100:.0f}cm "
                            "/ 近距稳定判定"
                        )
                    )
                    self.tour_status.setText(
                        f"货架对齐：{self._active_target.waypoint_id} / "
                        f"{alignment_detail}"
                    )
                    self._append_log(
                        f"Nav2 到达 {self._active_target.waypoint_id}，"
                        + (
                            "开始货架面角度校正（保持当前前后距离）"
                            if yaw_only
                            else (
                                "开始激光前沿精调（"
                                f"{self._active_target.target_clearance_m * 100:.0f}cm / "
                                "近距稳定）"
                            )
                        )
                    )
                    return
        self._goal_terminal_handled = True
        waypoint = self._active_target
        self._active_target = None
        row = self._active_target_row
        is_map_goal = self._active_is_map_goal
        self._active_target_row = None
        self._active_is_map_goal = False
        completed_phase = self._active_motion_phase
        self._active_motion_phase = "idle"
        duration = time.monotonic() - self._active_started_at

        if status != "succeeded" or not self.telemetry.has_map_pose:
            failure_detail = status if status != "succeeded" else "缺少 map 位姿"
            if (
                completed_phase == "shelf_alignment"
                and self.telemetry.shelf_align_message
            ):
                failure_detail = (
                    f"{status}（{self.telemetry.shelf_align_message}）"
                )
            self._last_waypoint_outcome = (
                waypoint.waypoint_id,
                False,
                failure_detail,
            )
            self._fail_count += 1
            if row is not None:
                self.waypoint_table.item(row, 6).setText("失败")
                self.waypoint_table.item(row, 7).setText(failure_detail)
            if is_map_goal:
                self.map_goal_status.setText(f"临时测点失败：{status}")
            self.result_status.setText(
                f"通过 {self._pass_count} / 失败 {self._fail_count}"
            )
            self._tour_active = False
            self._tour_queue.clear()
            self.tour_status.setText(f"巡游失败：{waypoint.waypoint_id}")
            self._append_log(
                f"航点 {waypoint.waypoint_id} 失败: {failure_detail}"
            )
            if (
                self._pick_navigation_test_stage == "navigating_to_shelf"
                and waypoint.waypoint_id == self._pick_navigation_test_source_id
            ):
                self._fail_mock_pick_navigation_test(
                    f"货架导航/角度对齐失败：{failure_detail}"
                )
            return

        position_error = math.hypot(
            self.telemetry.map_x - waypoint.x,
            self.telemetry.map_y - waypoint.y,
        )
        yaw_error = abs(normalize_angle(self.telemetry.map_yaw - waypoint.yaw))
        evaluation = evaluate_goal(
            self.profile.acceptance, position_error, yaw_error
        )
        acceptance = self.profile.acceptance
        # Shelf alignment deliberately moves away from the coarse Nav2 pose.
        # Its successful Action result is authoritative: the server owns the
        # filtering, close-range hysteresis, and safety checks. Reapplying only
        # the nominal GUI tolerance here can reject a result the server safely
        # accepted near the lower clearance boundary.
        final_pose_passed = (
            True
            if completed_phase == "shelf_alignment"
            else evaluation.passed
        )
        run_passed = (
            final_pose_passed
            and self._active_max_cte <= acceptance.path_cross_track_m
            and self._active_max_icp_step <= acceptance.max_icp_step_m
            and self._active_max_icp_yaw <= acceptance.max_icp_step_yaw_rad
        )
        if run_passed:
            self._pass_count += 1
            result_text = "通过"
        else:
            self._fail_count += 1
            result_text = "超差"
        self._last_waypoint_outcome = (
            waypoint.waypoint_id,
            run_passed,
            result_text,
        )
        if row is not None:
            self.waypoint_table.item(row, 6).setText(result_text)
            self.waypoint_table.item(row, 7).setText(
                (
                    f"净距 {self.telemetry.shelf_clearance * 100:.1f}cm / "
                    f"精停角差 {math.degrees(self.telemetry.shelf_yaw_error):.1f}° / "
                    f"{duration:.1f}s"
                    if completed_phase == "shelf_alignment"
                    else f"{position_error * 100:.1f}cm / "
                    f"{math.degrees(yaw_error):.1f}° / {duration:.1f}s"
                )
            )
            self.waypoint_table.item(row, 8).setText(
                f"{self._active_max_cte * 100:.1f}cm"
            )
            self.waypoint_table.item(row, 9).setText(
                f"{self._active_max_icp_step * 100:.1f}cm / "
                f"{math.degrees(self._active_max_icp_yaw):.1f}°"
            )
        if is_map_goal:
            self.map_goal_status.setText(
                f"{result_text}：位置 {position_error * 100:.1f} cm / "
                f"朝向 {math.degrees(yaw_error):.1f} deg / "
                f"最大横差 {self._active_max_cte * 100:.1f} cm / "
                f"耗时 {duration:.1f} s"
            )
        self.result_status.setText(
            f"通过 {self._pass_count} / 失败 {self._fail_count}"
        )
        self._append_log(
            f"航点 {waypoint.waypoint_id} {result_text}: "
            f"{position_error * 100:.2f} cm, {math.degrees(yaw_error):.2f} deg, "
            f"max CTE {self._active_max_cte * 100:.2f} cm, "
            f"max ICP {self._active_max_icp_step * 100:.2f} cm / "
            f"{math.degrees(self._active_max_icp_yaw):.2f} deg, "
            f"safety interventions {self._active_safety_interventions}, {duration:.1f} s"
        )

        if (
            self._pick_navigation_test_stage == "navigating_to_shelf"
            and waypoint.waypoint_id == self._pick_navigation_test_source_id
        ):
            # For the hardware handoff workflow we only require the local
            # shelf-align action to complete before testing the downstream
            # taught-posture/pick/retreat chain. Keep the stricter benchmark
            # result above visible as a warning, but do not block pregrasp
            # during flow-only integration tests.
            flow_alignment_accepted = completed_phase == "shelf_alignment"
            if not run_passed and not flow_alignment_accepted:
                self._fail_mock_pick_navigation_test(f"货架导航/精停{result_text}")
                return
            self._rebase_shelf_traverse_targets_to_current_anchor()
            if not self.ros_node or not self.ros_node.send_shelf_view_goal(
                waypoint.waypoint_id,
                self._pick_navigation_test_level,
                self._taught_posture_arm_name(self._pick_navigation_test_level),
            ):
                self._fail_mock_pick_navigation_test("货架观察姿态 Action 下发失败")
                return
            self._pick_navigation_test_stage = "moving_to_pregrasp"
            if run_passed:
                self.pregrasp_result_status.setText(
                    f"观察姿态结果：正在进入第 {self._pick_navigation_test_level} 层姿态"
                )
                self.pick_chain_status.setText(
                    f"联调状态：精停完成，正在进入第 {self._pick_navigation_test_level} 层观察姿态"
                )
            else:
                self.pregrasp_result_status.setText(
                    "观察姿态结果：精停超差但继续流程联调 "
                    f"/ 净距 {self.telemetry.shelf_clearance * 100:.1f}cm "
                    f"/ 角差 {math.degrees(self.telemetry.shelf_yaw_error):.1f}°"
                )
                self.pick_chain_status.setText(
                    f"联调状态：精停超差但继续进入第 {self._pick_navigation_test_level} 层观察姿态"
                )
            self.pregrasp_result_status.setStyleSheet("color:#f1c75b;")
            self._append_log(
                f"货架精停{'通过' if run_passed else '超差但流程联调放行'}，"
                f"开始进入第 {self._pick_navigation_test_level} 层观察姿态"
            )
            return

        if self._tour_active:
            self._tour_completed += 1
            if not run_passed:
                self._tour_active = False
                self._tour_queue.clear()
                self.tour_status.setText(f"精度超差：{waypoint.waypoint_id}")
                return
            self._tour_waiting = True
            QTimer.singleShot(
                max(1, int(self._active_settle_sec * 1000)),
                self._resume_tour_after_settle,
            )

    def _resume_tour_after_settle(self) -> None:
        self._tour_waiting = False
        self._dispatch_next_waypoint()

    def closeEvent(self, event) -> None:
        joint_page = getattr(self, "joint_control_page", None)
        if joint_page is not None and joint_page.freedrive_running():
            answer = QMessageBox.question(
                self,
                "退出拖动模式",
                "拖动模式仍在运行。请托住机械臂，退出时将恢复原 Kp/Kd、"
                "控制模式和 MoveIt。继续退出？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            if not joint_page.shutdown_freedrive():
                QMessageBox.critical(
                    self,
                    "拖动模式未退出",
                    "等待 freedrive_node 恢复控制参数超时，控制台将保持打开。",
                )
                event.ignore()
                return
        super().closeEvent(event)


def main(args=None) -> None:
    profile_path = default_profile_path()
    profile = load_benchmark_profile(profile_path)
    rclpy.init(args=args)
    node = NavigationRosNode()
    ros_executor = RosExecutorThread(node)
    ros_executor.start()
    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    window = CompetitionMainWindow(node, profile, profile_path)
    window.show()
    exit_code = app.exec_()
    window._stop_motion()
    if getattr(window, "joint_control_page", None) is not None:
        window.joint_control_page.shutdown_freedrive()
    node.shutdown_manual_joint_control()
    ros_executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
