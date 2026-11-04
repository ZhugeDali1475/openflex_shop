"""Read-only grasp backend panel for the competition console."""

from PyQt5.QtCore import Qt, QTimer, QUrl, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QCompleter,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QHBoxLayout,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .web_scene import QWebEngineView, WEBENGINE_IMPORT_ERROR
from .scene_client import SceneRosClient


class GraspDebugTab(QWidget):
    sku_catalog_loaded = pyqtSignal(object, str)

    def __init__(self, ros_node=None, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self._task_locked = False
        self._sku_catalog = {}
        self._sku_request_pending = False
        self.scene_client = None
        if ros_node is not None and hasattr(ros_node, "create_client"):
            self.scene_client = SceneRosClient(ros_node)
        self.sku_catalog_loaded.connect(self._apply_sku_catalog)
        layout = QHBoxLayout(self)
        splitter = QSplitter(Qt.Horizontal, self)
        control_page = QWidget(splitter)
        control_page.setMaximumWidth(410)
        control_layout = QVBoxLayout(control_page)
        backend = QGroupBox("抓取后端状态")
        form = QFormLayout(backend)
        self.backend_label = QLabel("real detect + real pick + mock place（放置未实现）")
        self.backend_label.setWordWrap(True)
        form.addRow("Backend", self.backend_label)
        self.worker_label = QLabel("等待 /detect_object、/pick_object 和两个常驻 worker")
        self.worker_label.setWordWrap(True)
        form.addRow("Worker", self.worker_label)
        control_layout.addWidget(backend)

        controls = QGroupBox("抓取调试参数")
        controls_form = QFormLayout(controls)
        self.arm_combo = QComboBox()
        self.arm_combo.addItems(["right", "left"])
        controls_form.addRow("手臂", self.arm_combo)
        self.sku_combo = QComboBox()
        self.sku_combo.setEditable(True)
        self.sku_combo.setInsertPolicy(QComboBox.NoInsert)
        self.sku_combo.lineEdit().setPlaceholderText("搜索并选择数据库 SKU")
        self.sku_combo.completer().setCaseSensitivity(Qt.CaseInsensitive)
        self.sku_combo.completer().setFilterMode(Qt.MatchContains)
        self.sku_combo.completer().setCompletionMode(QCompleter.PopupCompletion)
        # Compatibility alias for existing GUI automation.
        self.sku_edit = self.sku_combo.lineEdit()
        self.reload_sku_button = QPushButton("刷新")
        self.reload_sku_button.clicked.connect(self._reload_sku_catalog)
        sku_row = QWidget()
        sku_layout = QHBoxLayout(sku_row)
        sku_layout.setContentsMargins(0, 0, 0, 0)
        sku_layout.addWidget(self.sku_combo, 1)
        sku_layout.addWidget(self.reload_sku_button)
        controls_form.addRow("SKU", sku_row)
        self.qwen_prompt_edit = QLineEdit()
        self.qwen_prompt_edit.setReadOnly(True)
        self.qwen_prompt_edit.setPlaceholderText("选择 SKU 后自动读取数据库")
        controls_form.addRow("Qwen 提示词", self.qwen_prompt_edit)
        self.grasp_method_edit = QLineEdit("夹爪")
        self.grasp_method_edit.setReadOnly(True)
        controls_form.addRow("抓取方式", self.grasp_method_edit)
        self.initial_raise = self._height_spin()
        controls_form.addRow("预抓取前上抬", self.initial_raise)
        self.pregrasp_height = self._height_spin(30.0)
        controls_form.addRow("预抓取位抬高", self.pregrasp_height)
        self.grasp_height = self._height_spin(30.0)
        controls_form.addRow("抓取位抬高", self.grasp_height)
        self.retreat_height = self._height_spin(40.0)
        controls_form.addRow("回撤预抓取抬高", self.retreat_height)
        self.approach_distance = self._distance_spin()
        controls_form.addRow("预抓取到抓取距离", self.approach_distance)
        self.execute_check = QCheckBox("允许执行（默认关闭，现场安全确认后再开）")
        self.execute_check.setChecked(False)
        self.execute_check.setEnabled(True)
        self.execute_check.toggled.connect(self._execution_unlock_changed)
        controls_form.addRow("执行锁", self.execute_check)
        self.request_button = QPushButton("仅检测")
        self.request_button.clicked.connect(self._send_detect)
        controls_form.addRow(self.request_button)
        self.plan_button = QPushButton("仅规划")
        self.plan_button.clicked.connect(self._send_plan)
        controls_form.addRow(self.plan_button)
        self.execute_button = QPushButton("执行抓取")
        self.execute_button.setEnabled(False)
        self.execute_button.setStyleSheet(
            "QPushButton:enabled { background: #b3261e; color: white; font-weight: 700; }"
        )
        self.execute_button.clicked.connect(self._send_execute)
        controls_form.addRow(self.execute_button)
        self.cancel_button = QPushButton("取消")
        self.cancel_button.clicked.connect(self._cancel)
        controls_form.addRow(self.cancel_button)
        control_layout.addWidget(controls)
        self.status_label = QLabel("未发送请求。所有运动命令仍必须经过正式 ROS Action。")
        self.status_label.setWordWrap(True)
        self.status_label.setAlignment(Qt.AlignTop)
        control_layout.addWidget(self.status_label)
        control_layout.addStretch(1)
        splitter.addWidget(control_page)
        if QWebEngineView is None:
            visualizer = QLabel(f"抓取可视化需要 QtWebEngine：{WEBENGINE_IMPORT_ERROR}", splitter)
            visualizer.setAlignment(Qt.AlignCenter)
            visualizer.setWordWrap(True)
        else:
            visualizer = QWebEngineView(splitter)
            self._visualizer_url = QUrl("http://127.0.0.1:8765/")
            self._visualizer_loaded = False
            visualizer.loadFinished.connect(self._visualizer_load_finished)
            visualizer.setUrl(self._visualizer_url)
        splitter.addWidget(visualizer)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([360, 1180])
        self.visualizer = visualizer
        layout.addWidget(splitter)
        self.arm_combo.currentTextChanged.connect(self._invalidate_plan)
        self.sku_combo.currentTextChanged.connect(self._sku_changed)
        self.sku_combo.currentTextChanged.connect(self._invalidate_plan)
        for control in (
            self.initial_raise,
            self.pregrasp_height,
            self.grasp_height,
            self.retreat_height,
            self.approach_distance,
        ):
            control.valueChanged.connect(self._invalidate_plan)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start(200)
        self._visualizer_retry_timer = QTimer(self)
        self._visualizer_retry_timer.timeout.connect(self._retry_visualizer)
        self._visualizer_retry_timer.start(1500)
        self._sku_retry_timer = QTimer(self)
        self._sku_retry_timer.timeout.connect(self._request_sku_catalog)
        if self.scene_client is not None:
            self._sku_retry_timer.start(1000)
            self._request_sku_catalog()

    @staticmethod
    def _height_spin(default_mm: float = 0.0):
        control = QDoubleSpinBox()
        control.setRange(0.0, 200.0)
        control.setDecimals(1)
        control.setSingleStep(1.0)
        control.setValue(default_mm)
        control.setSuffix(" mm")
        return control

    @staticmethod
    def _distance_spin():
        control = QDoubleSpinBox()
        control.setRange(0.0, 300.0)
        control.setDecimals(1)
        control.setSingleStep(5.0)
        control.setValue(0.0)
        control.setSpecialValueText("自动")
        control.setSuffix(" mm")
        return control

    def _visualizer_load_finished(self, success: bool):
        self._visualizer_loaded = bool(success)
        if success and hasattr(self, "_visualizer_retry_timer"):
            self._visualizer_retry_timer.stop()

    def _retry_visualizer(self):
        if (
            QWebEngineView is not None
            and isinstance(self.visualizer, QWebEngineView)
            and not getattr(self, "_visualizer_loaded", False)
        ):
            self.visualizer.setUrl(self._visualizer_url)

    def _reload_sku_catalog(self):
        if self.scene_client is None:
            self.status_label.setText("数据库场景客户端不可用。")
            return
        self._sku_retry_timer.start(1000)
        self._request_sku_catalog()

    def _request_sku_catalog(self):
        if (
            self.scene_client is None
            or self._sku_request_pending
            or not self.scene_client.snapshot_service_ready
        ):
            return
        self._sku_request_pending = True
        future = self.scene_client.request_snapshot()

        def finished(value):
            try:
                snapshot = SceneRosClient.snapshot_from_response(value.result())
                self.sku_catalog_loaded.emit(snapshot["skus"], "")
            except Exception as error:
                self.sku_catalog_loaded.emit([], str(error))

        future.add_done_callback(finished)

    def _apply_sku_catalog(self, skus, error: str):
        self._sku_request_pending = False
        if error:
            self.status_label.setText(f"读取 SKU 数据库失败：{error}")
            return
        current = self.sku_combo.currentText().strip()
        self._sku_catalog = {str(item["sku"]): dict(item) for item in skus}
        self.sku_combo.blockSignals(True)
        self.sku_combo.clear()
        self.sku_combo.addItems(list(self._sku_catalog))
        if current in self._sku_catalog:
            self.sku_combo.setCurrentText(current)
        elif self.sku_combo.count():
            self.sku_combo.setCurrentIndex(0)
        self.sku_combo.blockSignals(False)
        self._sku_changed(self.sku_combo.currentText())
        self._sku_retry_timer.stop()

    def _sku_changed(self, sku: str):
        item = self._sku_catalog.get(str(sku).strip())
        self.qwen_prompt_edit.setText(
            str(item.get("qwen_grounding_prompt", "")) if item else ""
        )
        self.grasp_method_edit.setText(
            str(item.get("grasp_method", "夹爪")) if item else "夹爪"
        )

    def select_manual_pick(self, sku: str, arm_name: str = "right") -> bool:
        """Preselect a competition item without starting any grasp action."""
        normalized_sku = str(sku).strip()
        arm_index = self.arm_combo.findText(str(arm_name).strip().lower())
        if arm_index >= 0:
            self.arm_combo.setCurrentIndex(arm_index)
        sku_index = self.sku_combo.findText(normalized_sku, Qt.MatchFixedString)
        if sku_index >= 0:
            self.sku_combo.setCurrentIndex(sku_index)
        else:
            # Keep the requested value visible while the tab's independent
            # scene snapshot is still loading. _apply_sku_catalog preserves it
            # once the database SKU arrives.
            self.sku_combo.setCurrentText(normalized_sku)
        self._sku_changed(normalized_sku)
        return normalized_sku in self._sku_catalog

    def _invalidate_plan(self, *_args):
        if self.ros_node:
            invalidate_plan = getattr(self.ros_node, "invalidate_grasp_plan", None)
            if callable(invalidate_plan):
                invalidate_plan("debug")
            else:
                self.ros_node.grasp_debug_plan_result = None
        self.execute_check.setChecked(False)
        self.execute_button.setEnabled(False)

    def _execution_unlock_changed(self, checked: bool):
        if checked:
            self.status_label.setText(
                "真机执行已解锁：仅可执行当前已审查、未过期的单次计划。"
            )
        self._refresh_status()

    def _reject_selection(self, message: str):
        self.status_label.setText(message)
        if self.ros_node is not None:
            self.ros_node.grasp_debug_status = "invalid"
            self.ros_node.grasp_debug_message = message
        return None

    def _validate(self, require_prompt: bool = False):
        sku = self.sku_combo.currentText().strip()
        if not sku:
            return self._reject_selection("请先选择数据库 SKU。")
        if self.grasp_method_edit.text().strip() != "夹爪":
            return self._reject_selection(
                "该 SKU 配置为吸盘；当前比赛抓取链只支持夹爪。"
            )
        prompt = self.qwen_prompt_edit.text().strip()
        if require_prompt:
            if sku not in self._sku_catalog:
                return self._reject_selection("请选择数据库中存在的 SKU。")
            if not prompt:
                return self._reject_selection("该 SKU 没有配置 Qwen 专用提示词。")
        return sku, prompt

    def _send_detect(self):
        selection = self._validate(require_prompt=True)
        if selection and self.ros_node:
            sku, prompt = selection
            self.ros_node.send_grasp_detect_goal(
                sku, self.arm_combo.currentText(), prompt
            )

    def _send_plan(self):
        selection = self._validate()
        if selection and self.ros_node:
            sku, _prompt = selection
            self.execute_check.setChecked(False)
            self.ros_node.send_grasp_plan_goal(
                sku,
                self.arm_combo.currentText(),
                initial_raise_m=self.initial_raise.value() / 1000.0,
                pregrasp_height_offset_m=self.pregrasp_height.value() / 1000.0,
                grasp_height_offset_m=self.grasp_height.value() / 1000.0,
                retreat_height_offset_m=self.retreat_height.value() / 1000.0,
                approach_distance_m=self.approach_distance.value() / 1000.0,
            )

    def _send_execute(self):
        selection = self._validate()
        if not selection or not self.ros_node or not self.execute_check.isChecked():
            return
        sku, _prompt = selection
        arm = self.arm_combo.currentText()
        answer = QMessageBox.warning(
            self,
            "确认真机执行抓取",
            f"即将执行当前缓存计划。\n\n商品：{sku}\n手臂：{arm}\n"
            f"预抓取前上抬：{self.initial_raise.value():.1f} mm\n"
            f"预抓取位抬高：{self.pregrasp_height.value():.1f} mm\n"
            f"抓取位抬高：{self.grasp_height.value():.1f} mm\n"
            f"回撤预抓取抬高：{self.retreat_height.value():.1f} mm\n"
            f"预抓取到抓取距离："
            f"{'自动' if self.approach_distance.value() == 0.0 else f'{self.approach_distance.value():.1f} mm'}\n"
            "动作：预抓取 -> 直线进给 -> 夹持 -> 安全回撤\n"
            "底盘不会运动。执行前将重新验证商品、底盘和关节起始状态。\n\n"
            "确认工作区无人、急停可用并继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            self.execute_check.setChecked(False)
            return
        self.execute_check.setChecked(False)
        self.ros_node.send_grasp_execute_goal(sku, arm)

    def _cancel(self):
        if self.ros_node:
            self.ros_node.cancel_grasp_debug_goal()

    def _refresh_status(self):
        if not self.ros_node:
            return
        self._task_locked = any(
            handle is not None
            for handle in (
                self.ros_node.grasp_debug_goal_handle,
                self.ros_node.grasp_debug_pick_goal_handle,
                self.ros_node.grasp_debug_execute_goal_handle,
            )
        )
        detect = self.ros_node.detect_object_action_client.server_is_ready()
        pick = self.ros_node.pick_object_action_client.server_is_ready()
        sequence_plan = self.ros_node.plan_pick_action_client.server_is_ready()
        sequence_execute = self.ros_node.execute_pick_action_client.server_is_ready()
        pregrasp = self.ros_node.plan_pregrasp_action_client.server_is_ready()
        cartesian = self.ros_node.cartesian_approach_action_client.server_is_ready()
        self.worker_label.setText(
            f"Detect={'在线' if detect else '离线'} / "
            f"Pick={'在线' if pick else '离线'} / "
            f"Pregrasp={'在线' if pregrasp else '离线'} / "
            f"Cartesian={'在线' if cartesian else '离线'} / "
            f"执行={'在线' if sequence_execute else '离线'}"
        )
        self.status_label.setText(
            f"{self.ros_node.grasp_debug_status}: {self.ros_node.grasp_debug_message}"
        )
        plan_owner = str(
            getattr(self.ros_node, "grasp_debug_plan_owner", "") or ""
        )
        foreign_workflow = bool(plan_owner and plan_owner != "debug")
        self.request_button.setEnabled(
            detect and not self._task_locked and not foreign_workflow
        )
        self.plan_button.setEnabled(
            sequence_plan
            and pregrasp
            and cartesian
            and not self._task_locked
            and not foreign_workflow
        )
        planned = bool(
            self.ros_node.grasp_debug_plan_result
            and self.ros_node.grasp_debug_plan_result.plan_id
        )
        owned_plan = planned and (not plan_owner or plan_owner == "debug")
        self.execute_check.setEnabled(
            owned_plan and sequence_execute and not self._task_locked
        )
        if not owned_plan and self.execute_check.isChecked():
            self.execute_check.setChecked(False)
        self.execute_button.setEnabled(
            owned_plan
            and sequence_execute
            and self.execute_check.isChecked()
            and not self._task_locked
        )
