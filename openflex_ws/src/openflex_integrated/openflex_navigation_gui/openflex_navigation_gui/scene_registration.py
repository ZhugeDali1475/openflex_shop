"""Native Qt page for manual supermarket fixture registration on a ROS map."""

from __future__ import annotations

import math
from pathlib import Path

import yaml
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QStyle,
    QVBoxLayout,
    QWidget,
)

from .map_editor_core import parse_map_yaml, read_pgm
from .model import discover_map_names, resolve_map_paths
from .scene_client import SceneRosClient
from .scene_registration_canvas import SceneRegistrationCanvas
from .scene_registration_core import MapTransform, fingerprint_map_files
from .scene_store import SceneStore


class SceneRegistrationPage(QWidget):
    def __init__(
        self,
        ros_node=None,
        parent=None,
        *,
        map_root: str | Path | None = None,
        map_name: str = "",
    ) -> None:
        super().__init__(parent)
        self.store = SceneStore()
        self.scene_client = SceneRosClient(ros_node) if ros_node is not None else None
        self.grid = None
        self.map_transform = None
        self.yaml_path: Path | None = None
        self.pgm_path: Path | None = None
        self.map_name = ""
        self.map_fingerprint = ""
        self._pending_future = None
        self._pending_kind = ""
        self._edit_start_state = None
        self._updating_controls = False
        self._selected_key = None
        self._build_ui()

        self.future_timer = QTimer(self)
        self.future_timer.timeout.connect(self._poll_future)
        self.future_timer.start(50)
        self.scene_service_timer = QTimer(self)
        self.scene_service_timer.timeout.connect(self._retry_scene_service)
        self.scene_service_timer.start(500)
        if map_root is not None:
            self.set_map_selection(map_root, map_name, silent=True)
        elif self.scene_client is not None:
            QTimer.singleShot(0, self.refresh_scene)

    def _button(self, icon, tooltip: str, text: str = "") -> QPushButton:
        button = QPushButton(text)
        button.setIcon(self.style().standardIcon(icon))
        button.setToolTip(tooltip)
        return button

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        map_toolbar = QHBoxLayout()
        map_toolbar.addWidget(QLabel("地图根目录"))
        self.map_root_combo = QComboBox()
        self.map_root_combo.setEditable(True)
        self.map_root_combo.setMinimumWidth(280)
        self.map_root_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.map_root_combo.activated.connect(self._map_root_selected)
        if self.map_root_combo.lineEdit() is not None:
            self.map_root_combo.lineEdit().editingFinished.connect(
                self._map_root_selected
            )
        map_toolbar.addWidget(self.map_root_combo, 1)
        self.browse_root_button = self._button(
            QStyle.SP_DirOpenIcon, "选择地图根目录"
        )
        self.browse_root_button.clicked.connect(self.choose_map_root)
        map_toolbar.addWidget(self.browse_root_button)
        map_toolbar.addWidget(QLabel("地图"))
        self.map_name_combo = QComboBox()
        self.map_name_combo.setEditable(True)
        self.map_name_combo.setMinimumWidth(180)
        self.map_name_combo.activated.connect(self.load_selected_map)
        if self.map_name_combo.lineEdit() is not None:
            self.map_name_combo.lineEdit().editingFinished.connect(
                self.load_selected_map
            )
        map_toolbar.addWidget(self.map_name_combo)
        self.refresh_maps_button = self._button(
            QStyle.SP_BrowserReload, "刷新地图下拉列表"
        )
        self.refresh_maps_button.clicked.connect(
            lambda: self._refresh_map_choices(load_selected=True)
        )
        map_toolbar.addWidget(self.refresh_maps_button)
        self.open_map_button = self._button(QStyle.SP_DialogOpenButton, "选择 ROS 地图 YAML", "打开地图")
        self.open_map_button.clicked.connect(self.choose_map)
        map_toolbar.addWidget(self.open_map_button)
        root.addLayout(map_toolbar)

        toolbar = QHBoxLayout()
        self.refresh_button = self._button(QStyle.SP_BrowserReload, "重新获取数据库场景快照")
        self.refresh_button.clicked.connect(self.refresh_scene)
        self.undo_button = self._button(QStyle.SP_ArrowBack, "撤销")
        self.undo_button.clicked.connect(self.undo)
        self.redo_button = self._button(QStyle.SP_ArrowForward, "重做")
        self.redo_button.clicked.connect(self.redo)
        self.cancel_button = self._button(QStyle.SP_DialogCancelButton, "取消当前草稿", "取消草稿")
        self.cancel_button.clicked.connect(self.cancel_draft)
        self.apply_button = self._button(QStyle.SP_DialogApplyButton, "确认并原子提交布局", "确认布局")
        self.apply_button.clicked.connect(self.apply_layout)
        for button in (
            self.refresh_button,
            self.undo_button,
            self.redo_button,
            self.cancel_button,
            self.apply_button,
        ):
            toolbar.addWidget(button)
        self.zoom_out_button = QPushButton("−")
        self.zoom_out_button.setFixedWidth(32)
        self.zoom_out_button.setToolTip("缩小地图")
        self.zoom_out_button.clicked.connect(self.canvas_zoom_out)
        self.zoom_in_button = QPushButton("+")
        self.zoom_in_button.setFixedWidth(32)
        self.zoom_in_button.setToolTip("放大地图")
        self.zoom_in_button.clicked.connect(self.canvas_zoom_in)
        self.fit_map_button = QPushButton("适配")
        self.fit_map_button.setToolTip("完整显示地图")
        self.fit_map_button.clicked.connect(self.canvas_fit_map)
        toolbar.addSpacing(8)
        toolbar.addWidget(self.zoom_out_button)
        toolbar.addWidget(self.zoom_in_button)
        toolbar.addWidget(self.fit_map_button)
        toolbar.addStretch(1)
        self.map_scale_label = QLabel("比例 --")
        self.revision_label = QLabel("修订 --")
        self.status_label = QLabel("等待地图")
        toolbar.addWidget(self.map_scale_label)
        toolbar.addSpacing(12)
        toolbar.addWidget(self.revision_label)
        toolbar.addSpacing(12)
        toolbar.addWidget(self.status_label)
        root.addLayout(toolbar)

        splitter = QSplitter()
        self.canvas = SceneRegistrationCanvas()
        self.canvas.entity_selected.connect(self._select_entity)
        self.canvas.edit_started.connect(self._edit_started)
        self.canvas.pose_preview.connect(self._canvas_pose_preview)
        self.canvas.pose_committed.connect(self._canvas_pose_committed)
        splitter.addWidget(self.canvas)

        inspector = QWidget()
        inspector.setMinimumWidth(230)
        inspector.setMaximumWidth(300)
        inspector_layout = QVBoxLayout(inspector)
        inspector_layout.setContentsMargins(10, 4, 4, 4)
        inspector_layout.setSpacing(8)
        inspector_layout.addWidget(QLabel("实体"))
        self.entity_combo = QComboBox()
        self.entity_combo.currentIndexChanged.connect(self._combo_selected)
        inspector_layout.addWidget(self.entity_combo)
        self.entity_dimensions = QLabel("--")
        inspector_layout.addWidget(self.entity_dimensions)

        form = QFormLayout()
        self.world_x = self._pose_spin(" m")
        self.world_y = self._pose_spin(" m")
        self.yaw_degrees = self._pose_spin("°", minimum=-180.0, maximum=180.0, step=1.0)
        form.addRow("X", self.world_x)
        form.addRow("Y", self.world_y)
        form.addRow("Yaw", self.yaw_degrees)
        inspector_layout.addLayout(form)
        for control in (self.world_x, self.world_y, self.yaw_degrees):
            control.editingFinished.connect(self._pose_controls_changed)

        inspector_layout.addWidget(QLabel("face=0"))
        self.face_0_products = QLabel("--")
        self.face_0_products.setWordWrap(True)
        inspector_layout.addWidget(self.face_0_products)
        inspector_layout.addWidget(QLabel("face=1"))
        self.face_1_products = QLabel("--")
        self.face_1_products.setWordWrap(True)
        inspector_layout.addWidget(self.face_1_products)
        inspector_layout.addStretch(1)
        splitter.addWidget(inspector)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([1200, 260])
        root.addWidget(splitter, 1)
        self._update_actions()

    def canvas_zoom_in(self) -> None:
        self.canvas.zoom_in()

    def canvas_zoom_out(self) -> None:
        self.canvas.zoom_out()

    def canvas_fit_map(self) -> None:
        self.canvas.fit_map()

    @staticmethod
    def _pose_spin(suffix: str, minimum: float = -1000.0,
                   maximum: float = 1000.0, step: float = 0.05) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setSuffix(suffix)
        return spin

    def choose_map_root(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图根目录",
            self.map_root_combo.currentText() or str(Path.home()),
        )
        if selected:
            self.set_map_selection(selected, "", silent=False)

    def _map_root_selected(self, *_args) -> None:
        self._refresh_map_choices(load_selected=True)

    def _refresh_map_choices(
        self,
        preferred_name: str = "",
        *,
        load_selected: bool = False,
    ) -> None:
        root = self.map_root_combo.currentText().strip()
        current = preferred_name.strip() or self.map_name_combo.currentText().strip()
        names = discover_map_names(root) if root else []
        selected = current if current in names else (names[0] if names else current)
        self.map_name_combo.blockSignals(True)
        self.map_name_combo.clear()
        self.map_name_combo.addItems(names)
        self.map_name_combo.setEditText(selected)
        self.map_name_combo.blockSignals(False)
        if load_selected and selected:
            self.load_selected_map()

    def set_map_selection(
        self,
        map_root: str | Path,
        map_name: str,
        *,
        silent: bool = True,
    ) -> None:
        root = str(Path(map_root).expanduser())
        self.map_root_combo.blockSignals(True)
        if self.map_root_combo.findText(root) < 0:
            self.map_root_combo.addItem(root)
        self.map_root_combo.setCurrentText(root)
        self.map_root_combo.blockSignals(False)
        self._refresh_map_choices(map_name, load_selected=False)
        if self.map_name_combo.currentText().strip():
            self.load_selected_map(silent=silent)

    def load_selected_map(self, *_args, silent: bool = False) -> None:
        root = self.map_root_combo.currentText().strip()
        name = self.map_name_combo.currentText().strip()
        if not root or not name:
            return
        try:
            paths = resolve_map_paths(root, name)
            if not paths.yaml.is_file():
                raise FileNotFoundError(f"地图 YAML 不存在：{paths.yaml}")
        except Exception as error:
            self.status_label.setText("地图不可用")
            if not silent:
                QMessageBox.warning(self, "地图选择无效", str(error))
            return
        if self.yaml_path == paths.yaml.resolve() and self.map_name == paths.name:
            return
        self.load_map(paths.yaml, map_name=paths.name, show_error=not silent)

    def choose_map(self) -> None:
        selected, _ = QFileDialog.getOpenFileName(
            self,
            "选择 ROS 地图 YAML",
            str(self.yaml_path.parent if self.yaml_path else Path.home()),
            "ROS Map (*.yaml *.yml)",
        )
        if selected:
            self.load_map(Path(selected))

    @staticmethod
    def _image_path_from_yaml(yaml_path: Path) -> Path:
        payload = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        image = str(payload.get("image", "")).strip()
        if not image:
            raise ValueError("地图 YAML 缺少 image 字段")
        image_path = Path(image).expanduser()
        return image_path if image_path.is_absolute() else yaml_path.parent / image_path

    @staticmethod
    def _map_name_from_yaml(yaml_path: Path) -> str:
        if yaml_path.name in {"map.yaml", "map.yml"}:
            return yaml_path.parent.name
        return yaml_path.stem

    def load_map(
        self,
        yaml_path: Path,
        *,
        map_name: str = "",
        show_error: bool = True,
    ) -> bool:
        try:
            yaml_path = Path(yaml_path).expanduser().resolve()
            pgm_path = self._image_path_from_yaml(yaml_path).resolve()
            grid = read_pgm(pgm_path)
            transform = MapTransform.from_map(grid, parse_map_yaml(yaml_path))
            fingerprint = fingerprint_map_files(yaml_path, pgm_path)
        except Exception as error:
            self.status_label.setText("地图加载失败")
            if show_error:
                QMessageBox.critical(self, "地图加载失败", str(error))
            return False
        self.yaml_path = yaml_path
        self.pgm_path = pgm_path
        self.map_name = map_name.strip() or self._map_name_from_yaml(yaml_path)
        self.grid = grid
        self.map_transform = transform
        self.map_fingerprint = fingerprint
        self.canvas.set_map(grid, transform)
        self.map_scale_label.setText(
            f"{transform.resolution:.3f} m/px · {grid.width}×{grid.height} px"
        )
        self.status_label.setText(self.map_name)
        self.refresh_scene()
        return True

    def refresh_scene(self) -> None:
        if self.scene_client is None:
            self.status_label.setText("未连接 ROS")
            return
        if self._pending_future is not None:
            return
        if not self.scene_client.snapshot_service_ready:
            self.status_label.setText("等待场景服务")
            return
        self.scene_service_timer.stop()
        self._pending_future = self.scene_client.request_snapshot(
            self.map_name, self.map_fingerprint
        )
        self._pending_kind = "snapshot"
        self.status_label.setText("正在读取场景")

    def _retry_scene_service(self) -> None:
        if self.store.has_snapshot:
            self.scene_service_timer.stop()
            return
        if (
            self.scene_client is not None
            and self._pending_future is None
            and self.scene_client.snapshot_service_ready
        ):
            self.refresh_scene()

    def apply_layout(self) -> None:
        if self.scene_client is None or not self.store.draft:
            return
        if self._pending_future is not None:
            return
        if not self.scene_client.layout_service_ready:
            self.status_label.setText("等待布局服务")
            return
        self._pending_future = self.scene_client.apply_layout(self.store.layout_request())
        self._pending_kind = "apply"
        self.status_label.setText("正在提交布局")

    def _poll_future(self) -> None:
        if self._pending_future is None or not self._pending_future.done():
            return
        future = self._pending_future
        kind = self._pending_kind
        self._pending_future = None
        self._pending_kind = ""
        try:
            response = future.result()
            if kind == "snapshot":
                self._accept_snapshot(self.scene_client.snapshot_from_response(response))
            elif not response.success:
                if response.revision_conflict:
                    QMessageBox.warning(self, "场景已更新", response.error)
                    self.refresh_scene()
                else:
                    raise RuntimeError(response.error)
            else:
                self.status_label.setText(f"已提交修订 {response.scene_revision}")
                self.refresh_scene()
        except Exception as error:
            self.status_label.setText("操作失败")
            QMessageBox.critical(self, "场景服务错误", str(error))
        self._update_actions()

    def _accept_snapshot(self, snapshot: dict) -> None:
        self.store.load_snapshot(snapshot)
        self.revision_label.setText(f"修订 {self.store.scene_revision}")
        if self.grid is None or self.map_transform is None or self.yaml_path is None:
            self.status_label.setText("场景已加载")
            self._update_actions()
            return
        self.store.begin_registration(
            self.map_name, self.map_fingerprint, self.map_transform
        )
        self.canvas.set_entities(self.store.draft)
        self._rebuild_entity_combo()
        self.status_label.setText(self.map_name)
        self._update_actions()

    def _rebuild_entity_combo(self) -> None:
        self.entity_combo.blockSignals(True)
        self.entity_combo.clear()
        display_numbers = {"shelf": 0, "delivery_table": 0}
        for entity in self.store.draft:
            display_numbers[entity.kind] += 1
            label = f"{'货架' if entity.kind == 'shelf' else '交付台'} {display_numbers[entity.kind]}  {entity.name}"
            self.entity_combo.addItem(label, (entity.kind, entity.entity_id))
        self.entity_combo.blockSignals(False)
        if self.store.draft:
            self.entity_combo.setCurrentIndex(0)
            first = self.store.draft[0]
            self._select_entity(first.kind, first.entity_id)

    def _combo_selected(self, index: int) -> None:
        if index < 0:
            return
        key = self.entity_combo.itemData(index)
        if key:
            self.canvas.select_entity(*key)
            self._select_entity(*key)

    def _select_entity(self, kind: str, entity_id: int) -> None:
        try:
            entity = self.store.entity(kind, entity_id)
        except KeyError:
            return
        self._selected_key = (kind, entity_id)
        index = self.entity_combo.findData(self._selected_key)
        if index >= 0 and self.entity_combo.currentIndex() != index:
            self.entity_combo.blockSignals(True)
            self.entity_combo.setCurrentIndex(index)
            self.entity_combo.blockSignals(False)
        self._update_pose_controls(entity)

    def _update_pose_controls(self, entity) -> None:
        self._updating_controls = True
        self.world_x.setValue(entity.world_x)
        self.world_y.setValue(entity.world_y)
        self.yaw_degrees.setValue(math.degrees(entity.yaw))
        self.entity_dimensions.setText(f"{entity.size_x:.2f} m × {entity.size_y:.2f} m")
        self.face_0_products.setText("\n".join(entity.face_0_representatives) or "--")
        self.face_1_products.setText("\n".join(entity.face_1_representatives) or "--")
        self._updating_controls = False

    def _pose_controls_changed(self) -> None:
        if self._updating_controls or self._selected_key is None:
            return
        self.store.update_pose(
            *self._selected_key,
            world_x=self.world_x.value(),
            world_y=self.world_y.value(),
            yaw=math.radians(self.yaw_degrees.value()),
        )
        self.canvas.refresh_entities(self.store.draft)
        self._update_actions()

    def _edit_started(self, kind: str, entity_id: int) -> None:
        del kind, entity_id
        self._edit_start_state = list(self.store.draft)

    def _canvas_pose_preview(self, kind: str, entity_id: int,
                             world_x: float, world_y: float, yaw: float) -> None:
        entity = self.store.update_pose(
            kind, entity_id, world_x=world_x, world_y=world_y, yaw=yaw,
            record_history=False,
        )
        if self._selected_key == (kind, entity_id):
            self._update_pose_controls(entity)

    def _canvas_pose_committed(self, kind: str, entity_id: int,
                               world_x: float, world_y: float, yaw: float) -> None:
        self._canvas_pose_preview(kind, entity_id, world_x, world_y, yaw)
        if self._edit_start_state is not None:
            self.store.commit_history_state(self._edit_start_state)
        self._edit_start_state = None
        self._update_actions()

    def undo(self) -> None:
        if self.store.undo():
            self.canvas.refresh_entities(self.store.draft)
            if self._selected_key:
                self._select_entity(*self._selected_key)
        self._update_actions()

    def redo(self) -> None:
        if self.store.redo():
            self.canvas.refresh_entities(self.store.draft)
            if self._selected_key:
                self._select_entity(*self._selected_key)
        self._update_actions()

    def cancel_draft(self) -> None:
        if (
            self.store.has_snapshot
            and self.yaml_path is not None
            and self.map_transform is not None
        ):
            self.store.begin_registration(
                self.map_name, self.map_fingerprint, self.map_transform
            )
            self.canvas.refresh_entities(self.store.draft)
            if self._selected_key:
                self._select_entity(*self._selected_key)
        self._update_actions()

    def _update_actions(self) -> None:
        pending = self._pending_future is not None
        self.undo_button.setEnabled(self.store.can_undo and not pending)
        self.redo_button.setEnabled(self.store.can_redo and not pending)
        self.cancel_button.setEnabled(bool(self.store.draft) and self.store.is_dirty and not pending)
        self.apply_button.setEnabled(
            bool(self.store.draft) and self.scene_client is not None and not pending
        )
        self.refresh_button.setEnabled(self.scene_client is not None and not pending)
