"""Qt dialog for safe, reversible editing of Nav2 PGM maps."""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
)

from .map_editor_core import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GridMap,
    edit_summary,
    fill_component,
    paint_circle,
    paint_line,
    parse_map_yaml,
    read_pgm,
    save_with_backup,
)


MODE_VALUES = {
    "清为空闲（白）": FREE,
    "标为障碍（黑）": OCCUPIED,
    "设为未知（灰）": UNKNOWN,
}
TOOL_VALUES = {
    "画笔": "brush",
    "直线": "line",
    "连通区域": "component",
}


class MapCanvas(QLabel):
    edit_started = pyqtSignal()
    edit_finished = pyqtSignal()
    content_changed = pyqtSignal()
    cursor_changed = pyqtSignal(int, int, int)
    status_changed = pyqtSignal(str)

    def __init__(self, grid: GridMap, parent=None):
        super().__init__(parent)
        self.grid = grid
        self.zoom = 4
        self.tool = "brush"
        self.target = FREE
        self.radius = 1
        self.line_start = None
        self.painting = False
        self.last_cell = None
        self.cursor_x = grid.width // 2
        self.cursor_y = grid.height // 2
        self.keyboard_mode = False
        self.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)
        self.setFocusPolicy(Qt.StrongFocus)
        self.render_map()

    def render_map(self) -> None:
        image = QImage(
            bytes(self.grid.pixels),
            self.grid.width,
            self.grid.height,
            self.grid.width,
            QImage.Format_Grayscale8,
        ).copy()
        pixmap = QPixmap.fromImage(image).scaled(
            self.grid.width * self.zoom,
            self.grid.height * self.zoom,
            Qt.IgnoreAspectRatio,
            Qt.FastTransformation,
        )
        self.setPixmap(pixmap)
        self.setFixedSize(pixmap.size())
        self.update()
        if self.keyboard_mode:
            self.cursor_changed.emit(self.cursor_x, self.cursor_y, self.grid.value(self.cursor_x, self.cursor_y))

    def set_grid(self, grid: GridMap) -> None:
        self.grid = grid
        self.line_start = None
        self.render_map()

    def set_zoom(self, zoom: int) -> None:
        self.zoom = max(1, min(12, zoom))
        self.render_map()

    def cancel_line(self) -> None:
        self.line_start = None
        self.update()

    def _cell(self, event) -> tuple[int, int] | None:
        x = event.x() // self.zoom
        y = event.y() // self.zoom
        if 0 <= x < self.grid.width and 0 <= y < self.grid.height:
            return int(x), int(y)
        return None

    def _paint(self, cell: tuple[int, int]) -> None:
        if cell == self.last_cell:
            return
        self.last_cell = cell
        if paint_circle(self.grid, cell[0], cell[1], self.radius, self.target):
            self.render_map()
            self.content_changed.emit()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.RightButton:
            self.cancel_line()
            self.status_changed.emit("已取消直线起点")
            return
        if event.button() != Qt.LeftButton:
            return
        cell = self._cell(event)
        if cell is None:
            return
        if self.tool == "brush":
            self.edit_started.emit()
            self.painting = True
            self.last_cell = None
            self._paint(cell)
        elif self.tool == "component":
            self.edit_started.emit()
            before = bytes(self.grid.pixels)
            changed = fill_component(self.grid, cell[0], cell[1], self.target)
            if changed > 2000:
                self.grid.pixels = bytearray(before)
                changed = 0
                self.render_map()
                self.status_changed.emit(
                    "该连通区域超过 2000 格，已拒绝填充；请改用画笔，避免误清整片未知区"
                )
            if changed:
                self.render_map()
                self.content_changed.emit()
                self.status_changed.emit(f"已修改连通区域，共 {changed} 格")
            self.edit_finished.emit()
        elif self.line_start is None:
            self.line_start = cell
            self.status_changed.emit(f"直线起点 {cell}；再次左击完成，右击取消")
            self.update()
        else:
            self.edit_started.emit()
            changed = paint_line(self.grid, self.line_start, cell, self.radius, self.target)
            self.line_start = None
            if changed:
                self.render_map()
                self.content_changed.emit()
                self.status_changed.emit(f"已绘制直线，共修改 {changed} 格")
            self.edit_finished.emit()

    def mouseMoveEvent(self, event) -> None:
        cell = self._cell(event)
        if cell is not None:
            self.cursor_changed.emit(cell[0], cell[1], self.grid.value(cell[0], cell[1]))
            if self.painting and event.buttons() & Qt.LeftButton:
                self._paint(cell)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.LeftButton and self.painting:
            self.painting = False
            self.last_cell = None
            self.edit_finished.emit()

    def wheelEvent(self, event) -> None:
        self.set_zoom(self.zoom + (1 if event.angleDelta().y() > 0 else -1))
        event.accept()

    def keyPressEvent(self, event) -> None:
        key = event.key()
        if key in (Qt.Key_Up, Qt.Key_W):
            self.cursor_y = max(0, self.cursor_y - 1)
            self.keyboard_mode = True
            self.cursor_changed.emit(self.cursor_x, self.cursor_y, self.grid.value(self.cursor_x, self.cursor_y))
            self.update()
        elif key in (Qt.Key_Down, Qt.Key_S):
            self.cursor_y = min(self.grid.height - 1, self.cursor_y + 1)
            self.keyboard_mode = True
            self.cursor_changed.emit(self.cursor_x, self.cursor_y, self.grid.value(self.cursor_x, self.cursor_y))
            self.update()
        elif key in (Qt.Key_Left, Qt.Key_A):
            self.cursor_x = max(0, self.cursor_x - 1)
            self.keyboard_mode = True
            self.cursor_changed.emit(self.cursor_x, self.cursor_y, self.grid.value(self.cursor_x, self.cursor_y))
            self.update()
        elif key in (Qt.Key_Right, Qt.Key_D):
            self.cursor_x = min(self.grid.width - 1, self.cursor_x + 1)
            self.keyboard_mode = True
            self.cursor_changed.emit(self.cursor_x, self.cursor_y, self.grid.value(self.cursor_x, self.cursor_y))
            self.update()
        elif key == Qt.Key_Space or key == Qt.Key_Return or key == Qt.Key_Enter:
            if self.keyboard_mode:
                if self.tool == "brush":
                    self.edit_started.emit()
                    self.last_cell = None
                    cell = (self.cursor_x, self.cursor_y)
                    if paint_circle(self.grid, cell[0], cell[1], self.radius, self.target):
                        self.render_map()
                        self.content_changed.emit()
                    self.edit_finished.emit()
                elif self.tool == "component":
                    self.edit_started.emit()
                    before = bytes(self.grid.pixels)
                    changed = fill_component(self.grid, self.cursor_x, self.cursor_y, self.target)
                    if changed > 2000:
                        self.grid.pixels = bytearray(before)
                        changed = 0
                        self.render_map()
                        self.status_changed.emit(
                            "该连通区域超过 2000 格，已拒绝填充；请改用画笔，避免误清整片未知区"
                        )
                    if changed:
                        self.render_map()
                        self.content_changed.emit()
                        self.status_changed.emit(f"已修改连通区域，共 {changed} 格")
                    self.edit_finished.emit()
                elif self.line_start is None:
                    self.line_start = (self.cursor_x, self.cursor_y)
                    self.status_changed.emit(f"直线起点 ({self.cursor_x}, {self.cursor_y})；移动后再次按空格完成，Esc取消")
                    self.update()
                else:
                    self.edit_started.emit()
                    changed = paint_line(self.grid, self.line_start, (self.cursor_x, self.cursor_y), self.radius, self.target)
                    self.line_start = None
                    if changed:
                        self.render_map()
                        self.content_changed.emit()
                        self.status_changed.emit(f"已绘制直线，共修改 {changed} 格")
                    self.edit_finished.emit()
        elif key == Qt.Key_Escape:
            if self.line_start is not None:
                self.cancel_line()
                self.status_changed.emit("已取消直线起点")
        elif key == Qt.Key_Plus or key == Qt.Key_Equal:
            self.set_zoom(self.zoom + 1)
        elif key == Qt.Key_Minus:
            self.set_zoom(self.zoom - 1)
        else:
            event.ignore()

    def mouseMoveEvent(self, event) -> None:
        cell = self._cell(event)
        if cell is not None:
            self.keyboard_mode = False
            self.cursor_changed.emit(cell[0], cell[1], self.grid.value(cell[0], cell[1]))
            if self.painting and event.buttons() & Qt.LeftButton:
                self._paint(cell)
            self.update()

        super().paintEvent(event)
        if self.line_start is None and not self.keyboard_mode:
            return
        painter = QPainter(self)
        if self.line_start is not None:
            painter.setPen(QPen(QColor("#ffcc33"), 2))
            x, y = self.line_start
            painter.drawEllipse(
                x * self.zoom,
                y * self.zoom,
                max(2, self.zoom),
                max(2, self.zoom),
            )
        if self.keyboard_mode:
            painter.setPen(QPen(QColor("#00ff00"), 2))
            painter.drawRect(
                self.cursor_x * self.zoom,
                self.cursor_y * self.zoom,
                self.zoom,
                self.zoom,
            )


class MapEditorDialog(QDialog):
    def __init__(
        self,
        pgm_path: Path,
        yaml_path: Path,
        parent=None,
        save_allowed: Optional[Callable[[], bool]] = None,
    ):
        super().__init__(parent)
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.grid = read_pgm(pgm_path)
        self.saved_pixels = bytes(self.grid.pixels)
        self.metadata = parse_map_yaml(yaml_path)
        self.undo_stack: list[bytes] = []
        self.redo_stack: list[bytes] = []
        self.pending_snapshot = None
        self.saved = False
        self.backup_path = None
        self.save_allowed = save_allowed

        self.setWindowTitle(f"编辑 Nav2 地图 — {pgm_path.parent.name}")
        self.resize(1080, 820)
        self.setMinimumSize(820, 640)
        self._build_ui()
        self._update_state()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        controls = QGridLayout()

        self.tool_combo = QComboBox()
        self.tool_combo.addItems(TOOL_VALUES)
        self.tool_combo.currentTextChanged.connect(self._settings_changed)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(MODE_VALUES)
        self.mode_combo.currentTextChanged.connect(self._settings_changed)
        self.radius_spin = QSpinBox()
        self.radius_spin.setRange(0, 20)
        self.radius_spin.setValue(1)
        self.radius_spin.setSuffix(" 格")
        self.radius_spin.valueChanged.connect(self._settings_changed)
        self.zoom_spin = QSpinBox()
        self.zoom_spin.setRange(1, 12)
        self.zoom_spin.setValue(4)
        self.zoom_spin.setSuffix("×")
        self.zoom_spin.valueChanged.connect(self._zoom_changed)

        controls.addWidget(QLabel("工具"), 0, 0)
        controls.addWidget(self.tool_combo, 0, 1)
        controls.addWidget(QLabel("内容"), 0, 2)
        controls.addWidget(self.mode_combo, 0, 3)
        controls.addWidget(QLabel("半径"), 0, 4)
        controls.addWidget(self.radius_spin, 0, 5)
        controls.addWidget(QLabel("缩放"), 0, 6)
        controls.addWidget(self.zoom_spin, 0, 7)

        self.undo_button = QPushButton("撤销")
        self.undo_button.clicked.connect(self._undo)
        self.redo_button = QPushButton("重做")
        self.redo_button.clicked.connect(self._redo)
        self.reset_button = QPushButton("恢复到已保存")
        self.reset_button.clicked.connect(self._reset)
        controls.addWidget(self.undo_button, 1, 0, 1, 2)
        controls.addWidget(self.redo_button, 1, 2, 1, 2)
        controls.addWidget(self.reset_button, 1, 4, 1, 2)
        root.addLayout(controls)

        warning = QLabel(
            "安全提示：把黑色/灰色改成白色会让全局静态地图认为该处可通行。"
            "实时 /scan 局部代价地图和 collision_monitor 仍会工作，但玻璃也可能无法被实时激光可靠检测。"
            "只清理已经在现场确认长期为空的区域。"
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#f1c75b; padding:6px; border:1px solid #806d2d;")
        root.addWidget(warning)

        self.canvas = MapCanvas(self.grid)
        self.canvas.edit_started.connect(self._begin_edit)
        self.canvas.edit_finished.connect(self._finish_edit)
        self.canvas.content_changed.connect(self._update_state)
        self.canvas.cursor_changed.connect(self._cursor_changed)
        self.canvas.status_changed.connect(self._set_status)
        scroll = QScrollArea()
        scroll.setWidget(self.canvas)
        scroll.setWidgetResizable(False)
        scroll.setAlignment(Qt.AlignCenter)
        root.addWidget(scroll, 1)

        self.cursor_label = QLabel()
        self.stats_label = QLabel()
        self.status_label = QLabel("左键或空格编辑；方向键/WASD移动；鼠标滚轮或+/-缩放；直线工具右键或Esc取消")
        root.addWidget(self.cursor_label)
        root.addWidget(self.stats_label)
        root.addWidget(self.status_label)

        actions = QHBoxLayout()
        actions.addStretch(1)
        self.save_button = QPushButton("保存并备份原图")
        self.save_button.setObjectName("primaryButton")
        self.save_button.clicked.connect(self._save)
        close_button = QPushButton("关闭")
        close_button.clicked.connect(self.reject)
        actions.addWidget(self.save_button)
        actions.addWidget(close_button)
        root.addLayout(actions)
        self._settings_changed()

    def _settings_changed(self) -> None:
        self.canvas.tool = TOOL_VALUES[self.tool_combo.currentText()]
        self.canvas.target = MODE_VALUES[self.mode_combo.currentText()]
        self.canvas.radius = self.radius_spin.value()
        if self.canvas.tool != "line":
            self.canvas.cancel_line()

    def _zoom_changed(self, value: int) -> None:
        self.canvas.set_zoom(value)

    def _begin_edit(self) -> None:
        self.pending_snapshot = bytes(self.grid.pixels)

    def _finish_edit(self) -> None:
        if self.pending_snapshot is None:
            return
        if self.pending_snapshot != bytes(self.grid.pixels):
            self.undo_stack.append(self.pending_snapshot)
            self.undo_stack = self.undo_stack[-100:]
            self.redo_stack.clear()
        self.pending_snapshot = None
        self._update_state()

    def _replace_pixels(self, pixels: bytes) -> None:
        self.grid.pixels = bytearray(pixels)
        self.canvas.set_grid(self.grid)
        self._update_state()

    def _undo(self) -> None:
        if not self.undo_stack:
            return
        self.redo_stack.append(bytes(self.grid.pixels))
        self._replace_pixels(self.undo_stack.pop())

    def _redo(self) -> None:
        if not self.redo_stack:
            return
        self.undo_stack.append(bytes(self.grid.pixels))
        self._replace_pixels(self.redo_stack.pop())

    def _reset(self) -> None:
        if bytes(self.grid.pixels) == self.saved_pixels:
            return
        self.undo_stack.append(bytes(self.grid.pixels))
        self.redo_stack.clear()
        self._replace_pixels(self.saved_pixels)

    def _cursor_changed(self, x: int, y: int, value: int) -> None:
        names = {OCCUPIED: "障碍", UNKNOWN: "未知", FREE: "空闲"}
        text = f"栅格 ({x}, {y})  值 {value}（{names.get(value, '灰度')}）"
        if self.metadata.resolution is not None and self.metadata.origin is not None:
            origin_x, origin_y, _ = self.metadata.origin
            world_x = origin_x + (x + 0.5) * self.metadata.resolution
            world_y = origin_y + (self.grid.height - y - 0.5) * self.metadata.resolution
            text += f"  世界坐标 ({world_x:.3f}, {world_y:.3f})"
        self.cursor_label.setText(text)

    def _set_status(self, text: str) -> None:
        self.status_label.setText(text)

    def _update_state(self) -> None:
        summary = edit_summary(self.saved_pixels, bytes(self.grid.pixels))
        counts = self.grid.counts()
        self.stats_label.setText(
            f"尺寸 {self.grid.width}×{self.grid.height} | "
            f"障碍 {counts[OCCUPIED]} | 未知 {counts[UNKNOWN]} | 空闲 {counts[FREE]} | "
            f"待保存修改 {summary['changed']} 格"
        )
        self.undo_button.setEnabled(bool(self.undo_stack))
        self.redo_button.setEnabled(bool(self.redo_stack))
        self.save_button.setEnabled(summary["changed"] > 0)

    def _save(self) -> None:
        if self.save_allowed is not None and not self.save_allowed():
            QMessageBox.critical(
                self,
                "导航正在使用地图",
                "检测到 map_server/controller_server 正在运行。请先停止导航，再保存地图。",
            )
            return
        current = bytes(self.grid.pixels)
        summary = edit_summary(self.saved_pixels, current)
        cleared = summary["occupied_to_free"] + summary["unknown_to_free"]
        if cleared:
            answer = QMessageBox.warning(
                self,
                "确认清除静态占用",
                f"将把 {summary['occupied_to_free']} 个障碍格和 "
                f"{summary['unknown_to_free']} 个未知格改为空闲。\n\n"
                "这会影响 Nav2 全局静态地图。请确认这些位置在真实环境中长期没有障碍物。",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        try:
            self.backup_path = save_with_backup(self.pgm_path, self.grid)
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return
        self.saved_pixels = current
        self.saved = True
        self.undo_stack.clear()
        self.redo_stack.clear()
        self._update_state()
        backup_text = str(self.backup_path) if self.backup_path else "无（原文件不存在）"
        QMessageBox.information(
            self,
            "地图已保存",
            f"已保存：{self.pgm_path}\n备份：{backup_text}\n\n"
            "如果导航已经启动，需要重启 map_server/导航流程才会加载新地图。",
        )

    def reject(self) -> None:
        if bytes(self.grid.pixels) != self.saved_pixels:
            answer = QMessageBox.question(
                self,
                "放弃未保存修改？",
                "地图仍有未保存修改，确定关闭？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        super().reject()
