"""Map-backed pose picker for competition navigation stations."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Sequence

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
)

from .benchmark import BenchmarkWaypoint
from .competition_mission import grid_to_world, world_to_grid
from .goal_safety import GoalSafetyGeometry, GoalSafetyMap
from .map_editor_core import FREE, GridMap, MapMetadata, parse_map_yaml, read_pgm


class WaypointCanvas(QLabel):
    pose_changed = pyqtSignal(float, float, float, bool)

    def __init__(
        self,
        grid: GridMap,
        metadata: MapMetadata,
        stations: Sequence[BenchmarkWaypoint],
        pose: tuple[float, float, float],
        safety_map: GoalSafetyMap | None = None,
        parent=None,
    ):
        super().__init__(parent)
        self.grid = grid
        self.metadata = metadata
        self.stations = list(stations)
        self.pose = pose
        self.safety_map = safety_map
        self.zoom = 4
        self.drag_start: Optional[tuple[int, int]] = None
        self.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.setCursor(Qt.CrossCursor)
        self.setMouseTracking(True)
        self.render_map()

    def set_zoom(self, zoom: int) -> None:
        self.zoom = max(1, min(12, zoom))
        self.render_map()

    def _cell(self, event) -> tuple[int, int] | None:
        x = event.x() // self.zoom
        y = event.y() // self.zoom
        if 0 <= x < self.grid.width and 0 <= y < self.grid.height:
            return int(x), int(y)
        return None

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
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        if self.safety_map is not None:
            forbidden, boundary = self.safety_map.masks(self.pose[2])
            overlay = QImage(
                self.grid.width,
                self.grid.height,
                QImage.Format_ARGB32,
            )
            overlay.fill(Qt.transparent)
            for index, blocked in enumerate(forbidden):
                if not blocked:
                    continue
                x = index % self.grid.width
                y = index // self.grid.width
                overlay.setPixelColor(
                    x,
                    y,
                    QColor(255, 30, 30, 235 if boundary[index] else 55),
                )
            painter.drawImage(pixmap.rect(), overlay)
        grouped_labels: dict[tuple[int, int], list[str]] = {}
        for station in self.stations:
            cell = world_to_grid(station.x, station.y, self.grid, self.metadata)
            if cell is None:
                continue
            grouped_labels.setdefault(cell, []).append(station.waypoint_id)
            self._draw_pose(painter, cell, station.yaw, QColor("#2f80ed"), "")
        for cell, labels in grouped_labels.items():
            self._draw_label(painter, cell, " / ".join(labels), QColor("#2f80ed"))
        selected = world_to_grid(self.pose[0], self.pose[1], self.grid, self.metadata)
        if selected is not None:
            self._draw_pose(painter, selected, self.pose[2], QColor("#e25555"), "")
        painter.end()
        self.setPixmap(pixmap)
        self.setFixedSize(pixmap.size())

    def _draw_pose(
        self,
        painter: QPainter,
        cell: tuple[int, int],
        yaw: float,
        color: QColor,
        label: str,
    ) -> None:
        cx = (cell[0] + 0.5) * self.zoom
        cy = (cell[1] + 0.5) * self.zoom
        length = max(12.0, self.zoom * 4.0)
        ex = cx + math.cos(yaw) * length
        ey = cy - math.sin(yaw) * length
        painter.setPen(QPen(color, 3))
        painter.setBrush(color)
        painter.drawEllipse(int(cx - 4), int(cy - 4), 8, 8)
        painter.drawLine(int(cx), int(cy), int(ex), int(ey))
        if label:
            painter.drawText(int(cx + 6), int(cy - 6), label)

    def _draw_label(
        self,
        painter: QPainter,
        cell: tuple[int, int],
        label: str,
        color: QColor,
    ) -> None:
        cx = (cell[0] + 0.5) * self.zoom
        cy = (cell[1] + 0.5) * self.zoom
        painter.setPen(QPen(color, 1))
        painter.drawText(int(cx + 6), int(cy - 6), label)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.drag_start = self._cell(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() != Qt.LeftButton or self.drag_start is None:
            return
        end = self._cell(event)
        start = self.drag_start
        self.drag_start = None
        if end is None:
            return
        x, y = grid_to_world(start[0], start[1], self.grid, self.metadata)
        dx = end[0] - start[0]
        dy = start[1] - end[1]
        yaw = self.pose[2] if dx * dx + dy * dy < 4 else math.atan2(dy, dx)
        self.pose = (x, y, yaw)
        is_safe = (
            self.safety_map.is_safe(x, y, yaw)
            if self.safety_map is not None
            else self.grid.value(start[0], start[1]) >= FREE
        )
        self.pose_changed.emit(x, y, yaw, is_safe)
        self.render_map()


class WaypointPickerDialog(QDialog):
    def __init__(
        self,
        pgm_path: Path,
        yaml_path: Path,
        stations: Sequence[BenchmarkWaypoint],
        initial_pose: tuple[float, float, float],
        parent=None,
        safety_geometry: GoalSafetyGeometry | None = None,
    ):
        super().__init__(parent)
        self.grid = read_pgm(pgm_path)
        self.metadata = parse_map_yaml(yaml_path)
        if self.metadata.resolution is None or self.metadata.origin is None:
            raise ValueError("地图 YAML 缺少 resolution 或 origin")
        self.selected_pose = initial_pose
        self.safety_map = (
            GoalSafetyMap(self.grid, self.metadata, safety_geometry)
            if safety_geometry is not None
            else None
        )
        initial_cell = world_to_grid(
            initial_pose[0], initial_pose[1], self.grid, self.metadata
        )
        self.selected_is_free = (
            self.safety_map.is_safe(*initial_pose)
            if self.safety_map is not None
            else (
                initial_cell is not None
                and self.grid.value(initial_cell[0], initial_cell[1]) >= FREE
            )
        )
        self.setWindowTitle(f"选择比赛站点 - {pgm_path.parent.name}")
        self.resize(1040, 800)
        self.setMinimumSize(760, 600)

        root = QVBoxLayout(self)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("缩放"))
        zoom = QSpinBox()
        zoom.setRange(1, 12)
        zoom.setValue(4)
        zoom.setSuffix("x")
        controls.addWidget(zoom)
        controls.addWidget(
            QLabel("左键按住拖动：起点是车体中心，拖动方向是车头方向；短点按保留原方向")
        )
        if self.safety_map is not None:
            geometry = self.safety_map.geometry
            controls.addWidget(
                QLabel(
                    "红线内禁设目标（车体外沿到障碍至少 "
                    f"{geometry.clearance_m * 100:.0f} cm）"
                )
            )
        controls.addStretch(1)
        root.addLayout(controls)

        self.canvas = WaypointCanvas(
            self.grid, self.metadata, stations, initial_pose, self.safety_map
        )
        zoom.valueChanged.connect(self.canvas.set_zoom)
        self.canvas.pose_changed.connect(self._pose_changed)
        scroll = QScrollArea()
        scroll.setWidget(self.canvas)
        scroll.setWidgetResizable(False)
        scroll.setAlignment(Qt.AlignCenter)
        root.addWidget(scroll, 1)

        self.pose_label = QLabel()
        root.addWidget(self.pose_label)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)
        self.ok_button = buttons.button(QDialogButtonBox.Ok)
        self._update_pose_label()

    def _pose_changed(self, x: float, y: float, yaw: float, is_free: bool) -> None:
        self.selected_pose = (x, y, yaw)
        self.selected_is_free = is_free
        self._update_pose_label()

    def _update_pose_label(self) -> None:
        x, y, yaw = self.selected_pose
        state = "安全目标" if self.selected_is_free else "红线禁设区、障碍或未知区"
        self.pose_label.setText(
            f"X {x:.3f} m | Y {y:.3f} m | Yaw {math.degrees(yaw):.1f} deg | {state}"
        )
        self.pose_label.setStyleSheet(
            f"color:{'#5fd38d' if self.selected_is_free else '#ff6b6b'};"
        )
        self.ok_button.setEnabled(self.selected_is_free)
