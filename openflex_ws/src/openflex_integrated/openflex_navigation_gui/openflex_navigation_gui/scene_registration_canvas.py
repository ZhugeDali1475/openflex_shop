"""Interactive QGraphicsView for manual shelf and delivery-table registration."""

from __future__ import annotations

import math

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QGraphicsItem,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
)

from .map_editor_core import GridMap
from .scene_registration_core import MapTransform
from .scene_store import SceneEntity


class MapOriginItem(QGraphicsItem):
    """Draw the ROS map-frame origin with physically scaled one-metre axes."""

    def __init__(self, transform: MapTransform, axis_length_m: float = 1.0) -> None:
        super().__init__()
        origin = QPointF(*transform.world_to_pixel(0.0, 0.0))
        world_x = QPointF(*transform.world_to_pixel(axis_length_m, 0.0))
        world_y = QPointF(*transform.world_to_pixel(0.0, axis_length_m))
        self.x_axis = world_x - origin
        self.y_axis = world_y - origin
        self.setPos(origin)
        self._label("map (0, 0)", QColor("#ffeb3b"), QPointF(5.0, 4.0))
        self._label("+X  1 m", QColor("#ef5350"), self.x_axis + QPointF(3.0, 1.0))
        self._label("+Y  1 m", QColor("#43a047"), self.y_axis + QPointF(3.0, 1.0))

    def _label(self, text: str, color: QColor, position: QPointF) -> None:
        label = QGraphicsSimpleTextItem(text, self)
        label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        label.setBrush(QBrush(color))
        font = label.font()
        font.setPointSize(8)
        font.setBold(True)
        label.setFont(font)
        label.setPos(position)

    def boundingRect(self) -> QRectF:
        xs = (0.0, self.x_axis.x(), self.y_axis.x())
        ys = (0.0, self.x_axis.y(), self.y_axis.y())
        margin = 8.0
        return QRectF(
            min(xs) - margin,
            min(ys) - margin,
            max(xs) - min(xs) + margin * 2,
            max(ys) - min(ys) + margin * 2,
        )

    @staticmethod
    def _draw_arrow(painter: QPainter, endpoint: QPointF, color: QColor) -> None:
        length = math.hypot(endpoint.x(), endpoint.y())
        if length <= 0.0:
            return
        unit_x = endpoint.x() / length
        unit_y = endpoint.y() / length
        base = endpoint - QPointF(unit_x * 6.0, unit_y * 6.0)
        normal = QPointF(-unit_y * 3.0, unit_x * 3.0)
        painter.setPen(QPen(color, 0))
        painter.setBrush(QBrush(color))
        painter.drawLine(QPointF(), endpoint)
        painter.drawPolygon(QPolygonF((endpoint, base + normal, base - normal)))

    def paint(self, painter: QPainter, option, widget=None) -> None:
        del option, widget
        painter.setPen(QPen(QColor("#ffeb3b"), 0))
        painter.setBrush(QBrush(QColor("#ffeb3b")))
        painter.drawEllipse(QPointF(), 2.8, 2.8)
        self._draw_arrow(painter, self.x_axis, QColor("#ef5350"))
        self._draw_arrow(painter, self.y_axis, QColor("#43a047"))


class RegistrationEntityItem(QGraphicsItem):
    HANDLE_RADIUS = 3.0
    HANDLE_OFFSET = 9.0

    def __init__(self, canvas: "SceneRegistrationCanvas", entity: SceneEntity) -> None:
        super().__init__()
        self.canvas = canvas
        self.entity = entity
        self.width_px = entity.size_x / canvas.map_transform.resolution
        self.height_px = entity.size_y / canvas.map_transform.resolution
        self._center = QPointF(self.width_px * 0.5, -self.height_px * 0.5)
        self._handle = QPointF(self.width_px * 0.5, -self.height_px - self.HANDLE_OFFSET)
        self._rotating = False
        self._updating = True
        self.setFlags(
            QGraphicsItem.ItemIsSelectable
            | QGraphicsItem.ItemIsMovable
            | QGraphicsItem.ItemSendsGeometryChanges
        )
        self.setAcceptHoverEvents(True)
        self.setTransformOriginPoint(self._center)
        self._name_label = self._label("", QColor("#ffffff"), self._center)
        self._axis_x_label = self._label("+X", QColor("#ef5350"), QPointF(8, 0))
        self._axis_y_label = self._label("+Y", QColor("#66bb6a"), QPointF(0, -8))
        self._face_0_label = self._label("F0", QColor("#ffffff"), QPointF(0, -self.height_px * 0.5))
        self._face_1_label = self._label(
            "F1", QColor("#ffffff"), QPointF(self.width_px, -self.height_px * 0.5)
        )
        self.setToolTip(self._tooltip())
        self.set_entity(entity)
        self._updating = False

    def _label(self, text: str, color: QColor, position: QPointF) -> QGraphicsSimpleTextItem:
        label = QGraphicsSimpleTextItem(text, self)
        label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        label.setBrush(QBrush(color))
        font = label.font()
        font.setPointSize(8)
        label.setFont(font)
        label.setPos(position)
        label.setZValue(2)
        return label

    def _tooltip(self) -> str:
        representatives = [
            *self.entity.face_0_representatives,
            *self.entity.face_1_representatives,
        ]
        return "\n".join([self.entity.name, *representatives])

    def boundingRect(self) -> QRectF:
        margin = self.HANDLE_OFFSET + self.HANDLE_RADIUS + 4
        return QRectF(
            -margin,
            -self.height_px - margin,
            self.width_px + margin * 2,
            self.height_px + margin * 2,
        )

    def shape(self):
        path = super().shape()
        path.addRect(QRectF(0, -self.height_px, self.width_px, self.height_px))
        path.addEllipse(self._handle, self.HANDLE_RADIUS, self.HANDLE_RADIUS)
        return path

    def _rotation_offset(self, rotation_degrees: float) -> QPointF:
        radians = math.radians(rotation_degrees)
        cosine = math.cos(radians)
        sine = math.sin(radians)
        delta_x = -self._center.x()
        delta_y = -self._center.y()
        return QPointF(
            self._center.x() + delta_x * cosine - delta_y * sine,
            self._center.y() + delta_x * sine + delta_y * cosine,
        )

    def set_entity(self, entity: SceneEntity) -> None:
        self.entity = entity
        self._name_label.setText(entity.name)
        shelf = entity.kind == "shelf"
        self._face_0_label.setVisible(shelf)
        self._face_1_label.setVisible(shelf)
        self._updating = True
        rotation = self.canvas.map_transform.world_yaw_to_scene_degrees(entity.yaw)
        self.setRotation(rotation)
        origin_x, origin_y = self.canvas.map_transform.world_to_pixel(
            entity.world_x, entity.world_y
        )
        offset = self._rotation_offset(rotation)
        self.setPos(origin_x - offset.x(), origin_y - offset.y())
        self.setToolTip(self._tooltip())
        self._updating = False
        self.update()

    def current_pose(self) -> tuple[float, float, float]:
        origin = self.mapToScene(QPointF(0.0, 0.0))
        world_x, world_y = self.canvas.map_transform.pixel_to_world(origin.x(), origin.y())
        yaw = self.canvas.map_transform.scene_degrees_to_world_yaw(self.rotation())
        return world_x, world_y, yaw

    def itemChange(self, change, value):
        result = super().itemChange(change, value)
        if change == QGraphicsItem.ItemSelectedHasChanged and bool(value):
            self.canvas.entity_selected.emit(self.entity.kind, self.entity.entity_id)
        elif change == QGraphicsItem.ItemPositionHasChanged and not self._updating:
            self.canvas._emit_pose(self, committed=False)
        return result

    def mousePressEvent(self, event) -> None:
        self.setSelected(True)
        self.canvas.edit_started.emit(self.entity.kind, self.entity.entity_id)
        if math.hypot(
            event.pos().x() - self._handle.x(), event.pos().y() - self._handle.y()
        ) <= self.HANDLE_RADIUS + 4:
            self._rotating = True
            self.setFlag(QGraphicsItem.ItemIsMovable, False)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if not self._rotating:
            super().mouseMoveEvent(event)
            return
        center = self.mapToScene(self._center)
        delta = event.scenePos() - center
        pointer_angle = math.degrees(math.atan2(delta.y(), delta.x()))
        handle_angle = math.degrees(math.atan2(
            self._handle.y() - self._center.y(),
            self._handle.x() - self._center.x(),
        ))
        self.setRotation(pointer_angle - handle_angle)
        self.canvas._emit_pose(self, committed=False)
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if self._rotating:
            self._rotating = False
            self.setFlag(QGraphicsItem.ItemIsMovable, True)
            self.canvas._emit_pose(self, committed=True)
            event.accept()
            return
        super().mouseReleaseEvent(event)
        self.canvas._emit_pose(self, committed=True)

    def hoverMoveEvent(self, event) -> None:
        near_handle = math.hypot(
            event.pos().x() - self._handle.x(), event.pos().y() - self._handle.y()
        ) <= self.HANDLE_RADIUS + 4
        self.setCursor(Qt.CrossCursor if near_handle else Qt.OpenHandCursor)
        super().hoverMoveEvent(event)

    def paint(self, painter: QPainter, option, widget=None) -> None:
        del option, widget
        shelf = self.entity.kind == "shelf"
        fill = QColor("#d9982f" if shelf else "#2f9e73")
        fill.setAlpha(105)
        border = QColor("#64b5f6") if self.isSelected() else QColor("#f2f2f2")
        painter.setPen(QPen(border, 0))
        painter.setBrush(QBrush(fill))
        painter.drawRect(QRectF(0, -self.height_px, self.width_px, self.height_px))

        painter.setPen(QPen(QColor("#ef5350"), 0))
        painter.setBrush(QBrush(QColor("#ef5350")))
        painter.drawEllipse(QPointF(0, 0), 1.8, 1.8)
        arrow_x = min(max(self.width_px * 0.45, 8.0), 24.0)
        arrow_y = min(max(self.height_px * 0.30, 8.0), 24.0)
        painter.drawLine(QPointF(0, 0), QPointF(arrow_x, 0))
        self._axis_x_label.setPos(QPointF(arrow_x + 1, 0))
        painter.setPen(QPen(QColor("#66bb6a"), 0))
        painter.drawLine(QPointF(0, 0), QPointF(0, -arrow_y))
        self._axis_y_label.setPos(QPointF(0, -arrow_y))

        painter.setPen(QPen(QColor("#64b5f6"), 0))
        painter.setBrush(QBrush(QColor("#20252a")))
        painter.drawLine(self._center, self._handle)
        painter.drawEllipse(self._handle, self.HANDLE_RADIUS, self.HANDLE_RADIUS)


class SceneRegistrationCanvas(QGraphicsView):
    entity_selected = pyqtSignal(str, int)
    edit_started = pyqtSignal(str, int)
    pose_preview = pyqtSignal(str, int, float, float, float)
    pose_committed = pyqtSignal(str, int, float, float, float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setScene(QGraphicsScene(self))
        self.map_transform: MapTransform | None = None
        self._map_item: QGraphicsPixmapItem | None = None
        self._map_origin_item: MapOriginItem | None = None
        self._entity_items: dict[tuple[str, int], RegistrationEntityItem] = {}
        self._panning = False
        self._pan_position = None
        self._zoom_factor = 1.0
        self.setMinimumSize(760, 540)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QColor("#17191c"))

    def set_map(self, grid: GridMap, transform: MapTransform) -> None:
        self.scene().clear()
        self._entity_items.clear()
        self.map_transform = transform
        image = QImage(
            bytes(grid.pixels), grid.width, grid.height, grid.width, QImage.Format_Grayscale8
        ).copy()
        self._map_item = self.scene().addPixmap(QPixmap.fromImage(image))
        self._map_item.setZValue(0)
        self._map_origin_item = MapOriginItem(transform)
        self._map_origin_item.setZValue(20)
        self.scene().addItem(self._map_origin_item)
        self.scene().setSceneRect(QRectF(0, 0, grid.width, grid.height))
        self.fit_map()

    def fit_map(self) -> None:
        if self._map_item is None:
            return
        self.resetTransform()
        self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
        self._zoom_factor = 1.0

    def zoom_by(self, factor: float) -> None:
        target = min(12.0, max(0.25, self._zoom_factor * factor))
        applied = target / self._zoom_factor
        if math.isclose(applied, 1.0):
            return
        self.scale(applied, applied)
        self._zoom_factor = target

    def zoom_in(self) -> None:
        self.zoom_by(1.25)

    def zoom_out(self) -> None:
        self.zoom_by(0.8)

    def set_entities(self, entities: list[SceneEntity]) -> None:
        if self.map_transform is None:
            return
        for item in self._entity_items.values():
            self.scene().removeItem(item)
        self._entity_items.clear()
        for entity in entities:
            item = RegistrationEntityItem(self, entity)
            item.setZValue(10)
            self.scene().addItem(item)
            self._entity_items[(entity.kind, entity.entity_id)] = item

    def refresh_entities(self, entities: list[SceneEntity]) -> None:
        keys = {(entity.kind, entity.entity_id) for entity in entities}
        if keys != set(self._entity_items):
            self.set_entities(entities)
            return
        for entity in entities:
            self._entity_items[(entity.kind, entity.entity_id)].set_entity(entity)

    def select_entity(self, kind: str, entity_id: int) -> None:
        item = self._entity_items.get((kind, entity_id))
        if item is not None:
            item.setSelected(True)
            self.centerOn(item)

    def _emit_pose(self, item: RegistrationEntityItem, committed: bool) -> None:
        world_x, world_y, yaw = item.current_pose()
        signal = self.pose_committed if committed else self.pose_preview
        signal.emit(item.entity.kind, item.entity.entity_id, world_x, world_y, yaw)

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if delta == 0:
            return
        self.zoom_by(1.2 if delta > 0 else 1 / 1.2)
        event.accept()

    def resizeEvent(self, event) -> None:
        keep_fitted = math.isclose(self._zoom_factor, 1.0)
        super().resizeEvent(event)
        if keep_fitted and self._map_item is not None:
            self.fit_map()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.RightButton:
            self._panning = True
            self._pan_position = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._panning and self._pan_position is not None:
            delta = event.pos() - self._pan_position
            self._pan_position = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self._panning and event.button() == Qt.RightButton:
            self._panning = False
            self._pan_position = None
            self.unsetCursor()
            event.accept()
            return
        super().mouseReleaseEvent(event)
