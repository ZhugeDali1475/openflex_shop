"""Read-only adapter for the teammate-owned supermarket shelf database.

The database is an external source of semantic inventory and nominal object
positions.  This module intentionally opens SQLite in read-only mode and does
not import the external repository, so the ROS task package cannot migrate or
otherwise mutate the competition database at startup.
"""

from contextlib import closing
from dataclasses import dataclass
import math
from pathlib import Path
import sqlite3
from typing import Iterable, List, Optional, Tuple
from urllib.parse import quote


@dataclass(frozen=True)
class InventorySlot:
    """One product instance resolved from ``shelf_inventory.db``."""

    slot_id: str
    sku: str
    shelf_id: int
    shelf_name: str
    face: int
    level: int
    y_cm: float
    width_cm: Optional[float]
    height_cm: Optional[float]
    image_dir: str
    world_x: float
    world_y: float
    world_z: float
    shelf_length: float


class ShelfInventoryDatabase:
    """Minimal schema-compatible, read-only view of supermarket_scene data."""

    DEFAULT_SHELF_LENGTH = 1.86
    DEFAULT_SHELF_WIDTH = 0.80
    DEFAULT_BOTTOM_CLEARANCE = 0.05
    DEFAULT_LEVEL_SPACING = 0.40
    DEFAULT_PANEL_THICK = 0.02
    DEFAULT_BACK_THICK = 0.005
    DEFAULT_SHELF_DEPTH_NORMAL = 0.30
    DEFAULT_SHELF_DEPTH_BOTTOM = 0.40

    def __init__(self, path: str):
        database_path = Path(path).expanduser()
        if not database_path.is_file():
            raise ValueError(f"shelf inventory database does not exist: {database_path}")
        self.path = str(database_path.resolve())
        self._validate_schema()

    def _connect(self) -> sqlite3.Connection:
        uri = f"file:{quote(self.path, safe='/')}?mode=ro"
        connection = sqlite3.connect(uri, uri=True, timeout=2.0)
        connection.row_factory = sqlite3.Row
        return connection

    @staticmethod
    def format_slot_id(shelf_id: int, face: int, level: int, y_cm: float) -> str:
        return f"{int(shelf_id)}-{int(face)}-{int(level)}-{float(y_cm):g}"

    @staticmethod
    def parse_slot_id(slot_id: str) -> Tuple[int, int, int, float]:
        parts = str(slot_id).split("-")
        if len(parts) != 4:
            raise ValueError(
                f"invalid inventory slot id {slot_id!r}; expected shelf-face-level-y_cm"
            )
        shelf_id, face, level = (int(parts[index]) for index in range(3))
        y_cm = float(parts[3])
        if shelf_id < 0 or face not in (0, 1) or level < 0 or y_cm < 0.0:
            raise ValueError(f"invalid inventory slot id {slot_id!r}")
        return shelf_id, face, level, y_cm

    @staticmethod
    def _columns(connection: sqlite3.Connection, table: str) -> set:
        return {str(row["name"]) for row in connection.execute(f"PRAGMA table_info({table})")}

    def _validate_schema(self) -> None:
        with closing(self._connect()) as connection:
            required = {
                "sku_catalog": {"sku"},
                "shelf_groups": {"id", "name", "world_x", "world_y", "yaw", "shelf_type_id"},
                "shelf_types": {
                    "id", "shelf_length", "shelf_width", "bottom_clearance",
                    "level_spacing", "panel_thick", "back_thick",
                    "shelf_depth_normal", "shelf_depth_bottom",
                },
                "shelf_inventory": {"shelf_id", "face", "level", "y_cm"},
            }
            for table, columns in required.items():
                actual = self._columns(connection, table)
                missing = sorted(columns - actual)
                if missing:
                    raise ValueError(
                        f"shelf inventory table {table!r} is missing columns: {missing}"
                    )
            inventory_columns = self._columns(connection, "shelf_inventory")
            if not ({"actual_sku", "sku"} & inventory_columns):
                raise ValueError(
                    "shelf_inventory requires current actual_sku or legacy sku"
                )
            if "height_cm" not in inventory_columns and "z_offset_cm" not in inventory_columns:
                raise ValueError(
                    "shelf_inventory requires current height_cm or legacy z_offset_cm"
                )

    def list_catalog_skus(self) -> List[str]:
        """Return the normalized SKU whitelist without mutating the database."""
        with closing(self._connect()) as connection:
            rows = connection.execute(
                "SELECT sku FROM sku_catalog ORDER BY sku"
            ).fetchall()
        return [
            value
            for row in rows
            if (value := str(row["sku"] or "").strip())
            and value.lower() != "unknown"
        ]

    def find_sku_slots(self, sku: str) -> List[InventorySlot]:
        """Return every exact-SKU instance with its nominal pose in ``map``."""

        sku = str(sku).strip()
        if not sku:
            return []
        with closing(self._connect()) as connection:
            columns = self._columns(connection, "shelf_inventory")
            height_expr = "si.height_cm" if "height_cm" in columns else "NULL"
            width_expr = "si.width_cm" if "width_cm" in columns else "NULL"
            image_expr = "si.image_dir" if "image_dir" in columns else "''"
            z_offset_expr = "si.z_offset_cm" if "z_offset_cm" in columns else "NULL"
            sku_column = "actual_sku" if "actual_sku" in columns else "sku"
            rows = connection.execute(
                f"""
                SELECT si.shelf_id, sg.name AS shelf_name,
                       sg.world_x, sg.world_y, sg.yaw,
                       si.face, si.level, si.y_cm, si.{sku_column} AS sku,
                       {width_expr} AS width_cm,
                       {height_expr} AS height_cm,
                       {z_offset_expr} AS z_offset_cm,
                       {image_expr} AS image_dir,
                       COALESCE(st.shelf_length, ?) AS shelf_length,
                       COALESCE(st.shelf_width, ?) AS shelf_width,
                       COALESCE(st.bottom_clearance, ?) AS bottom_clearance,
                       COALESCE(st.level_spacing, ?) AS level_spacing,
                       COALESCE(st.panel_thick, ?) AS panel_thick,
                       COALESCE(st.back_thick, ?) AS back_thick,
                       COALESCE(st.shelf_depth_normal, ?) AS shelf_depth_normal,
                       COALESCE(st.shelf_depth_bottom, ?) AS shelf_depth_bottom
                FROM shelf_inventory si
                JOIN shelf_groups sg ON sg.id = si.shelf_id
                LEFT JOIN shelf_types st ON st.id = sg.shelf_type_id
                WHERE si.{sku_column} = ?
                ORDER BY si.shelf_id, si.face, si.level, si.y_cm
                """,
                (
                    self.DEFAULT_SHELF_LENGTH,
                    self.DEFAULT_SHELF_WIDTH,
                    self.DEFAULT_BOTTOM_CLEARANCE,
                    self.DEFAULT_LEVEL_SPACING,
                    self.DEFAULT_PANEL_THICK,
                    self.DEFAULT_BACK_THICK,
                    self.DEFAULT_SHELF_DEPTH_NORMAL,
                    self.DEFAULT_SHELF_DEPTH_BOTTOM,
                    sku,
                ),
            ).fetchall()

        slots: List[InventorySlot] = []
        for row in rows:
            level = int(row["level"])
            face = int(row["face"])
            depth = (
                float(row["shelf_depth_bottom"])
                if level == 0
                else float(row["shelf_depth_normal"])
            )
            if face == 0:
                local_x = float(row["back_thick"]) / 2.0 + depth / 2.0
            else:
                local_x = (
                    float(row["shelf_width"])
                    - float(row["back_thick"]) / 2.0
                    - depth / 2.0
                )
            local_y = float(row["y_cm"]) / 100.0
            surface_z = (
                float(row["bottom_clearance"])
                + level * float(row["level_spacing"])
                + float(row["panel_thick"]) / 2.0
            )
            if row["height_cm"] is not None:
                local_z = surface_z + float(row["height_cm"]) / 200.0
            else:
                # Legacy schema stored the product-centre offset above the board.
                local_z = surface_z + float(row["z_offset_cm"] or 0.0) / 100.0

            yaw = float(row["yaw"])
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            world_x = float(row["world_x"]) + local_x * cos_yaw - local_y * sin_yaw
            world_y = float(row["world_y"]) + local_x * sin_yaw + local_y * cos_yaw
            y_cm = float(row["y_cm"])
            slots.append(
                InventorySlot(
                    slot_id=self.format_slot_id(row["shelf_id"], face, level, y_cm),
                    sku=str(row["sku"]),
                    shelf_id=int(row["shelf_id"]),
                    shelf_name=str(row["shelf_name"]),
                    face=face,
                    level=level,
                    y_cm=y_cm,
                    width_cm=(
                        None if row["width_cm"] is None else float(row["width_cm"])
                    ),
                    height_cm=(
                        None if row["height_cm"] is None else float(row["height_cm"])
                    ),
                    image_dir=str(row["image_dir"] or ""),
                    world_x=world_x,
                    world_y=world_y,
                    world_z=local_z,
                    shelf_length=float(row["shelf_length"]),
                )
            )
        return slots

    def select_sku_slot(
        self,
        sku: str,
        available_faces: Iterable[Tuple[int, int]],
    ) -> Optional[InventorySlot]:
        """Select a deterministic, mapped instance for a pick-order line.

        Lower levels are preferred for safer initial integration.  Within one
        level, the instance nearest the longitudinal centre of its shelf is
        preferred so a face-level approach waypoint has the best reach margin.
        """

        allowed = {(int(shelf_id), int(face)) for shelf_id, face in available_faces}
        candidates = [
            slot
            for slot in self.find_sku_slots(sku)
            if (slot.shelf_id, slot.face) in allowed
        ]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda slot: (
                slot.level,
                abs(slot.y_cm - slot.shelf_length * 50.0),
                slot.shelf_id,
                slot.face,
                slot.y_cm,
            ),
        )
