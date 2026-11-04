import sqlite3
import tempfile
import unittest
from pathlib import Path

from openarmx_task_executor.shelf_inventory_db import ShelfInventoryDatabase


def create_database(path: str, legacy: bool = False) -> None:
    connection = sqlite3.connect(path)
    connection.executescript(
        """
        CREATE TABLE shelf_types (
            id INTEGER PRIMARY KEY, name TEXT, shelf_length REAL,
            shelf_width REAL, shelf_height REAL, num_levels INTEGER,
            bottom_clearance REAL, level_spacing REAL, panel_thick REAL,
            back_thick REAL, shelf_depth_normal REAL, shelf_depth_bottom REAL
        );
        CREATE TABLE shelf_groups (
            id INTEGER PRIMARY KEY, name TEXT, world_x REAL, world_y REAL,
            yaw REAL, shelf_type_id INTEGER
        );
        CREATE TABLE sku_catalog (sku TEXT PRIMARY KEY);
        """
    )
    if legacy:
        connection.execute(
            "CREATE TABLE shelf_inventory (shelf_id INTEGER, face INTEGER, level INTEGER, "
            "y_cm REAL, z_offset_cm REAL, sku TEXT)"
        )
    else:
        connection.execute(
            "CREATE TABLE shelf_inventory (shelf_id INTEGER, face INTEGER, level INTEGER, "
            "y_cm REAL, expected_sku TEXT, actual_sku TEXT, width_cm REAL, "
            "height_cm REAL, image_dir TEXT)"
        )
    connection.execute(
        "INSERT INTO shelf_types VALUES (1, 'standard', 1.86, 0.8, 1.65, 5, "
        "0.05, 0.4, 0.02, 0.005, 0.3, 0.4)"
    )
    connection.execute("INSERT INTO shelf_groups VALUES (1, 'shelf', 1.0, 2.0, 0.0, 1)")
    connection.execute("INSERT INTO sku_catalog VALUES ('demo')")
    if legacy:
        connection.execute("INSERT INTO shelf_inventory VALUES (1, 0, 2, 50, 8.0, 'demo')")
    else:
        connection.execute(
            "INSERT INTO shelf_inventory VALUES "
            "(1, 0, 2, 50, 'demo', 'demo', 10, 8, 'img')"
        )
    connection.commit()
    connection.close()


class ShelfInventoryDatabaseTest(unittest.TestCase):
    def test_lists_catalog_skus_in_read_only_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "inventory.db")
            create_database(path)
            connection = sqlite3.connect(path)
            connection.executemany(
                "INSERT INTO sku_catalog VALUES (?)",
                [("unknown",), ("茶饮",), ("",)],
            )
            connection.commit()
            connection.close()

            database = ShelfInventoryDatabase(path)

            self.assertEqual(["demo", "茶饮"], database.list_catalog_skus())

    def test_current_schema_pose_and_deterministic_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "inventory.db")
            create_database(path)
            database = ShelfInventoryDatabase(path)
            slot = database.select_sku_slot("demo", [(1, 0)])
            self.assertIsNotNone(slot)
            self.assertEqual("1-0-2-50", slot.slot_id)
            self.assertAlmostEqual(slot.world_x, 1.1525)
            self.assertAlmostEqual(slot.world_y, 2.50)
            self.assertAlmostEqual(slot.world_z, 0.90)

    def test_legacy_z_offset_schema_is_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "legacy.db")
            create_database(path, legacy=True)
            slot = ShelfInventoryDatabase(path).find_sku_slots("demo")[0]
            # surface=0.05 + 2*0.4 + 0.01, plus legacy centre offset=0.08
            self.assertAlmostEqual(slot.world_z, 0.94)

    def test_real_supermarket_scene_database_is_read_only_and_has_layers(self):
        path = Path("/home/openflex/openflex_shop/supermarket_scene/shelf_inventory.db")
        if not path.is_file():
            self.skipTest("teammate supermarket_scene database is not present")
        database = ShelfInventoryDatabase(str(path))
        slots = database.find_sku_slots("小青柠汁饮料")
        self.assertTrue(slots)
        self.assertEqual({3}, {slot.level for slot in slots})
        with self.assertRaises(sqlite3.OperationalError):
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
                connection.execute("create table should_not_exist (id integer)")


if __name__ == "__main__":
    unittest.main()
