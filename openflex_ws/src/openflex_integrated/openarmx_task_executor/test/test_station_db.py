import pathlib
import tempfile
import unittest

from openarmx_task_executor.station_db import StationDatabase


class StationDatabaseTest(unittest.TestCase):
    def test_loads_sample_stations(self):
        package_dir = pathlib.Path(__file__).resolve().parents[1]
        database = StationDatabase.from_yaml(str(package_dir / "config" / "stations.yaml"))

        source = database.get("shelf_A")
        destination = database.get("drop_A")
        slot = database.slot("shelf_A", "demo_object")

        self.assertIsNotNone(source)
        self.assertIsNotNone(destination)
        self.assertIsNotNone(slot)
        self.assertEqual(source.marker_id, 10)
        self.assertEqual(slot.arm_name, "left")
        self.assertAlmostEqual(source.target_marker_pose.x, 0.65)
        self.assertIsNotNone(destination.place_pose)

    def test_unknown_station_and_slot_are_none(self):
        package_dir = pathlib.Path(__file__).resolve().parents[1]
        database = StationDatabase.from_yaml(str(package_dir / "config" / "stations.yaml"))

        self.assertIsNone(database.get("missing"))
        self.assertIsNone(database.slot("shelf_A", "missing"))

    def test_missing_arm_name_remains_invalid_instead_of_guessing_left(self):
        content = """
stations:
  shelf:
    nav_pose: {frame_id: map, x: 0.0, y: 0.0, yaw: 0.0}
    slots:
      product:
        pose: {frame_id: base_link, x: 0.5, y: 0.0, z: 0.8, yaw: 0.0}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "stations.yaml"
            path.write_text(content, encoding="utf-8")
            slot = StationDatabase.from_yaml(str(path)).slot("shelf", "product")

        self.assertEqual("", slot.arm_name)

    def test_supermarket_test_07_uses_scan_alignment_and_station_retreat(self):
        package_dir = pathlib.Path(__file__).resolve().parents[1]
        database = StationDatabase.from_yaml(
            str(package_dir / "config" / "stations_supermarket_test_07.yaml")
        )

        station = database.get("SHELF_TABLE_APPROACH")

        self.assertIsNotNone(station)
        self.assertEqual(station.alignment_mode, "shelf_scan")
        self.assertAlmostEqual(station.shelf_target_clearance, 0.05)
        self.assertTrue(station.retreat_after_pick)
        self.assertTrue(station.retreat_after_place)
        self.assertAlmostEqual(station.retreat_distance, 0.15)
        self.assertAlmostEqual(station.nav_pose.yaw, -1.5707963267948966)
        self.assertIsNotNone(database.slot("SHELF_TABLE_APPROACH", "mock_item"))
        self.assertAlmostEqual(
            database.level_lift_position("SHELF_TABLE_APPROACH", None, 2),
            0.1066645,
        )
        level_3 = database.level_arm_joint_positions(
            "SHELF_TABLE_APPROACH", 3, "right"
        )
        self.assertEqual(len(level_3), 7)
        self.assertAlmostEqual(level_3["openarmx_right_joint1"], -0.7114080786705017)

    def test_layer_lift_calibration_prefers_slot_override(self):
        content = """
stations:
  shelf:
    nav_pose: {frame_id: map, x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
    level_lift_positions: {0: -0.50, 2: -0.10}
    slots:
      product:
        pose: {frame_id: base_link, x: 0.5, y: 0.0, z: 0.9, yaw: 0.0}
        level: 2
        nominal_lift_position: -0.08
      "*":
        pose: {frame_id: base_link, x: 0.5, y: 0.0, z: 0.5, yaw: 0.0}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "stations.yaml"
            path.write_text(content, encoding="utf-8")
            database = StationDatabase.from_yaml(str(path))
        slot = database.slot("shelf", "product")
        wildcard = database.slot("shelf", "dynamic")
        self.assertEqual(2, slot.level)
        self.assertAlmostEqual(
            -0.08, database.level_lift_position("shelf", slot, slot.level)
        )
        self.assertAlmostEqual(
            -0.10, database.level_lift_position("shelf", wildcard, 2)
        )
        self.assertIsNone(database.level_lift_position("shelf", wildcard, 1))

    def test_layer_arm_calibration_is_arm_specific_and_copied(self):
        content = """
stations:
  shelf:
    nav_pose: {frame_id: map, x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
    level_arm_positions:
      2:
        right: {openarmx_right_joint1: -0.5, openarmx_right_joint2: 0.25}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "stations.yaml"
            path.write_text(content, encoding="utf-8")
            database = StationDatabase.from_yaml(str(path))

        target = database.level_arm_joint_positions("shelf", 2, "right")
        self.assertEqual(
            target,
            {"openarmx_right_joint1": -0.5, "openarmx_right_joint2": 0.25},
        )
        target["openarmx_right_joint1"] = 1.0
        self.assertAlmostEqual(
            database.level_arm_joint_positions("shelf", 2, "right")[
                "openarmx_right_joint1"
            ],
            -0.5,
        )
        self.assertEqual(database.level_arm_joint_positions("shelf", 2, "left"), {})

    def test_removed_anygrasp_strategies_are_rejected(self):
        for strategy in ("anygrasp", "mock_or_anygrasp"):
            content = f"""
stations:
  shelf:
    nav_pose: {{frame_id: map, x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}}
    slots:
      product:
        pose: {{frame_id: base_link, x: 0.5, y: 0.0, z: 0.9, yaw: 0.0}}
        grasp_strategy: {strategy}
"""
            with tempfile.TemporaryDirectory() as directory:
                path = pathlib.Path(directory) / "stations.yaml"
                path.write_text(content, encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "removed grasp_strategy"):
                    StationDatabase.from_yaml(str(path))

    def test_supermarket_configs_use_explicit_graspnet_or_mock_strategies(self):
        package_dir = pathlib.Path(__file__).resolve().parents[1]
        mujoco = StationDatabase.from_yaml(
            str(package_dir / "config" / "stations_supermarket_mujoco.yaml")
        )
        self.assertEqual(
            mujoco.slot("replenishment_box", "item").grasp_strategy,
            "graspnet_basket",
        )
        self.assertEqual(
            mujoco.slot("shelf_a_pick", "item").grasp_strategy,
            "graspnet_shelf",
        )

        test_08 = StationDatabase.from_yaml(
            str(package_dir / "config" / "stations_supermarket_test_08.yaml")
        )
        self.assertEqual(
            test_08.slot("SHELF_A", "item").grasp_strategy,
            "graspnet_shelf",
        )
        self.assertEqual(test_08.slot("SHELF_C", "item").arm_name, "left")
        self.assertEqual(
            test_08.slot("delivery_counter", "item").grasp_strategy,
            "mock",
        )

        test_09 = StationDatabase.from_yaml(
            str(package_dir / "config" / "stations_supermarket_test_09.yaml")
        )
        self.assertEqual(
            set(test_09.get("SHELF-A").level_arm_positions[0]),
            {"left", "right"},
        )
        self.assertEqual(
            set(test_09.get("SHELF-A").level_arm_positions[4]),
            {"left", "right"},
        )
        self.assertEqual(
            set(test_09.get("SHELF-A").level_arm_positions),
            {0, 2, 3, 4},
        )
        self.assertAlmostEqual(
            test_09.get("SHELF-A").level_lift_positions[3], 0.29981
        )
        self.assertAlmostEqual(
            test_09.get("SHELF-A").shelf_target_clearance, 0.10
        )
        delivery_c = test_09.get("jiaofu2")
        self.assertEqual(set(delivery_c.level_arm_positions), {0, 10, 11})
        self.assertEqual(
            set(delivery_c.level_arm_positions[10]), {"left", "right"}
        )
        self.assertEqual(
            set(delivery_c.level_arm_positions[11]), {"left", "right"}
        )
        self.assertAlmostEqual(
            delivery_c.level_arm_positions[10]["right"]["openarmx_right_joint1"],
            -0.243,
        )
        self.assertAlmostEqual(
            delivery_c.level_arm_positions[11]["right"]["openarmx_right_joint1"],
            0.526,
        )
        self.assertTrue(
            all(
                "finger" not in joint_name
                for level in delivery_c.level_arm_positions.values()
                for arm in level.values()
                for joint_name in arm
            )
        )

    def test_mission_overlay_replaces_pose_alignment_and_retreat(self):
        base = """
stations:
  delivery_counter:
    nav_pose: {frame_id: map, x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
    alignment: {enabled: false, mode: none}
    retreat: {after_place: false, distance: 0.10, speed: 0.03}
    place_pose: {frame_id: base_link, x: 0.55, y: 0.0, z: 0.75, yaw: 0.0}
"""
        mission = """
schema_version: 2
map_name: arena
stations:
  - id: delivery_counter
    role: delivery_table
    pose: {x: 2.32, y: 0.37, yaw: -1.57079632679}
    action:
      fine_align:
        enabled: true
        mode: front_surface_scan
        target_clearance_m: 0.20
        clearance_tolerance_m: 0.01
        yaw_tolerance_deg: 1.0
        timeout_sec: 20.0
      retreat:
        after_place: true
        distance_m: 0.15
        speed_mps: 0.04
"""
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "stations.yaml").write_text(base, encoding="utf-8")
            (root / "mission.yaml").write_text(mission, encoding="utf-8")
            database = StationDatabase.from_yaml(
                str(root / "stations.yaml"), str(root / "mission.yaml")
            )
        station = database.get("delivery_counter")
        self.assertAlmostEqual(station.nav_pose.x, 2.32)
        self.assertEqual(station.alignment_mode, "shelf_scan")
        self.assertAlmostEqual(station.shelf_target_clearance, 0.20)
        self.assertTrue(station.retreat_after_place)
        self.assertAlmostEqual(station.retreat_distance, 0.15)
        self.assertAlmostEqual(station.retreat_speed, 0.04)
        self.assertIsNotNone(station.place_pose)


if __name__ == "__main__":
    unittest.main()
