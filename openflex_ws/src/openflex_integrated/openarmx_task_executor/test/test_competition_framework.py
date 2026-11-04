import pathlib
import unittest

from openarmx_task_executor.competition_config import CompetitionConfig
from openarmx_task_executor.competition_state import CompetitionRunState
from openarmx_task_executor.station_db import StationDatabase


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]


class CompetitionConfigTest(unittest.TestCase):
    def test_example_config_references_valid_stations(self):
        config = CompetitionConfig.from_yaml(
            str(PACKAGE_DIR / "config" / "competition.yaml")
        )
        stations = StationDatabase.from_yaml(
            str(PACKAGE_DIR / "config" / "stations.yaml")
        )
        referenced = set()
        for phase in config.phases.values():
            referenced.update(
                [phase.entry_station, phase.judging_station, *phase.scan_tour]
            )
        for route in config.sku_routes.values():
            referenced.update([route.source_station, route.destination_station])
        self.assertEqual([], sorted(name for name in referenced if not stations.get(name)))

    def test_sku_route_prefers_specification(self):
        config = CompetitionConfig.from_yaml(
            str(PACKAGE_DIR / "config" / "competition.yaml")
        )
        self.assertEqual("shelf_a_pick", config.route_for("milk", "250ml").source_station)
        self.assertEqual("milk", config.route_for("milk", "250ml").source_slot)

    def test_wildcard_slot_supports_dynamic_perception_ids(self):
        stations = StationDatabase.from_yaml(
            str(PACKAGE_DIR / "config" / "stations.yaml")
        )
        self.assertIsNotNone(stations.slot("replenishment_box", "runtime_object_17"))


class CompetitionRunStateTest(unittest.TestCase):
    def test_three_phases_require_order_and_same_run_id(self):
        state = CompetitionRunState()
        self.assertFalse(state.can_start("run", 2))
        state.start("run", 1)
        self.assertFalse(state.can_start("run", 1))
        state.complete()
        self.assertFalse(state.can_start("other", 2))
        self.assertTrue(state.can_start("run", 2))
        state.start("run", 2)
        state.complete()
        state.start("run", 3)
        state.complete()
        self.assertEqual(4, state.expected_phase)

    def test_failure_is_latched(self):
        state = CompetitionRunState()
        state.start("run", 1)
        state.fail()
        self.assertFalse(state.can_start("run", 1))
        self.assertEqual(1, state.failed_phase)

    def test_dry_run_does_not_advance_phase(self):
        state = CompetitionRunState()
        state.start("run", 1)
        state.release_dry_run()
        self.assertEqual(1, state.expected_phase)
        self.assertTrue(state.can_start("run", 1))

    def test_reset_clears_completed_or_failed_run(self):
        state = CompetitionRunState()
        state.start("run", 1)
        state.fail()
        state.reset()
        self.assertEqual("", state.run_id)
        self.assertEqual(1, state.expected_phase)
        self.assertEqual(0, state.failed_phase)
