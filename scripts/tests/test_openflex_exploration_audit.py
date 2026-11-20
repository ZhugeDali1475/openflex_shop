import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "exploration_audit",
    Path(__file__).resolve().parents[1] / "audit_openflex_exploration.py",
)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)

from openflex_roboos_bridge.mapping_contract import OCCUPIED  # noqa: E402


@pytest.fixture(scope="module")
def report():
    return audit.run_audit()


def test_audit_passes_and_is_strictly_offline(report):
    assert report["passed"] is True
    assert report["robot_motion_executed"] is False
    assert report["ros_started"] is False
    assert report["preset_route_used"] is False
    assert report["next_position_source"] == "current_map_frontier_information_gain_only"
    assert report["localization_source"] == "simulation_ground_truth_not_an_estimator"


def test_scenario_outcomes(report):
    outcomes = {item["name"]: item["outcome"] for item in report["scenarios"]}
    assert outcomes == {
        "room_target_absent": "target_not_found",
        "room_target_present": "target_found",
        "sealed_pocket_map_incomplete": "map_incomplete",
        "localization_lost": "localization_lost",
        "deadline_timeout": "timeout",
        "steps_exhausted": "steps_exhausted",
        "cancelled": "cancelled",
    }


def test_exploration_advances_and_grows_coverage(report):
    absent = {item["name"]: item for item in report["scenarios"]}["room_target_absent"]
    coverages = [step["coverage_fraction"] for step in absent["trajectory"]]
    assert len(coverages) >= 3
    assert coverages == sorted(coverages)
    assert coverages[-1] >= 0.99
    assert absent["final_coverage"] == pytest.approx(1.0)


def test_next_position_is_map_derived_not_preset(report):
    present = {item["name"]: item for item in report["scenarios"]}["room_target_present"]
    for step in present["trajectory"][:-1]:
        assert step["goal_xy"] is not None
        assert step["information_gain"] > 0.0


def test_rays_stop_on_walls():
    rows = ["####", "#..#", "####"]
    endpoints = audit.cast_rays(audit.parse_world(rows), 4, 3, (1, 1), resolution_m=0.1,
                                radius_m=0.5)
    # every endpoint lands on a grid cell, and at least one is the surrounding wall
    assert endpoints
    assert any(abs(point[0] - 0.3) < 1e-6 for point in endpoints)


def test_audit_is_deterministic():
    def stripped():
        report = audit.run_audit()
        report.pop("generated_at")
        return report

    assert stripped() == stripped()
