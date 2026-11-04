from pathlib import Path

import pytest

from openflex_navigation_gui.benchmark import (
    evaluate_goal,
    expand_tour,
    find_waypoint_by_role,
    load_benchmark_profile,
    waypoint_landmarks,
)


PROFILE = (
    Path(__file__).parents[3]
    / "openflex_chassis"
    / "navigation_layer"
    / "swerve_navigation"
    / "config"
    / "supermarket_test_03_benchmark.yaml"
)
PRECISION_PROFILE = PROFILE.with_name("supermarket_competition_precision_test.yaml")
TEST_05_PROFILE = PROFILE.with_name("supermarket_test_05_competition.yaml")


def test_supermarket_benchmark_profile_is_coherent():
    profile = load_benchmark_profile(PROFILE)
    assert profile.profile_id == "supermarket_test_03"
    assert profile.map_name == "supermarket_test_03"
    assert set(profile.waypoints) == {"C0", "W", "D", "C180", "N"}
    assert profile.acceptance.goal_position_m == pytest.approx(0.05)
    assert profile.acceptance.goal_yaw_rad == pytest.approx(0.05)


def test_tour_expansion_preserves_order_and_repeat_count():
    profile = load_benchmark_profile(PROFILE)
    expanded = expand_tour(profile, "baseline", 2)
    assert [item.waypoint_id for item in expanded[:6]] == [
        "C0",
        "W",
        "D",
        "C180",
        "N",
        "C0",
    ]
    assert len(expanded) == 12


def test_goal_evaluation_uses_competition_precision_window():
    profile = load_benchmark_profile(PROFILE)
    assert evaluate_goal(profile.acceptance, 0.049, 0.049).passed
    assert not evaluate_goal(profile.acceptance, 0.051, 0.049).passed
    assert not evaluate_goal(profile.acceptance, 0.049, 0.051).passed


def test_landmarks_merge_heading_variants_at_the_same_position():
    profile = load_benchmark_profile(PROFILE)
    landmarks = waypoint_landmarks(list(profile.waypoints.values()))
    assert len(landmarks) == 4
    assert any(label == "C0/C180" for _, _, label in landmarks)


def test_precision_profile_has_unique_start_and_shelf_targets():
    profile = load_benchmark_profile(PRECISION_PROFILE)
    start = find_waypoint_by_role(profile, "start_position")
    assert start is not None
    assert start.waypoint_id == "START"
    assert start.x == pytest.approx(0.20)
    assert start.y == pytest.approx(1.30)
    assert profile.initial_x == pytest.approx(0.0)
    assert profile.initial_y == pytest.approx(0.0)
    assert profile.initial_yaw == pytest.approx(0.0)
    assert set(profile.waypoints) == {
        "START",
        "SHELF1",
        "SHELF2",
        "SHELF3",
        "RETURN",
    }


def test_supermarket_test_05_profile_uses_safe_vehicle_centers():
    profile = load_benchmark_profile(TEST_05_PROFILE)
    start = find_waypoint_by_role(profile, "start_position")
    assert profile.map_name == "supermarket_test_05"
    assert profile.launch_file == "supermarket_test_05_benchmark.launch.py"
    assert start is None
    assert set(profile.waypoints) == {"SHELF4", "0"}
    assert profile.tours["safe_stations"].sequence == ("SHELF4", "0")
    assert profile.transport_length_m == pytest.approx(0.92)
    assert profile.transport_width_m == pytest.approx(0.90)
    assert profile.goal_obstacle_clearance_m == pytest.approx(0.05)
