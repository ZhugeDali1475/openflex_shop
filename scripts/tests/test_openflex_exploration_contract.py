import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.exploration_contract import (  # noqa: E402
    CANCELLED, LOCALIZATION_LOST, MAP_INCOMPLETE, NEXT_POSITION, STEPS_EXHAUSTED, TARGET_FOUND,
    TARGET_NOT_FOUND, TIMEOUT, ExplorationError, ExplorationSearch, find_frontier_clusters,
    information_gain, select_frontier,
)
from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, LocalizationQuality, MapSnapshot, ground_truth_localization,
)

_CODES = {"#": OCCUPIED, ".": FREE, "?": UNKNOWN}
_LOCALIZED = ground_truth_localization()


def make_snapshot(rows, *, resolution=0.1, origin=(0.0, 0.0), version=0, localization=None):
    cells = tuple(_CODES[character] for row in rows for character in row)
    return MapSnapshot(frame_id="map", resolution_m=resolution, origin_xy=origin,
                       width=len(rows[0]), height=len(rows), occupancy=cells,
                       map_version=version, localization=localization or _LOCALIZED, stamp_sec=0.0)


OPEN = [
    "??????",
    "??????",
    "??..??",
    "??..??",
    "??????",
    "??????",
]

TWO_PATCHES = [
    "????????????",
    "??..????..??",
    "??..????..??",
    "????????????",
]


def test_frontier_clusters_and_information_gain():
    snapshot = make_snapshot(OPEN)
    clusters = find_frontier_clusters(snapshot, sensor_radius_m=0.3)
    assert len(clusters) == 1
    assert clusters[0].cell_count == 4
    assert clusters[0].information_gain > 0.0
    assert information_gain(snapshot, clusters[0].centroid_xy, 0.3) == clusters[0].information_gain


def test_select_frontier_prefers_information_and_breaks_ties_by_distance():
    snapshot = make_snapshot(TWO_PATCHES)
    clusters = find_frontier_clusters(snapshot, sensor_radius_m=0.25)
    assert len(clusters) == 2
    assert clusters[0].information_gain == clusters[1].information_gain  # symmetric map
    candidate = select_frontier(snapshot, (0.0, 0.0), sensor_radius_m=0.25)
    assert candidate.goal_xy == clusters[0].centroid_xy  # nearer patch wins the tie
    from_right = select_frontier(snapshot, (1.1, 0.0), sensor_radius_m=0.25)
    assert from_right.goal_xy == clusters[1].centroid_xy
    # selection always returns a reachable candidate with the maximum gain
    assert candidate.information_gain == max(cluster.information_gain for cluster in clusters)
    # a reach budget below every cluster yields no goal
    assert select_frontier(snapshot, (0.0, 0.0), sensor_radius_m=0.25, max_reach_m=0.1) is None


def test_no_frontier_returns_none():
    assert select_frontier(make_snapshot(["..", ".."]), (0.0, 0.0)) is None


def test_exploration_plans_then_reports_target_found():
    search = ExplorationSearch(frame_id="map")
    snapshot = make_snapshot(OPEN, version=1)
    decision = search.plan(snapshot, robot_xy=(0.0, 0.0), now_sec=1.0)
    assert decision.outcome == NEXT_POSITION and not decision.finished
    assert decision.map_version == 1 and decision.information_gain > 0.0
    found = search.plan(snapshot, robot_xy=(0.0, 0.0), now_sec=1.5, target_observed=True)
    assert found.finished and found.outcome == TARGET_FOUND
    assert search.report()["robot_motion_executed"] is False


def test_localization_quality_gates_exploration():
    lost = LocalizationQuality(converged=False, source="amcl", position_rmse_m=0.30,
                               yaw_rmse_rad=0.10)
    search = ExplorationSearch(frame_id="map")
    decision = search.plan(make_snapshot(OPEN, localization=lost), robot_xy=(0.0, 0.0),
                           now_sec=1.0)
    assert decision.outcome == LOCALIZATION_LOST


def test_map_incomplete_and_target_not_found_are_distinct():
    empty = ExplorationSearch(frame_id="map")
    assert empty.plan(make_snapshot(["????"] * 4), robot_xy=(0.0, 0.0),
                      now_sec=1.0).outcome == MAP_INCOMPLETE
    complete = ExplorationSearch(frame_id="map")
    assert complete.plan(make_snapshot(["...."] * 4), robot_xy=(0.0, 0.0),
                         now_sec=1.0).outcome == TARGET_NOT_FOUND


def test_timeout_steps_and_cancel_are_terminal():
    timeout = ExplorationSearch(frame_id="map", deadline_sec=10.0)
    assert timeout.plan(make_snapshot(OPEN), robot_xy=(0.0, 0.0),
                        now_sec=20.0).outcome == TIMEOUT
    stepped = ExplorationSearch(frame_id="map", max_steps=1)
    assert stepped.plan(make_snapshot(OPEN), robot_xy=(0.0, 0.0), now_sec=1.0).outcome == NEXT_POSITION
    assert stepped.plan(make_snapshot(OPEN), robot_xy=(0.0, 0.0),
                        now_sec=1.1).outcome == STEPS_EXHAUSTED
    cancelled = ExplorationSearch(frame_id="map")
    assert cancelled.cancel().outcome == CANCELLED
    with pytest.raises(ExplorationError):
        cancelled.plan(make_snapshot(OPEN), robot_xy=(0.0, 0.0), now_sec=1.0)


def test_frame_mismatch_and_reuse_are_rejected():
    search = ExplorationSearch(frame_id="map")
    other = MapSnapshot(frame_id="odom", resolution_m=0.1, origin_xy=(0.0, 0.0), width=1, height=1,
                        occupancy=(UNKNOWN,), map_version=0, localization=_LOCALIZED, stamp_sec=0.0)
    with pytest.raises(ExplorationError):
        search.plan(other, robot_xy=(0.0, 0.0), now_sec=1.0)
    search.plan(make_snapshot(OPEN), robot_xy=(0.0, 0.0), now_sec=1.0)
    search.plan(make_snapshot(OPEN), robot_xy=(0.0, 0.0), now_sec=1.0, target_observed=True)
    with pytest.raises(ExplorationError):
        search.plan(make_snapshot(OPEN), robot_xy=(0.0, 0.0), now_sec=2.0)


def test_exploration_is_deterministic():
    def run():
        search = ExplorationSearch(frame_id="map")
        decisions = []
        for version in range(4):
            decisions.append(search.plan(make_snapshot(OPEN, version=version),
                                         robot_xy=(0.0, 0.0), now_sec=float(version)))
        return [decision.to_dict() for decision in decisions], search.report()["history"]

    first = run()
    second = run()
    assert first == second
