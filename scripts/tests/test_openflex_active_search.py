import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.active_search import (  # noqa: E402
    AMBIGUOUS, CANCELLED, FOUND, INVALID_DEPTH, NOT_FOUND, NOT_SETTLED, STALE_FRAME,
    TIMEOUT, ActiveSearch, ActiveSearchError, SearchDecision,
)
from openflex_roboos_bridge.shelf_viewpoints import (  # noqa: E402
    ShelfViewpointError, Viewpoint, generate_shelf_viewpoints, shelf_front_geometry,
    visible_span_at,
)
from openflex_roboos_bridge.visual_grasp_planning import Aabb  # noqa: E402
from openflex_roboos_bridge.visual_grasp_scene import load_shelf_scene  # noqa: E402

SCENE_PATH = ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/sim_shelf_scene.yaml"
# Head camera horizontal FOV from the sim intrinsics (fx=171.38, width=320).
H_FOV = 2.0 * math.atan(160.0 / 171.37776080905377)


def shelf_obstacles():
    return load_shelf_scene(SCENE_PATH).obstacles


class FakeResolution:
    def __init__(self, outcome, reason, detail="detail", track=None, object_id=None):
        self.outcome, self.reason, self.selection_reason = outcome, reason, detail
        self.selected_perception_track_id, self.selected_object_id = track, object_id


def resolution_for(object_y, viewpoint, *, objects=(), track="track_1"):
    contenders = (float(object_y),) + tuple(float(value) for value in objects)
    seen = [value for value in contenders
            if viewpoint.covered_y[0] <= value <= viewpoint.covered_y[1]]
    if len(seen) > 1:
        return FakeResolution("failed", "ambiguous_candidates", "two visible red candidates")
    if seen:
        return FakeResolution("authorized", "target_visible", "unique visible red candidate", track)
    return FakeResolution("failed", "target_absent", "no target-colour candidate")


def test_viewpoints_come_only_from_shelf_geometry():
    obstacles = shelf_obstacles()
    viewpoints = generate_shelf_viewpoints(
        obstacles, standoff_m=0.75, lateral_step_m=0.30, horizontal_fov_rad=H_FOV, clearance_m=0.05)
    front_x, y_min, y_max = shelf_front_geometry(obstacles)
    assert len(viewpoints) >= 2
    for viewpoint in viewpoints:
        assert viewpoint.position_xy[0] == pytest.approx(front_x - 0.75)
        assert viewpoint.position_xy[1] == pytest.approx(
            0.5 * (viewpoint.covered_y[0] + viewpoint.covered_y[1]))
    # union of covered windows must span the shelf extent
    assert min(box.covered_y[0] for box in viewpoints) <= y_min + 1e-9
    assert max(box.covered_y[1] for box in viewpoints) >= y_max - 1e-9
    assert all(viewpoint.standoff_m == 0.75 for viewpoint in viewpoints)


def test_visible_span_matches_field_of_view():
    assert visible_span_at(2.0, math.pi / 2) == pytest.approx(4.0)
    with pytest.raises(ShelfViewpointError):
        visible_span_at(0.0, H_FOV)
    with pytest.raises(ShelfViewpointError):
        visible_span_at(1.0, math.pi)


def test_viewpoint_generation_rejects_degenerate_inputs():
    with pytest.raises(ShelfViewpointError):
        generate_shelf_viewpoints([], standoff_m=0.75, lateral_step_m=0.3,
                                  horizontal_fov_rad=H_FOV, clearance_m=0.05)
    with pytest.raises(ShelfViewpointError):
        generate_shelf_viewpoints(shelf_obstacles(), standoff_m=0.0, lateral_step_m=0.3,
                                  horizontal_fov_rad=H_FOV, clearance_m=0.05)
    with pytest.raises(ShelfViewpointError):
        generate_shelf_viewpoints(shelf_obstacles(), standoff_m=0.75, lateral_step_m=-0.3,
                                  horizontal_fov_rad=H_FOV, clearance_m=0.05)


def test_viewpoint_inside_the_shelf_is_rejected():
    box = Aabb((0.0, -0.5, 0.0), (1.0, 0.5, 1.0))
    with pytest.raises(ShelfViewpointError):
        generate_shelf_viewpoints([box], standoff_m=0.05, lateral_step_m=0.2,
                                  horizontal_fov_rad=H_FOV, clearance_m=0.5)


def machine(**overrides):
    viewpoints = generate_shelf_viewpoints(
        shelf_obstacles(), standoff_m=0.75, lateral_step_m=0.30,
        horizontal_fov_rad=H_FOV, clearance_m=0.05)
    values = dict(viewpoints=viewpoints, max_frame_age_sec=2.0, min_depth_valid_fraction=0.2)
    values.update(overrides)
    return ActiveSearch(**values)


def test_search_finds_a_unique_visible_target():
    search = machine()
    viewpoint = search.next_viewpoint()
    assert viewpoint is not None
    decision = search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.5,
                             depth_valid_fraction=0.9,
                             resolution=resolution_for(0.0, viewpoint))
    assert decision.finished and decision.outcome == FOUND
    assert decision.selected_perception_track_id == "track_1"
    assert search.report()["robot_motion_executed"] is False


def test_search_scans_every_viewpoint_then_fails_safely():
    search = machine()
    decisions = []
    while not search.finished:
        viewpoint = search.next_viewpoint()
        decisions.append(search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.1,
                                       depth_valid_fraction=0.9,
                                       resolution=resolution_for(99.0, viewpoint)))
    assert search.outcome == NOT_FOUND
    assert len(decisions) == len(search.viewpoints)
    assert search.report()["viewpoints_observed"] == len(search.viewpoints)


def test_ambiguous_result_stops_immediately():
    search = machine()
    viewpoint = search.next_viewpoint()
    decision = search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.1,
                             depth_valid_fraction=0.9,
                             resolution=FakeResolution("failed", "ambiguous_candidates", "two reds"))
    assert decision.finished and decision.outcome == AMBIGUOUS
    assert search.next_viewpoint() is None


@pytest.mark.parametrize("case,expected", [
    ("stale", STALE_FRAME), ("depth", INVALID_DEPTH)])
def test_stale_frame_and_invalid_depth_fail_closed(case, expected):
    search = machine()
    viewpoint = search.next_viewpoint()
    if case == "stale":
        decision = search.submit(viewpoint, frame_timestamp_sec=90.0, now_sec=100.0,
                                 depth_valid_fraction=0.9,
                                 resolution=resolution_for(0.0, viewpoint))
    else:
        decision = search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.1,
                                 depth_valid_fraction=0.01,
                                 resolution=resolution_for(0.0, viewpoint))
    assert decision.finished and decision.outcome == expected


def test_cancellation_and_deadline_are_terminal():
    search = machine()
    assert search.cancel().outcome == CANCELLED
    search = machine(deadline_sec=100.0)
    viewpoint = search.next_viewpoint()
    decision = search.submit(viewpoint, frame_timestamp_sec=0.0, now_sec=200.0,
                             depth_valid_fraction=0.9,
                             resolution=resolution_for(0.0, viewpoint))
    assert decision.outcome == STALE_FRAME  # the stale check runs before the deadline
    search = machine(deadline_sec=100.0)
    viewpoint = search.next_viewpoint()
    decision = search.submit(viewpoint, frame_timestamp_sec=99.9, now_sec=100.5,
                             depth_valid_fraction=0.9,
                             resolution=resolution_for(0.0, viewpoint))
    assert decision.outcome == TIMEOUT


def test_wrong_viewpoint_is_rejected():
    search = machine()
    first = search.next_viewpoint()
    second = search.viewpoints[1]
    with pytest.raises(ActiveSearchError):
        search.submit(second, frame_timestamp_sec=100.0, now_sec=100.1,
                      depth_valid_fraction=0.9, resolution=resolution_for(0.0, first))


def test_random_slots_repeated_trials():
    """Randomised product positions never change the viewpoint plan."""

    import numpy as np

    rng = np.random.default_rng(20261120)
    front_x, y_min, y_max = shelf_front_geometry(shelf_obstacles())
    found = ambiguous = missed = 0
    for _ in range(120):
        object_y = float(rng.uniform(y_min + 0.02, y_max - 0.02))
        search = machine()
        while not search.finished:
            viewpoint = search.next_viewpoint()
            search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.1,
                          depth_valid_fraction=0.9, resolution=resolution_for(object_y, viewpoint))
        if search.outcome == FOUND:
            found += 1
        elif search.outcome == AMBIGUOUS:
            ambiguous += 1
        elif search.outcome in (NOT_FOUND,):
            missed += 1
        else:
            raise AssertionError(search.outcome)
    assert missed == 0 and ambiguous == 0
    assert found == 120
    # A product outside every observed window is reported, never grasped.
    search = machine()
    while not search.finished:
        viewpoint = search.next_viewpoint()
        search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.1,
                      depth_valid_fraction=0.9, resolution=resolution_for(50.0, viewpoint))
    assert search.outcome == NOT_FOUND


def test_two_products_in_one_view_are_ambiguous_not_grasped():
    viewpoints = generate_shelf_viewpoints(
        shelf_obstacles(), standoff_m=0.75, lateral_step_m=0.30,
        horizontal_fov_rad=H_FOV, clearance_m=0.05)
    search = ActiveSearch(viewpoints=viewpoints)
    viewpoint = search.next_viewpoint()
    decision = search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.1,
                             depth_valid_fraction=0.9,
                             resolution=resolution_for(0.0, viewpoint, objects=(0.05,)))
    assert decision.outcome == AMBIGUOUS


def test_unsettled_frame_is_fail_closed():
    search = machine()
    viewpoint = search.next_viewpoint()
    decision = search.submit(viewpoint, frame_timestamp_sec=100.0, now_sec=100.1,
                             depth_valid_fraction=0.9, settled=False,
                             resolution=resolution_for(0.0, viewpoint))
    assert decision.finished and decision.outcome == NOT_SETTLED
    assert search.next_viewpoint() is None
    assert search.report()["history"][-1]["settled"] is False


def test_viewpoint_plan_with_a_blind_spot_is_rejected():
    left = Viewpoint("left", (0.0, -1.0), 0.0, 0.75, (-1.5, -0.5))
    right = Viewpoint("right", (0.0, 1.0), 0.0, 0.75, (0.5, 1.5))
    with pytest.raises(ActiveSearchError):
        ActiveSearch(viewpoints=(left, right))
    # touching windows are accepted
    ActiveSearch(viewpoints=(left, Viewpoint("mid", (0.0, 0.0), 0.0, 0.75, (-0.5, 0.5))))


def test_decision_is_serialisable():
    decision = SearchDecision(True, FOUND, "unique", "shelf_view_00", "track_1")
    assert decision.to_dict()["selected_perception_track_id"] == "track_1"
    assert decision.to_dict()["finished"] is True
