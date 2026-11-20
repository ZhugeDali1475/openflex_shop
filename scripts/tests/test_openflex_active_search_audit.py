import importlib.util
import math
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "active_search_audit",
    Path(__file__).resolve().parents[1] / "audit_openflex_active_search.py",
)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


@pytest.fixture(scope="module")
def report():
    return audit.run_audit()


def test_audit_passes_and_is_strictly_offline(report):
    assert report["passed"] is True
    assert report["robot_motion_executed"] is False
    assert report["ros_started"] is False
    assert report["authorization_reads_slot_coordinates"] is False
    assert report["physical_verification"] is False
    assert report["viewpoint_source"] == "static_shelf_geometry_only"


def test_head_camera_quaternion_is_unit():
    norm = math.sqrt(sum(component * component
                         for component in audit.HEAD_CAMERA_QUATERNION_XYZW))
    assert norm == pytest.approx(1.0)


def test_viewpoint_plan_comes_only_from_shelf_geometry():
    obstacles, viewpoints = audit.plan_viewpoints()
    assert len(viewpoints) == 4
    assert all(abs(box.covered_y[0]) <= 1.2 and abs(box.covered_y[1]) <= 1.2 for box in viewpoints)
    # windows are contiguous, so no blind spot is left between stands
    windows = sorted(box.covered_y for box in viewpoints)
    for (_, high), (low, _) in zip(windows, windows[1:]):
        assert low <= high + 1e-9


def test_synthetic_render_drives_the_real_contract():
    _, viewpoints = audit.plan_viewpoints()
    intrinsics = audit.head_intrinsics()
    single = audit.observe(viewpoints[0], [0.0], intrinsics=intrinsics)
    assert single["resolution"].outcome == "authorized"
    assert single["resolution"].selected_perception_track_id
    double = audit.observe(viewpoints[0], [0.0, 0.15], intrinsics=intrinsics)
    assert double["resolution"].reason == "ambiguous_candidates"
    blue = audit.observe(viewpoints[0], [0.0], intrinsics=intrinsics, color=audit.BLUE)
    assert blue["resolution"].reason == "wrong_class"


def test_scanning_uses_more_than_one_viewpoint(report):
    by_name = {item["name"]: item for item in report["scenarios"]}
    assert by_name["hidden_position_+0.60"]["outcome"] == "found"
    assert len(by_name["hidden_position_+0.60"]["steps"]) == 2
    assert by_name["outside_shelf"]["outcome"] == "not_found"
    assert len(by_name["outside_shelf"]["steps"]) == len(report["viewpoints"])
    assert by_name["two_reds_ambiguous"]["outcome"] == "ambiguous"
    assert by_name["not_settled"]["outcome"] == "not_settled"
