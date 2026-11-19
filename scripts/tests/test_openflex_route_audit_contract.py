from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.route_audit_contract import (  # noqa: E402
    RouteAuditError,
    RouteCandidate,
    audit_route_with_guard,
    audit_route_with_attachment_evidence,
    make_competition_route_manifest,
)


def routes():
    return {
        "pickup": RouteCandidate("pickup_zone", "pickup-v1",
                                 ((0.1, 0.2, 0.0), (0.3, 0.2, 0.0))),
        "left": RouteCandidate("sim_delivery_top_left", "left-v1",
                               ((-0.4, 0.2, 1.5), (-0.6, 0.2, 1.5))),
        "right": RouteCandidate("sim_delivery_top_right", "right-v1",
                                ((0.4, 0.2, -1.5), (0.6, 0.2, -1.5))),
    }


def test_manifest_keeps_three_versioned_stations_distinct_and_non_executable():
    result = make_competition_route_manifest(**routes())
    assert set(result["routes"]) == {"pickup_zone", "sim_delivery_top_left", "sim_delivery_top_right"}
    assert result["execution_enabled"] is False
    assert result["routes"]["sim_delivery_top_left"]["version"] != result["routes"]["sim_delivery_top_right"]["version"]


def test_left_and_right_cannot_reuse_same_waypoint_sequence():
    values = routes()
    values["right"] = RouteCandidate("sim_delivery_top_right", "right-v1",
                                     values["left"].waypoints_xyyaw)
    with pytest.raises(RouteAuditError, match="cannot share"):
        make_competition_route_manifest(**values)


def test_guard_audit_never_enables_execution_or_claims_physical_validation():
    class FakeGuard:
        def check(self, joints, start, target, payload=None, via=None):
            return {"accepted": True, "scope": "fake_static_check", "via": via}

    route = routes()["left"]
    result = audit_route_with_guard(FakeGuard(), object(), (0.0, 0.0, 0.0), route,
                                    payload={"frame_id": "map"})
    assert result["accepted"] is True
    assert result["execution_enabled"] is False
    assert result["physical_navigation_validated"] is False
    assert result["payload_mode"] == "carried_red_object"


def test_measured_attachment_evidence_is_the_only_payload_route_input():
    class FakeGuard:
        def check(self, joints, start, target, payload=None, via=None):
            assert payload["frame_id"] == "map"
            assert payload["position_m"] == [1.0, 2.0, 3.0]
            return {"accepted": True, "scope": "fake_payload_static_check"}

    diagnostic = {
        "source": "mujoco_diagnostic_only_NOT_PHYSICAL",
        "stamp": {"sec": 1, "nanosec": 2},
        "object_id": "thesis_item_01", "attached_arm": "right",
        "object_pose": {"frame_id": "map", "position_m": [1.0, 2.0, 3.0],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "attached_hand_pose": {"frame_id": "map", "position_m": [0.9, 2.0, 3.0],
                                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "attachment_transform": {"parent_frame": "map", "child_frame": "openarmx_right_hand",
                                 "translation_m": [0.1, 0.0, 0.0],
                                 "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
    }
    result = audit_route_with_attachment_evidence(
        FakeGuard(), object(), (0.0, 0.0, 0.0), routes()["left"], diagnostic)
    assert result["accepted"] is True
    assert result["payload_mode"] == "carried_red_object_measured_attachment"
    assert result["payload_evidence"]["independently_measured"] is True


def test_payload_route_rejects_oracle_only_diagnostic():
    diagnostic = {"source": "mujoco_ground_truth_oracle_NOT_PHYSICAL",
                  "object_id": "thesis_item_01", "attached_arm": "right"}
    with pytest.raises(ValueError, match="diagnostic stream"):
        audit_route_with_attachment_evidence(
            object(), object(), (0.0, 0.0, 0.0), routes()["left"], diagnostic)
