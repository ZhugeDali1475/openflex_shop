"""Versioned, planning-only route audit for the two-table competition half."""

from dataclasses import dataclass
import math
from typing import Mapping, Sequence, Tuple

from .attachment_evidence import validate_attachment_evidence


class RouteAuditError(ValueError):
    pass


@dataclass(frozen=True)
class RouteCandidate:
    station_id: str
    version: str
    waypoints_xyyaw: Tuple[Tuple[float, float, float], ...]

    def validate(self) -> None:
        if self.station_id not in {"pickup_zone", "sim_delivery_top_left", "sim_delivery_top_right"}:
            raise RouteAuditError(f"unsupported competition station: {self.station_id}")
        if not isinstance(self.version, str) or not self.version.strip():
            raise RouteAuditError("route version is required")
        if not self.waypoints_xyyaw:
            raise RouteAuditError("route must contain at least one waypoint")
        for waypoint in self.waypoints_xyyaw:
            if len(waypoint) != 3 or not all(math.isfinite(float(value)) for value in waypoint):
                raise RouteAuditError("route waypoints must contain finite x/y/yaw")


def make_competition_route_manifest(*, pickup: RouteCandidate,
                                    left: RouteCandidate, right: RouteCandidate) -> Mapping[str, object]:
    routes = (pickup, left, right)
    for route in routes:
        route.validate()
    if len({route.station_id for route in routes}) != 3:
        raise RouteAuditError("pickup and left/right delivery routes must be distinct station IDs")
    if left.waypoints_xyyaw == right.waypoints_xyyaw:
        raise RouteAuditError("left and right destinations cannot share a route")
    return {
        "schema_version": 1,
        "execution_enabled": False,
        "physical_navigation_validated": False,
        "routes": {route.station_id: {
            "version": route.version,
            "waypoints_xyyaw": [list(item) for item in route.waypoints_xyyaw],
            "validated_empty_robot": False,
            "validated_with_payload": False,
        } for route in routes},
        "limitations": [
            "Discrete and controller-law preflight only",
            "No ROS action or MuJoCo dynamics were started",
            "A failed station remains non-executable until a new audited version passes",
        ],
    }


def audit_route_with_guard(guard, joint_states, start_xyyaw: Sequence[float],
                           route: RouteCandidate, *, payload=None) -> dict:
    """Run the existing static/controller-law guard only; never dispatch motion."""
    route.validate()
    if len(start_xyyaw) != 3 or not all(math.isfinite(float(value)) for value in start_xyyaw):
        raise RouteAuditError("start pose must contain finite x/y/yaw")
    result = guard.check(joint_states, start_xyyaw, route.waypoints_xyyaw[-1],
                         payload=payload, via=route.waypoints_xyyaw[:-1])
    return {
        "station_id": route.station_id,
        "route_version": route.version,
        "payload_mode": "carried_red_object" if payload is not None else "empty_robot",
        "accepted": bool(result.get("accepted")),
        "guard_result": result,
        "execution_enabled": False,
        "physical_navigation_validated": False,
    }


def audit_route_with_attachment_evidence(guard, joint_states,
                                         start_xyyaw: Sequence[float],
                                         route: RouteCandidate,
                                         diagnostic: Mapping[str, object], *,
                                         object_id: str = "thesis_item_01",
                                         arm: str = "right") -> dict:
    """Screen a carried-object route from one validated diagnostic sample.

    The adapter intentionally passes only the measured object pose to the
    existing MuJoCo guard.  The hand pose and relative transform remain in the
    audit record; they are not recomputed from an oracle grasp candidate.
    """
    evidence = validate_attachment_evidence(diagnostic, object_id=object_id, arm=arm)
    result = audit_route_with_guard(
        guard, joint_states, start_xyyaw, route,
        payload={"frame_id": "map", "position_m": evidence["object_pose"]["position_m"],
                 "orientation_xyzw": evidence["object_pose"]["orientation_xyzw"]})
    result["payload_evidence"] = evidence
    result["payload_mode"] = "carried_red_object_measured_attachment"
    return result
