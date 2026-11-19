#!/usr/bin/env python3
"""Audit one carried-object route from a saved measured diagnostic sample.

The command is intentionally fail-closed: no diagnostic JSON means no payload
audit, and the diagnostic must contain the same-sample attachment evidence
published by the MuJoCo bridge.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import mujoco


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"
sys.path.insert(0, str(BRIDGE))
from openflex_roboos_bridge.navigation_guard import NavigationGuard  # noqa: E402
from openflex_roboos_bridge.route_audit_contract import (  # noqa: E402
    RouteCandidate,
    audit_route_with_attachment_evidence,
)


def run(scene: Path, routes_path: Path, diagnostic_path: Path,
        live_state_path: Path, destination: str) -> dict:
    routes = json.loads(routes_path.read_text(encoding="utf-8"))
    diagnostic = json.loads(diagnostic_path.read_text(encoding="utf-8"))
    live_state = json.loads(live_state_path.read_text(encoding="utf-8"))
    route_data = routes["route_manifest"]["routes"].get(destination)
    if route_data is None:
        raise ValueError(f"route audit has no destination: {destination}")
    route = RouteCandidate(
        destination, route_data["version"],
        tuple(tuple(float(value) for value in waypoint)
              for waypoint in route_data["waypoints_xyyaw"]))
    guard = NavigationGuard(scene)
    sources = live_state.get("sources", {})
    required_sources = ("odometry", "joint_states", "grasp_status", "simulation_diagnostic")
    if live_state.get("holding") != "thesis_item_01":
        raise ValueError("live state does not verify the unique red object as held")
    if any(sources.get(key, {}).get("status") != "fresh" for key in required_sources):
        raise ValueError("live odometry, joints, grasp and diagnostic must all be fresh")
    stamps = [sources[key].get("source_stamp_ns") for key in required_sources]
    if any(stamp is None for stamp in stamps) or len(set(stamps)) != 1:
        raise ValueError("live payload route inputs must share one source timestamp")
    live_diagnostic = sources["simulation_diagnostic"].get("data", {})
    if live_diagnostic != diagnostic:
        raise ValueError("diagnostic file must match the same-sample live state diagnostic")
    scene_hash = hashlib.sha256(scene.read_bytes()).hexdigest()
    if live_state.get("scene_registry", {}).get("sources", {}).get("model_sha256") != scene_hash:
        raise ValueError("live state scene hash does not match the audited MJCF")
    joint_data = diagnostic.get("joint_state", {})
    joints = SimpleNamespace(name=joint_data.get("names", []),
                             position=joint_data.get("position_rad_or_m", []))
    odometry = sources["odometry"]["data"]["pose"]["pose"]
    orientation = odometry["orientation"]
    start = (
        float(odometry["position"]["x"]),
        float(odometry["position"]["y"]),
        math.atan2(2 * (orientation["w"] * orientation["z"]
                        + orientation["x"] * orientation["y"]),
                   1 - 2 * (orientation["y"] ** 2 + orientation["z"] ** 2)),
    )
    result = audit_route_with_attachment_evidence(
        guard, joints, start, route, diagnostic)
    return {
        "schema_version": 1,
        "scene": str(scene),
        "scene_sha256": scene_hash,
        "routes": str(routes_path),
        "diagnostic": str(diagnostic_path),
        "diagnostic_sha256": hashlib.sha256(diagnostic_path.read_bytes()).hexdigest(),
        "live_state": str(live_state_path),
        "live_state_sha256": hashlib.sha256(live_state_path.read_bytes()).hexdigest(),
        "source_stamp_ns": stamps[0],
        "destination": destination,
        "start_pose_xyyaw": list(start),
        "joint_state_source": "same-sample live MuJoCo diagnostic joint state",
        "audit": result,
        "execution_enabled": False,
        "mujoco_dynamics_steps": 0,
        "ros_accessed": False,
        "limitations": [
            "Static/controller-law screening only",
            "Nominal arm joints are used for collision sampling",
            "A passed payload route does not authorize navigation",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--routes", type=Path, required=True)
    parser.add_argument("--diagnostic", type=Path, required=True)
    parser.add_argument("--live-state", type=Path, required=True)
    parser.add_argument("--destination", choices=["sim_delivery_top_left", "sim_delivery_top_right"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = run(args.scene.resolve(strict=True), args.routes.resolve(strict=True),
                 args.diagnostic.resolve(strict=True), args.live_state.resolve(strict=True),
                 args.destination)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"accepted": report["audit"]["accepted"],
                      "payload_mode": report["audit"]["payload_mode"],
                      "execution_enabled": report["execution_enabled"]}))


if __name__ == "__main__":
    main()
