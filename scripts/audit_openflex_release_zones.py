#!/usr/bin/env python3
"""Derive geometric release-point candidates from the fixed v7 scene only."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"
MUJOCO_PKG = ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"
sys.path.insert(0, str(BRIDGE))
sys.path.insert(0, str(MUJOCO_PKG))
from openflex_roboos_bridge.release_zone_contract import plan_release_zone  # noqa: E402


def audit(scene: Path, route_audit: Path) -> dict:
    route_data = json.loads(route_audit.read_text(encoding="utf-8"))
    root = ET.parse(scene).getroot()
    world = root.find("worldbody")
    if world is None:
        raise ValueError("scene is missing worldbody")
    targets = {
        "sim_delivery_top_left": ("thesis_delivery_table", "thesis_delivery_top"),
        "sim_delivery_top_right": ("counter_top_right", "counter_top_right_top"),
    }
    report = {}
    for destination, (body_name, geom_name) in targets.items():
        body = world.find(f"body[@name='{body_name}']")
        geom = body.find(f"geom[@name='{geom_name}']") if body is not None else None
        if body is None or geom is None or geom.get("type") != "box":
            raise ValueError(f"missing table box geometry: {body_name}/{geom_name}")
        body_pos = [float(value) for value in body.get("pos", "0 0 0").split()]
        geom_pos = [float(value) for value in geom.get("pos", "0 0 0").split()]
        half = [float(value) for value in geom.get("size", "").split()]
        surface_center = [body_pos[i] + geom_pos[i] for i in range(3)]
        surface_center[2] += half[2]
        route = route_data["route_manifest"]["routes"][destination]
        base_pose = route["waypoints_xyyaw"][-1]
        candidate = plan_release_zone(
            destination, surface_center_xyz=surface_center,
            surface_dimensions_xy=[2 * half[0], 2 * half[1]],
            base_xy=base_pose[:2], version=f"{route['version']}-release-v1")
        report[destination] = {
            "table_body": body_name, "surface_geom": geom_name,
            "surface_center_xyz": list(candidate.surface_center_xyz),
            "surface_dimensions_xy": [2 * value for value in candidate.surface_half_size_xy],
            "route_terminal_base_xyyaw": base_pose,
            "release_center_candidate_xyz": list(candidate.release_center_xyz),
            "release_zone_version": candidate.version,
            "reachable_by_planar_radius_model_only": True,
            "arm_ik_validated": False,
            "full_robot_collision_validated": False,
            "execution_enabled": False,
        }
    if report["sim_delivery_top_left"]["release_center_candidate_xyz"] == report[
            "sim_delivery_top_right"]["release_center_candidate_xyz"]:
        raise ValueError("left and right release centers unexpectedly match")
    return {
        "schema_version": 1,
        "scene": str(scene),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "route_audit": str(route_audit),
        "release_zone_candidates": report,
        "execution_enabled": False,
        "mujoco_dynamics_steps": 0,
        "ros_accessed": False,
        "limitations": [
            "Reachability uses a planar radius estimate, not OpenFlex arm IK",
            "No collision-aware arm trajectory or release/settling test was run",
            "Candidates are not authorized for motion",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=ROOT /
                        "local_dependencies/openflex_thesis/retail_fixture_single_red_runtime_20261113_v7/scene.xml")
    parser.add_argument("--routes", type=Path, default=ROOT /
                        "local_dependencies/openflex_thesis/p3_route_audit_20261114/route_audit.json")
    parser.add_argument("--output", type=Path, default=ROOT /
                        "local_dependencies/openflex_thesis/p3_route_audit_20261114/release_zone_candidates.json")
    args = parser.parse_args()
    report = audit(args.scene.resolve(strict=True), args.routes.resolve(strict=True))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({key: value["release_center_candidate_xyz"] for key, value in
                      report["release_zone_candidates"].items()}))


if __name__ == "__main__":
    main()
