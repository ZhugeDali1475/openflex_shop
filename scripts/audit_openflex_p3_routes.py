#!/usr/bin/env python3
"""Run P3 route screening without ROS or MuJoCo dynamics stepping.

The guard uses the compiled MJCF and nominal qpos0 arm joints.  Results are
planning evidence only; payload checks remain blocked until a measured grasp
attachment transform exists.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"
sys.path.insert(0, str(BRIDGE))
from openflex_roboos_bridge.navigation_guard import NavigationGuard  # noqa: E402
from openflex_roboos_bridge.route_audit_contract import (  # noqa: E402
    RouteCandidate,
    audit_route_with_guard,
    make_competition_route_manifest,
)


def run(scene: Path) -> dict:
    guard = NavigationGuard(scene)
    names, positions = [], []
    for joint in range(guard.model.njnt):
        if guard.model.jnt_type[joint] not in (mujoco.mjtJoint.mjJNT_HINGE,
                                               mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        name = mujoco.mj_id2name(guard.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if name in {"base_x", "base_y", "base_yaw"}:
            continue
        names.append(name)
        positions.append(float(guard.model.qpos0[guard.model.jnt_qposadr[joint]]))
    joints = type("NominalJointState", (), {"name": names, "position": positions})()
    start = (0.0, 3.6, 4.71238898038469)
    routes = {
        "pickup_zone": RouteCandidate("pickup_zone", "v7-half-pickup-20261114-a",
                                       ((0.0, 2.5, 0.0), (-0.2, 2.0, 0.0), (0.2, 0.53, 0.0))),
        "sim_delivery_top_left": RouteCandidate(
            "sim_delivery_top_left", "v7-left-20261114-b",
            ((0.0, 2.5, 0.0), (-2.4, 2.7, 0.0), (-2.4, 2.7, math.pi / 2))),
        "sim_delivery_top_right": RouteCandidate(
            "sim_delivery_top_right", "v7-right-20261114-b",
            ((0.0, 2.5, 0.0), (2.4, 2.7, 0.0), (2.4, 2.7, math.pi / 2))),
    }
    manifest = make_competition_route_manifest(pickup=routes["pickup_zone"],
                                                left=routes["sim_delivery_top_left"],
                                                right=routes["sim_delivery_top_right"])
    audits = {key: audit_route_with_guard(guard, joints, start, route)
              for key, route in routes.items()}
    return {
        "schema_version": 1,
        "scope": "offline NavigationGuard route screening, nominal MJCF qpos0 joints",
        "scene": str(scene),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "start_pose_xyyaw": list(start),
        "joint_state_source": "MJCF qpos0, not live measured ROS state",
        "execution_enabled": False, "mujoco_dynamics_steps": 0, "ros_accessed": False,
        "route_manifest": manifest,
        "empty_robot_audits": audits,
        "payload_audits": {"status": "blocked_no_independently_measured_attachment_transform",
                           "validated": False},
        "limitations": [
            "Static sampled/controller-law screening only, not continuous-path proof",
            "No live joint state, ROS action, or MuJoCo dynamics used",
            "No payload transform was fabricated; payload route screening remains pending",
            "A passed empty-robot route does not authorize motion or validate release reachability",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=ROOT /
                        "local_dependencies/openflex_thesis/retail_fixture_single_red_runtime_20261113_v7/scene.xml")
    parser.add_argument("--output", type=Path, default=ROOT /
                        "local_dependencies/openflex_thesis/p3_route_audit_20261114/route_audit.json")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = run(args.scene.resolve(strict=True))
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({key: value["accepted"] for key, value in result["empty_robot_audits"].items()}))


if __name__ == "__main__":
    main()
