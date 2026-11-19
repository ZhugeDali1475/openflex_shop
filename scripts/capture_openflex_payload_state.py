#!/usr/bin/env python3
"""Persist one fresh, synchronized attached-object MuJoCo state for route audit."""

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_runtime import request_json, save  # noqa: E402
from openflex_roboos_bridge.attachment_evidence import validate_attachment_evidence  # noqa: E402


def capture(scene: Path, output: Path) -> dict:
    if os.environ.get("ROS_DOMAIN_ID") != "83":
        raise RuntimeError("Payload evidence capture requires isolated ROS domain 83")
    snapshot = request_json("http://127.0.0.1:18552/state")
    if snapshot.get("environment") != "mujoco_simulation" or snapshot.get("physical_verification") is not False:
        raise ValueError("Expected a simulation-only snapshot, never physical evidence")
    if snapshot.get("holding") != "thesis_item_01":
        raise ValueError("Fresh independent state does not verify the unique red object is held")
    sources = snapshot.get("sources", {})
    required = ("odometry", "joint_states", "grasp_status", "simulation_diagnostic")
    if any(sources.get(name, {}).get("status") != "fresh" for name in required):
        raise ValueError("Odometry, joints, grasp and diagnostic must all be fresh")
    stamps = [sources[name].get("source_stamp_ns") for name in required]
    if any(stamp is None for stamp in stamps) or len(set(stamps)) != 1:
        raise ValueError("Payload inputs must come from the same source timestamp")
    diagnostic = sources["simulation_diagnostic"].get("data")
    attachment = validate_attachment_evidence(diagnostic, object_id="thesis_item_01", arm="right")
    if diagnostic.get("safety_stop") is not None:
        raise ValueError("Active MuJoCo safety stop blocks payload route audit")
    scene_hash = hashlib.sha256(scene.read_bytes()).hexdigest()
    registry = snapshot.get("scene_registry", {}).get("sources", {})
    if registry.get("model_sha256") != scene_hash:
        raise ValueError("Live scene model hash differs from the requested route-audit scene")
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite payload evidence: {output}")
    output.mkdir(parents=True)
    save(output / "live_state.json", snapshot)
    save(output / "diagnostic.json", diagnostic)
    manifest = {
        "schema_version": 1,
        "captured_at": datetime.now().astimezone().isoformat(),
        "scope": "fresh_same_sample_isolated_MuJoCo_payload_state_NOT_PHYSICAL",
        "ros_domain_id": 83,
        "request_id": snapshot.get("request_id"),
        "source_stamp_ns": stamps[0],
        "scene": str(scene),
        "scene_sha256": scene_hash,
        "live_state_sha256": hashlib.sha256((output / "live_state.json").read_bytes()).hexdigest(),
        "diagnostic_sha256": hashlib.sha256((output / "diagnostic.json").read_bytes()).hexdigest(),
        "holding": snapshot["holding"],
        "attachment_evidence": attachment,
        "safety_stop": diagnostic.get("safety_stop"),
        "execution_enabled": False,
        "mujoco_dynamics_steps": 0,
    }
    save(output / "manifest.json", manifest)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = capture(args.scene.expanduser().resolve(strict=True), args.output.expanduser().resolve())
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
