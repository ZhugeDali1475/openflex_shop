#!/usr/bin/env python3
"""Audit whether an old single-red run contains usable measured payload pose data.

This deliberately does not infer a gripper-to-object transform from an oracle
grasp pose.  It only reports fields actually present in the saved trace.
"""

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRACE = ROOT / "local_dependencies/openflex_roboos/runs/phase1_single_red_fixed_20261113_221656/tool_trace.json"


def audit(trace_path: Path) -> dict:
    trace = json.loads(trace_path.read_text(encoding="utf-8"))
    grasp_entries = [entry for entry in trace
                     if entry.get("selected_tool") == "grasp_object"]
    retained_events = []
    for entry in grasp_entries:
        terminal = entry.get("terminal_result", {})
        retained_events.extend(event for event in terminal.get("events", [])
                               if event.get("kind") == "retained_object_verified")
    post_grasp_states = []
    for entry in trace:
        result = entry.get("raw_result", {})
        sources = result.get("sources", {}) if isinstance(result, dict) else {}
        diagnostic = sources.get("simulation_diagnostic", {}).get("data", {})
        if diagnostic.get("attached_arm") is not None:
            post_grasp_states.append({
                "selected_tool": entry.get("selected_tool"),
                "attached_arm": diagnostic.get("attached_arm"),
                "object_pose": diagnostic.get("object_pose"),
                "source": diagnostic.get("source"),
            })
    measured_pose_available = any(
        state.get("object_pose") and state.get("source") == "mujoco_diagnostic_only_NOT_PHYSICAL"
        for state in post_grasp_states)
    return {
        "schema_version": 1,
        "trace": str(trace_path),
        "trace_sha256": hashlib.sha256(trace_path.read_bytes()).hexdigest(),
        "grasp_entry_count": len(grasp_entries),
        "independent_retention_events": len(retained_events),
        "post_grasp_attached_states": post_grasp_states,
        "measured_attachment_transform_available": False,
        "measured_attached_object_pose_available": measured_pose_available,
        "payload_route_audit_authorized": False,
        "blocking_reasons": [
            "Saved baseline has retention evidence but no post-grasp attached object pose in the trace",
            "Oracle grasp pose cannot be promoted to an independently measured attachment transform",
            "No payload route result is generated from this audit",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--output", type=Path, default=ROOT /
                        "local_dependencies/openflex_thesis/p3_route_audit_20261114/payload_evidence_audit.json")
    args = parser.parse_args()
    report = audit(args.trace.resolve(strict=True))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"measured_attachment_transform_available": report[
        "measured_attachment_transform_available"], "payload_route_audit_authorized": report[
        "payload_route_audit_authorized"]}))


if __name__ == "__main__":
    main()
