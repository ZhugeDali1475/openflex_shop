"""Isolated simulator trials with task-terminal-first result classification."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tarfile
import sys
import time

import yaml

from .statistics import wilson_interval


TERMINAL = re.compile(r"RECOVERY_TASK_RESULT (\{.*\})")


def classify_log(text, outer_timeout=False):
    records = []
    for match in TERMINAL.finditer(text):
        try:
            records.append(json.loads(match.group(1)))
        except json.JSONDecodeError:
            continue
    if records:
        identities = {record["task_id"] for record in records}
        outcomes = {record["success"] for record in records}
        if len(identities) != 1 or len(outcomes) != 1:
            return {"status": "invalid_terminal", "success": None}
        record = records[-1]
        return {**record, "status": "success" if record["success"] else "failed"}
    legacy = re.findall(r"MUJOCO_MISSION_(SUCCESS|FAILED)\b", text)
    if legacy:
        if len(set(legacy)) != 1:
            return {"status": "invalid_terminal", "success": None}
        duration = re.search(r"MUJOCO_MISSION_SUCCESS duration=([0-9.]+)", text)
        return {"status": "success" if legacy[-1] == "SUCCESS" else "failed",
                "success": legacy[-1] == "SUCCESS",
                "duration_sec": float(duration.group(1)) if duration else None,
                "legacy_terminal": True}
    return {"status": "outer_timeout" if outer_timeout else "infrastructure_error", "success": None}


def stop_process_group(process):
    for requested, grace in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, requested)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            continue
        if requested != signal.SIGKILL:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        break


def run_trial(root, output, scenario, method, seed, timeout_sec, model_path=""):
    root, output = Path(root).resolve(), Path(output).resolve()
    identifier = f"{scenario['scenario_id']}__{method}__{seed}"
    trial = output / identifier
    trial.mkdir(parents=True, exist_ok=False)
    event_path = trial / "recovery.jsonl"
    forced_action = method.removeprefix("probe_") if method.startswith("probe_") else ""
    selector_method = "random" if forced_action else method
    if method.startswith("learned") and not model_path:
        raise ValueError("learned trials require an explicitly supplied model")
    model_path = str(Path(model_path).resolve()) if model_path else ""
    parameters = {
        "mission_orchestrator": {"ros__parameters": {
            "enable_recovery_research": method != "original", "recovery_method": selector_method if method != "original" else "fixed",
            "recovery_forced_action": forced_action,
            "recovery_local_only": scenario.get("local_only", True),
            "recovery_model_path": model_path, "recovery_log_path": str(event_path),
            "recovery_scenario_id": scenario["scenario_id"], "recovery_seed": seed,
            "recovery_partition": scenario.get("partition", "development"),
            "recovery_budget_sec": scenario.get("budget_sec", 300.0),
            "recovery_evidence_class": "mujoco_oracle_geometry_proxy",
            "moveit_grasp_timeout_sec": scenario.get("grasp_timeout_sec", 120.0),
            "recovery_grasp_reserve_sec": scenario.get("grasp_reserve_sec", 90.0),
            "reobserve_cost_sec": scenario.get("reobserve_cost_sec", 35.0),
            "reposition_cost_sec": scenario.get("reposition_cost_sec", 35.0),
        }},
        "thesis_mujoco_mission_client": {"ros__parameters": {
            "task_id": identifier, "max_recovery_attempts": scenario.get("max_recovery_attempts", 2),
            "pickup_observation_pose": scenario.get("pickup_pose", [0.0, 0.0, 0.0]),
            "coarse_top_k": scenario.get("top_k", 3), "refined_top_k": scenario.get("top_k", 3),
        }},
        "openflex_thesis_mujoco_bridge": {"ros__parameters": {
            "random_seed": seed,
            "observation_position_noise_std_m": scenario.get("observation_noise_m", 0.0),
            "object_nominal_pose": scenario.get("object_pose", [1.12, 0.0, 0.84, 3.141592653589793]),
        }},
        "mujoco_oracle_anygrasp": {"ros__parameters": {
            "refined_lateral_spread_m": scenario.get("candidate_spread_m", 0.01),
            "refined_candidate_offsets_m": scenario.get("candidate_offsets_m", [0.0]),
        }},
        "tc_wam_server": {"ros__parameters": {
            "distance_min_m": scenario.get("distance_min_m", 0.6),
            "distance_max_m": scenario.get("distance_max_m", 1.0),
            "distance_step_m": scenario.get("distance_step_m", 0.2),
            "lateral_min_m": scenario.get("lateral_min_m", -0.3),
            "lateral_max_m": scenario.get("lateral_max_m", 0.3),
            "lateral_step_m": scenario.get("lateral_step_m", 0.15),
        }},
    }
    config_path = trial / "parameters.yaml"
    config_path.write_text(yaml.safe_dump(parameters, sort_keys=True), encoding="utf-8")
    version = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = subprocess.check_output(["git", "diff", "--binary"], cwd=root)
    manifest = {"schema_version": 1, "task_id": identifier, "scenario": scenario, "method": method,
                "seed": seed, "code_revision": version, "tracked_diff_sha256": hashlib.sha256(dirty).hexdigest(),
                "timeout_sec": timeout_sec, "evidence_class": "mujoco_oracle_geometry_proxy"}
    source_paths = sorted(path for path in (root / "openflex_ws/src/openflex_thesis").rglob("*")
                          if path.is_file() and path.suffix in (".py", ".yaml", ".xml", ".action", ".msg", ".srv", ".cfg")
                          and "__pycache__" not in path.parts)
    source_paths.append(root / "scripts/run_thesis_moveit_mission.sh")
    manifest["source_sha256"] = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in source_paths}
    with tarfile.open(trial / "source_snapshot.tar.gz", "w:gz") as archive:
        for path in source_paths:
            archive.add(path, arcname=str(path.relative_to(root)))
    (trial / "tracked.patch").write_bytes(dirty)
    manifest["model_sha256"] = hashlib.sha256(Path(model_path).read_bytes()).hexdigest() if model_path else None
    manifest["parameters_sha256"] = hashlib.sha256(config_path.read_bytes()).hexdigest()
    (trial / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(100 + seed % 100)
    environment["ROS_LOCALHOST_ONLY"] = "1"
    if method.startswith("learned"):
        learning_site = root / f".venvs/recovery/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
        if not learning_site.exists():
            raise ValueError("learning runtime missing; use scripts/run_recovery_research.sh setup")
        environment["PYTHONPATH"] = str(learning_site) + os.pathsep + environment.get("PYTHONPATH", "")
    command = ["bash", "scripts/run_thesis_moveit_mission.sh", f"experiment_config:={config_path}"]
    started = time.monotonic()
    timed_out = False
    interrupted = False
    log_path = trial / "runtime.log"
    with log_path.open("w") as stream:
        process = subprocess.Popen(command, cwd=root, env=environment, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while process.poll() is None:
                text = log_path.read_text(errors="replace")
                if TERMINAL.search(text):
                    break
                if time.monotonic() - started >= timeout_sec:
                    timed_out = True
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            interrupted = True
        finally:
            stop_process_group(process)
    result = {**manifest, **classify_log(log_path.read_text(errors="replace"), timed_out),
              "wall_duration_sec": time.monotonic() - started, "process_returncode": process.returncode}
    if interrupted and result["success"] is None:
        result["status"] = "interrupted"
    result["interrupted"] = interrupted
    if event_path.exists():
        events = [json.loads(line) for line in event_path.read_text().splitlines()]
        local = [event for event in events if event["event"] == "local_terminal"]
        result["local_success"] = bool(local and local[-1]["success"])
        result["recovery_decisions"] = sum(event["event"] == "decision" for event in events)
        result["recovery_success"] = any(event["event"] == "transition" and event.get("label") == 1 for event in events)
        result["first_failure"] = next((event for event in events if event["event"] == "failure"), None)
        result["first_failure_context"] = next((event for event in events if event["event"] == "failure_context"), None)
        result["local_duration_sec"] = local[-1].get("duration_sec") if local else None
        transitions = [event for event in events if event["event"] == "transition"]
        result["recovery_duration_sec"] = sum(event["duration_sec"] for event in transitions)
        result["repeated_failure_calls"] = sum(event["option"]["features"].get("repeat_count", 0) > 0
                                              for event in events if event["event"] == "decision")
        result["extra_base_distance_m"] = sum(event.get("distance_m", 0.0) for event in events if event["event"] == "base_motion")
        result["status_counts"] = {status: sum(event.get("outcome") == status for event in transitions)
                                   for status in ("budget_exhausted", "unsafe_or_unverified", "grasp_verified")}
    (trial / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return result


def summarize(results):
    report = {}
    for method in sorted(set(record["method"] for record in results)):
        values = [record for record in results if record["method"] == method]
        valid = [record for record in values if record["success"] is not None]
        successes = sum(record["success"] for record in valid)
        recovered = [record for record in valid if record.get("recovery_decisions", 0)
                     or record.get("first_failure") or not record["success"]]
        report[method] = {"runs": len(values), "valid_task_terminals": len(valid), "successes": successes,
                          "success_rate": successes / len(valid) if valid else None,
                          "wilson_95": wilson_interval(successes, len(valid)) if valid else None,
                          "infrastructure_or_outer_timeout": len(values) - len(valid),
                          "recovery_decisions": sum(record.get("recovery_decisions", 0) for record in values),
                          "recovery_success_rate": sum(record.get("recovery_success", False) for record in recovered) / len(recovered) if recovered else None,
                          "first_fault_trials": len(recovered),
                          "attempted_recovery_trials": sum(record.get("recovery_decisions", 0) > 0 for record in valid),
                          "repeated_failure_calls": sum(record.get("repeated_failure_calls", 0) for record in values),
                          "recovery_duration_sec_all": [record.get("recovery_duration_sec") for record in values],
                          "extra_base_distance_m_all": [record.get("extra_base_distance_m") for record in values],
                          "failure_codes": {str(code): sum(record.get("error_code") == code for record in valid)
                                            for code in sorted(set(record.get("error_code", 0) for record in valid))}}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".")
    parser.add_argument("--output", required=True)
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--timeout", type=float, default=420.0)
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    matrix = json.loads(Path(args.matrix).read_text())
    results = []
    for trial in matrix[:args.limit]:
        result = run_trial(args.root, args.output, trial["scenario"], trial["method"], trial["seed"],
                           args.timeout, trial.get("model_path", ""))
        results.append(result)
        report = summarize(results)
        Path(args.output, "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps({"task_id": result["task_id"], "status": result["status"],
                          "recovery_decisions": result.get("recovery_decisions", 0)}), flush=True)
        if result["interrupted"]:
            break


if __name__ == "__main__":
    main()
