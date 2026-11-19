#!/usr/bin/env python3
"""Rebuild the OpenFlex P6 evaluation matrices from recorded evidence.

This is a *read-only* reducer: it never starts the stack, it only re-reads the
committed run artifacts under ``local_dependencies/`` and rewrites a JSON/CSV
matrix.  A second machine (or a later revision) can therefore rebuild the exact
same tables and re-check every number against the run logs.

Honest accounting rules, applied uniformly:

* every attempt stays in the denominator; a failure is never rewritten as a
  success;
* a batch is either *reported* or listed in ``excluded_batches`` **with the
  reason** it is out of the reported denominator (harness bug, pre-fix code,
  ablation configuration, invalid scene state);
* durations/fields that the evidence never recorded are emitted as ``null``,
  not guessed;
* the MuJoCo isolation boundary (``physical_verification=false``) is carried on
  every row.

Usage::

    python3 scripts/aggregate_openflex_p6_matrix.py --output-root <dir>
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNS = ROOT / "local_dependencies" / "openflex_roboos" / "runs"
THESIS = ROOT / "local_dependencies" / "openflex_thesis" / "multiskill_execution_20261114"

V9_SCENE_SHA = "6db3a9bfad3fa6f937856521ece259e06fe1efc578db858251cb630a82257995"
OBJECT_ID = "thesis_item_01"
DESTINATION_ID = "sim_delivery_top_left"
MOTION_TOOLS = {"navigate_to_target", "grasp_object", "place_to_affordance"}
TERMINAL_STATUSES = {"succeeded", "failed", "canceled"}

# --- Reported batches -------------------------------------------------------
COMPOSITE_BATCHES = {
    # driver: scripts/run_openflex_composite_delivery.py (navigate + grasp + place)
    "composite_fetch_deliver_p4_20261116": "P4 组合驱动（无 GUI）",
    "composite_fetch_deliver_gui_20261117": "P4 组合驱动（GUI，查看器修复前）",
    "composite_fetch_deliver_gui_fixed_20261117": "P4 组合驱动（GUI，查看器修复后）",
    "composite_fetch_deliver_gui_round1_20261117": "P4 组合驱动（GUI 单轮复跑）",
}
SEARCH_BATCHES = {
    # driver: scripts/run_openflex_search_and_pick.py --only match / mismatch
    "search_and_pick_p43_20261117_v2": "P4.3 官方批次（逐轮重置）",
    "search_and_pick_20261117_first": "P4.3 首轮（含 mismatch 负例）",
    "dispatch_fix_verify_20261118": "第 15 项修复后批次 A",
    "dispatch_fix_verify2_20261118": "第 15 项修复后批次 B",
    "dispatch_fix_gilattr_20261118": "第 15 项修复 + GIL 归因诊断批次 C",
}
EXCLUDED_BATCHES = {
    "composite_fetch_deliver_p4_prelifecyclefix_20261116":
        "生命周期修复前代码（Attachment precheck 缺失），非最终配置",
    "composite_fetch_deliver_ablation_nav_on_20261116":
        "消融配置（nav_on），不是被评测的最终配置",
    "composite_fetch_deliver_harnessbug_20261116":
        "测试夹具 bug（多次夹具修复轮），非 Skill 结果",
    "composite_fetch_deliver_harnessbug2_20261116":
        "测试夹具 bug",
    "composite_fetch_deliver_harnessbug3_20261116":
        "测试夹具 bug",
    "composite_fetch_deliver_harnessbug4_20261116":
        "测试夹具 bug",
    "composite_fetch_deliver_harnessbug5_20261116":
        "测试夹具 bug",
    "search_and_pick_20261117_match_retry":
        "复用了未重置场景，基座位姿偏离观测位，slot 落在图像外（设置无效）",
    "search_and_pick_p43_20261117":
        "驱动 readiness bug：只轮询 18552，两次操作 Connection refused",
    "p6_model_20261118_01":
        "栈未带 v9 场景配置启动：MCP scene_registry 目标原点为旧值 [1.12,0,0.84]，"
        "与 v9 实际原点 [0.9475,0.53,0.943] 不符（设置无效，未产生任何运动）",
}


def _recorded_stop(result, terminated):
    """Recover the recorded ``stopped_confirmed`` for an online model round.

    The delivery driver never accepts a motion terminal unless it reports
    ``stopped_confirmed is True``, and the failure path embeds the cancelled
    skill audit (``'stopped_confirmed': True/False``) inside the error string.
    We therefore *read back* the recorded value instead of guessing:

    * any ``terminal_result`` present in the trace is checked directly;
    * otherwise the value is parsed from the recorded failure feedback;
    * a round that ended with ``completion_verified`` implies the driver's hard
      per-motion stop check passed, so it is recorded as ``True``;
    * anything else stays ``None`` (unknown, never assumed).
    """
    if terminated:
        return all(bool(item.get("stopped_confirmed")) for item in terminated)
    match = re.search(r"'stopped_confirmed':\s*(True|False)", result.get("error") or "")
    if match:
        return match.group(1) == "True"
    if result.get("stopped_confirmed") is not None:
        return result["stopped_confirmed"]
    if result.get("success") and result.get("completion_verified"):
        return True
    return None


def _code_rev() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True,
            check=True,
        ).stdout.strip()
    except Exception:  # pragma: no cover - git always present in the study tree
        return "unknown"


def _load(path: Path):
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def _rounds_of_composite(batch: str):
    summary = _load(RUNS / batch / "summary.json")
    for entry in summary.get("rounds", []):
        round_dir = ROOT / entry["dir"]
        events_path = round_dir / "events.json"
        yield entry, (events_path if events_path.is_file() else None)


def _atomic_skills_from_events(events):
    """Submit events carry tool+request_id; terminal poll results carry outcome."""
    submitted = {}
    for event in events:
        tool = event.get("tool")
        result = event.get("result") or {}
        if tool in MOTION_TOOLS and result.get("request_id"):
            submitted[result["request_id"]] = (tool, result.get("arguments") or {})
    for event in events:
        result = event.get("result") or {}
        if event.get("tool") != "get_skill_result":
            continue
        if result.get("status") not in TERMINAL_STATUSES:
            continue
        request_id = (result.get("arguments") or {}).get("request_id")
        tool, arguments = submitted.get(request_id, (None, {}))
        started, finished = result.get("started_at"), result.get("finished_at")
        yield {
            "tool": tool,
            "request_id": request_id,
            "arguments": arguments,
            "status": result.get("status"),
            "success": bool(result.get("success")),
            "stopped_confirmed": result.get("stopped_confirmed"),
            "duration_sec": (round(finished - started, 1)
                             if started is not None and finished is not None else None),
            "detail": result.get("detail"),
            "grasp_verified": result.get("grasp_verified"),
            "place_verified": result.get("place_verified"),
        }


def _failure_type(text):
    if not text:
        return None
    lowered = text.lower()
    if "missing/stale telemetry" in lowered:
        return "telemetry_stale"
    if "revoked" in lowered or "status 6" in lowered or "acknowledgement" in lowered:
        return "moveit_exclusive_control_revoked"
    if "execute_grasp" in lowered or "grasp_object" in lowered:
        return "grasp_stage_failed"
    if "precheck" in lowered:
        return "attachment_precheck_missing"
    if "goal" in lowered and "reject" in lowered:
        return "controller_goal_rejected"
    if "master exited before dispatch" in lowered:
        return "master_exited_before_dispatch"
    if "budget" in lowered:
        return "budget_exhausted"
    if "out of memory" in lowered or "cuda" in lowered:
        return "gpu_oom"
    if "connection refused" in lowered:
        return "driver_dependency_unavailable"
    return "other"


def collect(new_batch=None):
    rows, atomic, excluded, provenance = [], [], {}, {
        "code_rev_at_aggregation": _code_rev(),
        "aggregated_at": datetime.now(timezone.utc).isoformat(),
        "v9_scene_sha256": V9_SCENE_SHA,
        "object_id": OBJECT_ID,
        "destination_id": DESTINATION_ID,
        "physical_verification": False,
        "root": str(ROOT),
    }

    # --- composite driver family ------------------------------------------
    for batch, label in COMPOSITE_BATCHES.items():
        for entry, events_path in _rounds_of_composite(batch):
            events = _load(events_path) if events_path else []
            times = [event.get("time") for event in events if event.get("time")]
            duration = None
            if len(times) >= 2:
                try:
                    fmt = "%Y-%m-%dT%H:%M:%S.%f%z"
                    start = datetime.strptime(times[0], fmt)
                    end = datetime.strptime(times[-1], fmt)
                    duration = round((end - start).total_seconds(), 1)
                except ValueError:
                    duration = None
            error = entry.get("error") or ""
            skills = list(_atomic_skills_from_events(events))
            stopped_confirmed = None
            if skills:
                stopped_confirmed = all(bool(skill["stopped_confirmed"]) for skill in skills)
            rows.append({
                "family": "composite_driver",
                "batch": batch,
                "label": label,
                "round": entry.get("round"),
                "run_id": entry.get("run_id"),
                "passed": bool(entry.get("passed")),
                "failure_stage": entry.get("failure_stage"),
                "failure_type": _failure_type(error),
                "error": error or None,
                "duration_sec": duration,
                "stopped_confirmed": stopped_confirmed,
                "scene_sha256": V9_SCENE_SHA,
                "physical_verification": False,
            })
            for skill in skills:
                skill.update({
                    "family": "composite_driver", "batch": batch,
                    "round": entry.get("round"), "run_id": entry.get("run_id"),
                    "scene_sha256": V9_SCENE_SHA, "physical_verification": False,
                })
                atomic.append(skill)

    # --- search_and_pick family -------------------------------------------
    search_sources = []
    for batch, label in SEARCH_BATCHES.items():
        top = RUNS / batch / "summary.json"
        if top.is_file():
            search_sources.append((batch, label, top))
        else:
            for round_dir in sorted((RUNS / batch).glob("round*")):
                summary = round_dir / "summary.json"
                if summary.is_file():
                    search_sources.append((batch, label, summary))
    for batch, label, summary_path in search_sources:
        summary = _load(summary_path)
        for entry in summary.get("rounds", []):
            terminal = entry.get("terminal") or {}
            search = terminal.get("search")
            if isinstance(search, dict):
                search_outcome, search_reason = search.get("outcome"), search.get("reason")
            else:
                search_outcome, search_reason = search, None
            detail = terminal.get("detail") or ""
            negative = search_outcome == "failed"
            rows.append({
                # A fail-closed negative control (wrong colour / occluded /
                # empty slot) is reported separately: "no motion" is the wanted
                # outcome there, so folding it into the fetch success rate would
                # silently inflate it.
                "family": ("search_and_pick_negative_control" if negative
                           else "search_and_pick"),
                "batch": batch,
                "label": label,
                "round": entry.get("round"),
                "run_id": entry.get("request_id"),
                "passed": bool(entry.get("passed")),
                "failure_stage": None if terminal.get("success") else "search_and_pick",
                "failure_type": _failure_type(detail),
                "error": detail or None,
                "search_outcome": search_outcome,
                "search_reason": search_reason,
                "duration_sec": (round(terminal["finished_at"] - terminal["started_at"], 1)
                                 if terminal.get("started_at") is not None
                                 and terminal.get("finished_at") is not None else None),
                "stopped_confirmed": terminal.get("stopped_confirmed"),
                "scene_sha256": V9_SCENE_SHA,
                "physical_verification": False,
            })

    # --- fixed-policy 3-atomic-tool family (P5) ---------------------------
    live = _load(THESIS / "p5_roboos_integration_20261117" / "live_runs.json")
    for run in live.get("runs", []):
        if run.get("policy") != "fixed":
            continue
        error = run.get("error_head") or ""
        rows.append({
            "family": "fixed_policy_3_atomic_tools",
            "batch": "p5_roboos_integration_20261117",
            "label": "P5 固定策略（navigate/grasp/place）",
            "round": run.get("run"),
            "run_id": run.get("run"),
            "passed": bool(run.get("passed")),
            "failure_stage": None if run.get("passed") else "grasp",
            "failure_type": _failure_type(error),
            "error": error or None,
            "duration_sec": None,
            "stopped_confirmed": next(
                (step.get("stopped_confirmed") for step in run.get("steps", [])
                 if step.get("success") is False), None),
            "scene_sha256": V9_SCENE_SHA,
            "physical_verification": False,
        })
        for step in run.get("steps", []):
            if step.get("tool") not in MOTION_TOOLS:
                continue
            atomic.append({
                "family": "fixed_policy_3_atomic_tools",
                "batch": "p5_roboos_integration_20261117",
                "round": run.get("run"), "run_id": run.get("run"),
                "tool": step.get("tool"), "request_id": None,
                "arguments": step.get("arguments") or {},
                "status": step.get("status"), "success": bool(step.get("success")),
                "stopped_confirmed": step.get("stopped_confirmed"),
                "duration_sec": None, "detail": None,
                "grasp_verified": None, "place_verified": None,
                "scene_sha256": V9_SCENE_SHA, "physical_verification": False,
            })

    # --- model policy (no online evidence this round) ---------------------
    for run in live.get("runs", []):
        if run.get("policy") != "model":
            continue
        rows.append({
            # Recorded before the local-model VRAM fix: the model server could
            # not coexist with the MuJoCo GPU renderer, so every attempt died in
            # dispatch with no model decision at all.
            "family": "model_policy_before_vram_fix",
            "batch": "p5_roboos_integration_20261117",
            "label": "P5 模型策略（RoboBrain 决策）",
            "round": run.get("run"), "run_id": run.get("run"),
            "passed": bool(run.get("passed", False)),
            "failure_stage": "master_dispatch",
            "failure_type": _failure_type(run.get("error_head") or "gpu_oom"),
            "error": run.get("error_head"),
            "duration_sec": None, "stopped_confirmed": run.get("stopped_confirmed"),
            "scene_sha256": V9_SCENE_SHA, "physical_verification": False,
        })

    # --- model policy, online after the VRAM fix --------------------------
    for run_dir in sorted(RUNS.glob("p6_model_*")):
        result_path = run_dir / "result.json"
        if not result_path.is_file():
            continue
        if run_dir.name in EXCLUDED_BATCHES:
            continue
        result = _load(result_path)
        if result.get("policy") != "model" or "GlobalAgent" not in str(result.get("scope", "")):
            continue
        budget = result.get("budget") or {}
        spent = budget.get("spent") or {}
        tool_trace_path = run_dir / "tool_trace.json"
        steps, tools, terminated = [], [], []
        if tool_trace_path.is_file():
            steps = _load(tool_trace_path)
            tools = [step.get("selected_tool") for step in steps]
            terminated = [step.get("terminal_result") for step in steps
                          if step.get("terminal_result")]
        stopped = _recorded_stop(result, terminated)
        rows.append({
            "family": "model_policy_online",
            "batch": run_dir.name,
            "label": "P6 模型策略在线（RoboBrain2.5-4B 决策）",
            "round": run_dir.name, "run_id": result.get("task_id"),
            "passed": bool(result.get("passed", False)),
            "failure_stage": None if result.get("success") else "model_loop",
            "failure_type": _failure_type(result.get("error") or ""),
            "error": result.get("error"),
            "duration_sec": None,
            "stopped_confirmed": stopped,
            "completion_verified": bool(result.get("completion_verified", False)),
            "model_steps": spent.get("model_steps"),
            "motion_calls": spent.get("motion_calls"),
            "model_tools": " ".join(str(tool) for tool in tools),
            "scene_sha256": V9_SCENE_SHA, "physical_verification": False,
        })

    for batch, reason in EXCLUDED_BATCHES.items():
        if (RUNS / batch).is_dir():
            excluded[batch] = reason

    # --- a fresh batch, if one was produced after the fix -----------------
    if new_batch:
        batch_dir = RUNS / new_batch
        for round_dir in sorted(batch_dir.glob("round*")):
            summary_path = round_dir / "summary.json"
            if not summary_path.is_file():
                continue
            for entry in _load(summary_path).get("rounds", []):
                terminal = entry.get("terminal") or {}
                detail = terminal.get("detail") or ""
                rows.append({
                    "family": "search_and_pick_after_fix",
                    "batch": new_batch, "label": new_batch,
                    "round": f"{round_dir.name}", "run_id": entry.get("request_id"),
                    "passed": bool(entry.get("passed")),
                    "failure_stage": None if terminal.get("success") else "grasp",
                    "failure_type": _failure_type(detail),
                    "error": detail or None,
                    "duration_sec": (round(terminal["finished_at"] - terminal["started_at"], 1)
                                     if terminal.get("started_at") is not None
                                     and terminal.get("finished_at") is not None else None),
                    "stopped_confirmed": terminal.get("stopped_confirmed"),
                    "scene_sha256": V9_SCENE_SHA, "physical_verification": False,
                })

    return rows, atomic, excluded, provenance


def summarize(rows):
    groups = {}
    for row in rows:
        key = row["family"]
        group = groups.setdefault(key, {"total": 0, "passed": 0, "stopped_confirmed": 0,
                                        "failure_types": {}})
        group["total"] += 1
        group["passed"] += 1 if row["passed"] else 0
        if row.get("stopped_confirmed"):
            group["stopped_confirmed"] += 1
        if not row["passed"]:
            failure = row.get("failure_type") or "unknown"
            group["failure_types"][failure] = group["failure_types"].get(failure, 0) + 1
    for group in groups.values():
        group["rate"] = round(group["passed"] / group["total"], 4) if group["total"] else None
    return groups


def summarize_atomic(atomic):
    groups = {}
    for row in atomic:
        key = row["tool"]
        group = groups.setdefault(key, {"total": 0, "succeeded": 0, "failures": {}})
        group["total"] += 1
        if row["success"]:
            group["succeeded"] += 1
        else:
            status = row.get("status") or "unknown"
            group["failures"][status] = group["failures"].get(status, 0) + 1
    for group in groups.values():
        group["rate"] = round(group["succeeded"] / group["total"], 4) if group["total"] else None
    return groups


def summarize_by_batch(rows):
    """Per-batch totals: the host load differs between batches, so a single
    family rate can hide a confound; both views are published."""
    batches = {}
    for row in rows:
        key = f"{row['batch']} [{row['family']}]"
        group = batches.setdefault(key, {"family": row["family"], "total": 0, "passed": 0,
                                         "failure_types": {}})
        group["total"] += 1
        group["passed"] += 1 if row["passed"] else 0
        if not row["passed"]:
            failure = row.get("failure_type") or "unknown"
            group["failure_types"][failure] = group["failure_types"].get(failure, 0) + 1
    for group in batches.values():
        group["rate"] = round(group["passed"] / group["total"], 4) if group["total"] else None
    return batches


def _write_csv(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--new-batch", default=None,
                        help="optional batch dir name produced after the item-15 fix")
    args = parser.parse_args()

    out = Path(args.output_root)
    out.mkdir(parents=True, exist_ok=True)
    rows, atomic, excluded, provenance = collect(args.new_batch)

    matrix = {
        "provenance": provenance,
        "reported_batches": {**COMPOSITE_BATCHES, **SEARCH_BATCHES,
                             "p5_roboos_integration_20261117": "P5 固定策略在线批次"},
        "excluded_batches": excluded,
        "group_summary": summarize(rows),
        "batch_summary": summarize_by_batch(rows),
        "atomic_skill_summary": summarize_atomic(atomic),
        "rows": rows,
        "atomic_skill_rows": atomic,
    }
    (out / "experiment_matrix.json").write_text(
        json.dumps(matrix, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")

    _write_csv(out / "composite_rounds.csv", rows, [
        "family", "batch", "label", "round", "run_id", "passed", "failure_stage",
        "failure_type", "search_outcome", "search_reason", "duration_sec",
        "stopped_confirmed", "model_steps", "motion_calls", "model_tools", "error",
        "scene_sha256", "physical_verification",
    ])
    _write_csv(out / "atomic_skills.csv", atomic, [
        "family", "batch", "round", "run_id", "tool", "request_id", "status",
        "success", "stopped_confirmed", "duration_sec", "grasp_verified",
        "place_verified", "detail", "scene_sha256", "physical_verification",
    ])
    (out / "excluded_batches.json").write_text(
        json.dumps(excluded, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")

    print(json.dumps({"output_root": str(out), "rounds": len(rows),
                      "atomic_skill_rows": len(atomic),
                      "group_summary": matrix["group_summary"],
                      "atomic_skill_summary": matrix["atomic_skill_summary"]},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
