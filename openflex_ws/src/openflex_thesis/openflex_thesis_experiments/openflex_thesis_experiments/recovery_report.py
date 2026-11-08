"""Evidence audit, matched comparisons and a conservative research go/no-go gate."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
from statistics import mean

from .recovery_benchmark import classify_log, summarize


def bootstrap_difference(differences, seed=7, repetitions=5000):
    if not differences:
        return None
    generator = random.Random(seed)
    estimates = sorted(mean(generator.choices(differences, k=len(differences)))
                       for _ in range(repetitions))
    return {"pairs": len(differences), "mean_difference": mean(differences),
            "bootstrap_95": [estimates[int(0.025 * repetitions)], estimates[int(0.975 * repetitions)]]}


def paired_comparison(results, reference, proposed):
    by_method = {method: {(record["scenario"]["scenario_id"], record["seed"]): record
                         for record in results if record["method"] == method}
                 for method in (reference, proposed)}
    differences = []
    rejected = 0
    for key in by_method[reference].keys() & by_method[proposed].keys():
        first, second = by_method[reference][key], by_method[proposed][key]
        if (first["success"] is None or second["success"] is None
                or not first.get("source_sha256")
                or first.get("source_sha256") != second.get("source_sha256")
                or first["scenario"] != second["scenario"]):
            rejected += 1
            continue
        differences.append(int(second["success"]) - int(first["success"]))
    return {"reference": reference, "proposed": proposed,
            "success_difference": bootstrap_difference(differences), "rejected_pairs": rejected}


def probe_gate(results):
    by_scene = {}
    for record in results:
        if record["method"].startswith("probe_"):
            by_scene.setdefault(record["scenario"]["scenario_id"], []).append(record)
    comparable = []
    winners = set()
    advantage_pairs = set()
    for scene, values in by_scene.items():
        attempted = [record for record in values if record.get("recovery_decisions", 0)
                     and record["success"] is not None and record.get("first_failure_context")]
        if len(attempted) < 2:
            continue
        first = attempted[0]
        context = first["first_failure_context"]
        matched = all(record.get("source_sha256") == first.get("source_sha256")
                      and record["first_failure"]["reason"] == first["first_failure"]["reason"]
                      and math.dist(record["first_failure_context"]["base"], context["base"]) <= 0.03
                      and math.dist(record["first_failure_context"]["target"], context["target"]) <= 0.01
                      for record in attempted)
        if not matched:
            continue
        outcomes = {record["method"]: record.get("recovery_success", False) for record in attempted}
        comparable.append({"scenario_id": scene, "outcomes": outcomes})
        if len(set(outcomes.values())) > 1:
            winners.update(method for method, success in outcomes.items() if success)
            advantage_pairs.update((first, second) for first, success in outcomes.items()
                                   for second, other in outcomes.items() if success and not other)
    reasons = []
    if len(comparable) < 10:
        reasons.append("fewer than ten matched failure configurations with multiple actually executed actions")
    if not any((second, first) in advantage_pairs for first, second in advantage_pairs):
        reasons.append("cross-state reversal of action advantages is not established")
    collection_reasons = list(reasons)
    if not any(record.get("repeated_failure_calls", 0) for record in results):
        reasons.append("repeated-failure and memory benefit evidence is missing")
    return {"ready_for_expansion": not reasons, "ready_for_collection": not collection_reasons,
            "matched_configurations": comparable,
            "distinct_advantaged_actions": sorted(winners), "reasons": reasons,
            "limitation": "state proximity is not an exact simulator/planner snapshot replay"}


def report(paths):
    results = [json.loads(Path(path).read_text()) for path in paths]
    identifiers = [record["task_id"] for record in results]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("duplicate task identities cannot be counted as independent trials")
    return {"summary": summarize(results), "research_gate": probe_gate(results),
            "comparisons": [paired_comparison(results, reference, "learned_memory") for reference in (
                "original", "heuristic", "heuristic_memory", "learned")],
            "trial_paths": [str(path) for path in paths],
            "evidence_class": "mujoco_oracle_geometry_proxy",
            "claim": "descriptive simulator results only; no automatic claim of improvement or real-robot validity"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="*")
    parser.add_argument("--audit-legacy")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.audit_legacy:
        records = []
        for path in sorted(Path(args.audit_legacy).glob("run_*.log")):
            content = path.read_text(errors="replace")
            records.append({"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                            **classify_log(content, True)})
        result = {"records": records, "runs": len(records),
                  "successes": sum(record["success"] is True for record in records),
                  "failures": sum(record["success"] is False for record in records),
                  "claim": "historical task-terminal audit, not a recovery-algorithm comparison"}
    else:
        if not args.inputs:
            parser.error("result.json inputs or --audit-legacy are required")
        result = report(args.inputs)
    Path(args.output).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result.get("research_gate", {key: result[key] for key in ("runs", "successes", "failures") if key in result}), ensure_ascii=False))


if __name__ == "__main__":
    main()
