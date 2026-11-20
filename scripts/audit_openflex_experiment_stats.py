#!/usr/bin/env python3
"""Offline audit of the L1/L2 trial-statistics contract.

Runs a deterministic synthetic batch through ``trial_statistics.run_batch`` and
reduces it with ``layer_report``.  The point of the audit is the *accounting*:
L1 and L2 are reported as separate columns, a merged success rate is refused,
every attempt stays in the denominator, failures are attributed to a stage, and
harness errors are counted rather than dropped.

This is a statistics/contract audit only; it starts no ROS and moves nothing.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.trial_statistics import (  # noqa: E402
    CONDITIONS, TrialRecord, TrialSpec, TrialStatisticsError, combine_layers, layer_report,
    run_batch,
)

# (success probability on L1, stage attributed to a failure)
_MODEL = {
    "visible": (0.95, "grasp"),
    "occluded": (0.55, "perception"),
    "ambiguous": (0.90, "perception"),
    "invalid_depth": (0.85, "perception"),
    "empty": (0.95, "perception"),
    "unreachable_grasp": (0.30, "grasp"),
    "wrong_class": (0.90, "perception"),
    "random_slot": (0.85, "exploration"),
}


def build_specs(*, seed: int = 20261120, per_condition: int = 8) -> tuple:
    rng = np.random.default_rng(seed)
    specs = []
    for layer in ("L1", "L2"):
        for condition in CONDITIONS:
            base_p, stage = _MODEL[condition]
            probability = base_p if layer == "L1" else max(0.0, base_p - 0.25)
            for index in range(per_condition):
                success = bool(rng.random() < probability)
                specs.append(TrialSpec(
                    trial_id=f"{layer}_{condition}_{index:02d}", layer=layer,
                    evidence_source="pure_vision", condition=condition,
                    params={"success": success, "stage": stage,
                            "duration_sec": float(rng.uniform(1.0, 9.0)),
                            "localization_error_m": float(rng.uniform(0.0, 0.02))}))
    return tuple(specs)


def execute(spec: TrialSpec) -> TrialRecord:
    params = spec.params
    return TrialRecord(spec.trial_id, spec.layer, spec.evidence_source, spec.condition,
                       success=bool(params["success"]),
                       failure_stage="none" if params["success"] else str(params["stage"]),
                       duration_sec=float(params["duration_sec"]),
                       localization_error_m=float(params["localization_error_m"]))


def run_audit(*, seed: int = 20261120, per_condition: int = 8) -> dict:
    # Two extra specs whose executor raises: they must surface as counted harness failures.
    def flaky_execute(spec):
        if spec.trial_id.endswith("harness_probe"):
            raise RuntimeError("injected sensor drop")
        return execute(spec)

    specs = list(build_specs(seed=seed, per_condition=per_condition))
    specs.append(TrialSpec("L1_visible_harness_probe", "L1", "pure_vision", "visible"))
    specs.append(TrialSpec("L2_random_slot_harness_probe", "L2", "pure_vision", "random_slot"))
    records = run_batch(specs, flaky_execute)

    report = layer_report(records)
    layers = {layer: {source: metrics.to_dict() for source, metrics in sources.items()}
              for layer, sources in report.items()}

    def slice_of(layer):
        return report[layer]["pure_vision"]

    l1, l2 = slice_of("L1"), slice_of("L2")
    l1_rates = l1.success_rate
    l2_rates = l2.success_rate

    merge_refused = False
    try:
        combine_layers(records)
    except TrialStatisticsError:
        merge_refused = True

    checks = {
        "both_layers_present": {"L1", "L2"}.issubset(report.keys()),
        "failures_in_denominator": all(metrics.successes + metrics.failures == metrics.trials
                                       for metrics in (l1, l2)),
        "attribution_sums_to_failures": all(sum(metrics.attribution.values()) == metrics.failures
                                            for metrics in (l1, l2)),
        "layers_are_distinct": l1_rates != l2_rates,
        "merged_rate_forbidden": merge_refused,
        "pure_vision_only": all(source == "pure_vision"
                                for sources in report.values() for source in sources),
        "harness_failures_counted": any(record.failure_stage == "harness" for record in records),
    }

    audit = {
        "scope": "phase_C2_L1_L2_statistics_contract_offline_NOT_PHYSICAL",
        "physical_verification": False,
        "robot_motion_executed": False,
        "ros_started": False,
        "synthetic_batch": True,
        "generator": {"seed": seed, "per_condition": per_condition, "conditions": list(CONDITIONS),
                      "note": "synthetic determinism seed; not a physical result"},
        "trials_total": len(records),
        "layers": layers,
        "guards": {"layers_separate": True, "merge_forbidden": merge_refused,
                   "failures_in_denominator": checks["failures_in_denominator"],
                   "evidence_sources_separate": True, "combined_rate": None},
        "checks": checks,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    audit["passed"] = all(checks.values())
    return audit


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument("--seed", type=int, default=20261120)
    parser.add_argument("--per-condition", type=int, default=8)
    arguments = parser.parse_args(argv)
    audit = run_audit(seed=arguments.seed, per_condition=arguments.per_condition)
    text = json.dumps(audit, ensure_ascii=False, indent=2)
    if arguments.output:
        Path(arguments.output).write_text(text + "\n", encoding="utf-8")
        print(arguments.output)
    else:
        print(text)
    print(json.dumps({"passed": audit["passed"],
                      "L1_success_rate": audit["layers"]["L1"]["pure_vision"]["success_rate"],
                      "L2_success_rate": audit["layers"]["L2"]["pure_vision"]["success_rate"],
                      "checks": audit["checks"]}, ensure_ascii=False, indent=2), file=sys.stderr)
    return 0 if audit["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
