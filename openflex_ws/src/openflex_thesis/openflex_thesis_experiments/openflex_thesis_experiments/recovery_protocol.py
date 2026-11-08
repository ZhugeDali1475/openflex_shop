"""Generate development probes and a separate frozen evaluation matrix."""

import argparse
import json
import random
from pathlib import Path


def scenarios(phase="probe"):
    values = []
    for index, noise in enumerate((0.005, 0.01, 0.02, 0.035, 0.05)):
        values.append({"scenario_id": f"observation_{index}", "family": "observation",
                       "observation_noise_m": noise})
    for index, lateral in enumerate((-0.2, -0.1, 0.1, 0.2, 0.3)):
        values.append({"scenario_id": f"base_{index}", "family": "base",
                       "pickup_pose": [0.0, lateral, 0.0]})
    for index, offsets in enumerate(([0.03, 0.08, 0.0], [0.0, 0.15, 0.0],
                                     [0.0, 0.25, 0.0], [0.1, 0.1, 0.1], [0.25, 0.25, 0.25])):
        values.append({"scenario_id": f"candidates_{index}", "family": "candidates",
                       "candidate_offsets_m": offsets})
    if phase != "probe":
        generator = random.Random(2027 if phase == "collect" else 2028)
        values = []
        for family in ("observation", "base", "candidates"):
            for index in range(20 if phase == "collect" else 30):
                value = {"scenario_id": f"{phase}_{family}_{index}", "family": family}
                if family == "observation":
                    value["observation_noise_m"] = generator.uniform(0.005, 0.05)
                elif family == "base":
                    value["pickup_pose"] = [generator.uniform(-0.03, 0.03),
                                              generator.uniform(-0.3, 0.3), generator.uniform(-0.1, 0.1)]
                else:
                    value["candidate_offsets_m"] = [0.0, generator.uniform(0.025, 0.25),
                                                      generator.uniform(-0.02, 0.02)]
                values.append(value)
    return [{"budget_sec": 420.0, "max_recovery_attempts": 2 if phase != "probe" else 1,
             "top_k": 3, **value} for value in values]


def matrix(phase, learned_model="", no_memory_model=""):
    methods = ("probe_reobserve", "probe_switch_grasp", "probe_reposition") if phase == "probe" else (
        ("random",) if phase == "collect" else ("original", "heuristic", "heuristic_memory", "learned", "learned_memory"))
    if phase == "evaluate" and not (learned_model and no_memory_model):
        raise ValueError("evaluation requires independently trained memory/no-memory models")
    seed_offset = {"probe": 1000, "collect": 2000, "evaluate": 3000}[phase]
    return [{"scenario": {**scenario, "partition": "held_out" if phase == "evaluate" else "development"},
             "method": method, "seed": seed_offset + index,
             "model_path": learned_model if method == "learned_memory" else no_memory_model if method == "learned" else ""}
            for index, scenario in enumerate(scenarios(phase)) for method in methods]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("probe", "collect", "evaluate"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--model", default="")
    parser.add_argument("--no-memory-model", default="")
    args = parser.parse_args()
    Path(args.output).write_text(json.dumps(matrix(args.phase, args.model, args.no_memory_model), indent=2) + "\n")


if __name__ == "__main__":
    main()
