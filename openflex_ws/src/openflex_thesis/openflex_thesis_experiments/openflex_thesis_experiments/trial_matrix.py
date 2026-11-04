"""Expand a compact YAML protocol into a deterministic JSONL trial manifest."""

import argparse
from itertools import product
import json
from pathlib import Path
from typing import Dict, Iterable, List

import yaml


def expand_experiment(experiment: dict) -> List[dict]:
    name = str(experiment["name"])
    repetitions = int(experiment.get("repetitions", 1))
    if repetitions <= 0:
        raise ValueError(f"{name}: repetitions must be positive")
    factors: Dict[str, list] = experiment.get("factors", {})
    keys = list(factors.keys())
    values = [list(factors[key]) for key in keys]
    if any(not value for value in values):
        raise ValueError(f"{name}: factor levels must not be empty")
    combinations = product(*values) if values else [tuple()]
    rows = []
    sequence = 0
    for combination in combinations:
        condition = dict(zip(keys, combination))
        for repetition in range(1, repetitions + 1):
            sequence += 1
            rows.append(
                {
                    "trial_id": f"{name}-{sequence:04d}",
                    "experiment": name,
                    "repetition": repetition,
                    "condition": condition,
                    "required_metrics": list(experiment.get("required_metrics", [])),
                }
            )
    return rows


def expand_protocol(protocol: dict) -> List[dict]:
    rows: List[dict] = []
    seen = set()
    for experiment in protocol.get("experiments", []):
        for row in expand_experiment(experiment):
            if row["trial_id"] in seen:
                raise ValueError(f"duplicate trial_id: {row['trial_id']}")
            seen.add(row["trial_id"])
            row["protocol_version"] = protocol.get("protocol_version", "unknown")
            rows.append(row)
    return rows


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("protocol", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    with args.protocol.open("r", encoding="utf-8") as stream:
        protocol = yaml.safe_load(stream) or {}
    rows = expand_protocol(protocol)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    print(f"wrote {len(rows)} trials to {args.output}")
