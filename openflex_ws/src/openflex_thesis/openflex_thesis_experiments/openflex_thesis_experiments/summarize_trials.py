"""Summarize terminal results by experiment condition with Wilson intervals."""

import argparse
from collections import defaultdict
import json
from pathlib import Path

from .statistics import wilson_interval


def load_runs(root: Path):
    for metadata_path in sorted(root.glob("*/metadata.json")):
        result_path = metadata_path.parent / "result.json"
        if not result_path.exists():
            continue
        with metadata_path.open("r", encoding="utf-8") as stream:
            metadata = json.load(stream)
        with result_path.open("r", encoding="utf-8") as stream:
            result = json.load(stream)
        yield metadata, result


def summarize(root: Path):
    groups = defaultdict(list)
    for metadata, result in load_runs(root):
        condition = json.dumps(metadata.get("condition", {}), ensure_ascii=False, sort_keys=True)
        groups[(metadata.get("experiment", "unknown"), condition)].append(result)
    rows = []
    for (experiment, condition), results in sorted(groups.items()):
        successes = sum(bool(result.get("success")) for result in results)
        lower, upper = wilson_interval(successes, len(results))
        rows.append(
            {
                "experiment": experiment,
                "condition": json.loads(condition),
                "trials": len(results),
                "successes": successes,
                "success_rate": successes / len(results),
                "wilson_95_low": lower,
                "wilson_95_high": upper,
                "mean_duration_sec": sum(float(r.get("duration_sec", 0.0)) for r in results) / len(results),
            }
        )
    return rows


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    rows = summarize(args.run_root)
    rendered = json.dumps(rows, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
