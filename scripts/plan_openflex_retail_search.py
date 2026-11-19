"""Create an offline target-search plan from a retail fixture manifest."""

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"))

from openflex_thesis_mujoco.retail_search_planner import build_search_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--current-base", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest_path = args.manifest.expanduser().resolve(strict=True)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    plan = build_search_plan(manifest["products"], args.target, tuple(args.current_base))
    plan["manifest_path"] = str(manifest_path)
    plan["manifest_scene_sha256"] = manifest.get("scene_sha256")
    output = args.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "candidates": plan["candidate_instance_ids"],
                      "observations": len(plan["observations"]), "execution_enabled": False},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
