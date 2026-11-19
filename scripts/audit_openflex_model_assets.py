"""Read-only audit of the deployable grasp/perception model assets, without ROS.

Checks the on-disk ``openflex_models`` tree against its ``SHA256SUMS`` manifest and
records license/provenance for each asset.  It never downloads or restores
assets, never imports torch/transformers, and never starts ROS or simulation.
"""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath


ROOT = Path(__file__).resolve().parents[1]
ROOT_NAME = "openflex_models"

# Mirrors openflex_grasp/openflex_grasp/backend_health.py so runtime preflight and
# this audit agree on what "the model root is populated" means.
REQUIRED_FILES = (
    "Qwen3.5-4B/config.json",
    "sam2.1-hiera-small/config.json",
    "graspnet-baseline/checkpoint-rs.tar",
    "SHA256SUMS",
)

LICENSE_TABLE = {
    "Qwen3.5-4B": {
        "license": "Apache-2.0",
        "evidence": "LICENSE file present; README metadata license: apache-2.0",
    },
    "sam2.1-hiera-small": {
        "license": "Apache-2.0",
        "evidence": "README metadata license: apache-2.0; no standalone LICENSE file shipped",
    },
    "graspnet-baseline": {
        "license": "SJTU NonCommercial Research",
        "evidence": "LICENSE: academic/non-profit, noncommercial research use only",
    },
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalize_manifest_path(raw, root_name=ROOT_NAME):
    """Drop any foreign prefix so ``/x/y/openflex_models/a`` becomes ``a``."""

    parts = PurePosixPath(str(raw).strip()).parts
    if root_name in parts:
        index = len(parts) - 1 - parts[::-1].index(root_name)
        return "/".join(parts[index + 1:])
    if parts and parts[0] == "/":
        return "/".join(parts[1:])
    return "/".join(parts)


def parse_sums(text, root_name=ROOT_NAME):
    """Parse a ``sha256sum`` manifest into a ``{relative_path: digest}`` mapping."""

    entries = {}
    for line in str(text).splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split(None, 1)
        if len(fields) != 2:
            continue
        digest, raw_path = fields
        entries[normalize_manifest_path(raw_path, root_name)] = digest.lower()
    return entries


def _anygrasp_assets(model_root):
    if not model_root.is_dir():
        return []
    return sorted(str(path.relative_to(model_root))
                  for path in model_root.rglob("*") if "anygrasp" in path.name.lower())


def audit(model_root, *, verify_hashes=True):
    model_root = Path(model_root).expanduser()
    report = {
        "model_root": str(model_root),
        "exists": model_root.is_dir(),
        "required_files": {name: (model_root / name).is_file() for name in REQUIRED_FILES},
        "licenses": LICENSE_TABLE,
        "anygrasp_assets_present": _anygrasp_assets(model_root),
        "robot_motion_executed": False,
    }
    sums_path = model_root / "SHA256SUMS"
    if not sums_path.is_file():
        report["manifest"] = {"present": False}
        report["hash_verification"] = {"checked": 0, "matched": 0, "mismatched": [], "skipped": True}
        return report
    raw_text = sums_path.read_text(encoding="utf-8")
    entries = parse_sums(raw_text)
    absolute = [line.split(None, 1)[1] for line in raw_text.splitlines()
                if len(line.split(None, 1)) == 2 and line.split(None, 1)[1].startswith("/")]
    report["manifest"] = {
        "present": True,
        "path": str(sums_path),
        "entries": len(entries),
        "path_portable": not absolute,
        "foreign_prefix_examples": absolute[:1],
    }
    if not verify_hashes:
        report["hash_verification"] = {"checked": 0, "matched": 0, "mismatched": [], "skipped": True}
        return report
    matched, mismatched, absent = 0, [], []
    for relative, digest in sorted(entries.items()):
        candidate = model_root / relative
        if not candidate.is_file():
            absent.append(relative)
        elif sha256(candidate) == digest:
            matched += 1
        else:
            mismatched.append(relative)
    report["hash_verification"] = {
        "checked": len(entries),
        "matched": matched,
        "mismatched": mismatched,
        "absent": absent,
        "skipped": False,
    }
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-root", default=str(ROOT / ROOT_NAME))
    parser.add_argument("--no-verify", action="store_true",
                        help="skip hashing; only check presence and licenses")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    report = audit(args.model_root, verify_hashes=not args.no_verify)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
        print(args.output)
    else:
        print(text)


if __name__ == "__main__":
    main()
