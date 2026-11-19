"""Read-only runtime audit of the GraspNet/perception virtualenvs, without ROS.

The audit never imports torch into the *system* interpreter: it shells out to the
migrated virtualenvs so a broken CUDA extension or a stale interpreter cannot
crash this process.  Every check is fail-soft -- a missing venv is reported as
``skipped`` rather than raising -- so the script is safe to run on hosts where
the migrated environment has not been installed.

It verifies, in order of increasing cost:

* the virtualenv ``bin`` scripts carry a shebang that is valid on this host
  (migration copied scripts that still pointed at the old ``/home/openflex`` root);
* the rebuilt CUDA extension modules (``pointnet2._ext``, ``knn_pytorch``) load
  on this interpreter and their forward kernels run on the local GPU;
* the GraspNet checkpoint loads and produces a finite candidate set on the
  bundled ``doc/example_data`` sample;
* the perception venv can import torch/transformers.

It never starts ROS or simulation, never downloads assets and never executes
robot motion.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GRASPNET_VENV = ROOT / ".venvs" / "openflex_graspnet"
DEFAULT_PERCEPTION_VENV = ROOT / ".venvs" / "openflex_perception"
DEFAULT_MODEL_ROOT = ROOT / "openflex_models"
DEFAULT_GRASPNET_REPO = ROOT / "openflex_ws" / "src" / "openflex_grasp" / "graspnet-baseline"
DEFAULT_CHECKPOINT = DEFAULT_MODEL_ROOT / "graspnet-baseline" / "checkpoint-rs.tar"

PROBE_MARKER = "OPENFLEX_PROBE_JSON="

# Shebangs imported from the old deployment root are dead on this host.  Only the
# ``/home/openflex`` root is treated as foreign; a venv deliberately installed
# elsewhere is out of scope for this audit.
FOREIGN_SHEBANG_PREFIXES = ("/home/openflex/",)


def venv_python(venv):
    return Path(venv).expanduser() / "bin" / "python"


def foreign_shebangs(bin_dir, prefixes=FOREIGN_SHEBANG_PREFIXES):
    """Return ``bin/*`` scripts whose shebang still points at a foreign root."""

    bin_dir = Path(bin_dir)
    if not bin_dir.is_dir():
        return []
    found = []
    for entry in sorted(bin_dir.iterdir()):
        if entry.is_symlink() or not entry.is_file():
            continue
        try:
            with entry.open("rb") as stream:
                first = stream.readline(1024)
        except OSError:
            continue
        if not first.startswith(b"#!"):
            continue
        text = first.decode("utf-8", "replace").strip()
        if any(text == "#!" + prefix.rstrip("/") or text.startswith("#!" + prefix)
               for prefix in prefixes):
            found.append({"script": entry.name, "shebang": text[2:].strip()})
    return found


def parse_probe(stdout, marker=PROBE_MARKER):
    """Extract the last ``marker`` + JSON payload emitted by a probe subprocess."""

    payload = None
    for line in str(stdout).splitlines():
        if line.startswith(marker):
            payload = line[len(marker):].strip()
    if payload is None:
        return None
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return None


def run_python(python, code, *, args=(), timeout=1800, env=None, cwd=None):
    """Run ``code`` with the given interpreter; returns the completed process."""

    command = [str(python), "-c", code, *[str(arg) for arg in args]]
    return subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
        cwd=None if cwd is None else str(cwd),
    )


CUDA_OPS_PROBE = r'''
import json, sys, traceback
config = json.loads(sys.argv[1])
report = {"probe": "cuda_ops", "ops": {}, "errors": []}
try:
    import torch  # must precede the extensions so libc10 resolves
    report["torch"] = torch.__version__
    report["cuda_available"] = bool(torch.cuda.is_available())
    report["device_capability"] = (
        list(torch.cuda.get_device_capability(0)) if torch.cuda.is_available() else None
    )
    if config.get("require_cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA required but torch.cuda.is_available() is false")
    import pointnet2._ext as ext
    import knn_pytorch.knn_pytorch as knn
    report["pointnet2_ext"] = ext.__file__
    report["knn_module"] = knn.__file__
    device = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(0)
    b, n, c = 2, 64, 3
    xyz = torch.rand(b, n, 3, device=device)
    idx = ext.furthest_point_sampling(xyz.contiguous(), 16)
    feat = torch.rand(b, c, n, device=device)
    gathered = ext.gather_points(feat.contiguous(), idx.contiguous())
    new_xyz = torch.rand(b, 8, 3, device=device)
    gidx = ext.ball_query(new_xyz.contiguous(), xyz.contiguous(), 0.5, 4)
    grouped = ext.group_points(feat.contiguous(), gidx.contiguous())
    dist, tidx = ext.three_nn(new_xyz.contiguous(), xyz.contiguous())
    weight = torch.rand(b, 8, 3, device=device)
    interp = ext.three_interpolate(feat.contiguous(), tidx.contiguous(), weight.contiguous())
    rot = torch.eye(3, device=device).repeat(b, 8, 1, 1).contiguous()
    cidx = ext.cylinder_query(new_xyz.contiguous(), xyz.contiguous(), rot, 0.5, 0.5, 0.5, 4)
    ref = torch.rand(1, 3, 256, device=device)
    qry = torch.rand(1, 3, 32, device=device)
    inds = torch.empty(1, 8, 32, dtype=torch.long, device=device)
    knn.knn(ref.contiguous(), qry.contiguous(), inds)
    report["ops"] = {
        "device": device,
        "furthest_point_sampling": tuple(idx.shape) == (b, 16),
        "gather_points": tuple(gathered.shape) == (b, c, 16),
        "ball_query": tuple(gidx.shape) == (b, 8, 4),
        "group_points": tuple(grouped.shape) == (b, c, 8, 4),
        "cylinder_query": tuple(cidx.shape) == (b, 8, 4),
        "three_nn": tuple(dist.shape) == (b, 8, 3) and tuple(tidx.shape) == (b, 8, 3),
        "three_interpolate": tuple(interp.shape) == (b, c, 8),
        "knn": tuple(inds.shape) == (1, 8, 32),
    }
    report["ok"] = all(report["ops"].values())
except Exception:
    report["ok"] = False
    report["errors"].append(traceback.format_exc())
print("OPENFLEX_PROBE_JSON=" + json.dumps(report))
sys.exit(0 if report.get("ok") else 3)
'''


INFERENCE_PROBE = r'''
import json, sys, traceback
from pathlib import Path
config = json.loads(sys.argv[1])
report = {"probe": "checkpoint_inference", "errors": []}
try:
    import numpy as np
    from PIL import Image
    import scipy.io as scio
    sys.path.insert(0, config["package_root"])
    from openflex_grasp.graspnet_backend import GraspNetBackend
    data = Path(config["example_data"])
    meta = scio.loadmat(str(data / "meta.mat"))
    intrinsic = np.asarray(meta["intrinsic_matrix"], dtype=np.float64)
    factor = float(np.asarray(meta["factor_depth"]).reshape(-1)[0])
    depth = np.asarray(Image.open(data / "depth.png"))
    mask = np.asarray(Image.open(data / "workspace_mask.png").convert("L"))
    depth_m = depth.astype(np.float64) / factor
    fx, fy = intrinsic[0, 0], intrinsic[1, 1]
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]
    rows, cols = np.where((mask > 0) & (depth_m > 0))
    z = depth_m[rows, cols]
    points = np.stack(
        [(cols - cx) * z / fx, (rows - cy) * z / fy, z], axis=1
    ).astype(np.float32)
    report["scene_points"] = int(points.shape[0])
    backend = GraspNetBackend(config["repo"], config["checkpoint"])
    predictions = backend.predict(points)
    report["candidate_shape"] = list(predictions.shape)
    report["all_finite"] = bool(np.isfinite(predictions).all())
    if predictions.shape[0]:
        report["score_min"] = float(predictions[:, 0].min())
        report["score_max"] = float(predictions[:, 0].max())
    else:
        report["score_min"] = None
        report["score_max"] = None
    report["ok"] = bool(predictions.shape[0] > 0 and report["all_finite"])
except Exception:
    report["ok"] = False
    report["errors"].append(traceback.format_exc())
print("OPENFLEX_PROBE_JSON=" + json.dumps(report))
sys.exit(0 if report.get("ok") else 3)
'''


PERCEPTION_PROBE = r'''
import json, sys, traceback
report = {"probe": "perception_import", "errors": []}
try:
    import torch
    report["torch"] = torch.__version__
    report["cuda_available"] = bool(torch.cuda.is_available())
    import transformers
    report["transformers"] = transformers.__version__
    report["ok"] = True
except Exception:
    report["ok"] = False
    report["errors"].append(traceback.format_exc())
print("OPENFLEX_PROBE_JSON=" + json.dumps(report))
sys.exit(0 if report.get("ok") else 3)
'''


def _probe(python, code, payload, *, timeout, runner=run_python):
    if not python.is_file():
        return {"status": "skipped", "reason": f"interpreter not found: {python}"}
    args = [json.dumps(payload)] if payload is not None else ()
    try:
        proc = runner(python, code, args=args, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"status": "error", "reason": f"probe timed out after {timeout}s"}
    report = parse_probe(proc.stdout)
    if report is None:
        return {
            "status": "error",
            "reason": "probe produced no parseable payload",
            "returncode": proc.returncode,
            "stderr_tail": proc.stderr[-1500:],
        }
    report = dict(report)
    report["status"] = "pass" if report.get("ok") else "fail"
    report["returncode"] = proc.returncode
    if not report.get("ok") and proc.stderr:
        report["stderr_tail"] = proc.stderr[-1500:]
    return report


def audit(
    *,
    graspnet_venv=DEFAULT_GRASPNET_VENV,
    perception_venv=DEFAULT_PERCEPTION_VENV,
    graspnet_repo=DEFAULT_GRASPNET_REPO,
    checkpoint=DEFAULT_CHECKPOINT,
    timeout=1800,
    check_ops=True,
    check_inference=True,
    check_perception=True,
    runner=run_python,
):
    """Collect runtime facts for the migrated venvs.  Never raises for env issues."""

    graspnet_python = venv_python(graspnet_venv)
    perception_python = venv_python(perception_venv)

    report = {
        "graspnet_venv": str(Path(graspnet_venv).expanduser()),
        "perception_venv": str(Path(perception_venv).expanduser()),
        "graspnet_python": str(graspnet_python),
        "graspnet_python_present": graspnet_python.is_file(),
        "perception_python_present": perception_python.is_file(),
        "graspnet_foreign_shebangs": foreign_shebangs(Path(graspnet_venv).expanduser() / "bin"),
        "perception_foreign_shebangs": foreign_shebangs(Path(perception_venv).expanduser() / "bin"),
        "robot_motion_executed": False,
        "ros_started": False,
    }

    ops_probe = _probe(
        graspnet_python,
        CUDA_OPS_PROBE,
        {"require_cuda": False},
        timeout=timeout,
        runner=runner,
    ) if check_ops else {"status": "skipped", "reason": "check disabled"}
    report["cuda_ops"] = ops_probe

    if check_inference and graspnet_python.is_file():
        repo = Path(graspnet_repo).expanduser()
        example_data = repo / "doc" / "example_data"
        checkpoint_path = Path(checkpoint).expanduser()
        if not example_data.is_dir() or not checkpoint_path.is_file():
            report["checkpoint_inference"] = {
                "status": "skipped",
                "reason": "bundled example_data or checkpoint not present",
                "example_data": str(example_data),
                "checkpoint": str(checkpoint_path),
            }
        else:
            report["checkpoint_inference"] = _probe(
                graspnet_python,
                INFERENCE_PROBE,
                {
                    "package_root": str(Path(graspnet_repo).expanduser().parent / "openflex_grasp"),
                    "repo": str(repo),
                    "checkpoint": str(checkpoint_path),
                    "example_data": str(example_data),
                },
                timeout=timeout,
                runner=runner,
            )
    elif check_inference:
        report["checkpoint_inference"] = {
            "status": "skipped",
            "reason": f"interpreter not found: {graspnet_python}",
        }
    else:
        report["checkpoint_inference"] = {"status": "skipped", "reason": "check disabled"}

    report["perception_import"] = (
        _probe(perception_python, PERCEPTION_PROBE, None, timeout=timeout, runner=runner)
        if check_perception
        else {"status": "skipped", "reason": "check disabled"}
    )

    report["overall_ok"] = all(
        report[key]["status"] in ("pass", "skipped")
        for key in ("cuda_ops", "checkpoint_inference", "perception_import")
    ) and not report["graspnet_foreign_shebangs"] and not report["perception_foreign_shebangs"]
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graspnet-venv", default=str(DEFAULT_GRASPNET_VENV))
    parser.add_argument("--perception-venv", default=str(DEFAULT_PERCEPTION_VENV))
    parser.add_argument("--graspnet-repo", default=str(DEFAULT_GRASPNET_REPO))
    parser.add_argument("--checkpoint", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--timeout", type=int, default=1800)
    parser.add_argument("--skip-ops", action="store_true")
    parser.add_argument("--skip-inference", action="store_true")
    parser.add_argument("--skip-perception", action="store_true")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    report = audit(
        graspnet_venv=args.graspnet_venv,
        perception_venv=args.perception_venv,
        graspnet_repo=args.graspnet_repo,
        checkpoint=args.checkpoint,
        timeout=args.timeout,
        check_ops=not args.skip_ops,
        check_inference=not args.skip_inference,
        check_perception=not args.skip_perception,
    )
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
        print(args.output)
    else:
        print(text)
    return 0 if report["overall_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
