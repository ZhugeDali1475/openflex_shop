import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location(
    "graspnet_runtime",
    Path(__file__).resolve().parents[1] / "audit_openflex_graspnet_runtime.py",
)
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)

ROOT = Path(__file__).resolve().parents[2]
REAL_GRASPNET_VENV = ROOT / ".venvs" / "openflex_graspnet"
REAL_PERCEPTION_VENV = ROOT / ".venvs" / "openflex_perception"
graspnet_env_present = pytest.mark.skipif(
    not runtime.venv_python(REAL_GRASPNET_VENV).is_file(),
    reason="migrated openflex_graspnet venv is not installed on this host",
)


def _payload(probe, **fields):
    body = {"probe": probe, "ok": True, "errors": []}
    body.update(fields)
    return runtime.PROBE_MARKER + __import__("json").dumps(body)


def _make_venv(root, name, *, shebang=None):
    bin_dir = root / name / "bin"
    bin_dir.mkdir(parents=True)
    python = bin_dir / "python"
    python.write_text("#!/bin/sh\nexit 0\n")
    if shebang:
        script = bin_dir / "pip"
        script.write_text(shebang + "\n")
    return root / name


def test_foreign_shebangs_flags_stale_root_and_ignores_symlinks(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    (bin_dir / "pip").write_text("#!/home/openflex/openflex_shop/.venvs/x/bin/python\n")
    (bin_dir / "python").write_text("#!/home/wgb/openflex_shop/.venvs/x/bin/python\n")
    (bin_dir / "link").symlink_to("python")
    found = runtime.foreign_shebangs(bin_dir)
    assert found == [
        {"script": "pip", "shebang": "/home/openflex/openflex_shop/.venvs/x/bin/python"}
    ]


def test_foreign_shebangs_missing_dir_is_empty(tmp_path):
    assert runtime.foreign_shebangs(tmp_path / "nope") == []


def test_parse_probe_uses_last_payload_and_tolerates_garbage():
    out = "noise\n" + runtime.PROBE_MARKER + '{"a": 1}\n' + runtime.PROBE_MARKER + '{"a": 2}\n'
    assert runtime.parse_probe(out) == {"a": 2}
    assert runtime.parse_probe("nothing here") is None
    assert runtime.parse_probe(runtime.PROBE_MARKER + "not json") is None


def _stub_runner(payloads, *, returncode=0):
    def runner(python, code, *, args=(), timeout=1800, env=None, cwd=None):
        for key, payload in payloads.items():
            if key in code:
                return SimpleNamespace(returncode=returncode, stdout=payload, stderr="")
        return SimpleNamespace(returncode=returncode, stdout="", stderr="")
    return runner


def test_audit_passes_with_stubbed_probes(tmp_path, monkeypatch):
    _make_venv(tmp_path, "graspnet")
    _make_venv(tmp_path, "perception")
    repo = tmp_path / "repo"
    (repo / "doc" / "example_data").mkdir(parents=True)
    checkpoint = tmp_path / "checkpoint-rs.tar"
    checkpoint.write_bytes(b"0" * 1_200_000)
    runner = _stub_runner({
        "cuda_ops": _payload("cuda_ops", ops={"knn": True}),
        "checkpoint_inference": _payload("checkpoint_inference", candidate_shape=[326, 17]),
        "perception_import": _payload("perception_import"),
    })
    report = runtime.audit(
        graspnet_venv=tmp_path / "graspnet",
        perception_venv=tmp_path / "perception",
        graspnet_repo=repo,
        checkpoint=checkpoint,
        runner=runner,
    )
    assert report["cuda_ops"]["status"] == "pass"
    assert report["checkpoint_inference"]["status"] == "pass"
    assert report["perception_import"]["status"] == "pass"
    assert report["overall_ok"] is True
    assert report["ros_started"] is False and report["robot_motion_executed"] is False


def test_audit_skips_when_venv_missing(tmp_path):
    report = runtime.audit(
        graspnet_venv=tmp_path / "missing",
        perception_venv=tmp_path / "missing",
        graspnet_repo=tmp_path / "repo",
        checkpoint=tmp_path / "checkpoint.tar",
    )
    assert report["graspnet_python_present"] is False
    assert report["cuda_ops"]["status"] == "skipped"
    assert report["checkpoint_inference"]["status"] == "skipped"
    assert report["perception_import"]["status"] == "skipped"
    assert report["overall_ok"] is True


def test_audit_flags_foreign_shebang(tmp_path):
    _make_venv(tmp_path, "graspnet", shebang="#!/home/openflex/openflex_shop/.venvs/x/bin/python")
    _make_venv(tmp_path, "perception")
    report = runtime.audit(
        graspnet_venv=tmp_path / "graspnet",
        perception_venv=tmp_path / "perception",
        check_ops=False,
        check_inference=False,
        check_perception=False,
    )
    assert report["graspnet_foreign_shebangs"]
    assert report["overall_ok"] is False


def test_audit_skips_inference_without_example_data(tmp_path):
    _make_venv(tmp_path, "graspnet")
    _make_venv(tmp_path, "perception")
    report = runtime.audit(
        graspnet_venv=tmp_path / "graspnet",
        perception_venv=tmp_path / "perception",
        graspnet_repo=tmp_path / "empty-repo",
        checkpoint=tmp_path / "absent.tar",
        check_ops=False,
        check_perception=False,
    )
    assert report["checkpoint_inference"]["status"] == "skipped"
    assert "example_data" in report["checkpoint_inference"]["reason"]


def test_audit_marks_failing_probe(tmp_path):
    _make_venv(tmp_path, "graspnet")
    _make_venv(tmp_path, "perception")
    failing = runtime.PROBE_MARKER + '{"probe": "cuda_ops", "ok": false, "errors": ["boom"]}'
    runner = _stub_runner({"cuda_ops": failing}, returncode=3)
    report = runtime.audit(
        graspnet_venv=tmp_path / "graspnet",
        perception_venv=tmp_path / "perception",
        check_inference=False,
        check_perception=False,
        runner=runner,
    )
    assert report["cuda_ops"]["status"] == "fail"
    assert report["overall_ok"] is False


def test_audit_reports_unparseable_probe(tmp_path):
    _make_venv(tmp_path, "graspnet")
    _make_venv(tmp_path, "perception")
    runner = _stub_runner({"cuda_ops": "totally broken"}, returncode=1)
    report = runtime.audit(
        graspnet_venv=tmp_path / "graspnet",
        perception_venv=tmp_path / "perception",
        check_inference=False,
        check_perception=False,
        runner=runner,
    )
    assert report["cuda_ops"]["status"] == "error"
    assert "stderr_tail" in report["cuda_ops"]


@graspnet_env_present
def test_real_graspnet_extension_and_perception_import():
    """Integration: the rebuilt extensions load and run on this host's GPU."""

    report = runtime.audit(
        graspnet_venv=REAL_GRASPNET_VENV,
        perception_venv=REAL_PERCEPTION_VENV,
        check_inference=False,
    )
    assert report["graspnet_foreign_shebangs"] == []
    assert report["cuda_ops"]["status"] == "pass"
    assert report["cuda_ops"]["ops"]["knn"] is True
    assert report["perception_import"]["status"] in ("pass", "skipped")
