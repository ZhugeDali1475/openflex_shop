import importlib.util
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    "model_assets", Path(__file__).resolve().parents[1] / "audit_openflex_model_assets.py"
)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)

REAL_ROOT = Path(__file__).resolve().parents[2] / "openflex_models"
real_root_present = pytest.mark.skipif(
    not (REAL_ROOT / "SHA256SUMS").is_file(),
    reason="openflex_models assets are not installed on this host",
)


def test_normalize_manifest_path_strips_foreign_root():
    assert audit.normalize_manifest_path(
        "/home/openflex/old/openflex_models/Qwen3.5-4B/LICENSE"
    ) == "Qwen3.5-4B/LICENSE"
    assert audit.normalize_manifest_path("graspnet-baseline/checkpoint-rs.tar") == \
        "graspnet-baseline/checkpoint-rs.tar"


def test_parse_sums_handles_absolute_and_relative_lines():
    text = (
        "# comment\n"
        "aaaa  /home/openflex/old/openflex_models/Qwen3.5-4B/config.json\n"
        "bbbb  sam2.1-hiera-small/model.safetensors\n"
        "\n"
        "malformed-line\n"
    )
    entries = audit.parse_sums(text)
    assert entries == {
        "Qwen3.5-4B/config.json": "aaaa",
        "sam2.1-hiera-small/model.safetensors": "bbbb",
    }


def test_audit_reports_presence_missing_and_anygrasp(tmp_path):
    (tmp_path / "Qwen3.5-4B").mkdir()
    (tmp_path / "Qwen3.5-4B" / "config.json").write_text("{}")
    (tmp_path / "graspnet-baseline").mkdir()
    (tmp_path / "graspnet-baseline" / "checkpoint-rs.tar").write_bytes(b"weights")
    (tmp_path / "anygrasp_sdk").mkdir()
    (tmp_path / "SHA256SUMS").write_text(
        f"{audit.sha256(tmp_path / 'Qwen3.5-4B' / 'config.json')}"
        "  /old/openflex_models/Qwen3.5-4B/config.json\n"
        f"{audit.sha256(tmp_path / 'graspnet-baseline' / 'checkpoint-rs.tar')}"
        "  /old/openflex_models/graspnet-baseline/checkpoint-rs.tar\n"
        "deadbeef  /old/openflex_models/Qwen3.5-4B/missing.json\n"
    )
    report = audit.audit(tmp_path)
    assert report["required_files"]["Qwen3.5-4B/config.json"] is True
    assert report["required_files"]["sam2.1-hiera-small/config.json"] is False
    assert report["manifest"]["path_portable"] is False
    assert report["manifest"]["entries"] == 3
    assert report["hash_verification"]["matched"] == 2
    assert report["hash_verification"]["absent"] == ["Qwen3.5-4B/missing.json"]
    assert report["anygrasp_assets_present"] == ["anygrasp_sdk"]
    assert report["robot_motion_executed"] is False


def test_audit_detects_hash_mismatch(tmp_path):
    (tmp_path / "SHA256SUMS").write_text(
        "0000000000000000000000000000000000000000000000000000000000000000"
        "  /old/openflex_models/Qwen3.5-4B/config.json\n")
    (tmp_path / "Qwen3.5-4B").mkdir()
    (tmp_path / "Qwen3.5-4B" / "config.json").write_text("{}")
    report = audit.audit(tmp_path)
    assert report["hash_verification"]["mismatched"] == ["Qwen3.5-4B/config.json"]


def test_audit_without_manifest_marks_verification_skipped(tmp_path):
    report = audit.audit(tmp_path)
    assert report["manifest"]["present"] is False
    assert report["hash_verification"]["skipped"] is True


@real_root_present
def test_real_model_root_is_integrity_verified_and_license_known():
    report = audit.audit(REAL_ROOT)
    assert report["exists"] is True
    assert all(report["required_files"].values())
    assert report["hash_verification"]["skipped"] is False
    assert report["hash_verification"]["mismatched"] == []
    assert report["hash_verification"]["absent"] == []
    assert report["hash_verification"]["matched"] == report["manifest"]["entries"] > 0
    assert report["anygrasp_assets_present"] == []
    for asset in ("Qwen3.5-4B", "sam2.1-hiera-small", "graspnet-baseline"):
        assert report["licenses"][asset]["license"]
