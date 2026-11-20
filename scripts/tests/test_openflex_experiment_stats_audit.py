import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "experiment_stats_audit",
    Path(__file__).resolve().parents[1] / "audit_openflex_experiment_stats.py",
)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


@pytest.fixture(scope="module")
def report():
    return audit.run_audit()


def test_audit_passes_and_is_offline(report):
    assert report["passed"] is True
    assert report["robot_motion_executed"] is False
    assert report["ros_started"] is False
    assert report["physical_verification"] is False
    assert report["synthetic_batch"] is True


def test_l1_and_l2_are_separate_columns(report):
    assert set(report["layers"]) == {"L1", "L2"}
    assert report["guards"]["combined_rate"] is None
    assert report["guards"]["merge_forbidden"] is True
    assert report["layers"]["L1"]["pure_vision"]["success_rate"] != \
        report["layers"]["L2"]["pure_vision"]["success_rate"]


def test_all_attempts_and_harness_errors_are_counted(report):
    total = sum(metrics["trials"] for layer in report["layers"].values()
                for metrics in layer.values())
    assert total == report["trials_total"]
    for layer in report["layers"].values():
        for metrics in layer.values():
            assert metrics["successes"] + metrics["failures"] == metrics["trials"]
            assert sum(metrics["attribution"].values()) == metrics["failures"]
            assert "harness" in metrics["attribution"]


def test_audit_is_deterministic_for_a_seed(report):
    again = audit.run_audit()
    again.pop("generated_at")
    report_copy = dict(report)
    report_copy.pop("generated_at")
    assert again == report_copy
