import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.trial_statistics import (  # noqa: E402
    TrialRecord, TrialSpec, TrialStatisticsError, combine_evidence_sources, combine_layers,
    layer_report, run_batch, slice_metrics,
)


def trial(trial_id, layer="L1", source="pure_vision", condition="visible", success=True,
          stage="none", duration=1.0, error=None, safety=False):
    return TrialRecord(trial_id, layer, source, condition, success, stage, duration, error, safety)


def test_record_validation():
    with pytest.raises(TrialStatisticsError):
        trial("t", layer="L3")
    with pytest.raises(TrialStatisticsError):
        trial("t", source="oracle")
    with pytest.raises(TrialStatisticsError):
        trial("t", condition="banana")
    with pytest.raises(TrialStatisticsError):
        trial("t", success=True, stage="perception")
    with pytest.raises(TrialStatisticsError):
        trial("t", success=False, stage="none")
    with pytest.raises(TrialStatisticsError):
        trial("t", duration=-1.0)
    with pytest.raises(TrialStatisticsError):
        trial("t", error=-0.5)


def test_failures_stay_in_the_denominator_and_are_attributed():
    records = [
        trial("a", success=True, duration=2.0, error=0.01),
        trial("b", success=False, stage="exploration", duration=4.0, error=0.02),
        trial("c", success=False, stage="exploration", duration=6.0),
        trial("d", success=False, stage="perception", duration=8.0),
    ]
    metrics = slice_metrics(records, layer="L1", evidence_source="pure_vision")
    assert metrics.trials == 4 and metrics.successes == 1 and metrics.failures == 3
    assert metrics.success_rate == pytest.approx(0.25)
    assert sum(metrics.attribution.values()) == metrics.failures
    assert metrics.attribution == {"exploration": 2, "perception": 1}
    assert metrics.mean_duration_sec == pytest.approx(5.0)
    assert metrics.mean_localization_error_m == pytest.approx(0.015)


def test_empty_slice_is_rejected():
    with pytest.raises(TrialStatisticsError):
        slice_metrics([trial("a")], layer="L2", evidence_source="pure_vision")


def test_layers_and_sources_are_never_merged():
    records = [
        trial("l1", layer="L1", success=True),
        trial("l2", layer="L2", success=False, stage="mapping"),
        trial("oracle", layer="L1", source="oracle_control", success=True),
    ]
    report = layer_report(records)
    assert set(report) == {"L1", "L2"}
    assert set(report["L1"]) == {"pure_vision", "oracle_control"}
    assert report["L1"]["pure_vision"].success_rate == pytest.approx(1.0)
    assert report["L2"]["pure_vision"].success_rate == pytest.approx(0.0)
    assert report["L1"]["oracle_control"].success_rate == pytest.approx(1.0)
    with pytest.raises(TrialStatisticsError):
        combine_layers(records)
    with pytest.raises(TrialStatisticsError):
        combine_evidence_sources(records)


def test_run_batch_counts_harness_errors_and_preserves_identity():
    specs = [
        TrialSpec("ok", "L1", "pure_vision", "visible"),
        TrialSpec("boom", "L2", "pure_vision", "occluded"),
        TrialSpec("mismatch", "L1", "pure_vision", "empty"),
    ]

    def execute(spec):
        if spec.trial_id == "ok":
            return trial("ok")
        if spec.trial_id == "boom":
            raise RuntimeError("sensor drop")
        return trial("something-else", layer="L2", success=False, stage="mapping")

    records = run_batch(specs, execute)
    assert [record.trial_id for record in records] == ["ok", "boom", "mismatch"]
    assert records[1] == TrialRecord("boom", "L2", "pure_vision", "occluded", success=False,
                                     failure_stage="harness",
                                     detail="RuntimeError: sensor drop")
    assert records[2].layer == "L1" and records[2].condition == "empty"
    assert records[2].failure_stage == "harness"
    metrics = slice_metrics(records, layer="L1", evidence_source="pure_vision")
    assert metrics.trials == 2 and metrics.failures == 1


def test_reports_are_deterministic():
    records = [trial("a", success=True), trial("b", success=False, stage="grasp"),
               trial("c", layer="L2", success=False, stage="mapping")]
    first = {layer: {source: metrics.to_dict() for source, metrics in sources.items()}
             for layer, sources in layer_report(records).items()}
    second = {layer: {source: metrics.to_dict() for source, metrics in sources.items()}
              for layer, sources in layer_report(records).items()}
    assert first == second
