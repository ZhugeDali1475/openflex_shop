"""ROS-free trial statistics with mandatory L1/L2 separation (phase C2/F).

Reports success metrics per ``layer`` (L1 known map / L2 unknown map) and per
``evidence_source`` (pure vision / oracle control) and *refuses* to merge them:
a single combined success rate is never produced.  Every attempt stays in the
denominator, a failure must name the stage it failed at, and a harness error is
counted (not discarded) as a failure of the trial it belongs to.

The module is middleware-free and deterministic so a batch can be reduced
offline from recorded trials.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Mapping, Optional, Sequence, Tuple

LAYERS = ("L1", "L2")
EVIDENCE_SOURCES = ("pure_vision", "oracle_control")
FAILURE_STAGES = ("perception", "localization", "mapping", "exploration", "grasp",
                  "navigation", "placement", "harness")
CONDITIONS = ("visible", "occluded", "empty", "wrong_class", "ambiguous", "invalid_depth",
              "unreachable_grasp", "random_slot")
_NO_FAILURE = "none"


class TrialStatisticsError(ValueError):
    """Raised when trial records or an aggregation request violate the contract."""


def _finite(value, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise TrialStatisticsError(f"{name} must be finite")
    return number


@dataclass(frozen=True)
class TrialRecord:
    """One attempt.  Failures must be attributed; successes carry no failure stage."""

    trial_id: str
    layer: str
    evidence_source: str
    condition: str
    success: bool
    failure_stage: str = _NO_FAILURE
    duration_sec: float = 0.0
    localization_error_m: Optional[float] = None
    safety_stop: bool = False
    detail: str = ""

    def __post_init__(self) -> None:
        if not str(self.trial_id).strip():
            raise TrialStatisticsError("trial_id must be a non-empty string")
        if self.layer not in LAYERS:
            raise TrialStatisticsError(f"layer must be one of {LAYERS}")
        if self.evidence_source not in EVIDENCE_SOURCES:
            raise TrialStatisticsError(f"evidence_source must be one of {EVIDENCE_SOURCES}")
        if self.condition not in CONDITIONS:
            raise TrialStatisticsError(f"condition must be one of {CONDITIONS}")
        if bool(self.success) and self.failure_stage != _NO_FAILURE:
            raise TrialStatisticsError("a successful trial cannot name a failure stage")
        if not bool(self.success) and self.failure_stage not in FAILURE_STAGES:
            raise TrialStatisticsError(
                "a failed trial must name a failure stage from "
                f"{FAILURE_STAGES} (got {self.failure_stage!r})")
        if _finite(self.duration_sec, "duration_sec") < 0.0:
            raise TrialStatisticsError("duration_sec must be non-negative")
        if self.localization_error_m is not None \
                and _finite(self.localization_error_m, "localization_error_m") < 0.0:
            raise TrialStatisticsError("localization_error_m must be non-negative")

    def to_dict(self) -> Mapping[str, object]:
        return {"trial_id": self.trial_id, "layer": self.layer,
                "evidence_source": self.evidence_source, "condition": self.condition,
                "success": bool(self.success), "failure_stage": self.failure_stage,
                "duration_sec": float(self.duration_sec),
                "localization_error_m": self.localization_error_m,
                "safety_stop": bool(self.safety_stop), "detail": self.detail}


@dataclass(frozen=True)
class Metrics:
    layer: str
    evidence_source: str
    trials: int
    successes: int
    failures: int
    success_rate: float
    failure_rate: float
    safety_stop_rate: float
    mean_duration_sec: float
    mean_localization_error_m: Optional[float]
    attribution: Mapping[str, int]
    conditions: Mapping[str, int]

    def to_dict(self) -> Mapping[str, object]:
        return {"layer": self.layer, "evidence_source": self.evidence_source,
                "trials": self.trials, "successes": self.successes, "failures": self.failures,
                "success_rate": self.success_rate, "failure_rate": self.failure_rate,
                "safety_stop_rate": self.safety_stop_rate,
                "mean_duration_sec": self.mean_duration_sec,
                "mean_localization_error_m": self.mean_localization_error_m,
                "attribution": dict(self.attribution), "conditions": dict(self.conditions)}


def slice_metrics(records: Sequence[TrialRecord], *, layer: str, evidence_source: str) -> Metrics:
    """Metrics for exactly one (layer, evidence_source) slice, failures included."""

    if layer not in LAYERS or evidence_source not in EVIDENCE_SOURCES:
        raise TrialStatisticsError("layer/evidence_source out of contract")
    selected = [record for record in records
                if record.layer == layer and record.evidence_source == evidence_source]
    if not selected:
        raise TrialStatisticsError(f"no trials for layer={layer} source={evidence_source}")
    trials = len(selected)
    successes = sum(1 for record in selected if record.success)
    failures = trials - successes
    durations = [record.duration_sec for record in selected]
    errors = [record.localization_error_m for record in selected
              if record.localization_error_m is not None]
    attribution: dict = {}
    for record in selected:
        if not record.success:
            attribution[record.failure_stage] = attribution.get(record.failure_stage, 0) + 1
    conditions: dict = {}
    for record in selected:
        conditions[record.condition] = conditions.get(record.condition, 0) + 1
    return Metrics(layer=layer, evidence_source=evidence_source, trials=trials,
                   successes=successes, failures=failures,
                   success_rate=round(successes / trials, 6),
                   failure_rate=round(failures / trials, 6),
                   safety_stop_rate=round(sum(1 for record in selected if record.safety_stop) / trials, 6),
                   mean_duration_sec=round(sum(durations) / trials, 6),
                   mean_localization_error_m=(round(sum(errors) / len(errors), 6) if errors else None),
                   attribution=dict(sorted(attribution.items())),
                   conditions=dict(sorted(conditions.items())))


def layer_report(records: Sequence[TrialRecord]) -> Mapping[str, Mapping[str, Metrics]]:
    """Per-layer, per-source metrics.  L1 and L2 are never combined."""

    report: dict = {}
    for layer in LAYERS:
        sources = [source for source in EVIDENCE_SOURCES
                   if any(record.layer == layer and record.evidence_source == source
                          for record in records)]
        if sources:
            report[layer] = {source: slice_metrics(records, layer=layer, evidence_source=source)
                             for source in sources}
    return report


def combine_layers(records: Sequence[TrialRecord]) -> Metrics:
    """Always raises: a merged L1+L2 success rate is forbidden by the plan."""

    raise TrialStatisticsError("merging L1 and L2 success rates is forbidden by the plan")


def combine_evidence_sources(records: Sequence[TrialRecord]) -> Metrics:
    """Always raises: pure-vision and oracle-control results must stay separate."""

    raise TrialStatisticsError(
        "merging pure_vision and oracle_control success rates is forbidden by the plan")


@dataclass(frozen=True)
class TrialSpec:
    trial_id: str
    layer: str
    evidence_source: str
    condition: str
    params: Mapping[str, object] = field(default_factory=dict)


def run_batch(specs: Sequence[TrialSpec], execute: Callable[[TrialSpec], TrialRecord]
              ) -> Tuple[TrialRecord, ...]:
    """Run trials in order; any error becomes a counted ``harness`` failure.

    A trial is never dropped: if ``execute`` raises or returns a record whose
    identity does not match the spec, the spec's identity is preserved and the
    attempt is recorded as a failure attributed to ``harness``.
    """

    results = []
    for spec in specs:
        try:
            record = execute(spec)
            if not isinstance(record, TrialRecord):
                raise TrialStatisticsError("execute must return a TrialRecord")
            if (record.trial_id, record.layer, record.evidence_source, record.condition) != \
                    (spec.trial_id, spec.layer, spec.evidence_source, spec.condition):
                raise TrialStatisticsError("execute returned a record for a different trial")
            results.append(record)
        except Exception as error:  # fail-closed: harness problems stay in the denominator
            results.append(TrialRecord(spec.trial_id, spec.layer, spec.evidence_source,
                                       spec.condition, success=False, failure_stage="harness",
                                       detail=f"{type(error).__name__}: {error}"))
    return tuple(results)
