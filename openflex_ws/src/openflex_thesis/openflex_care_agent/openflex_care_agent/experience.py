"""Task-local recovery choices, geometric failure memory and auditable records."""

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import random
import time
from typing import Mapping, Optional, Tuple


ACTIONS = ("reobserve", "switch_grasp", "reposition")
METHODS = ("fixed", "heuristic", "heuristic_memory", "learned", "learned_memory", "random")
FEATURES = (
    "observation_age", "confidence", "target_shift", "base_distance", "base_angle",
    "candidate_count", "score", "joint_margin", "robustness", "grasp_quality",
    "remaining_sec", "attempt", "repeat_count", "duration_sec", "distance_m",
)


@dataclass(frozen=True)
class RecoveryOption:
    action: str
    key: Tuple[float, ...]
    features: Mapping[str, float]
    safe: bool = True

    def __post_init__(self):
        if self.action not in ACTIONS:
            raise ValueError("unknown recovery action")
        if any(not math.isfinite(float(value)) for value in self.features.values()):
            raise ValueError("recovery features must be finite")
        if any(not math.isfinite(value) for value in self.key):
            raise ValueError("candidate geometry must be finite")
        if self.features.get("duration_sec", 0.0) < 0:
            raise ValueError("duration must be non-negative")


@dataclass(frozen=True)
class FailureRecord:
    object_id: str
    version: int
    target: Tuple[float, float, float]
    key: Tuple[float, ...]
    reason: str


class FailureMemory:
    def __init__(self, target_tolerance=0.05, geometry_tolerance=0.025):
        self.target_tolerance = target_tolerance
        self.geometry_tolerance = geometry_tolerance
        self.records = []

    def add(self, record: FailureRecord):
        self.records.append(record)

    def repeats(self, object_id, target, key):
        count = 0
        for record in self.records:
            if record.object_id != object_id or record.reason in (
                "stale_observation", "target_changed", "infrastructure", "cancelled"
            ):
                continue
            if math.dist(record.target, target) > self.target_tolerance:
                continue
            if len(record.key) != len(key) or not key:
                continue
            if max(abs(first - second) for first, second in zip(record.key, key)) <= self.geometry_tolerance:
                count += 0.25 if record.reason in ("planning_timeout", "ik_timeout") else 1
        return count


def feature_row(features, action, reason, stage):
    row = {name: float(features.get(name, 0.0)) for name in FEATURES}
    row.update({"action": action, "reason": reason, "stage": stage})
    return row


class RecoverySelector:
    def __init__(self, method="heuristic_memory", model_path="", seed=7, time_weight=0.2,
                 repeat_weight=0.15, duration_scale=120.0, forced_action=""):
        if method not in METHODS:
            raise ValueError("unknown recovery method")
        if duration_scale <= 0 or time_weight < 0 or repeat_weight < 0:
            raise ValueError("invalid utility configuration")
        self.method = method
        if forced_action and forced_action not in ACTIONS:
            raise ValueError("unknown forced action")
        self.forced_action = forced_action
        self.random = random.Random(seed)
        self.time_weight = time_weight
        self.repeat_weight = repeat_weight
        self.duration_scale = duration_scale
        self.model = None
        if method.startswith("learned"):
            if not model_path:
                raise ValueError("learned recovery requires a trained model")
            import joblib
            self.model = joblib.load(model_path)
            if self.model["features"] != list(FEATURES):
                raise ValueError("model feature schema mismatch")
            if self.model["use_memory"] != (method == "learned_memory"):
                raise ValueError("memory ablation requires a separately trained model")

    @property
    def uses_memory(self):
        return self.method in ("heuristic_memory", "learned_memory")

    def probability(self, option, reason, stage):
        features = dict(option.features)
        if not self.uses_memory:
            features["repeat_count"] = 0.0
        if self.model is not None:
            if self.model.get("drop_capability", False):
                for name in ("score", "joint_margin", "robustness", "grasp_quality", "candidate_count"):
                    features[name] = 0.0
            row = feature_row(features, option.action, reason, stage)
            return float(self.model["pipeline"].predict_proba([row])[0, 1])
        return max(0.0, min(1.0, 0.35 + 0.25 * features.get("score", 0.0)
                           + 0.15 * features.get("joint_margin", 0.0)
                           + 0.15 * features.get("robustness", 0.0)
                           + 0.10 * features.get("grasp_quality", 0.0)))

    def choose(self, options, reason, stage, remaining_sec, grasp_reserve_sec):
        valid = [option for option in options if option.safe
                 and option.features["duration_sec"] + grasp_reserve_sec <= remaining_sec]
        if not valid:
            return None
        if self.forced_action:
            return next((option for option in valid if option.action == self.forced_action), None)
        if self.method == "random":
            return self.random.choice(valid)
        if self.method == "fixed":
            preferred = "reobserve" if reason in ("stale_observation", "target_changed", "alignment") else "reposition"
            return next((option for option in valid if option.action == preferred), None)
        def utility(option):
            penalty = option.features.get("repeat_count", 0.0) if self.uses_memory else 0.0
            return (self.probability(option, reason, stage)
                    - self.time_weight * option.features["duration_sec"] / self.duration_scale
                    - self.repeat_weight * penalty)
        return max(valid, key=utility)


class RecoveryJournal:
    def __init__(self, path, task_id, metadata):
        self.path = Path(path) if path else None
        self.task_id = task_id
        self.metadata = dict(metadata)
        self.pending = None
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event, **values):
        record = {"schema_version": 1, "task_id": self.task_id, "event": event,
                  "monotonic_sec": time.monotonic(), **self.metadata, **values}
        if self.path:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        return record

    def begin(self, option, reason, stage, propensity=None):
        if self.pending is not None:
            raise RuntimeError("previous recovery transition has no outcome")
        self.pending = (time.monotonic(), option, reason, stage, propensity)
        self.write("decision", option=asdict(option), reason=reason, stage=stage, propensity=propensity)

    def finish(self, success: Optional[bool], outcome):
        if self.pending is None:
            return
        started, option, reason, stage, propensity = self.pending
        self.write("transition", action=option.action, features=dict(option.features),
                   reason=reason, stage=stage, label=None if success is None else int(success),
                   outcome=outcome, duration_sec=time.monotonic() - started, propensity=propensity)
        self.pending = None
