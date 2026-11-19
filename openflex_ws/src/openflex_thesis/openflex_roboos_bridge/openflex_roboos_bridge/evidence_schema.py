"""Separated perception and simulation-evaluation evidence records.

The online search path writes only the perception layer.  A later evaluator may
write a separate simulation layer, but a simulator identity can never be
silently inserted into the perception record.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional


class EvidenceSchemaError(ValueError):
    pass


def _reject_nonnull_sim_fields(value, path="record") -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            key_text = str(key)
            if key_text in {
                "mujoco_object_id", "object_id", "truth_object_id",
                "selected_object_id", "oracle_object_id", "mujoco_id",
            } and item is not None:
                raise EvidenceSchemaError(f"perception evidence contains simulator identity at {path}.{key_text}")
            _reject_nonnull_sim_fields(item, f"{path}.{key_text}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _reject_nonnull_sim_fields(item, f"{path}[{index}]")


@dataclass(frozen=True)
class PerceptionEvidence:
    request_id: str
    attempt: int
    frame_sha256: str
    frame_ref: str
    depth_ref: str
    record: Mapping[str, object]

    def to_dict(self) -> dict:
        if not self.request_id or int(self.attempt) < 1 or not self.frame_sha256:
            raise EvidenceSchemaError("perception evidence identity fields are required")
        if not self.frame_ref or not self.depth_ref:
            raise EvidenceSchemaError("perception RGB/depth references are required")
        _reject_nonnull_sim_fields(self.record)
        return {
            "evidence_layer": "perception",
            "request_id": self.request_id,
            "attempt": int(self.attempt),
            "frame_sha256": self.frame_sha256,
            "frame_ref": self.frame_ref,
            "depth_ref": self.depth_ref,
            "physical_verification": False,
            "source": "rgbd_visual_observation",
            "record": dict(self.record),
        }


@dataclass(frozen=True)
class SimulationEvaluationEvidence:
    request_id: str
    attempt: int
    mujoco_object_id: Optional[str] = None
    physical_verification: bool = False
    source: str = "not_available"

    def to_dict(self) -> dict:
        if not self.request_id or int(self.attempt) < 1:
            raise EvidenceSchemaError("evaluation evidence identity fields are required")
        if bool(self.physical_verification) and not self.mujoco_object_id:
            raise EvidenceSchemaError(
                "physical verification requires an evaluator-owned MuJoCo object id"
            )
        if not math.isfinite(float(self.attempt)):
            raise EvidenceSchemaError("attempt must be finite")
        return {
            "evidence_layer": "simulation_evaluation",
            "request_id": self.request_id,
            "attempt": int(self.attempt),
            "mujoco_object_id": self.mujoco_object_id,
            "physical_verification": bool(self.physical_verification),
            "source": self.source,
        }
