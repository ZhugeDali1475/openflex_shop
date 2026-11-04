"""Structured failure and outcome contracts used by the agent."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Optional

from .skills import SkillName


class FailureCode(str, Enum):
    NONE = "none"
    TARGET_NOT_FOUND = "target_not_found"
    TARGET_AMBIGUOUS = "target_ambiguous"
    TARGET_OCCLUDED = "target_occluded"
    STALE_MAP = "stale_map"
    SENSOR_STALE = "sensor_stale"
    LOCALIZATION_INVALID = "localization_invalid"
    NAVIGATION_FAILED = "navigation_failed"
    ALIGNMENT_FAILED = "alignment_failed"
    IK_UNREACHABLE = "ik_unreachable"
    COLLISION_BLOCKED = "collision_blocked"
    GRASP_FAILED = "grasp_failed"
    GRASP_NOT_ATTACHED = "grasp_not_attached"
    TRANSPORT_UNSAFE = "transport_unsafe"
    PLACE_FAILED = "place_failed"
    PLACE_VERIFY_FAILED = "place_verify_failed"
    CONTROL_CONFLICT = "control_conflict"
    TIMEOUT = "timeout"
    INTERNAL_ERROR = "internal_error"


class SkillStatus(str, Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    CANCELED = "canceled"


@dataclass(frozen=True)
class SkillOutcome:
    skill: SkillName
    status: SkillStatus
    postcondition_verified: bool
    elapsed_sec: float
    failure_code: FailureCode = FailureCode.NONE
    evidence_id: str = ""
    message: str = ""

    def __post_init__(self) -> None:
        if not isfinite(self.elapsed_sec) or self.elapsed_sec < 0.0:
            raise ValueError("elapsed_sec must be finite and non-negative")
        if self.status == SkillStatus.SUCCESS and not self.postcondition_verified:
            raise ValueError("a successful skill outcome must have a verified postcondition")
        if self.status == SkillStatus.SUCCESS and self.failure_code != FailureCode.NONE:
            raise ValueError("a successful skill outcome cannot carry a failure code")
        if self.status == SkillStatus.FAILURE and self.failure_code == FailureCode.NONE:
            raise ValueError("a failed skill outcome must carry a failure code")

    @property
    def successful(self) -> bool:
        return self.status == SkillStatus.SUCCESS and self.postcondition_verified


@dataclass(frozen=True)
class FailureRecord:
    failure_code: FailureCode
    skill: SkillName
    attempt: int
    evidence_id: str = ""
    message: str = ""
    elapsed_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.failure_code == FailureCode.NONE:
            raise ValueError("a failure record cannot contain FailureCode.NONE")
        if self.attempt < 1:
            raise ValueError("failure attempt must be at least 1")


def failure_for_outcome(outcome: SkillOutcome, attempt: int) -> Optional[FailureRecord]:
    if outcome.successful or outcome.failure_code == FailureCode.NONE:
        return None
    return FailureRecord(
        failure_code=outcome.failure_code,
        skill=outcome.skill,
        attempt=attempt,
        evidence_id=outcome.evidence_id,
        message=outcome.message,
        elapsed_sec=outcome.elapsed_sec,
    )
