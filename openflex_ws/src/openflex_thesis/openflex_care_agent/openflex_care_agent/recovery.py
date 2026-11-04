"""Failure-to-skill recovery routing and soft utility selection."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Dict, Iterable, Mapping, Optional, Tuple

from .belief import BeliefState
from .failure import FailureCode
from .safety import SafetyCritic
from .skills import SkillName, SkillRegistry


@dataclass(frozen=True)
class CapabilityEstimate:
    success_lcb: float
    information_gain: float
    duration_sec: float
    distance_m: float
    risk: float

    def __post_init__(self) -> None:
        for name, value in (
            ("success_lcb", self.success_lcb),
            ("information_gain", self.information_gain),
            ("duration_sec", self.duration_sec),
            ("distance_m", self.distance_m),
            ("risk", self.risk),
        ):
            if not isfinite(value):
                raise ValueError(f"{name} must be finite")
        if not 0.0 <= self.success_lcb <= 1.0:
            raise ValueError("success_lcb must be within [0, 1]")
        if not 0.0 <= self.information_gain <= 1.0:
            raise ValueError("information_gain must be within [0, 1]")
        if not 0.0 <= self.risk <= 1.0:
            raise ValueError("risk must be within [0, 1]")
        if self.duration_sec < 0.0 or self.distance_m < 0.0:
            raise ValueError("duration and distance must be non-negative")


@dataclass(frozen=True)
class UtilityWeights:
    success: float = 1.0
    information: float = 0.35
    duration: float = 0.02
    distance: float = 0.10
    risk: float = 1.5

    def __post_init__(self) -> None:
        values = (
            self.success,
            self.information,
            self.duration,
            self.distance,
            self.risk,
        )
        if any(not isfinite(value) or value < 0.0 for value in values):
            raise ValueError("utility weights must be finite and non-negative")
        if not any(values):
            raise ValueError("at least one utility weight must be positive")


@dataclass(frozen=True)
class RankedRecovery:
    skill: SkillName
    utility: float
    estimate: CapabilityEstimate


DEFAULT_ESTIMATES: Dict[SkillName, CapabilityEstimate] = {
    SkillName.RE_OBSERVE: CapabilityEstimate(0.65, 0.75, 4.0, 0.0, 0.05),
    SkillName.CHANGE_VIEW: CapabilityEstimate(0.60, 0.90, 8.0, 0.25, 0.12),
    SkillName.LOCAL_ALIGN_REGION: CapabilityEstimate(0.70, 0.10, 12.0, 0.20, 0.18),
    SkillName.FINE_ALIGN: CapabilityEstimate(0.68, 0.05, 15.0, 0.10, 0.18),
    SkillName.COMPUTE_CAPABILITY: CapabilityEstimate(0.72, 0.20, 8.0, 0.0, 0.03),
    SkillName.NAVIGATE_TO_STATION: CapabilityEstimate(0.65, 0.10, 25.0, 1.00, 0.20),
    SkillName.RETREAT_FROM_STATION: CapabilityEstimate(0.85, 0.05, 6.0, 0.15, 0.10),
    SkillName.PLAN_AND_GRASP: CapabilityEstimate(0.65, 0.0, 20.0, 0.0, 0.30),
    SkillName.PLACE_OBJECT: CapabilityEstimate(0.65, 0.0, 12.0, 0.0, 0.20),
    SkillName.LOOK_FOR_OBJECT: CapabilityEstimate(0.55, 0.80, 5.0, 0.0, 0.05),
    SkillName.ABORT_HOLD: CapabilityEstimate(1.00, 0.0, 0.0, 0.0, 0.0),
}


FAILURE_RECOVERY_MAP: Dict[FailureCode, Tuple[SkillName, ...]] = {
    FailureCode.TARGET_NOT_FOUND: (
        SkillName.RE_OBSERVE,
        SkillName.CHANGE_VIEW,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.TARGET_AMBIGUOUS: (
        SkillName.RE_OBSERVE,
        SkillName.CHANGE_VIEW,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.TARGET_OCCLUDED: (
        SkillName.CHANGE_VIEW,
        SkillName.RE_OBSERVE,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.STALE_MAP: (
        SkillName.RE_OBSERVE,
        SkillName.COMPUTE_CAPABILITY,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.SENSOR_STALE: (SkillName.ABORT_HOLD,),
    FailureCode.LOCALIZATION_INVALID: (SkillName.ABORT_HOLD,),
    FailureCode.NAVIGATION_FAILED: (
        SkillName.NAVIGATE_TO_STATION,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.ALIGNMENT_FAILED: (
        SkillName.LOCAL_ALIGN_REGION,
        SkillName.FINE_ALIGN,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.IK_UNREACHABLE: (
        SkillName.COMPUTE_CAPABILITY,
        SkillName.LOCAL_ALIGN_REGION,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.COLLISION_BLOCKED: (
        SkillName.COMPUTE_CAPABILITY,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.GRASP_FAILED: (
        SkillName.RETREAT_FROM_STATION,
        SkillName.RE_OBSERVE,
        SkillName.COMPUTE_CAPABILITY,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.GRASP_NOT_ATTACHED: (
        SkillName.RETREAT_FROM_STATION,
        SkillName.RE_OBSERVE,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.TRANSPORT_UNSAFE: (SkillName.ABORT_HOLD,),
    FailureCode.PLACE_FAILED: (
        SkillName.FINE_ALIGN,
        SkillName.PLACE_OBJECT,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.PLACE_VERIFY_FAILED: (
        SkillName.RE_OBSERVE,
        SkillName.FINE_ALIGN,
        SkillName.ABORT_HOLD,
    ),
    FailureCode.CONTROL_CONFLICT: (SkillName.ABORT_HOLD,),
    FailureCode.TIMEOUT: (SkillName.ABORT_HOLD,),
    FailureCode.INTERNAL_ERROR: (SkillName.ABORT_HOLD,),
}


class RecoveryPolicy:
    """Chooses only among safety-valid, failure-relevant skills."""

    def __init__(
        self,
        registry: SkillRegistry,
        safety: SafetyCritic,
        weights: UtilityWeights = UtilityWeights(),
        maximum_recovery_attempts: int = 3,
    ):
        if maximum_recovery_attempts < 1:
            raise ValueError("maximum_recovery_attempts must be at least 1")
        self.registry = registry
        self.safety = safety
        self.weights = weights
        self.maximum_recovery_attempts = maximum_recovery_attempts

    def utility(self, estimate: CapabilityEstimate) -> float:
        return (
            self.weights.success * estimate.success_lcb
            + self.weights.information * estimate.information_gain
            - self.weights.duration * estimate.duration_sec
            - self.weights.distance * estimate.distance_m
            - self.weights.risk * estimate.risk
        )

    def candidates(self, state: BeliefState, failure: FailureCode) -> Tuple[SkillName, ...]:
        if failure == FailureCode.NONE:
            raise ValueError("recovery requires a real failure code")
        attempt = state.attempts_for(failure)
        if attempt >= self.maximum_recovery_attempts:
            return (SkillName.ABORT_HOLD,)
        proposed = tuple(
            skill
            for skill in FAILURE_RECOVERY_MAP[failure]
            if skill != SkillName.ABORT_HOLD
        )
        valid = tuple(
            skill for skill in proposed if self.safety.evaluate(state, skill).allowed
        )
        if valid:
            return valid
        return (SkillName.ABORT_HOLD,)

    def rank(
        self,
        state: BeliefState,
        failure: FailureCode,
        estimates: Optional[Mapping[SkillName, CapabilityEstimate]] = None,
    ) -> Tuple[RankedRecovery, ...]:
        estimates = estimates or {}
        ranked = []
        for skill in self.candidates(state, failure):
            estimate = estimates.get(skill, DEFAULT_ESTIMATES.get(
                skill, CapabilityEstimate(0.0, 0.0, 999.0, 0.0, 1.0)
            ))
            ranked.append(RankedRecovery(skill, self.utility(estimate), estimate))
        return tuple(sorted(ranked, key=lambda item: (-item.utility, item.skill.value)))

    def choose(
        self,
        state: BeliefState,
        failure: FailureCode,
        estimates: Optional[Mapping[SkillName, CapabilityEstimate]] = None,
    ) -> RankedRecovery:
        ranked = self.rank(state, failure, estimates)
        if not ranked:
            raise RuntimeError("recovery policy produced no candidate")
        return ranked[0]
