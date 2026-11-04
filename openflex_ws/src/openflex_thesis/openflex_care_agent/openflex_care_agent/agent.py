"""Nominal phase policy and failure-aware top-level decision loop."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .belief import BeliefState, TaskPhase
from .failure import FailureCode
from .recovery import CapabilityEstimate, RecoveryPolicy
from .safety import SafetyCritic
from .skills import SkillName, SkillRegistry


@dataclass(frozen=True)
class Decision:
    skill: SkillName
    reason: str
    fallback: bool
    candidates: Tuple[SkillName, ...] = ()


NOMINAL_PHASE_SKILL = {
    TaskPhase.NAVIGATE_OBSERVE: SkillName.NAVIGATE_TO_STATION,
    TaskPhase.ACQUIRE_TARGET: SkillName.LOOK_FOR_OBJECT,
    TaskPhase.EVALUATE_CAPABILITY: SkillName.COMPUTE_CAPABILITY,
    TaskPhase.ALIGN_TO_READY: SkillName.LOCAL_ALIGN_REGION,
    TaskPhase.GRASP: SkillName.PLAN_AND_GRASP,
    TaskPhase.VERIFY_GRASP: SkillName.VERIFY_GRASP,
    TaskPhase.NAVIGATE_DELIVERY: SkillName.NAVIGATE_TO_DELIVERY,
    TaskPhase.PLACE: SkillName.PLACE_OBJECT,
    TaskPhase.VERIFY_PLACE: SkillName.VERIFY_PLACE,
}


class CAREAgent:
    """Pure decision core; ROS adapters execute the returned skills."""

    def __init__(
        self,
        registry: Optional[SkillRegistry] = None,
        maximum_recovery_attempts: int = 3,
    ):
        self.registry = registry or SkillRegistry()
        self.safety = SafetyCritic(self.registry)
        self.recovery = RecoveryPolicy(
            self.registry,
            self.safety,
            maximum_recovery_attempts=maximum_recovery_attempts,
        )

    def decide(
        self,
        state: BeliefState,
        failure: FailureCode = FailureCode.NONE,
        estimates: Optional[Mapping[SkillName, CapabilityEstimate]] = None,
    ) -> Decision:
        if state.phase in (TaskPhase.DONE, TaskPhase.HOLD, TaskPhase.FAILED):
            return Decision(
                SkillName.ABORT_HOLD,
                f"terminal phase {state.phase.value}",
                fallback=True,
            )
        if failure != FailureCode.NONE:
            ranked = self.recovery.rank(state, failure, estimates)
            best = ranked[0]
            fallback = best.skill == SkillName.ABORT_HOLD
            return Decision(
                best.skill,
                f"recover {failure.value} with utility {best.utility:.3f}",
                fallback=fallback,
                candidates=tuple(item.skill for item in ranked),
            )
        if state.phase == TaskPhase.RECOVERY and state.latest_failure is not None:
            ranked = self.recovery.rank(
                state, state.latest_failure.failure_code, estimates
            )
            best = ranked[0]
            return Decision(
                best.skill,
                f"resume recovery for {state.latest_failure.failure_code.value}",
                fallback=best.skill == SkillName.ABORT_HOLD,
                candidates=tuple(item.skill for item in ranked),
            )

        skill = NOMINAL_PHASE_SKILL.get(state.phase)
        if skill is None:
            return Decision(
                SkillName.ABORT_HOLD,
                f"no nominal skill for phase {state.phase.value}",
                fallback=True,
            )
        safety = self.safety.evaluate(state, skill)
        if not safety.allowed:
            return Decision(
                SkillName.ABORT_HOLD,
                "nominal skill blocked: " + ", ".join(safety.violations),
                fallback=True,
                candidates=(skill,),
            )
        return Decision(skill, f"nominal phase {state.phase.value}", fallback=False)
