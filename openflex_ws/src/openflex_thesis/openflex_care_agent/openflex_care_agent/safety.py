"""Deterministic safety critic for skill preconditions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from .belief import BeliefState
from .skills import Precondition, SkillName, SkillRegistry


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    violations: Tuple[str, ...]

    def require_allowed(self) -> None:
        if not self.allowed:
            raise PermissionError("; ".join(self.violations))


class SafetyCritic:
    """Checks physical and control preconditions without issuing commands."""

    def __init__(self, registry: SkillRegistry):
        self.registry = registry

    @staticmethod
    def _check(precondition: Precondition, state: BeliefState) -> bool:
        robot = state.robot
        target = state.target
        conditions = {
            Precondition.CAMERA_FRESH: robot.camera_fresh,
            Precondition.ODOMETRY_FRESH: robot.odometry_fresh,
            Precondition.JOINT_STATE_FRESH: robot.joint_state_fresh,
            Precondition.LOCALIZATION_VALID: robot.localization_valid,
            Precondition.BASE_STOPPED: robot.base_stopped,
            Precondition.BASE_AVAILABLE: robot.base_available,
            Precondition.MANIPULATION_AVAILABLE: robot.manipulation_available,
            Precondition.PERCEPTION_READY: robot.perception_ready,
            Precondition.MOVEIT_READY: robot.moveit_ready,
            Precondition.TIME_AVAILABLE: robot.remaining_time_sec > 0.0,
            Precondition.MAP_CONSISTENT: state.map_consistent,
            Precondition.TARGET_VISIBLE: target.visible,
            Precondition.GRASP_AUTHORIZED: target.grasp_authorized,
            Precondition.CAPABILITY_VALID: state.capability_valid,
            Precondition.READY_REGION_BOUND: state.ready_region_valid,
            Precondition.OBJECT_ATTACHED: state.object_attached,
            Precondition.DELIVERY_TARGET_VALID: state.delivery_target_valid,
        }
        return bool(conditions[precondition])

    def evaluate(self, state: BeliefState, skill: SkillName) -> SafetyDecision:
        contract = self.registry.contract(skill)
        violations: List[str] = []
        for precondition in sorted(contract.preconditions, key=lambda item: item.value):
            try:
                passed = self._check(precondition, state)
            except KeyError:
                passed = False
            if not passed:
                violations.append(precondition.value)
        return SafetyDecision(not violations, tuple(violations))

    def require(self, state: BeliefState, skill: SkillName) -> SafetyDecision:
        decision = self.evaluate(state, skill)
        decision.require_allowed()
        return decision

    def valid_skills(self, state: BeliefState) -> Tuple[SkillName, ...]:
        return tuple(
            skill for skill in SkillName if self.evaluate(state, skill).allowed
        )
