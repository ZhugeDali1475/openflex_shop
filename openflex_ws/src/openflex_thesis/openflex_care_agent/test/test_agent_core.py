import dataclasses

import pytest

from openflex_care_agent.agent import CAREAgent
from openflex_care_agent.belief import (
    BeliefState,
    ObjectBelief,
    RobotContext,
    TaskPhase,
    TaskSpec,
)
from openflex_care_agent.failure import (
    FailureCode,
    FailureRecord,
    SkillOutcome,
    SkillStatus,
)
from openflex_care_agent.recovery import CapabilityEstimate, RecoveryPolicy
from openflex_care_agent.safety import SafetyCritic
from openflex_care_agent.skills import (
    ControlOwner,
    SkillName,
    SkillRegistry,
)


def belief(**changes):
    state = BeliefState(
        task=TaskSpec("pick the red box", "item-1", 120.0),
        phase=TaskPhase.GRASP,
        target=ObjectBelief(
            object_id="item-1",
            visible=True,
            grasp_authorized=True,
            confidence=0.95,
            map_version=7,
            age_sec=0.1,
            position_m=(0.8, 0.0, 0.9),
        ),
        robot=RobotContext(
            camera_fresh=True,
            odometry_fresh=True,
            joint_state_fresh=True,
            localization_valid=True,
            base_stopped=True,
            base_available=False,
            manipulation_available=True,
            perception_ready=True,
            moveit_ready=True,
            remaining_time_sec=90.0,
            map_version=7,
        ),
        capability_valid=True,
        ready_region_valid=True,
        object_attached=False,
        delivery_target_valid=False,
    )
    return dataclasses.replace(state, **changes)


def test_registry_contracts_are_complete_and_typed():
    registry = SkillRegistry()
    grasp = registry.contract(SkillName.PLAN_AND_GRASP)
    assert grasp.owner == ControlOwner.MANIPULATION
    assert grasp.preconditions
    assert grasp.postconditions
    assert SkillName.VERIFY_GRASP in registry.successors(SkillName.PLAN_AND_GRASP)


def test_safety_rejects_moveit_when_base_is_not_stopped():
    state = dataclasses.replace(
        belief(), robot=dataclasses.replace(belief().robot, base_stopped=False)
    )
    decision = SafetyCritic(SkillRegistry()).evaluate(state, SkillName.PLAN_AND_GRASP)
    assert not decision.allowed
    assert "base_stopped" in decision.violations


def test_safety_rejects_stale_map_target():
    state = dataclasses.replace(
        belief(),
        target=dataclasses.replace(belief().target, map_version=6),
    )
    decision = SafetyCritic(SkillRegistry()).evaluate(
        state, SkillName.COMPUTE_CAPABILITY
    )
    assert not decision.allowed
    assert "map_consistent" in decision.violations


def test_abort_is_always_safe():
    state = dataclasses.replace(
        belief(),
        robot=dataclasses.replace(
            belief().robot,
            camera_fresh=False,
            odometry_fresh=False,
            localization_valid=False,
            remaining_time_sec=0.0,
        ),
    )
    assert SafetyCritic(SkillRegistry()).evaluate(
        state, SkillName.ABORT_HOLD
    ).allowed


def test_recovery_prefers_information_action_for_occlusion():
    state = dataclasses.replace(
        belief(),
        target=dataclasses.replace(belief().target, visible=False),
        phase=TaskPhase.RECOVERY,
    )
    policy = RecoveryPolicy(SkillRegistry(), SafetyCritic(SkillRegistry()))
    ranked = policy.rank(state, FailureCode.TARGET_OCCLUDED)
    assert ranked[0].skill in {
        SkillName.CHANGE_VIEW,
        SkillName.RE_OBSERVE,
    }
    assert all(item.skill != SkillName.PLAN_AND_GRASP for item in ranked)


def test_recovery_uses_precomputed_capability_estimates():
    state = dataclasses.replace(
        belief(),
        phase=TaskPhase.RECOVERY,
        target=dataclasses.replace(belief().target, visible=False),
        robot=dataclasses.replace(belief().robot, base_available=True),
    )
    policy = RecoveryPolicy(SkillRegistry(), SafetyCritic(SkillRegistry()))
    estimates = {
        SkillName.CHANGE_VIEW: CapabilityEstimate(0.9, 0.9, 2.0, 0.1, 0.01),
        SkillName.RE_OBSERVE: CapabilityEstimate(0.2, 0.1, 8.0, 0.0, 0.4),
    }
    chosen = policy.choose(state, FailureCode.TARGET_OCCLUDED, estimates)
    assert chosen.skill == SkillName.CHANGE_VIEW


def test_recovery_budget_fails_closed():
    registry = SkillRegistry()
    safety = SafetyCritic(registry)
    state = dataclasses.replace(
        belief(),
        phase=TaskPhase.RECOVERY,
        failure_history=tuple(
            FailureRecord(
                failure_code=FailureCode.TARGET_NOT_FOUND,
                skill=SkillName.LOOK_FOR_OBJECT,
                attempt=index,
            )
            for index in (1, 2, 3)
        ),
    )
    policy = RecoveryPolicy(
        registry, safety, maximum_recovery_attempts=3
    )
    assert policy.candidates(state, FailureCode.TARGET_NOT_FOUND) == (
        SkillName.ABORT_HOLD,
    )


def test_nominal_agent_rejects_blocked_grasp():
    state = dataclasses.replace(
        belief(), robot=dataclasses.replace(belief().robot, moveit_ready=False)
    )
    decision = CAREAgent().decide(state)
    assert decision.skill == SkillName.ABORT_HOLD
    assert decision.fallback


def test_successful_outcome_requires_postcondition():
    with pytest.raises(ValueError):
        SkillOutcome(
            skill=SkillName.LOOK_FOR_OBJECT,
            status=SkillStatus.SUCCESS,
            postcondition_verified=False,
            elapsed_sec=1.0,
        )
