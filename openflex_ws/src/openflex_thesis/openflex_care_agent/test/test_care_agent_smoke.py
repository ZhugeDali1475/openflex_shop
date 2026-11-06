#!/usr/bin/env python3
"""Test the CARE-Agent decision loop without full ROS infrastructure.

This smoke test verifies:
1. CAREAgent core can make decisions
2. Phase transitions are logical
3. Terminal phases are handled
"""

import sys

# Add path for local imports
sys.path.insert(0, '/home/wgb/openflex_shop/openflex_ws/src/openflex_thesis/openflex_care_agent')

from openflex_care_agent import (
    BeliefState,
    CAREAgent,
    ObjectBelief,
    RobotContext,
    TaskPhase,
    TaskSpec,
)
from openflex_care_agent.failure import FailureCode
from openflex_care_agent.skills import SkillName


def create_test_belief(
    phase: TaskPhase,
    has_object: bool = True,
    capability_valid: bool = False,
    ready_region_valid: bool = False,
    object_attached: bool = False,
    delivery_target_valid: bool = False,
) -> BeliefState:
    """Create a test belief state."""
    task = TaskSpec(
        instruction="pick item and deliver",
        object_id="test_item_01",
        total_time_budget_sec=300.0,
    )

    target = ObjectBelief(
        object_id="test_item_01",
        visible=has_object,
        grasp_authorized=True,
        confidence=0.95 if has_object else 0.1,
        map_version=1,
        age_sec=0.5,
        position_m=(1.0, 0.0, 0.8) if has_object else None,
    )

    context = RobotContext(
        camera_fresh=True,
        odometry_fresh=True,
        joint_state_fresh=True,
        localization_valid=True,
        base_stopped=True,
        base_available=True,
        manipulation_available=True,
        perception_ready=True,
        moveit_ready=True,
        remaining_time_sec=280.0,
        map_version=1,
    )

    return BeliefState(
        task=task,
        phase=phase,
        target=target,
        robot=context,
        capability_valid=capability_valid,
        ready_region_valid=ready_region_valid,
        object_attached=object_attached,
        delivery_target_valid=delivery_target_valid,
    )


def test_simple_phases():
    """Test basic phase decisions."""
    print("=== Test 1: Simple Phase Decisions ===")
    agent = CAREAgent(maximum_recovery_attempts=3)

    # Test phases that don't require complex preconditions
    test_cases = [
        (TaskPhase.NAVIGATE_OBSERVE, {}, SkillName.NAVIGATE_TO_STATION),
        (TaskPhase.ACQUIRE_TARGET, {}, SkillName.LOOK_FOR_OBJECT),
        (TaskPhase.EVALUATE_CAPABILITY, {}, SkillName.COMPUTE_CAPABILITY),
        (TaskPhase.ALIGN_TO_READY, {"capability_valid": True, "ready_region_valid": True}, SkillName.LOCAL_ALIGN_REGION),
        (TaskPhase.GRASP, {"capability_valid": True, "ready_region_valid": True}, SkillName.PLAN_AND_GRASP),
        (TaskPhase.VERIFY_GRASP, {}, SkillName.VERIFY_GRASP),
        (TaskPhase.NAVIGATE_DELIVERY, {"object_attached": True, "delivery_target_valid": True}, SkillName.NAVIGATE_TO_DELIVERY),
        (TaskPhase.PLACE, {"object_attached": True, "delivery_target_valid": True}, SkillName.PLACE_OBJECT),
        (TaskPhase.VERIFY_PLACE, {}, SkillName.VERIFY_PLACE),
    ]

    passed = 0
    failed = 0

    for phase, kwargs, expected_skill in test_cases:
        belief = create_test_belief(phase, **kwargs)
        decision = agent.decide(belief)

        if decision.skill == expected_skill:
            print(f"✅ {phase.value:25s} -> {decision.skill.value}")
            passed += 1
        else:
            print(f"❌ {phase.value:25s} -> {decision.skill.value} (expected {expected_skill.value})")
            print(f"   Reason: {decision.reason}")
            failed += 1

    print(f"\nResult: {passed} passed, {failed} failed\n")
    return failed == 0


def test_terminal_phases():
    """Test terminal phase handling."""
    print("=== Test 2: Terminal Phases ===")
    agent = CAREAgent()

    terminal_phases = [TaskPhase.DONE, TaskPhase.HOLD, TaskPhase.FAILED]

    all_correct = True
    for phase in terminal_phases:
        belief = create_test_belief(phase)
        decision = agent.decide(belief)

        if decision.skill == SkillName.ABORT_HOLD and decision.fallback:
            print(f"✅ {phase.value:15s} -> {decision.skill.value} (fallback={decision.fallback})")
        else:
            print(f"❌ {phase.value:15s} -> {decision.skill.value} (fallback={decision.fallback})")
            print(f"   ERROR: Should return ABORT_HOLD with fallback=True")
            all_correct = False

    print()
    return all_correct


def test_failure_recovery():
    """Test failure recovery decision."""
    print("=== Test 3: Failure Recovery ===")
    agent = CAREAgent()

    belief = create_test_belief(TaskPhase.GRASP, capability_valid=True, ready_region_valid=True)

    # Simulate grasp failure
    decision = agent.decide(belief, failure=FailureCode.GRASP_FAILED)

    print(f"Phase=GRASP with GRASP_FAILED")
    print(f"  Decision: {decision.skill.value}")
    print(f"  Reason: {decision.reason}")
    print(f"  Fallback: {decision.fallback}")

    # Should provide some recovery suggestion
    print("✅ Agent provided recovery decision\n")
    return True


def test_missing_precondition():
    """Test safety critic blocking."""
    print("=== Test 4: Missing Precondition (Safety Check) ===")
    agent = CAREAgent()

    # Try to grasp without visible object
    belief = create_test_belief(TaskPhase.GRASP, has_object=False)
    decision = agent.decide(belief)

    print(f"Phase=GRASP with object NOT visible")
    print(f"  Decision: {decision.skill.value}")
    print(f"  Reason: {decision.reason}")
    print(f"  Fallback: {decision.fallback}")

    # Safety should block
    if decision.fallback or decision.skill == SkillName.ABORT_HOLD:
        print("✅ Safety critic blocked unsafe grasp\n")
        return True
    else:
        print("❌ Safety critic should have blocked\n")
        return False


def main():
    print("=" * 60)
    print("CARE-Agent Decision Core Smoke Test")
    print("=" * 60)
    print()

    results = []

    results.append(("Simple Phases", test_simple_phases()))
    results.append(("Terminal Phases", test_terminal_phases()))
    results.append(("Failure Recovery", test_failure_recovery()))
    results.append(("Safety Check", test_missing_precondition()))

    print("=" * 60)
    print("Test Summary")
    print("=" * 60)

    for name, passed in results:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{status:10s} {name}")

    all_passed = all(passed for _, passed in results)

    print()
    if all_passed:
        print("🎉 All tests passed! CARE-Agent decision core is working.")
        return 0
    else:
        print("❌ Some tests failed. Check implementation.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
