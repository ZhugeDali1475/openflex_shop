from pathlib import Path
import sys


BRIDGE = Path(__file__).parents[2] / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"
sys.path.insert(0, str(BRIDGE))

import pytest

from openflex_roboos_bridge.skill_contracts import (
    ATOMIC_SKILLS,
    COMPOSITE_SKILLS,
    MCP_ATOMIC_TOOL_MAP,
    AtomicSkillSpec,
    CompositeSkillSpec,
    CompositeStep,
    CompositeExecution,
    SkillRequest,
    SkillResult,
    SkillStatus,
    validate_catalog,
    validate_status_transition,
)


def _atomic(name):
    return next(spec for spec in ATOMIC_SKILLS if spec.name == name)


def test_catalog_is_valid_and_has_two_layers():
    validate_catalog()
    assert {spec.layer.value for spec in ATOMIC_SKILLS} == {"atomic"}
    assert {spec.layer.value for spec in COMPOSITE_SKILLS} == {"composite"}
    assert {spec.name for spec in COMPOSITE_SKILLS} >= {"pick_object", "deliver_object", "fetch_and_deliver"}


def test_request_rejects_free_coordinate_style_arguments_and_accepts_registered_ids():
    request = SkillRequest(
        request_id="r1", skill_name="navigate", station_id="pickup_station",
        arguments={"station_id": "pickup_station"}, timeout_sec=30, control_mode="NAV2",
    )
    _atomic("navigate").validate(request)
    with pytest.raises(ValueError, match="missing|required"):
        _atomic("navigate").validate(SkillRequest(request_id="r2", skill_name="navigate"))
    with pytest.raises(ValueError, match="non-empty string"):
        SkillRequest(request_id="r3", skill_name="navigate", station_id="")


def test_atomic_skill_enforces_mode_timeout_and_registered_name():
    with pytest.raises(ValueError, match="control_mode"):
        _atomic("navigate").validate(SkillRequest(
            request_id="r1", skill_name="navigate", station_id="pickup_station",
            arguments={"station_id": "pickup_station"}, control_mode="PLACE"))
    with pytest.raises(ValueError, match="timeout_sec"):
        _atomic("navigate").validate(SkillRequest(
            request_id="r2", skill_name="navigate", station_id="pickup_station",
            arguments={"station_id": "pickup_station"}, timeout_sec=601, control_mode="NAV2"))
    assert MCP_ATOMIC_TOOL_MAP["place_to_affordance"] == "execute_place"


def test_result_requires_explicit_success_and_evidence():
    with pytest.raises(ValueError, match="success=true"):
        SkillResult(request_id="r1", skill_name="observe", status=SkillStatus.SUCCEEDED)
    with pytest.raises(ValueError, match="source"):
        SkillResult(request_id="r1", skill_name="observe", status=SkillStatus.FAILED, source="")
    result = SkillResult(
        request_id="r1", skill_name="observe", status=SkillStatus.SUCCEEDED,
        success=True, source="mujoco_snapshot", evidence_refs=("snapshot.json",),
        safety_verified=True, freshness_verified=True,
        independent_verification_ref="independent_check.json",
    )
    assert result.terminal and result.completion_verified


def test_motion_success_without_measured_stop_or_independent_check_is_not_verified():
    result = SkillResult(
        request_id="nav-1", skill_name="navigate", status=SkillStatus.SUCCEEDED,
        success=True, source="ros_action", evidence_refs=("nav.json",),
        safety_verified=True, freshness_verified=True,
        independent_verification_ref="nav_stop_check.json", stopped_confirmed=False,
    )
    assert not result.completion_verified


def test_status_transitions_are_monotonic_and_unknown_is_terminal():
    validate_status_transition(None, SkillStatus.ACCEPTED)
    validate_status_transition(SkillStatus.ACCEPTED, SkillStatus.RUNNING)
    validate_status_transition(SkillStatus.RUNNING, SkillStatus.SUCCEEDED)
    validate_status_transition(None, SkillStatus.UNKNOWN)
    with pytest.raises(ValueError, match="illegal"):
        validate_status_transition(SkillStatus.SUCCEEDED, SkillStatus.RUNNING)


def test_composite_plan_rejects_cycles_and_unknown_dependencies():
    CompositeSkillSpec("ok", "1", (CompositeStep("a"), CompositeStep("b", ("a",))), 2).validate()
    with pytest.raises(ValueError, match="unknown"):
        CompositeSkillSpec("bad", "1", (CompositeStep("a", ("missing",)),), 1).validate()
    with pytest.raises(ValueError, match="cycle"):
        CompositeSkillSpec("bad", "1", (CompositeStep("a", ("b",)), CompositeStep("b", ("a",))), 2).validate()
    with pytest.raises(ValueError, match="must stop"):
        CompositeSkillSpec("unsafe", "1", (CompositeStep("a"),), 1,
                           stop_on_non_success=False).validate()


def test_composite_result_can_reference_child_receipts():
    result = SkillResult(
        request_id="composite-1", skill_name="fetch_and_deliver", status=SkillStatus.SUCCEEDED,
        success=True, source="roboos_worker", evidence_refs=("final_check.json",),
        child_request_ids=("pick-1", "deliver-1"), safety_verified=True, freshness_verified=True,
    )
    assert result.child_request_ids == ("pick-1", "deliver-1")


def _result(name, status=SkillStatus.SUCCEEDED, *, facts=None):
    return SkillResult(
        request_id=f"{name}-request", skill_name=name, status=status,
        success=True if status is SkillStatus.SUCCEEDED else None,
        source="offline_contract_test", evidence_refs=(f"{name}.json",),
        facts=facts or {},
    )


def test_composite_execution_is_single_flight_and_requires_explicit_reset():
    spec = CompositeSkillSpec(
        "test_flow", "1", (CompositeStep("observe"), CompositeStep("navigate", ("observe",))), 2,
    )
    execution = CompositeExecution(spec, "flow-1", started_at=100.0)
    assert execution.next_step(now=100.0) == "observe"
    execution.start_step("observe", now=100.0)
    with pytest.raises(RuntimeError, match="already active"):
        execution.start_step("navigate", now=100.0)
    execution.record_result("observe", _result("observe"), now=100.0)
    execution.start_step("navigate", now=100.0)
    execution.record_result("navigate", _result("navigate"), now=100.0)
    assert execution.succeeded and execution.requires_reset
    with pytest.raises(RuntimeError, match="termination"):
        execution.start_step("observe", now=100.0)
    execution.reset("scene_reset_receipt.json")
    assert execution.next_step(now=100.0) == "observe"


def test_composite_execution_stops_on_unknown_motion_until_stop_is_confirmed():
    spec = CompositeSkillSpec(
        "motion_flow", "1", (CompositeStep("navigate"), CompositeStep("execute_place", ("navigate",))), 2,
    )
    execution = CompositeExecution(spec, "flow-unknown", started_at=100.0)
    execution.start_step("navigate", now=100.0)
    execution.record_result("navigate", _result("navigate", SkillStatus.UNKNOWN), now=100.0)
    assert execution.halted_reason and execution.requires_stop_confirmation
    with pytest.raises(RuntimeError, match="confirm stop"):
        execution.reset("reset.json")
    execution.confirm_stop("stop_receipt.json")
    execution.reset("scene_reset_receipt.json")


def test_composite_execution_enforces_fact_gate_and_replan_budget():
    spec = CompositeSkillSpec(
        "fact_flow", "1", (
            CompositeStep("observe"),
            CompositeStep("verify_attachment", ("observe",), {"attached": True}),
        ), 2, max_replans=1,
    )
    execution = CompositeExecution(spec, "flow-facts", started_at=100.0)
    execution.start_step("observe", now=100.0)
    execution.record_result("observe", _result("observe", facts={"attached": True}), now=100.0)
    assert execution.next_step(now=100.0) == "verify_attachment"
    execution.request_replan("new observation", now=100.0)
    execution.request_replan("budget exceeded", now=100.0)
    assert execution.halted_reason == "replan budget exhausted"
    with pytest.raises(RuntimeError, match="terminal"):
        execution.request_replan("after halt", now=100.0)


def test_every_composite_skill_declares_all_four_budgets():
    for spec in COMPOSITE_SKILLS:
        spec.validate()
        assert spec.max_steps > 0
        assert spec.max_observations >= 0 and spec.max_replans >= 0
        assert spec.max_motion_steps >= 0 and spec.total_timeout_sec > 0
    for bad in (-1, True, 1.5):
        with pytest.raises(ValueError, match="motion budget"):
            CompositeSkillSpec("bad_motion", "1", (CompositeStep("observe"),), 2,
                               max_motion_steps=bad).validate()


def test_motion_budget_exhaustion_halts_safely_with_an_auditable_terminal_state():
    spec = CompositeSkillSpec("bounded_motion", "1", (
        CompositeStep("observe"),
        CompositeStep("navigate", ("observe",)),
        CompositeStep("retreat", ("navigate",)),
        CompositeStep("get_state", ("retreat",)),
    ), 4, max_motion_steps=1)
    execution = CompositeExecution(spec, "flow-motion-budget", started_at=100.0)
    execution.start_step("observe", now=100.0)
    execution.record_result("observe", _result("observe"), now=100.0)
    execution.start_step("navigate", now=100.0)
    execution.record_result("navigate", _result("navigate"), now=100.0)
    assert execution.motion_count == 1 and execution.halted_reason is None
    execution.start_step("retreat", now=100.0)
    assert execution.halted_reason == "motion budget exhausted"
    assert execution.requires_stop_confirmation is True
    assert execution.terminal and execution.active_step is None
    assert execution.motion_count == 2
    with pytest.raises(RuntimeError, match="termination"):
        execution.start_step("get_state", now=100.0)
    execution.confirm_stop("stop_receipt.json")
    snapshot = execution.snapshot()
    assert snapshot["motion_count"] == 2
    assert snapshot["budgets"]["max_motion_steps"] == 1
    assert snapshot["budgets"]["total_timeout_sec"] == spec.total_timeout_sec
    execution.reset("scene_reset_receipt.json")
    assert execution.motion_count == 0


def test_nested_composite_step_counts_toward_the_motion_budget():
    spec = CompositeSkillSpec("nested_motion", "1", (
        CompositeStep("observe"),
        CompositeStep("pick_object", ("observe",)),
        CompositeStep("get_state", ("pick_object",)),
    ), 3, max_motion_steps=1)
    execution = CompositeExecution(spec, "flow-nested-motion", started_at=100.0)
    execution.start_step("observe", now=100.0)
    execution.record_result("observe", _result("observe"), now=100.0)
    execution.start_step("pick_object", now=100.0)
    execution.record_result("pick_object", _result("pick_object"), now=100.0)
    assert execution.motion_count == 1
    execution.start_step("get_state", now=100.0)
    assert execution.halted_reason is None and execution.motion_count == 1
