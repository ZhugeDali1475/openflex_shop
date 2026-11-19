"""Offline, evidence-carrying lifecycle for the single-red thesis scenario.

This is a design/test contract only: it does not update MuJoCo, ROS, or
MoveIt.  Runtime evidence must identify its source and remain auditable.
"""

from dataclasses import dataclass
from typing import Mapping, Tuple

from .competition_registry import TARGET_OBJECT_ID


class LifecycleTransitionError(ValueError):
    """Raised when a single-object lifecycle transition is not authorized."""


@dataclass(frozen=True)
class EvidenceReference:
    evidence_id: str
    source: str
    details: Mapping[str, object]

    def validate(self) -> None:
        if not isinstance(self.evidence_id, str) or not self.evidence_id.strip():
            raise LifecycleTransitionError("Every transition requires an evidence_id")
        if not isinstance(self.source, str) or not self.source.strip():
            raise LifecycleTransitionError("Every transition requires an evidence source")
        if not isinstance(self.details, Mapping):
            raise LifecycleTransitionError("Evidence details must be a mapping")


@dataclass(frozen=True)
class SingleRedLifecycle:
    object_id: str
    phase: str
    run_id: str
    evidence: Tuple[EvidenceReference, ...] = ()

    def validate(self) -> None:
        if self.object_id != TARGET_OBJECT_ID:
            raise LifecycleTransitionError("Only thesis_item_01 is in this lifecycle")
        if self.phase not in {"world", "selected", "approach", "attached", "transport", "placed"}:
            raise LifecycleTransitionError(f"Unknown lifecycle phase: {self.phase}")
        if not isinstance(self.run_id, str) or not self.run_id.strip():
            raise LifecycleTransitionError("run_id must be a nonempty string")
        for evidence in self.evidence:
            evidence.validate()


def initial_single_red_lifecycle(run_id: str) -> SingleRedLifecycle:
    state = SingleRedLifecycle(TARGET_OBJECT_ID, "world", run_id)
    state.validate()
    return state


def _advance(state: SingleRedLifecycle, expected: str, phase: str,
             evidence: EvidenceReference) -> SingleRedLifecycle:
    state.validate()
    evidence.validate()
    if state.phase != expected:
        raise LifecycleTransitionError(
            f"Transition to {phase} requires {expected}; current phase is {state.phase}")
    result = SingleRedLifecycle(state.object_id, phase, state.run_id,
                                state.evidence + (evidence,))
    result.validate()
    return result


def select_target(state: SingleRedLifecycle, evidence: EvidenceReference) -> SingleRedLifecycle:
    return _advance(state, "world", "selected", evidence)


def confirm_approach(state: SingleRedLifecycle, evidence: EvidenceReference) -> SingleRedLifecycle:
    return _advance(state, "selected", "approach", evidence)


def attach_target(state: SingleRedLifecycle, evidence: EvidenceReference) -> SingleRedLifecycle:
    if evidence.details.get("grasp_precheck_passed") is not True:
        raise LifecycleTransitionError("Attachment requires a passing grasp precheck reference")
    return _advance(state, "approach", "attached", evidence)


def begin_transport(state: SingleRedLifecycle, evidence: EvidenceReference) -> SingleRedLifecycle:
    if evidence.details.get("attachment_independently_verified") is not True:
        raise LifecycleTransitionError("Transport requires independent attachment verification")
    return _advance(state, "attached", "transport", evidence)


def confirm_placed(state: SingleRedLifecycle, evidence: EvidenceReference) -> SingleRedLifecycle:
    required = ("release_verified", "settled_on_registered_surface", "retreat_verified")
    if any(evidence.details.get(key) is not True for key in required):
        raise LifecycleTransitionError("Placement requires release, settling, and retreat evidence")
    if evidence.details.get("destination_id") not in {
            "sim_delivery_top_left", "sim_delivery_top_right", "delivery_zone"}:
        raise LifecycleTransitionError("Placement evidence must name a registered destination")
    return _advance(state, "transport", "placed", evidence)


def reset_after_audited_rebuild(state: SingleRedLifecycle,
                                evidence: EvidenceReference) -> SingleRedLifecycle:
    state.validate()
    evidence.validate()
    if state.phase != "placed":
        raise LifecycleTransitionError("Reset requires a completed placed state")
    if evidence.details.get("scene_rebuilt") is not True or evidence.details.get(
            "post_reset_audit_passed") is not True:
        raise LifecycleTransitionError("Reset requires explicit scene rebuild and post-reset audit")
    next_run_id = evidence.details.get("new_run_id")
    if not isinstance(next_run_id, str) or not next_run_id.strip() or next_run_id == state.run_id:
        raise LifecycleTransitionError("Reset must start a distinct, explicitly named run")
    result = SingleRedLifecycle(state.object_id, "world", next_run_id,
                                state.evidence + (evidence,))
    result.validate()
    return result
