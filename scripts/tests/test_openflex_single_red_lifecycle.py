from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.single_red_lifecycle import (  # noqa: E402
    EvidenceReference,
    LifecycleTransitionError,
    attach_target,
    begin_transport,
    confirm_approach,
    confirm_placed,
    initial_single_red_lifecycle,
    reset_after_audited_rebuild,
    select_target,
)


def ev(name, **details):
    return EvidenceReference(name, "offline_test", details)


def test_single_red_lifecycle_records_each_transition_and_destination():
    state = initial_single_red_lifecycle("run-a")
    state = select_target(state, ev("selection", candidate_unique=True))
    state = confirm_approach(state, ev("approach", stopped_confirmed=True))
    state = attach_target(state, ev("grasp", grasp_precheck_passed=True))
    state = begin_transport(state, ev("attachment", attachment_independently_verified=True))
    state = confirm_placed(state, ev("release", release_verified=True,
                                      settled_on_registered_surface=True,
                                      retreat_verified=True,
                                      destination_id="sim_delivery_top_left"))
    assert state.phase == "placed"
    assert [item.evidence_id for item in state.evidence] == [
        "selection", "approach", "grasp", "attachment", "release"]


def test_lifecycle_rejects_skipping_attachment_and_requires_explicit_reset():
    state = initial_single_red_lifecycle("run-a")
    state = select_target(state, ev("selection"))
    with pytest.raises(LifecycleTransitionError):
        begin_transport(state, ev("attachment", attachment_independently_verified=True))
    with pytest.raises(LifecycleTransitionError):
        reset_after_audited_rebuild(state, ev("reset", scene_rebuilt=True,
                                               post_reset_audit_passed=True, new_run_id="run-b"))


def test_reset_requires_new_run_and_audit_after_placement():
    state = initial_single_red_lifecycle("run-a")
    state = select_target(state, ev("selection"))
    state = confirm_approach(state, ev("approach"))
    state = attach_target(state, ev("grasp", grasp_precheck_passed=True))
    state = begin_transport(state, ev("attachment", attachment_independently_verified=True))
    state = confirm_placed(state, ev("release", release_verified=True,
                                      settled_on_registered_surface=True,
                                      retreat_verified=True,
                                      destination_id="delivery_zone"))
    with pytest.raises(LifecycleTransitionError):
        reset_after_audited_rebuild(state, ev("reset", scene_rebuilt=True,
                                               post_reset_audit_passed=True, new_run_id="run-a"))
    reset = reset_after_audited_rebuild(
        state, ev("reset", scene_rebuilt=True, post_reset_audit_passed=True, new_run_id="run-b"))
    assert reset.phase == "world" and reset.run_id == "run-b"
    assert reset.evidence[-1].evidence_id == "reset"


@pytest.mark.parametrize("details", [
    {"release_verified": True, "settled_on_registered_surface": True,
     "destination_id": "sim_delivery_top_left"},
    {"release_verified": True, "settled_on_registered_surface": True,
     "retreat_verified": True, "destination_id": "unregistered"},
])
def test_placement_requires_complete_registered_destination_evidence(details):
    state = initial_single_red_lifecycle("run-a")
    state = select_target(state, ev("selection"))
    state = confirm_approach(state, ev("approach"))
    state = attach_target(state, ev("grasp", grasp_precheck_passed=True))
    state = begin_transport(state, ev("attachment", attachment_independently_verified=True))
    with pytest.raises(LifecycleTransitionError):
        confirm_placed(state, ev("release", **details))
