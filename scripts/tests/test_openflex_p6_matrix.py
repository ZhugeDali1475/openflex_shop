"""Contract checks for the P6 evaluation matrix reducer.

The reducer must stay *auditable*: every reported attempt keeps the v9 scene hash
and the simulation boundary, excluded batches must carry a reason, and no attempt
may be silently dropped or rewritten as a success.
"""

import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "aggregate_openflex_p6_matrix.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("aggregate_openflex_p6_matrix", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def matrix():
    module = _load_module()
    rows, atomic, excluded, provenance = module.collect()
    return module, rows, atomic, excluded, provenance


def test_reducer_runs_without_starting_the_stack(matrix):
    module, rows, atomic, excluded, provenance = matrix
    assert rows and atomic
    assert provenance["physical_verification"] is False
    assert provenance["v9_scene_sha256"] == module.V9_SCENE_SHA


def test_every_reported_row_keeps_scene_hash_and_boundary(matrix):
    module, rows, atomic, _, _ = matrix
    for row in rows + atomic:
        assert row["scene_sha256"] == module.V9_SCENE_SHA
        assert row["physical_verification"] is False


def test_excluded_batches_are_listed_with_a_reason(matrix):
    _, rows, _, excluded, _ = matrix
    assert excluded, "expected excluded batches to be listed with reasons"
    assert all(reason.strip() for reason in excluded.values())
    reported = {row["batch"] for row in rows}
    assert not (set(excluded) & reported), "an excluded batch must not be reported"


def test_summaries_account_for_every_row(matrix):
    module, rows, atomic, _, _ = matrix
    groups = module.summarize(rows)
    assert sum(group["total"] for group in groups.values()) == len(rows)
    for group in groups.values():
        assert len(group["failure_types"]) == 0 or group["passed"] < group["total"]


def test_negative_controls_are_separated_from_fetch_success(matrix):
    module, rows, _, _, _ = matrix
    groups = module.summarize(rows)
    assert "search_and_pick_negative_control" in groups
    # A fail-closed control is the wanted outcome and must not be merged into the
    # fetch success rate it would otherwise inflate.
    fetch_total = groups["search_and_pick"]["total"]
    assert fetch_total == sum(1 for row in rows if row["family"] == "search_and_pick")


def test_atomic_navigation_never_reported_as_a_failure(matrix):
    module, _, atomic, _, _ = matrix
    summary = module.summarize_atomic(atomic)
    if "navigate_to_target" in summary:
        assert summary["navigate_to_target"]["rate"] == 1.0


def test_model_policy_online_is_separated_from_pre_vram_fix(matrix):
    module, rows, _, _, _ = matrix
    groups = module.summarize(rows)
    assert "model_policy_online" in groups
    assert "model_policy_before_vram_fix" in groups
    online = [row for row in rows if row["family"] == "model_policy_online"]
    assert online, "expected the post-VRAM-fix model rounds to be reported"
    for row in online:
        assert row["model_steps"] and row["model_steps"] > 0
        assert row["stopped_confirmed"] is True
    # The pre-fix attempts died in dispatch with no model decision at all and
    # must never be folded into the online family rate.
    assert groups["model_policy_before_vram_fix"]["rate"] == 0.0
    assert all(group["total"] > group["passed"] or group["passed"] == 0
               for key, group in groups.items() if key == "model_policy_online")


def test_recorded_stop_reads_back_the_recorded_value():
    module = _load_module()
    assert module._recorded_stop({"error": "x 'stopped_confirmed': True y"}, []) is True
    assert module._recorded_stop({"error": "x 'stopped_confirmed': False y"}, []) is False
    assert module._recorded_stop({"success": True, "completion_verified": True}, []) is True
    assert module._recorded_stop({}, []) is None
    assert module._recorded_stop(
        {}, [{"stopped_confirmed": True}, {"stopped_confirmed": False}]) is False
