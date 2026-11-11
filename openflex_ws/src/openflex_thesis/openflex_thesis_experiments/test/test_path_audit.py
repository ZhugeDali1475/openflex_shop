import json

import pytest

from openflex_thesis_experiments.path_audit import (
    audit_discrete_path,
    classify_invalid_indices,
    interpolate_and_audit_path,
    main,
    parse_moveit_path_failure,
)


@pytest.mark.parametrize(("indices", "total", "expected"), [
    ([], 5, "valid"),
    ([0], 5, "start"),
    ([4], 5, "goal"),
    ([0, 4], 5, "start_and_goal"),
    ([2], 5, "intermediate"),
    ([0, 2], 5, "start_and_intermediate"),
    ([2, 4], 5, "intermediate_and_goal"),
    ([0, 2, 4], 5, "start_and_intermediate_and_goal"),
    ([], 0, "unknown"),
])
def test_classify_invalid_indices(indices, total, expected):
    assert classify_invalid_indices(indices, total) == expected


def test_discrete_audit_retains_contacts_and_positions():
    result = audit_discrete_path([
        {"index": 0, "valid": True, "positions": {"j": 0.0}},
        {"index": 1, "valid": False, "contacts": ["shelf<->finger"],
         "positions": {"j": 0.5}},
        {"index": 2, "valid": True, "positions": {"j": 1.0}},
    ])
    assert result["category"] == "intermediate"
    assert result["invalid_indices"] == [1]
    assert result["invalid_states"][0]["contacts"] == ["shelf<->finger"]
    assert result["invalid_states"][0]["positions"] == {"j": 0.5}
    assert result["endpoint_states_checked"]
    assert result["intermediate_states_checked"]


def test_linear_diagnostic_sampler_catches_only_intermediate_collision():
    def checker(positions):
        if 0.4 < positions["j"] < 0.6:
            return False, ["shelf<->finger"]
        return True, []

    result = interpolate_and_audit_path({"j": 0.0}, {"j": 1.0}, checker, samples=11)
    assert result["category"] == "intermediate"
    assert result["invalid_indices"] == [5]
    assert result["sample_count"] == 11
    assert result["sampler"] == "linear_joint_space_diagnostic_only"


def test_sampler_requires_same_joints_and_two_samples():
    with pytest.raises(ValueError, match="at least two"):
        interpolate_and_audit_path({"j": 0.0}, {"j": 1.0}, lambda _: True, samples=1)
    with pytest.raises(ValueError, match="same joint names"):
        interpolate_and_audit_path({"j": 0.0}, {"k": 1.0}, lambda _: True)


def test_parse_moveit_failure_extracts_6301_evidence():
    log = """
    Computed path is not valid. Invalid states at index locations: [ 61 ] out of 156.
    Found a contact between 'mujoco_shelf_board_target_level' (type 'Object') and
    'openarmx_right_left_finger' (type 'Robot link'), which constitutes a collision.
    """
    result = parse_moveit_path_failure(log)
    assert result["parse_status"] == "parsed"
    failure = result["failures"][0]
    assert failure["invalid_indices"] == [61]
    assert failure["total_states"] == 156
    assert failure["category"] == "intermediate"
    assert failure["contacts"] == [
        "mujoco_shelf_board_target_level<->openarmx_right_left_finger"
    ]
    assert not result["continuous_safety_proven"]
    assert not result["endpoint_validity_proven"]


def test_parse_without_moveit_marker_is_unknown():
    result = parse_moveit_path_failure("planner returned no useful diagnostic")
    assert result["parse_status"] == "not_found"
    assert result["failures"] == []
    assert not result["continuous_safety_proven"]


@pytest.mark.parametrize("states", [[], [
    {"positions": {"joint": 0.0}},
    {"positions": {"joint": 1.0}, "valid": None},
]])
def test_unchecked_or_empty_paths_never_become_valid(states):
    result = audit_discrete_path(states)
    assert result["category"] == "unknown"
    assert not result["complete_sample_evidence"]
    assert not result["endpoint_states_checked"]


def test_mixed_known_failure_and_unknown_samples_keep_both():
    result = audit_discrete_path([
        {"positions": {"joint": 0.0}, "valid": False},
        {"positions": {"joint": 1.0}},
    ])
    assert result["category"] == "start"
    assert result["invalid_indices"] == [0]
    assert result["unknown_indices"] == [1]
    assert not result["complete_sample_evidence"]


@pytest.mark.parametrize("index,total", [(-1, 5), (5, 5), (1.1, 5), (True, 5), (0, 0)])
def test_out_of_range_indices_are_rejected(index, total):
    with pytest.raises(ValueError):
        classify_invalid_indices([index], total)


@pytest.mark.parametrize("record", [
    {"positions": {"joint": float("nan")}, "valid": True},
    {"positions": {"joint": float("inf")}, "valid": True},
    {"positions": {"joint": True}, "valid": True},
    {"positions": {"joint": 0.0}, "valid": "false"},
    {"positions": {"joint": 0.0}, "valid": 1},
    {"positions": {"joint": 0.0}, "contacts": "shelf<->finger"},
    {"positions": {}, "valid": True},
    {"positions": {"joint": 0.0}, "index": 1},
])
def test_malformed_samples_fail_closed(record):
    with pytest.raises(ValueError):
        audit_discrete_path([record])


def test_inconsistent_joint_sets_are_rejected():
    with pytest.raises(ValueError, match="same joint names"):
        audit_discrete_path([
            {"positions": {"first": 0.0}}, {"positions": {"second": 0.0}},
        ])


def test_checker_cannot_mutate_recorded_joint_positions():
    def checker(positions):
        positions["joint"] = 99.0
        return True

    result = interpolate_and_audit_path({"joint": 0.0}, {"joint": 1.0}, checker, samples=3)
    assert [state["positions"]["joint"] for state in result["states"]] == [0.0, 0.5, 1.0]
    assert not result["continuous_safety_proven"]


def test_separate_requests_do_not_mix_contact_pairs_or_execution_markers():
    log = """pregrasp target joints: {'joint': 0.1}
Invalid states at index locations: [ 2 ] out of 5.
Found a contact between 'old_shelf' (type 'Object') and 'old_finger' (type 'Robot link')
Completed listing of explanations for invalid states.
Not executing.
recovery_safe_park target joints: {'joint': 0.9}
Invalid states at index locations: [ 61 ] out of 156.
Found a contact between 'shelf' (type 'Object') and 'finger' (type 'Robot link')
Completed listing of explanations for invalid states.
Not executing.
"""
    result = parse_moveit_path_failure(log)
    assert result["failure_count"] == 2
    first, second = result["failures"]
    assert first["contacts"] == ["old_shelf<->old_finger"]
    assert second["contacts"] == ["shelf<->finger"]
    assert second["request_label"] == "recovery_safe_park"
    assert second["request_target_positions"] == {"joint": 0.9}
    assert second["line"] == 7
    assert second["execution_rejected_marker"]
    assert second["trajectory_positions"] is None
    assert second["request_start_positions"] is None


def test_missing_explanation_end_cannot_leak_next_request_contacts():
    result = parse_moveit_path_failure("""Invalid states at index locations: [ 1 ] out of 3.
next_move target joints: {'joint': 0.0}
Found a contact between 'other' (type 'Object') and 'link' (type 'Robot link')
Not executing.
""")
    assert result["failures"][0]["contacts"] == []
    assert not result["failures"][0]["execution_rejected_marker"]


@pytest.mark.parametrize("text", [
    "Invalid states at index locations: [ -1 ] out of 3.",
    "Invalid states at index locations: [ 3 ] out of 3.",
    "Invalid states at index locations: [ ] out of 3.",
    "Invalid states at index locations: [ lost log",
])
def test_malformed_failure_markers_do_not_become_positive_evidence(text):
    with pytest.raises(ValueError):
        parse_moveit_path_failure(text)


def test_cli_preserves_source_and_refuses_output_overwrite(tmp_path):
    source, output = tmp_path / "runtime.log", tmp_path / "audit.json"
    source.write_text("Invalid states at index locations: [ 1 ] out of 3.\n")
    original = source.read_bytes()
    main(["--log", str(source), "--output", str(output)])
    report = json.loads(output.read_text())
    assert report["failures"][0]["category"] == "intermediate"
    assert report["source_sha256"]
    with pytest.raises(FileExistsError):
        main(["--log", str(source), "--output", str(source)])
    with pytest.raises(FileExistsError):
        main(["--log", str(source), "--output", str(output)])
    assert source.read_bytes() == original


def test_cli_audits_recorded_states_without_invoking_a_checker(tmp_path):
    source, output = tmp_path / "states.json", tmp_path / "audit.json"
    source.write_text(json.dumps([
        {"positions": {"joint": 0.0}, "valid": True},
        {"positions": {"joint": 1.0}, "valid": False, "contacts": ["shelf<->finger"]},
    ]))
    main(["--states", str(source), "--output", str(output)])
    report = json.loads(output.read_text())
    assert report["category"] == "goal"
    assert report["invalid_states"][0]["positions"] == {"joint": 1.0}
