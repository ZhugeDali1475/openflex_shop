from openflex_thesis_experiments.recovery_benchmark import classify_log, summarize


def test_terminal_success_wins_over_outer_timeout():
    assert classify_log("MUJOCO_MISSION_SUCCESS duration=12.3s", True)["success"] is True


def test_no_terminal_is_not_task_failure():
    assert classify_log("running", True) == {"status": "outer_timeout", "success": None}


def test_conflicting_terminals_are_invalid():
    text = 'RECOVERY_TASK_RESULT {"task_id":"a","success":true}\nRECOVERY_TASK_RESULT {"task_id":"a","success":false}'
    assert classify_log(text)["status"] == "invalid_terminal"


def test_infrastructure_is_reported_not_silently_discarded():
    report = summarize([{"method": "fixed", "success": True}, {"method": "fixed", "success": None}])
    assert report["fixed"]["valid_task_terminals"] == 1
    assert report["fixed"]["infrastructure_or_outer_timeout"] == 1


def test_recovery_denominator_includes_failed_trials_without_available_actions():
    report = summarize([{"method": "fixed", "success": False, "recovery_decisions": 0},
                        {"method": "fixed", "success": True, "recovery_decisions": 1,
                         "recovery_success": True}])
    assert report["fixed"]["first_fault_trials"] == 2
    assert report["fixed"]["attempted_recovery_trials"] == 1
    assert report["fixed"]["recovery_success_rate"] == 0.5


def test_protocol_has_fifteen_independent_configurations():
    from openflex_thesis_experiments.recovery_protocol import matrix
    probes = matrix("probe")
    assert len(probes) == 45
    assert len(set(trial["scenario"]["scenario_id"] for trial in probes)) == 15
    assert {trial["seed"] for trial in probes}.isdisjoint(
        {trial["seed"] for trial in matrix("collect")})
    evaluation = matrix("evaluate", "memory-model", "no-memory-model")
    assert len(evaluation) == 450
    assert {trial["scenario"]["scenario_id"] for trial in matrix("collect")}.isdisjoint(
        {trial["scenario"]["scenario_id"] for trial in evaluation})
