from openflex_thesis_experiments.recovery_report import bootstrap_difference, paired_comparison, probe_gate


def test_bootstrap_constant_difference():
    assert bootstrap_difference([1, 1, 1], repetitions=100)["bootstrap_95"] == [1, 1]


def test_gate_cannot_pass_on_a_single_successful_demo():
    gate = probe_gate([{"method": "probe_switch_grasp", "scenario": {"scenario_id": "one"},
                        "success": True, "recovery_decisions": 1}])
    assert not gate["ready_for_expansion"]


def test_different_source_versions_are_not_paired():
    first = {"method": "heuristic", "scenario": {"scenario_id": "one"}, "seed": 1,
             "success": False, "source_sha256": {"code": "old"}}
    second = {**first, "method": "learned_memory", "success": True, "source_sha256": {"code": "new"}}
    comparison = paired_comparison([first, second], "heuristic", "learned_memory")
    assert comparison["success_difference"] is None
    assert comparison["rejected_pairs"] == 1
