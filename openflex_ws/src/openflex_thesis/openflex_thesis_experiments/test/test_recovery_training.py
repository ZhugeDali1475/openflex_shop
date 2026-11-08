import pytest

from openflex_thesis_experiments.recovery_training import grouped_split, rows


def records():
    return [{"scenario_id": f"scene_{index}", "task_id": f"task_{index}", "label": outcome,
             "features": {"repeat_count": 3, "score": 0.8}, "action": "reobserve",
             "reason": "alignment", "stage": "align"}
            for index in range(10) for outcome in (0, 1)]


def test_grouped_split_never_leaks_tasks_or_scenarios():
    splits = grouped_split(records())
    groups = [set(record["scenario_id"] for record in values) for values in splits.values()]
    assert not groups[0] & groups[1]
    assert not groups[0] & groups[2]
    assert not groups[1] & groups[2]
    assert sum(len(values) for values in splits.values()) == 20


def test_small_dataset_is_rejected():
    with pytest.raises(ValueError):
        grouped_split(records()[:4])


def test_ablation_removes_memory_and_capability():
    row = rows(records()[:1], False, True)[0]
    assert row["repeat_count"] == 0
    assert row["score"] == 0


def test_training_and_inference_roundtrip_is_software_only(tmp_path):
    pytest.importorskip("sklearn")
    from openflex_care_agent.experience import RecoveryOption, RecoverySelector
    from openflex_thesis_experiments.recovery_training import fit_models
    report = fit_models(records(), tmp_path)
    assert 0.0 <= report["test_brier"] <= 1.0
    selector = RecoverySelector("learned_memory", str(tmp_path / "model.joblib"))
    probability = selector.probability(RecoveryOption("reobserve", (), {"duration_sec": 4.0}), "alignment", "align")
    assert 0.0 <= probability <= 1.0
    with pytest.raises(ValueError):
        RecoverySelector("learned", str(tmp_path / "model.joblib"))


def test_formal_test_data_and_duplicate_transitions_are_rejected(tmp_path):
    import json
    from openflex_thesis_experiments.recovery_training import read_transitions
    record = {"event": "transition", "label": 1, "evidence_class": "mujoco_oracle_geometry_proxy",
              "scenario_id": "scene", "task_id": "task", "monotonic_sec": 1.0, "partition": "held_out"}
    path = tmp_path / "events.jsonl"
    path.write_text(json.dumps(record) + "\n")
    with pytest.raises(ValueError):
        read_transitions([path])
    record["partition"] = "development"
    path.write_text((json.dumps(record) + "\n") * 2)
    with pytest.raises(ValueError):
        read_transitions([path])
