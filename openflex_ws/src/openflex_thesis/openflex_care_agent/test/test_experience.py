import pytest

from openflex_care_agent.experience import (
    FailureMemory, FailureRecord, RecoveryJournal, RecoveryOption, RecoverySelector,
)


def option(action="reobserve", duration=4.0, repeat=0):
    return RecoveryOption(action, (0.0, 0.0), {"duration_sec": duration, "repeat_count": repeat})


def test_budget_includes_next_grasp():
    selector = RecoverySelector()
    assert selector.choose([option()], "alignment", "align", 10, 7) is None
    assert selector.choose([option()], "alignment", "align", 11, 7) is not None


def test_memory_ignores_observation_failure_and_decays_on_motion():
    memory = FailureMemory()
    memory.add(FailureRecord("item", 1, (0, 0, 0), (0, 0), "stale_observation"))
    assert memory.repeats("item", (0, 0, 0), (0, 0)) == 0
    memory.add(FailureRecord("item", 1, (0, 0, 0), (0, 0), "ik_collision"))
    assert memory.repeats("item", (0, 0, 0), (0, 0)) == 1
    assert memory.repeats("item", (1, 0, 0), (0, 0)) == 0
    assert memory.repeats("other", (0, 0, 0), (0, 0)) == 0


def test_memory_changes_choice_but_ablation_does_not():
    options = [option("switch_grasp", repeat=2), option("reobserve")]
    assert RecoverySelector("heuristic").choose(options, "planning", "grasp", 100, 20).action == "switch_grasp"
    assert RecoverySelector("heuristic_memory").choose(options, "planning", "grasp", 100, 20).action == "reobserve"


def test_model_is_required():
    with pytest.raises(ValueError):
        RecoverySelector("learned")


def test_journal_labels_only_completed_transition(tmp_path):
    journal = RecoveryJournal(tmp_path / "events.jsonl", "task", {})
    journal.begin(option(), "alignment", "align")
    with pytest.raises(RuntimeError):
        journal.begin(option(), "alignment", "align")
    journal.finish(None, "infrastructure")
    journal.begin(option(), "alignment", "align")
    journal.finish(True, "grasp_verified")
    import json
    transitions = [json.loads(line) for line in (tmp_path / "events.jsonl").read_text().splitlines()
                   if json.loads(line)["event"] == "transition"]
    assert [record["label"] for record in transitions] == [None, 1]


def test_nonfinite_features_rejected():
    with pytest.raises(ValueError):
        RecoveryOption("reobserve", (), {"duration_sec": float("nan")})
