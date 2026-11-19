from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.evidence_schema import (  # noqa: E402
    EvidenceSchemaError,
    PerceptionEvidence,
    SimulationEvaluationEvidence,
)


def perception(**updates):
    values = dict(
        request_id="run_1", attempt=1, frame_sha256="abc123",
        frame_ref="frame.png", depth_ref="depth.png",
        record={"target_observation": {"mujoco_object_id": None}, "physical_verification": False},
    )
    values.update(updates)
    return PerceptionEvidence(**values)


def test_perception_and_evaluation_layers_are_distinct():
    visual = perception().to_dict()
    evaluation = SimulationEvaluationEvidence("run_1", 1).to_dict()
    assert visual["evidence_layer"] == "perception"
    assert visual["physical_verification"] is False
    assert evaluation["evidence_layer"] == "simulation_evaluation"
    assert evaluation["mujoco_object_id"] is None
    assert evaluation["physical_verification"] is False


def test_perception_rejects_nonnull_simulator_identity():
    with pytest.raises(EvidenceSchemaError, match="simulator identity"):
        perception(record={"target_observation": {"selected_object_id": "thesis_item_01"}}).to_dict()


def test_evaluation_requires_identity_for_verified_truth():
    with pytest.raises(EvidenceSchemaError, match="object id"):
        SimulationEvaluationEvidence("run_1", 1, physical_verification=True).to_dict()
    record = SimulationEvaluationEvidence(
        "run_1", 1, mujoco_object_id="thesis_item_01", physical_verification=True,
        source="simulator_contact_evaluator",
    ).to_dict()
    assert record["mujoco_object_id"] == "thesis_item_01"
