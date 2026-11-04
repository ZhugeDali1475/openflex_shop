from openflex_thesis_experiments.trial_matrix import expand_protocol
from pathlib import Path

import yaml


def test_factorial_expansion_is_deterministic():
    protocol = {
        "protocol_version": "v1",
        "experiments": [
            {"name": "demo", "repetitions": 2, "factors": {"method": ["a", "b"]}}
        ],
    }
    rows = expand_protocol(protocol)
    assert [row["trial_id"] for row in rows] == [
        "demo-0001", "demo-0002", "demo-0003", "demo-0004"
    ]


def test_frozen_template_covers_final_layered_pipeline_and_ablations():
    path = Path(__file__).resolve().parents[1] / "config" / "experiment_protocol.yaml"
    with path.open("r", encoding="utf-8") as stream:
        protocol = yaml.safe_load(stream)
    names = {item["name"] for item in protocol["experiments"]}
    assert "anygrasp_repeatability" in names
    assert "tcwam_method_comparison" in names
    assert "local_alignment_comparison" in names
    assert "refined_grasp_ablation" in names
    end_to_end = next(item for item in protocol["experiments"] if item["name"] == "end_to_end")
    assert "nav2_anygrasp_tcwam_act_moveit" in end_to_end["factors"]["pipeline"]
