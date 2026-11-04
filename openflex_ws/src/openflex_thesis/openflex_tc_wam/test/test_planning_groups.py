from pathlib import Path

import yaml


def test_both_active_arm_groups_jointly_include_lift():
    path = Path(__file__).resolve().parents[1] / "config" / "planning_groups.yaml"
    with path.open("r", encoding="utf-8") as stream:
        groups = yaml.safe_load(stream)["planning_groups"]
    assert set(groups) == {"left_arm_with_lift", "right_arm_with_lift"}
    for name, contract in groups.items():
        assert contract["joint_names"][0] == "lift_joint"
        assert len(contract["joint_names"]) == 8
        assert len(contract["joint_lower"]) == 8
        assert len(contract["joint_upper"]) == 8
        assert contract["ik_link"].endswith("_hand_tcp")
        assert name.endswith("_arm_with_lift")
