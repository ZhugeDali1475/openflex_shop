from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"


def load(name):
    with (CONFIG / name).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_canonical_dimensions_match_declared_blocks():
    schema = load("act_schema.yaml")
    observation = schema["numeric_observation"]
    assert sum(block["dimension"] for block in observation["blocks"]) == 24
    assert observation["dimension"] == 24
    assert len(schema["action"]["order"]) == 3
    assert schema["action"]["dimension"] == 3
    assert schema["scope"]["act_may_command_lift_arm_or_gripper"] is False


def test_lerobot_target_schema_is_base_only_and_safe_topic():
    schema = load("lerobot_thesis_m2_schema.yaml")
    assert schema["requires_bridge_patch"] is True
    assert schema["observations"]["arms"]["right"]["enabled"] is False
    assert schema["observations"]["arms"]["left"]["enabled"] is False
    assert schema["observations"]["lift"]["enabled"] is False
    assert len(schema["observations"]["tc_wam"]["fields"]) == 18
    velocity = schema["actions"]["chassis"]["velocity"]
    assert velocity["command_topic"] == "/thesis/act_cmd_vel"
    assert schema["expected_dimensions"] == {
        "numeric_observation": 24,
        "action": 3,
    }
