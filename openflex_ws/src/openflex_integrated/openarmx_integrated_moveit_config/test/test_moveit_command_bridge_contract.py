import importlib.util
from pathlib import Path


def _load_bridge_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "moveit_command_bridge.py"
    spec = importlib.util.spec_from_file_location("moveit_command_bridge_contract", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_executor_keeps_control_plane_workers_during_three_domain_motion():
    bridge = _load_bridge_module()

    assert bridge.BRIDGE_EXECUTOR_THREADS >= len(bridge.TRAJECTORY_DOMAINS) + 2


def test_gripper_feedback_rejects_wrapped_or_out_of_range_positions():
    bridge = _load_bridge_module()

    assert bridge.gripper_position_is_valid(0.0)
    assert bridge.gripper_position_is_valid(0.044)
    assert not bridge.gripper_position_is_valid(-0.5271)
    assert not bridge.gripper_position_is_valid(0.10)


def test_trajectory_limit_spillover_is_accepted_and_clamped():
    bridge = _load_bridge_module()
    joint = "openarmx_right_joint4"

    assert bridge.trajectory_position_is_valid(joint, 1.80048)
    assert bridge.clamp_trajectory_position(joint, 1.80048) == 1.8
    assert not bridge.trajectory_position_is_valid(joint, 1.802)
