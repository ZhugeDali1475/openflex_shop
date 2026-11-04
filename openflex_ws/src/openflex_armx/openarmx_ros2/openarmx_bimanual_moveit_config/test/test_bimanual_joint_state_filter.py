from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from sensor_msgs.msg import JointState


SCRIPT = Path(__file__).parents[1] / "scripts" / "bimanual_joint_state_filter.py"
SPEC = spec_from_file_location("bimanual_joint_state_filter", SCRIPT)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_filtered_joint_state_removes_non_bimanual_joints():
    source = JointState()
    source.name = [
        "openarmx_head_yaw_joint",
        "openarmx_right_joint1",
        "lift_joint",
        "openarmx_left_finger_joint1",
    ]
    source.position = [1.0, 2.0, 3.0, 4.0]
    source.velocity = [0.1, 0.2, 0.3, 0.4]

    result = MODULE.filtered_joint_state(source)

    assert result.name == ["openarmx_right_joint1", "openarmx_left_finger_joint1"]
    assert list(result.position) == [2.0, 4.0]
    assert list(result.velocity) == [0.2, 0.4]
    assert list(result.effort) == []


def test_filtered_joint_state_rejects_malformed_arrays():
    source = JointState()
    source.name = ["openarmx_right_joint1", "openarmx_right_joint2"]
    source.position = [1.0]

    try:
        MODULE.filtered_joint_state(source)
    except ValueError as error:
        assert "position has 1 values for 2 names" in str(error)
    else:
        raise AssertionError("malformed JointState should be rejected")
