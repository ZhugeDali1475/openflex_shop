import pytest

from openflex_grasp.arm_context import arm_context


@pytest.mark.parametrize("arm_name", ["left", "right"])
def test_shelf_approach_axis_is_horizontal_and_forward(arm_name):
    context = arm_context(arm_name)

    assert context.planning_group == f"{arm_name}_arm_with_lift"
    assert context.planning_joint_names == ("lift_joint",) + context.joint_names
    assert context.approach_axis == (1.0, 0.0, 0.0)
    assert context.approach_axis[2] == 0.0
