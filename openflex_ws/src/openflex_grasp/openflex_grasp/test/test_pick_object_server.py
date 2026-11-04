import asyncio
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from openarmx_integrated_moveit_msgs.action import PlanPregrasp
from openflex_grasp.arm_context import arm_context
from openflex_grasp.pick_object_server import PickObjectServer


@pytest.mark.parametrize("arm_name", ["left", "right"])
def test_cartesian_start_state_keeps_optional_lift_joint(arm_name):
    context = arm_context(arm_name)
    planned = JointState()
    planned.name = list(context.planning_joint_names)
    planned.position = [0.1] * len(planned.name)

    result = PickObjectServer._cartesian_start_state(planned, context)

    assert tuple(result.name) == context.planning_joint_names
    assert list(result.position) == pytest.approx([0.1] * 8)


@pytest.mark.parametrize("arm_name", ["left", "right"])
def test_pick_object_pregrasp_requests_arm_with_lift(arm_name):
    context = arm_context(arm_name)
    captured = []
    planned_result = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        message="ok",
        STAGE_A=PlanPregrasp.Result.STAGE_A,
        STAGE_B=PlanPregrasp.Result.STAGE_B,
        selected_stage=PlanPregrasp.Result.STAGE_B,
        arm_goal=JointState(
            name=list(context.planning_joint_names),
            position=[0.1] * 8,
        ),
    )

    class Client:
        @staticmethod
        def wait_for_server(timeout_sec):
            return timeout_sec == pytest.approx(2.0)

        @staticmethod
        async def send_goal_async(goal):
            captured.append(goal)

            class Sent:
                accepted = True

                @staticmethod
                async def get_result_async():
                    return SimpleNamespace(result=planned_result)

            return Sent()

    server = object.__new__(PickObjectServer)
    server._plan = Client()
    server.get_parameter = lambda name: SimpleNamespace(
        value={"velocity_scale": 0.08, "acceleration_scale": 0.08}[name]
    )

    planned = asyncio.run(
        PickObjectServer._plan_pregrasp(
            server,
            PoseStamped(),
            arm_name,
            PlanPregrasp.Goal.ARM_WITH_LIFT,
            False,
        )
    )

    assert planned is planned_result
    assert captured[0].max_dof == PlanPregrasp.Goal.ARM_WITH_LIFT
    assert captured[0].base_yaw_mode == PlanPregrasp.Goal.CURRENT_STATE
