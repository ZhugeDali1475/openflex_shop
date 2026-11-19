from threading import RLock
import time
from types import MethodType, SimpleNamespace

from control_msgs.action import GripperCommand
import mujoco
from openflex_thesis_msgs.msg import ControlMode
import pytest
from rclpy.action import GoalResponse

from openflex_thesis_mujoco.bridge_node import GRIPPER_JOINTS, MujocoBridgeNode


@pytest.fixture
def gripper():
    name = GRIPPER_JOINTS['right']
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody><body><joint name="{name}" type="slide"/>
        <geom type="sphere" size=".01" mass="1"/></body></worldbody>
        <actuator><position name="finger_pos" joint="{name}" kp="10"/></actuator></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[0] = 0.02
    parameters = {'gripper_goal_tolerance_m': 0.003, 'gripper_action_timeout_sec': 4.0}
    instance = SimpleNamespace(model=model, data=data, lock=RLock(), control_mode_lock=RLock(), safety_stop=None, pending_place=None,
        control_mode=ControlMode(mode=ControlMode.MOVEIT, inputs_healthy=True), control_mode_receipt=time.monotonic(),
        actual_twist=[0.0] * 3, active_grippers=set(), attached_arm=None,
        joint_ids={name: 0}, actuator_ids={name: 0}, gripper_targets={'right': 0.04},
        get_parameter=lambda parameter: SimpleNamespace(value=parameters[parameter]))
    for method in ['_joint_position', '_arm_control_valid', '_gripper_goal', '_execute_gripper', '_run_gripper']:
        setattr(instance, method, MethodType(getattr(MujocoBridgeNode, method), instance))
    return instance


def goal(position):
    request = GripperCommand.Goal()
    request.command.position, request.command.max_effort = position, 20.0
    return request


def test_cancel_stops_gripper_at_measured_position_and_releases_ownership(gripper):
    request = goal(0.0)
    assert gripper._gripper_goal(request, 'right') == GoalResponse.ACCEPT
    history = []
    handle = SimpleNamespace(request=request, is_cancel_requested=True,
        canceled=lambda: history.append('canceled'), abort=lambda: history.append('aborted'),
        succeed=lambda: history.append('succeeded'))
    result = gripper._execute_gripper(handle, 'right')
    assert history == ['canceled'] and not result.reached_goal and not result.stalled
    assert gripper.gripper_targets['right'] == pytest.approx(0.02)
    assert gripper.data.ctrl[0] == pytest.approx(0.02)
    assert not gripper.active_grippers


def test_concurrent_gripper_commands_are_rejected(gripper):
    assert gripper._gripper_goal(goal(0.0), 'right') == GoalResponse.ACCEPT
    assert gripper._gripper_goal(goal(0.04), 'right') == GoalResponse.REJECT


@pytest.mark.parametrize('position', [-0.1, 0.1, float('nan'), float('inf')])
def test_invalid_gripper_positions_are_rejected(gripper, position):
    assert gripper._gripper_goal(goal(position), 'right') == GoalResponse.REJECT


def test_gripper_cannot_acquire_control_during_place_or_stale_mode(gripper):
    gripper.control_mode.mode = ControlMode.PLACE
    assert gripper._gripper_goal(goal(0.04), 'right') == GoalResponse.REJECT
    gripper.control_mode.mode = ControlMode.MOVEIT
    gripper.control_mode_receipt -= 2
    assert gripper._gripper_goal(goal(0.04), 'right') == GoalResponse.REJECT
