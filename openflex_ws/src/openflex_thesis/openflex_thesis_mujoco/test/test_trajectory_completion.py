from threading import RLock
import time
from types import MethodType, SimpleNamespace
from unittest.mock import patch

from control_msgs.action import FollowJointTrajectory
from openflex_thesis_msgs.msg import ControlMode
import mujoco
import pytest
from rclpy.action import GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint

from openflex_thesis_mujoco.bridge_node import MujocoBridgeNode


@pytest.fixture
def bridge():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body><joint name="lift_joint" type="slide"/>
        <geom type="sphere" size=".01" mass="1"/></body></worldbody>
        <actuator><position name="lift_pos" joint="lift_joint" kp="10"/></actuator></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[0] = 0.2
    instance = SimpleNamespace(model=model, data=data, lock=RLock(), safety_stop=None,
        control_mode=ControlMode(mode=ControlMode.MOVEIT, inputs_healthy=True), control_mode_receipt=time.monotonic(),
        actual_twist=[0.0] * 3,
        active_controllers=set(), trajectories={}, trajectory_done={},
        joint_ids={'lift_joint': 0}, actuator_ids={'lift_joint': 0},
        get_parameter=lambda name: SimpleNamespace(value=1.0))
    for name in ['_joint_position', '_trajectory_goal', '_execute_trajectory', '_hold_controller_measured', '_arm_control_valid']:
        setattr(instance, name, MethodType(getattr(MujocoBridgeNode, name), instance))
    return instance


def goal(position=0.3):
    request = FollowJointTrajectory.Goal()
    request.trajectory.joint_names = ['lift_joint']
    request.trajectory.points = [JointTrajectoryPoint(positions=[position])]
    return request


def handle(request, canceled=False):
    history = []
    return SimpleNamespace(request=request, is_cancel_requested=canceled,
        succeed=lambda: history.append('succeeded'), abort=lambda: history.append('aborted'),
        canceled=lambda: history.append('canceled'), history=history)


def test_controller_ownership_is_reserved_before_execution(bridge):
    assert bridge._trajectory_goal(goal(), 'lift') == GoalResponse.ACCEPT
    assert bridge._trajectory_goal(goal(), 'lift') == GoalResponse.REJECT


def test_non_finite_goal_is_rejected_without_owning_controller(bridge):
    assert bridge._trajectory_goal(goal(float('nan')), 'lift') == GoalResponse.REJECT
    assert not bridge.active_controllers


def test_controller_requires_fresh_exclusive_moveit_mode(bridge):
    bridge.control_mode.mode = ControlMode.HOLD
    assert bridge._trajectory_goal(goal(), 'lift') == GoalResponse.REJECT
    bridge.control_mode.mode = ControlMode.MOVEIT
    bridge.control_mode_receipt -= 2
    assert bridge._trajectory_goal(goal(), 'lift') == GoalResponse.REJECT


def test_canceled_trajectory_holds_measured_position_and_releases_mutex(bridge):
    request = goal()
    bridge._trajectory_goal(request, 'lift')
    current = handle(request, canceled=True)
    bridge._execute_trajectory(current, 'lift')
    assert current.history == ['canceled']
    assert bridge.data.ctrl[0] == pytest.approx(0.2)
    assert not bridge.trajectories and not bridge.active_controllers and not bridge.trajectory_done


def test_timeout_holds_measured_position_and_does_not_leave_running_sampler(bridge):
    request = goal()
    bridge._trajectory_goal(request, 'lift')
    current = handle(request)
    with patch('openflex_thesis_mujoco.bridge_node.time.monotonic', side_effect=[0.0, 10.0]):
        result = bridge._execute_trajectory(current, 'lift')
    assert current.history == ['aborted'] and result.error_code != 0
    assert bridge.data.ctrl[0] == pytest.approx(0.2)
    assert not bridge.trajectories and not bridge.active_controllers


def test_sampler_completion_without_measured_arrival_cannot_succeed(bridge):
    request = goal(position=0.4)
    bridge._trajectory_goal(request, 'lift')
    current = handle(request)

    def completed_sampler(delay):
        for token in bridge.trajectory_done:
            bridge.trajectory_done[token] = True

    with patch('openflex_thesis_mujoco.bridge_node.time.monotonic', side_effect=[0.0, 0.0, 1.0, 10.0]), \
            patch('openflex_thesis_mujoco.bridge_node.time.sleep', side_effect=completed_sampler):
        bridge._execute_trajectory(current, 'lift')
    assert current.history == ['aborted']


def test_safety_stop_aborts_and_clears_controller_state(bridge):
    request = goal()
    bridge.active_controllers.add('lift')
    bridge.safety_stop = {'contact': 'arm_shelf'}
    current = handle(request)
    bridge._execute_trajectory(current, 'lift')
    assert current.history == ['aborted']
    assert not bridge.trajectories and not bridge.active_controllers
