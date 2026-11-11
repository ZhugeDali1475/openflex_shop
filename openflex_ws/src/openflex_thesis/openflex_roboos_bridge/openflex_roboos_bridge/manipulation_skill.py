from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from openflex_thesis_msgs.msg import ControlMode, GraspStatus
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory


class ManipulationSkill:
    def __init__(self, node):
        if os.environ.get('OPENFLEX_ENABLE_SIM_MANIPULATION') != '1':
            raise RuntimeError('Explicit isolated simulation manipulation opt-in required')
        self.node = node
        self.root = Path(__file__).resolve().parents[5]
        if not (self.root / 'scripts/probe_openflex_grasp_planning.py').is_file():
            raise RuntimeError('Cannot locate reviewed planning probes')
        self.clients = {name: ActionClient(node, FollowJointTrajectory, endpoint, callback_group=node.group)
            for name, endpoint in [('lift', '/lift_joint_controller/follow_joint_trajectory'),
                                   ('arm', '/right_joint_trajectory_controller/follow_joint_trajectory')]}
        self.gripper = ActionClient(node, GripperCommand, '/right_gripper_controller/gripper_cmd', callback_group=node.group)
        self.listener = node.create_subscription(GraspStatus, '/thesis/grasp_status',
            lambda message: node.receive('grasp', message), qos_profile_sensor_data, callback_group=node.group)
        self.mode_listener = node.create_subscription(ControlMode, '/thesis/control_mode',
            lambda message: node.receive('mode', message), QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE), callback_group=node.group)
        self.handles = []
        self.deadline = 0.0
        self.base_lock = None
        self.released_target = None

    def submit(self, arguments):
        if set(arguments) != {'object_id', 'request_id', 'timeout_sec'} or arguments['object_id'] != 'thesis_item_01':
            raise ValueError('Only the verified single simulation object is supported')
        if isinstance(arguments['timeout_sec'], bool) or not isinstance(arguments['timeout_sec'], (int, float)):
            raise ValueError('timeout_sec must be a JSON number')
        timeout = float(arguments['timeout_sec'])
        if not math.isfinite(timeout) or not 1 <= timeout <= 600:
            raise ValueError('timeout_sec must be 1-600')
        arguments = dict(arguments, timeout_sec=timeout)
        record, accepted = self.node.store.begin(arguments['request_id'], 'grasp_object', arguments)
        if accepted:
            threading.Thread(target=self.execute, args=(arguments,), daemon=True).start()
        return record

    def gate(self, attached=False):
        self.node.healthy(allow_attached=attached)
        if self.node.store.cancel_event.is_set():
            raise InterruptedError('Manipulation canceled')
        if time.monotonic() >= self.deadline:
            raise TimeoutError('Manipulation deadline exceeded')
        if self.node.guard is None:
            raise RuntimeError('Missing simulation geometry guard')
        if self.released_target is not None and not self.node.placement.placement_still_verified(self.released_target):
            raise RuntimeError('Released payload moved or lost stable support during retreat; stop immediately')
        if self.base_lock is not None:
            mode = self.node.sample('mode')
            twist = self.node.sample('odometry').twist.twist
            if (mode.mode != ControlMode.MOVEIT or math.hypot(twist.linear.x, twist.linear.y) > 0.02
                    or abs(twist.angular.z) > 0.03 or math.dist(self.node.base_pose()[:2], self.base_lock[:2]) > 0.01):
                raise RuntimeError('MoveIt exclusive mode or measured stationary base lock revoked')

    def probe(self, request_id, script, extra):
        self.gate(attached=True)
        run_path = self.root / 'local_dependencies/openflex_roboos/runs' / request_id
        run_path.mkdir(exist_ok=True)
        log_path = run_path / f'{script}_{time.time_ns()}.log'
        with log_path.open('w') as stream:
            process = subprocess.Popen(['/usr/bin/python3', str(self.root / 'scripts' / script), *extra],
                cwd=self.root, stdout=stream, stderr=subprocess.STDOUT)
            try:
                while process.poll() is None:
                    self.gate(attached=True)
                    time.sleep(0.05)
            except Exception:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                raise
        output_lines = log_path.read_text().strip().splitlines()
        paths = [Path(line) for line in output_lines if line.startswith(str(self.root)) and line.endswith('/result.json')]
        if process.returncode != 0 or len(paths) != 1:
            raise RuntimeError(f'Planning-only probe failed; inspect {log_path}')
        report = json.loads(paths[0].read_text())
        self.node.store.event(request_id, 'planning_probe', {'script': script, 'report': str(paths[0]),
            'blocked_at': report.get('blocked_at'), 'error': report.get('error')})
        return report, paths[0]

    def wait_action(self, client, goal, attached=False):
        self.gate(attached)
        if not client.wait_for_server(timeout_sec=3):
            raise RuntimeError('Reviewed simulation controller unavailable')
        future = client.send_goal_async(goal)
        try:
            handle = self.node.await_future(future, 3)
        except Exception:
            future.add_done_callback(lambda done: done.result().cancel_goal_async()
                if done.result() and done.result().accepted else None)
            raise RuntimeError('Unknown goal acceptance; manual stop audit required')
        if handle is None or not handle.accepted:
            raise RuntimeError('Controller goal rejected')
        result = handle.get_result_async()
        self.handles.append((handle, result))
        while not result.done():
            self.gate(attached)
            time.sleep(0.02)
        response = result.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Controller terminal status {response.status}')
        self.handles.remove((handle, result))
        return response.result

    def joints_stopped(self, timeout=5):
        deadline, since = time.monotonic() + timeout, None
        while time.monotonic() < deadline:
            diagnostic = self.node.sample('diagnostic')
            velocities = diagnostic['joint_state']['velocity_rad_or_m_s']
            if velocities and all(math.isfinite(value) and abs(value) <= 0.03 for value in velocities):
                since = since or time.monotonic()
                if time.monotonic() - since >= 0.5:
                    return True
            else:
                since = None
            time.sleep(0.02)
        return False

    def execute_trajectory(self, request_id, phase, attached):
        trajectory = JointTrajectory()
        set_message_fields(trajectory, phase['planning_response']['trajectory']['joint_trajectory'])
        if not phase.get('trajectory_samples_valid') or not phase.get('mujoco_environment_samples_valid'):
            raise RuntimeError('Both full-robot and exact simulator trajectory checks are required')
        measured = dict(zip(self.node.sample('joints').name, self.node.sample('joints').position))
        for name, expected in zip(trajectory.joint_names, trajectory.points[0].positions):
            if abs(measured[name] - expected) > (0.005 if name == 'lift_joint' else 0.015):
                raise RuntimeError('Robot moved since the verified trajectory start')
        pending = []
        try:
            self.node.store.event(request_id, 'trajectory_start', {'phase': phase['phase'],
                'planned_points': len(trajectory.points), 'attached': attached,
                'duration_sec': trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec / 1e9})
            for controller, names in [('lift', ['lift_joint']), ('arm', [f'openarmx_right_joint{index}' for index in range(1, 8)])]:
                selected = [trajectory.joint_names.index(name) for name in names]
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = names
                for point in trajectory.points:
                    converted = deepcopy(point)
                    converted.positions = [point.positions[index] for index in selected]
                    converted.velocities, converted.accelerations, converted.effort = [], [], []
                    goal.trajectory.points.append(converted)
                client = self.clients[controller]
                if not client.wait_for_server(timeout_sec=3):
                    raise RuntimeError('Paired arm/lift controller unavailable')
                sent = client.send_goal_async(goal)
                try:
                    handle = self.node.await_future(sent, 3)
                except Exception:
                    sent.add_done_callback(lambda done: done.result().cancel_goal_async()
                        if done.result() and done.result().accepted else None)
                    raise RuntimeError('Unknown goal acceptance; manual stop audit required')
                if handle is None or not handle.accepted:
                    raise RuntimeError('Paired arm/lift trajectory rejected')
                result = handle.get_result_async()
                self.handles.append((handle, result))
                pending.append((handle, result))
            while any(not future.done() for _, future in pending):
                self.gate(attached)
                time.sleep(0.02)
            for handle, future in pending:
                response = future.result()
                if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != 0:
                    raise RuntimeError(f'Real trajectory execution did not succeed: {response.status}: {response.result.error_string}')
                self.handles.remove((handle, future))
            if not self.joints_stopped():
                raise RuntimeError('Measured joints did not stop after trajectory')
            self.node.store.event(request_id, 'trajectory_executed', {'phase': phase['phase'],
                'measured_stationary': True, 'controllers': ['lift', 'right_arm']})
        except Exception:
            for handle, future in pending:
                if not future.done():
                    handle.cancel_goal_async()
            raise

    def command_gripper(self, position, attached):
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = position, 20.0
        result = self.wait_action(self.gripper, goal, attached)
        if not (result.reached_goal or result.stalled):
            raise RuntimeError('Gripper result does not confirm the requested action')

    def retained(self):
        status = self.node.sample('grasp')
        diagnostic = self.node.sample('diagnostic')
        return bool(status.object_id == 'thesis_item_01' and status.verified and status.contact_detected
                    and status.gripper_closed and status.object_retained and diagnostic.get('attached_arm') == 'right')

    def execute(self, arguments):
        request_id = arguments['request_id']
        self.deadline = time.monotonic() + float(arguments['timeout_sec'])
        status, success, detail, stopped, uncertain = 'failed', False, '', False, False
        try:
            self.gate()
            self.node.set_mode(ControlMode.HOLD)
            if not self.node.stopped() or not self.joints_stopped():
                raise RuntimeError('Initial measured base and joints must be stationary')
            planned, source = self.probe(request_id, 'probe_openflex_grasp_planning.py', ['--search-base', '--refined'])
            if not planned.get('tc_wam_success') or not planned.get('endpoints_valid'):
                raise RuntimeError('No independently validated base/grasp candidate')
            checked, _ = self.probe(request_id, 'probe_openflex_grasp_trajectory.py', [str(source)])
            if not checked.get('attached_transport_path_valid'):
                raise RuntimeError(f'Whole pickup path rejected: {checked.get("blocked_at")}')
            candidate = planned['actions'][1]['result']['plan']['best_base_pose']['pose']
            rotation = candidate['orientation']
            target = [candidate['position']['x'], candidate['position']['y'],
                      math.atan2(2 * rotation['w'] * rotation['z'], 1 - 2 * rotation['z'] ** 2)]
            navigation = self.node.execute({'request_id': request_id, 'timeout_sec': min(90, self.deadline - time.monotonic())},
                                           target_override=target, finalize=False)
            if not navigation['success']:
                if navigation['status'] == 'canceled':
                    raise InterruptedError('Grasp approach navigation canceled')
                if navigation['status'] == 'timed_out':
                    raise TimeoutError('Grasp approach navigation timed out')
                if navigation['status'] == 'unknown':
                    raise RuntimeError('Unknown goal acceptance or navigation terminal state')
                raise RuntimeError(f'Grasp approach navigation failed: {navigation}')
            self.node.set_mode(ControlMode.MOVEIT)
            self.base_lock = self.node.base_pose()
            self.command_gripper(0.044, False)
            planned, source = self.probe(request_id, 'probe_openflex_grasp_planning.py', ['--refined'])
            if not planned.get('tc_wam_success') or not planned.get('endpoints_valid'):
                raise RuntimeError('Fresh refined plan at the measured aligned base is not feasible')
            checked, _ = self.probe(request_id, 'probe_openflex_grasp_trajectory.py', [str(source)])
            if not checked.get('attached_transport_path_valid'):
                raise RuntimeError(f'Fresh whole pickup path rejected: {checked.get("blocked_at")}')
            for phase in checked['phases'][:2]:
                self.execute_trajectory(request_id, phase, False)
            self.command_gripper(0.0, True)
            deadline = min(self.deadline, time.monotonic() + 8)
            while not self.retained() and time.monotonic() < deadline:
                self.gate(attached=True)
                time.sleep(0.02)
            if not self.retained():
                raise RuntimeError('Independent jaw-bracketed retained-object verification failed')
            self.node.store.event(request_id, 'retained_object_verified', {'provenance': 'MuJoCo jaw geometry proxy, not physical force sensing'})
            transport, _ = self.probe(request_id, 'probe_openflex_grasp_trajectory.py', [str(source), '--held-transport'])
            if not transport.get('attached_transport_path_valid'):
                raise RuntimeError(f'Actual attached-object escape rejected: {transport.get("blocked_at")}')
            for phase in transport['phases']:
                self.execute_trajectory(request_id, phase, True)
            if not self.retained():
                raise RuntimeError('Object not retained after actual transport posture execution')
            self.gate(attached=True)
            status, success = 'succeeded', True
        except InterruptedError as error:
            status, detail = 'canceled', str(error)
        except TimeoutError as error:
            status, detail = 'timed_out', str(error)
        except Exception as error:
            detail = str(error)
            uncertain = 'Unknown goal acceptance' in detail
        finally:
            try:
                for handle, future in self.handles:
                    handle.cancel_goal_async()
                    self.node.await_future(future, 5)
                self.node.set_mode(ControlMode.HOLD)
                stopped = self.node.stopped() and self.joints_stopped()
                if not stopped or uncertain:
                    status, success = 'unknown', False
            except Exception as error:
                status, success = 'unknown', False
                detail += f'; terminal stop unknown: {error}'
            self.handles.clear()
            self.base_lock = None
            self.node.store.finish(request_id, status, success=success and stopped, detail=detail,
                stopped_confirmed=stopped, physical_verification=False,
                grasp_verified=success and stopped, transport_safe=success and stopped,
                backend='existing_MuJoCo_controllers_MoveIt_planning_TC_WAM_Oracle_NOT_PHYSICAL')
