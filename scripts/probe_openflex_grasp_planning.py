#!/usr/bin/env python3
from copy import deepcopy
from datetime import datetime
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from openflex_thesis_msgs.action import ComputeTcWam, GenerateGrasps
from openflex_thesis_msgs.msg import AffordanceCandidateArray, ObjectMemory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from check_openflex_manipulation_safety import SafetyProbe


class PlanningProbe(SafetyProbe):
    def __init__(self):
        super().__init__()
        self.subscriptions_keepalive.extend([
            self.create_subscription(ObjectMemory, '/thesis/object_memory',
                lambda message: self.samples.__setitem__('memory', (deepcopy(message), time.monotonic())),
                qos_profile_sensor_data),
            self.create_subscription(AffordanceCandidateArray, '/thesis/tc_wam/candidates',
                lambda message: self.samples.__setitem__('candidates', (deepcopy(message), time.monotonic())), 10),
        ])
        self.generate = ActionClient(self, GenerateGrasps, '/thesis/generate_grasps')
        self.compute = ActionClient(self, ComputeTcWam, '/thesis/compute_tc_wam')
        self.action_evidence = []

    def wait_future(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('Planning action response unavailable; no execution authorized')
        return future.result()

    def action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError(f'Planning action unavailable: {client._action_name}')
        handle = self.wait_future(client.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError(f'Planning action rejected: {client._action_name}')
        future = handle.get_result_async()
        try:
            response = self.wait_future(future, timeout)
        except Exception:
            self.wait_future(handle.cancel_goal_async(), 10)
            raise
        self.action_evidence.append({'action': client._action_name, 'status': int(response.status),
                                     'result': message_to_ordereddict(response.result)})
        return response.result, response.status

    def run(self, model_path, search_base=False, refined=False):
        initial = self.probe(model_path)
        if not initial['state_valid']:
            raise RuntimeError('Measured full-robot start state is not collision free')
        deadline = time.monotonic() + 5
        while 'memory' not in self.samples and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        memory = self.fresh('memory')
        if (memory.object_id != 'thesis_item_01' or not memory.visible or not memory.grasp_authorized
                or memory.freshness < 0.5):
            raise RuntimeError('No fresh authorized single-object simulation memory')
        goal = GenerateGrasps.Goal()
        goal.target, goal.stage, goal.active_arm, goal.top_k = memory, (
            GenerateGrasps.Goal.REFINED if refined else GenerateGrasps.Goal.COARSE), 2, 3
        goal.require_physical_perception, goal.timeout_sec = False, 8.0
        generated, status = self.action(self.generate, goal, 15)
        if status != GoalStatus.STATUS_SUCCEEDED or not generated.success or not generated.grasps.grasps:
            raise RuntimeError('Oracle simulation grasp generation failed')
        compute = ComputeTcWam.Goal()
        compute.target, compute.grasps, compute.active_arm = memory, generated.grasps, 2
        compute.evaluate_robustness, compute.require_physical_evaluation = True, False
        compute.timeout_sec, compute.restrict_base = 150.0, not search_base
        compute.requested_base_pose = deepcopy(self.fresh('odometry').pose.pose)
        computed, status = self.action(self.compute, compute, 170)
        post = self.probe(model_path)
        report = {'trajectory_execution': False, 'robot_motion_executed': False,
                  'grasp_execution_authorized': False, 'oracle_perception_not_physical': True,
                  'initial_start_state': initial, 'post_query_start_state': post,
                  'actions': self.action_evidence, 'planning_action_status': int(status),
                  'search_base_without_executing_motion': search_base,
                  'tc_wam_success': bool(computed.success), 'message': computed.message}
        if 'candidates' in self.samples:
            report['published_candidates'] = message_to_ordereddict(self.samples['candidates'][0])
        if computed.success:
            plan = computed.plan
            validations = []
            for phase, positions in [('pregrasp', plan.pregrasp_joint_positions), ('grasp', plan.grasp_joint_positions)]:
                state = RobotState()
                state.joint_state = deepcopy(self.fresh('joints'))
                replacements = dict(zip(plan.joint_names, positions))
                state.joint_state.position = [replacements.get(name, position) for name, position
                                               in zip(state.joint_state.name, state.joint_state.position)]
                odometry = self.fresh('odometry')
                state.multi_dof_joint_state.header.frame_id = 'odom'
                state.multi_dof_joint_state.joint_names = ['base_planar_joint']
                transform = Transform()
                candidate = PoseStamped()
                candidate.header.frame_id = 'map'
                candidate.pose = deepcopy(plan.best_base_pose.pose)
                candidate.pose.position.z = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform.translation.z
                candidate_odom = do_transform_pose_stamped(candidate, self.tf_buffer.lookup_transform('odom', 'map', Time()))
                transform.translation.x, transform.translation.y = candidate_odom.pose.position.x, candidate_odom.pose.position.y
                transform.rotation = deepcopy(candidate_odom.pose.orientation)
                state.multi_dof_joint_state.transforms = [transform]
                request = GetStateValidity.Request()
                request.robot_state, request.group_name = state, ''
                checked = self.call(self.validity, request)
                validations.append({'phase': phase, 'valid': bool(checked.valid),
                                    'request': message_to_ordereddict(request),
                                    'response': message_to_ordereddict(checked)})
            report['independent_all_robot_endpoint_checks'] = validations
            report['endpoints_valid'] = all(check['valid'] for check in validations)
        report['remaining_gates'] = ['continuous path and attached-object clearance',
            'fresh refined grasps', 'exclusive MoveIt execution and cancellation',
            'independent retained-object verification', 'physical-pose release instead of teleport placement']
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--search-base', action='store_true')
    parser.add_argument('--refined', action='store_true')
    arguments = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=83')
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('grasp_planning_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    model_path = ROOT / 'local_dependencies/openflex_roboos/readonly_scene.xml'
    rclpy.init()
    node = PlanningProbe()
    report = {'grasp_execution_authorized': False}
    try:
        report = node.run(model_path, arguments.search_base, arguments.refined)
        report['model_sha256'] = hashlib.sha256(model_path.read_bytes()).hexdigest()
        report['probe_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    except Exception as error:
        report.update(error=str(error), actions=node.action_evidence)
        raise
    finally:
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(output / 'result.json')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
