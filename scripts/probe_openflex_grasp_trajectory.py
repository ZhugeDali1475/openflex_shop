#!/usr/bin/env python3
import argparse
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import mujoco

from geometry_msgs.msg import PoseStamped, Transform
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
import rclpy
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose_stamped

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_control'))
sys.path.insert(0, str(ROOT / 'scripts'))
from check_openflex_manipulation_safety import SafetyProbe
from openflex_motion_profile import scaling_for_phase, selected_profile
from openflex_roboos_bridge.navigation_guard import NavigationGuard
from openflex_roboos_bridge.trajectory_guard import sample_joint_path
from openflex_thesis_control.attached_object import relative_pose
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco'))
from openflex_thesis_mujoco.math_utils import compose_pose


class TrajectoryProbe(SafetyProbe):
    def run(self, source, model_path, held_transport=False):
        evidence = json.loads(source.read_text())
        if (evidence.get('model_sha256') != hashlib.sha256(model_path.read_bytes()).hexdigest()
                or not evidence.get('tc_wam_success') or not evidence.get('endpoints_valid')):
            raise RuntimeError('A matching successful planning-only candidate with independent endpoint checks is required')
        model_guard = NavigationGuard(model_path)
        initial = self.probe(model_path, allow_attached=held_transport)
        if not initial['state_valid']:
            raise RuntimeError('Measured robot start state is invalid')
        plan = evidence['actions'][1]['result']['plan']
        candidate = PoseStamped()
        set_message_fields(candidate, plan['best_base_pose'])
        orientation = candidate.pose.orientation
        yaw = math.atan2(2 * orientation.w * orientation.z, 1 - 2 * orientation.z ** 2)
        odometry = self.fresh('odometry')
        current_orientation = odometry.pose.pose.orientation
        current_yaw = math.atan2(2 * current_orientation.w * current_orientation.z, 1 - 2 * current_orientation.z ** 2)
        navigation = model_guard.check(self.fresh('joints'),
            [odometry.pose.pose.position.x, odometry.pose.pose.position.y, current_yaw],
            [candidate.pose.position.x, candidate.pose.position.y, yaw])
        for sample_index in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)
        state = RobotState()
        state.joint_state = deepcopy(self.fresh('joints'))
        candidate.pose.position.z = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform.translation.z
        candidate_odom = do_transform_pose_stamped(candidate, self.tf_buffer.lookup_transform('odom', 'map', Time()))
        transform = Transform()
        transform.translation.x, transform.translation.y = candidate_odom.pose.position.x, candidate_odom.pose.position.y
        transform.rotation = deepcopy(candidate_odom.pose.orientation)
        state.multi_dof_joint_state.header.frame_id = 'odom'
        state.multi_dof_joint_state.joint_names = ['base_planar_joint']
        state.multi_dof_joint_state.transforms = [transform]
        check = GetStateValidity.Request()
        check.robot_state, check.group_name = state, ''
        start_checked = self.call(self.validity, check)
        report = {'trajectory_execution': False, 'robot_motion_executed': False,
                  'grasp_execution_authorized': False, 'candidate_source': str(source),
                  'candidate_source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                  'model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
                  'measured_start_state': initial, 'candidate_base': plan['best_base_pose'],
                  'empty_approach_guard': navigation, 'hypothetical_candidate_start': message_to_ordereddict(start_checked),
                  'phases': [], 'motion_profile': selected_profile()}
        report['held_transport_only'] = held_transport
        if held_transport and initial['simulator_diagnostic'].get('attached_arm') != 'right':
            raise RuntimeError('Fresh measured right-hand attachment required for held transport planning')
        object_pose = initial['simulator_diagnostic']['object_pose']
        object_joint = mujoco.mj_name2id(model_guard.model, mujoco.mjtObj.mjOBJ_JOINT, 'thesis_object_free')
        object_address = model_guard.model.jnt_qposadr[object_joint]
        model_guard.data.qpos[object_address:object_address + 3] = object_pose['position_m']
        quaternion = object_pose['orientation_xyzw']
        model_guard.data.qpos[object_address + 3:object_address + 7] = [quaternion[3], *quaternion[:3]]
        if not navigation['accepted'] or not start_checked.valid:
            report['blocked_at'] = 'candidate_base_approach_or_start'
            return report
        def simulator_contacts(phase):
            contacts = []
            mujoco.mj_forward(model_guard.model, model_guard.data)
            for contact in model_guard.data.contact[:model_guard.data.ncon]:
                first, second = (int(model_guard.model.geom_bodyid[index])
                                 for index in (contact.geom1, contact.geom2))
                forbidden = ((model_guard.model.body_rootid[first] == model_guard.robot and second in model_guard.environment)
                    or (model_guard.model.body_rootid[second] == model_guard.robot and first in model_guard.environment))
                for robot_body, obstacle in [(first, second), (second, first)]:
                    if model_guard.model.body_rootid[robot_body] != model_guard.robot:
                        continue
                    robot_name = mujoco.mj_id2name(model_guard.model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
                    obstacle_name = mujoco.mj_id2name(model_guard.model, mujoco.mjtObj.mjOBJ_BODY, obstacle)
                    if obstacle == 0 and not robot_name.endswith('_wheel_link'):
                        forbidden = True
                    touch_links = {'openarmx_right_hand', 'openarmx_right_left_finger', 'openarmx_right_right_finger'}
                    if obstacle_name == 'thesis_object' and (phase not in ('grasp', 'transport') or robot_name not in touch_links):
                        forbidden = True
                object_body = mujoco.mj_name2id(model_guard.model, mujoco.mjtObj.mjOBJ_BODY, 'thesis_object')
                if phase == 'transport' and ((first == object_body and second in model_guard.environment)
                        or (second == object_body and first in model_guard.environment)
                        or (first == object_body and second == 0) or (second == object_body and first == 0)):
                    forbidden = True
                if forbidden and contact.dist <= 0:
                    contacts.append({'bodies': [mujoco.mj_id2name(model_guard.model, mujoco.mjtObj.mjOBJ_BODY, body)
                                               for body in (first, second)], 'distance_m': float(contact.dist)})
            return contacts

        planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        phases = [] if held_transport else [('pregrasp', plan['pregrasp_joint_positions']), ('grasp', plan['grasp_joint_positions'])]
        for phase, positions in phases:
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = plan['planning_group']
            request.motion_plan_request.start_state = deepcopy(state)
            request.motion_plan_request.allowed_planning_time = 8.0
            request.motion_plan_request.num_planning_attempts = 3
            velocity, acceleration = scaling_for_phase(phase)
            request.motion_plan_request.max_velocity_scaling_factor = velocity
            request.motion_plan_request.max_acceleration_scaling_factor = acceleration
            constraints = Constraints()
            for name, position in zip(plan['joint_names'], positions):
                constraint = JointConstraint()
                constraint.joint_name, constraint.position = name, position
                constraint.tolerance_above = constraint.tolerance_below = 0.001
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
            request.motion_plan_request.goal_constraints = [constraints]
            response = self.call(planner, request).motion_plan_response
            phase_report = {'phase': phase, 'request': message_to_ordereddict(request),
                            'planning_response': message_to_ordereddict(response),
                            'trajectory_samples_valid': False, 'mujoco_environment_samples_valid': False}
            report['phases'].append(phase_report)
            if response.error_code.val != 1 or not response.trajectory.joint_trajectory.points:
                report['blocked_at'] = phase + '_planning'
                return report
            trajectory = response.trajectory.joint_trajectory
            current_positions = dict(zip(state.joint_state.name, state.joint_state.position))
            samples = sample_joint_path(trajectory.joint_names,
                [[current_positions[name] for name in trajectory.joint_names]] +
                [point.positions for point in trajectory.points])
            for index, sample in enumerate(samples):
                for name, position in zip(trajectory.joint_names, sample):
                    model_guard.data.qpos[model_guard.offset(name)] = position
                contacts = simulator_contacts(phase)
                if contacts:
                    phase_report.update(rejected_sample=index, mujoco_contacts=contacts)
                    report['blocked_at'] = phase + '_mujoco_robot_environment_collision'
                    return report
                replacements = dict(zip(trajectory.joint_names, sample))
                state.joint_state.position = [replacements.get(name, position) for name, position
                    in zip(state.joint_state.name, state.joint_state.position)]
                check.robot_state = state
                checked = self.call(self.validity, check)
                if not checked.valid:
                    phase_report.update(rejected_sample=index, validity=message_to_ordereddict(checked))
                    report['blocked_at'] = phase + '_full_robot_trajectory_collision'
                    return report
            phase_report.update(trajectory_samples_valid=True, mujoco_environment_samples_valid=True, samples=len(samples))
        report['robot_only_paths_valid'] = not held_transport
        forward = self.create_client(GetPositionFK, '/compute_fk')
        request = GetPositionFK.Request()
        request.header.frame_id = 'map'
        request.fk_link_names = [plan['ik_link']]
        request.robot_state = state
        response = self.call(forward, request)
        if response.error_code.val != 1 or len(response.pose_stamped) != 1:
            raise RuntimeError('Cannot verify the planned attachment transform with real robot FK')
        tcp_pose = response.pose_stamped[0].pose
        offset, rotation = relative_pose(
            [tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z],
            [tcp_pose.orientation.x, tcp_pose.orientation.y, tcp_pose.orientation.z, tcp_pose.orientation.w],
            object_pose['position_m'], object_pose['orientation_xyzw'])
        attached = AttachedCollisionObject()
        attached.link_name = plan['ik_link']
        attached.touch_links = ['openarmx_right_hand', 'openarmx_right_left_finger', 'openarmx_right_right_finger']
        attached.object.header.frame_id = plan['ik_link']
        attached.object.id, attached.object.operation = 'planning_only_target_box', CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type, primitive.dimensions = SolidPrimitive.BOX, [0.07, 0.06, 0.16]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = offset
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
        attached.object.primitives, attached.object.primitive_poses = [primitive], [pose]
        state.attached_collision_objects = [attached]
        attached_start = deepcopy(state)
        attached_checks = []
        for phase in ['attached_grasp_start', 'legacy_transport_posture']:
            if phase == 'legacy_transport_posture':
                replacements = dict(zip(plan['joint_names'], [-0.10, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0]))
                state.joint_state.position = [replacements.get(name, position) for name, position
                    in zip(state.joint_state.name, state.joint_state.position)]
            check.robot_state = state
            checked = self.call(self.validity, check)
            attached_checks.append({'phase': phase, 'valid': bool(checked.valid),
                                    'request': message_to_ordereddict(check), 'response': message_to_ordereddict(checked)})
        report['attached_object_checks'] = attached_checks
        report['attached_endpoints_valid'] = all(check['valid'] for check in attached_checks)
        if not report['attached_endpoints_valid']:
            report['blocked_at'] = 'attached_object_full_robot_collision'
            return report
        state = deepcopy(attached_start)
        tcp_body = mujoco.mj_name2id(model_guard.model, mujoco.mjtObj.mjOBJ_BODY, plan['ik_link'])
        mujoco.mj_forward(model_guard.model, model_guard.data)
        tcp_quaternion = model_guard.data.xquat[tcp_body]
        attachment_position, attachment_rotation = relative_pose(model_guard.data.xpos[tcp_body],
            [*tcp_quaternion[1:], tcp_quaternion[0]], object_pose['position_m'], object_pose['orientation_xyzw'])

        def audit_attached_trajectory(phase_report, trajectory):
            current_positions = dict(zip(state.joint_state.name, state.joint_state.position))
            samples = sample_joint_path(trajectory.joint_names,
                [[current_positions[name] for name in trajectory.joint_names]] +
                [point.positions for point in trajectory.points])
            for index, sample in enumerate(samples):
                replacements = dict(zip(trajectory.joint_names, sample))
                state.joint_state.position = [replacements.get(name, position) for name, position
                    in zip(state.joint_state.name, state.joint_state.position)]
                check.robot_state = state
                checked = self.call(self.validity, check)
                if not checked.valid:
                    phase_report.update(rejected_sample=index, validity=message_to_ordereddict(checked))
                    report['blocked_at'] = phase_report['phase'] + '_full_robot_trajectory_collision'
                    return False
                for name, position in zip(trajectory.joint_names, sample):
                    model_guard.data.qpos[model_guard.offset(name)] = position
                mujoco.mj_forward(model_guard.model, model_guard.data)
                tcp_quaternion = model_guard.data.xquat[tcp_body]
                carried_position, carried_rotation = compose_pose(model_guard.data.xpos[tcp_body],
                    [*tcp_quaternion[1:], tcp_quaternion[0]], attachment_position, attachment_rotation)
                model_guard.data.qpos[object_address:object_address + 7] = [
                    *carried_position, carried_rotation[3], *carried_rotation[:3]]
                contacts = simulator_contacts('transport')
                if contacts:
                    phase_report.update(rejected_sample=index, mujoco_contacts=contacts)
                    report['blocked_at'] = phase_report['phase'] + '_mujoco_collision'
                    return False
            phase_report.update(trajectory_samples_valid=True, mujoco_environment_samples_valid=True, samples=len(samples))
            return True

        if 'lift_joint' in plan['joint_names']:
            lift_request = GetMotionPlan.Request()
            lift_request.motion_plan_request.group_name = plan['planning_group']
            lift_request.motion_plan_request.start_state = deepcopy(state)
            lift_request.motion_plan_request.allowed_planning_time = 8.0
            lift_request.motion_plan_request.num_planning_attempts = 3
            velocity, acceleration = scaling_for_phase('clearance_lift')
            lift_request.motion_plan_request.max_velocity_scaling_factor = velocity
            lift_request.motion_plan_request.max_acceleration_scaling_factor = acceleration
            lift_constraints = Constraints()
            measured_positions = dict(zip(state.joint_state.name, state.joint_state.position))
            for name in plan['joint_names']:
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = measured_positions[name] + (0.04 if name == 'lift_joint' else 0.0)
                constraint.tolerance_above = constraint.tolerance_below = 0.0005
                constraint.weight = 1.0
                lift_constraints.joint_constraints.append(constraint)
                if name != 'lift_joint':
                    path_constraint = deepcopy(constraint)
                    path_constraint.tolerance_above = path_constraint.tolerance_below = 0.002
                    lift_request.motion_plan_request.path_constraints.joint_constraints.append(path_constraint)
            lift_request.motion_plan_request.goal_constraints = [lift_constraints]
            lift_response = self.call(planner, lift_request).motion_plan_response
            lift_report = {'phase': 'clearance_lift', 'request': message_to_ordereddict(lift_request),
                           'planning_response': message_to_ordereddict(lift_response),
                           'trajectory_samples_valid': False, 'mujoco_environment_samples_valid': False}
            report['phases'].append(lift_report)
            if lift_response.error_code.val != 1 or not lift_response.trajectory.joint_trajectory.points:
                report['blocked_at'] = 'clearance_lift_planning'
                return report
            if not audit_attached_trajectory(lift_report, lift_response.trajectory.joint_trajectory):
                return report
            retreat_fk_request = GetPositionFK.Request()
            retreat_fk_request.header.frame_id = 'map'
            retreat_fk_request.fk_link_names = [plan['ik_link']]
            retreat_fk_request.robot_state = state
            retreat_fk = self.call(forward, retreat_fk_request)
            if retreat_fk.error_code.val != 1 or not retreat_fk.pose_stamped:
                raise RuntimeError('Cannot compute measured lifted TCP for shelf withdrawal')
            ik_client = self.create_client(GetPositionIK, '/compute_ik')
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = plan['planning_group']
            ik_request.ik_request.ik_link_name = plan['ik_link']
            ik_request.ik_request.robot_state = deepcopy(state)
            ik_request.ik_request.pose_stamped = deepcopy(retreat_fk.pose_stamped[0])
            ik_request.ik_request.pose_stamped.pose.position.x -= 0.18
            ik_request.ik_request.avoid_collisions = True
            ik_request.ik_request.timeout.sec = 1
            lifted_positions = dict(zip(state.joint_state.name, state.joint_state.position))
            lift_constraint = JointConstraint()
            lift_constraint.joint_name, lift_constraint.position = 'lift_joint', lifted_positions['lift_joint']
            lift_constraint.tolerance_above = lift_constraint.tolerance_below = 0.025
            lift_constraint.weight = 1.0
            ik_request.ik_request.constraints.joint_constraints = [lift_constraint]
            report['withdrawal_ik_attempts'] = []
            for lift_seed in [lifted_positions['lift_joint'], lifted_positions['lift_joint'] + 0.1,
                              lifted_positions['lift_joint'] - 0.1]:
                seeded_positions = dict(zip(state.joint_state.name, state.joint_state.position))
                seeded_positions['lift_joint'] = lift_seed
                ik_request.ik_request.robot_state.joint_state.position = [seeded_positions[name]
                    for name in state.joint_state.name]
                ik_request.ik_request.constraints.joint_constraints[0].position = lift_seed
                ik_response = self.call(ik_client, ik_request)
                report['withdrawal_ik_attempts'].append({'request': message_to_ordereddict(ik_request),
                                                       'response': message_to_ordereddict(ik_response)})
                if ik_response.error_code.val == 1:
                    break
            if ik_response.error_code.val != 1:
                report['blocked_at'] = 'shelf_withdrawal_ik'
                return report
            retreat_request = GetMotionPlan.Request()
            retreat_request.motion_plan_request.group_name = plan['planning_group']
            retreat_request.motion_plan_request.start_state = deepcopy(state)
            retreat_request.motion_plan_request.allowed_planning_time = 8.0
            retreat_request.motion_plan_request.num_planning_attempts = 3
            velocity, acceleration = scaling_for_phase('shelf_withdrawal')
            retreat_request.motion_plan_request.max_velocity_scaling_factor = velocity
            retreat_request.motion_plan_request.max_acceleration_scaling_factor = acceleration
            retreat_constraints = Constraints()
            retreat_positions = dict(zip(ik_response.solution.joint_state.name, ik_response.solution.joint_state.position))
            for name in plan['joint_names']:
                constraint = JointConstraint()
                constraint.joint_name, constraint.position = name, retreat_positions[name]
                constraint.tolerance_above = constraint.tolerance_below = 0.0005
                constraint.weight = 1.0
                retreat_constraints.joint_constraints.append(constraint)
            retreat_request.motion_plan_request.goal_constraints = [retreat_constraints]
            retreat_response = self.call(planner, retreat_request).motion_plan_response
            retreat_report = {'phase': 'shelf_withdrawal', 'request': message_to_ordereddict(retreat_request),
                              'planning_response': message_to_ordereddict(retreat_response),
                              'trajectory_samples_valid': False, 'mujoco_environment_samples_valid': False}
            report['phases'].append(retreat_report)
            if retreat_response.error_code.val != 1 or not retreat_response.trajectory.joint_trajectory.points:
                report['blocked_at'] = 'shelf_withdrawal_planning'
                return report
            if not audit_attached_trajectory(retreat_report, retreat_response.trajectory.joint_trajectory):
                return report
            object_geometry = mujoco.mj_name2id(model_guard.model, mujoco.mjtObj.mjOBJ_GEOM, 'thesis_object_geom')
            object_rotation = model_guard.data.geom_xmat[object_geometry].reshape(3, 3)
            object_front = model_guard.data.geom_xpos[object_geometry, 0] + sum(
                abs(object_rotation[0, axis]) * model_guard.model.geom_size[object_geometry, axis] for axis in range(3))
            shelf_board = mujoco.mj_name2id(model_guard.model, mujoco.mjtObj.mjOBJ_GEOM, 'thesis_shelf_board_0')
            shelf_front = model_guard.data.geom_xpos[shelf_board, 0] - model_guard.model.geom_size[shelf_board, 0]
            report['withdrawal_clearance_m'] = float(shelf_front - object_front)
            if report['withdrawal_clearance_m'] < 0.005:
                report['blocked_at'] = 'withdrawal_does_not_clear_the_shelf_front'
                return report
        transport_request = GetMotionPlan.Request()
        transport_request.motion_plan_request.group_name = plan['planning_group']
        transport_request.motion_plan_request.start_state = deepcopy(state)
        transport_request.motion_plan_request.allowed_planning_time = 8.0
        transport_request.motion_plan_request.num_planning_attempts = 3
        velocity, acceleration = scaling_for_phase('transport')
        transport_request.motion_plan_request.max_velocity_scaling_factor = velocity
        transport_request.motion_plan_request.max_acceleration_scaling_factor = acceleration
        transport_constraints = Constraints()
        for name, position in zip(plan['joint_names'], [-0.10, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0]):
            constraint = JointConstraint()
            constraint.joint_name, constraint.position = name, position
            constraint.tolerance_above = constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            transport_constraints.joint_constraints.append(constraint)
        transport_request.motion_plan_request.goal_constraints = [transport_constraints]
        transport_response = self.call(planner, transport_request).motion_plan_response
        transport_report = {'phase': 'transport', 'request': message_to_ordereddict(transport_request),
                            'planning_response': message_to_ordereddict(transport_response),
                            'trajectory_samples_valid': False, 'mujoco_environment_samples_valid': False}
        report['phases'].append(transport_report)
        if transport_response.error_code.val != 1 or not transport_response.trajectory.joint_trajectory.points:
            report['blocked_at'] = 'attached_transport_planning'
            return report
        if not audit_attached_trajectory(transport_report, transport_response.trajectory.joint_trajectory):
            return report
        report['attached_transport_path_valid'] = True
        report['remaining_gates'] = ['target-object collision geometry and attachment clearance',
            'fresh refined plan at measured aligned base', 'MuJoCo trajectory contact replay',
            'exclusive execution timeout and cancellation', 'verified release without pose teleportation']
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('candidate_report', type=Path)
    parser.add_argument('--held-transport', action='store_true')
    arguments = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=83')
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('grasp_trajectory_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    rclpy.init()
    node = TrajectoryProbe()
    report = {'grasp_execution_authorized': False}
    try:
        report = node.run(arguments.candidate_report, ROOT / 'local_dependencies/openflex_roboos/readonly_scene.xml', arguments.held_transport)
        report['probe_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(output / 'result.json')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
