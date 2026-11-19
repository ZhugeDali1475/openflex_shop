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
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
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

# A closed hand that is driven into the product shoves it out of the jaws
# instead of around it. The approach phases are therefore rejected as soon as
# the measured hand/finger geometry penetrates the target deeper than this.
APPROACH_PENETRATION_LIMIT_M = 0.002

# Folded right-arm carry posture used between the grasp and the delivery
# approach. The measured transport posture (-0.10 on lift_joint) held the
# 0.16 m product at z=0.838 m, i.e. level with the 0.841-0.861 m shelf slab, so
# the carried box swept 42 mm into the shelf while the base backed away on
# 2026-11-13. Lift +0.05 carries the same product at z=0.988 m, which the
# sampled base route and the attached transport path both clear.
CARRY_POSTURE = [0.05, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0]


def shelf_front_for_object(model_guard, object_geom_id):
    object_center = model_guard.data.geom_xpos[object_geom_id]
    object_rotation = model_guard.data.geom_xmat[object_geom_id].reshape(3, 3)
    object_half_size = model_guard.model.geom_size[object_geom_id]
    object_vertical_extent = sum(abs(object_rotation[2, axis]) * object_half_size[axis]
                                 for axis in range(3))
    object_bottom = float(object_center[2]) - object_vertical_extent
    candidates = []
    for geom_id in range(model_guard.model.ngeom):
        name = mujoco.mj_id2name(model_guard.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if not (name.startswith("shelf_group_") and "_shelf_L" in name and name.endswith("_body")):
            continue
        if model_guard.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        center = model_guard.data.geom_xpos[geom_id]
        shelf_rotation = model_guard.data.geom_xmat[geom_id].reshape(3, 3)
        if (abs(float(shelf_rotation[0, 0]) - 1.0) > 1e-5
                or abs(float(shelf_rotation[1, 1]) - 1.0) > 1e-5
                or abs(float(shelf_rotation[2, 2]) - 1.0) > 1e-5):
            raise RuntimeError("Rotated shelf support geometry needs explicit clearance handling")
        shelf_half_size = model_guard.model.geom_size[geom_id]
        surface_z = float(center[2]) + float(shelf_half_size[2])
        vertical_error = abs(surface_z - object_bottom)
        if vertical_error > 0.03:
            continue
        if name.endswith("_neg_body"):
            normal_x = -1.0
        elif name.endswith("_pos_body"):
            normal_x = 1.0
        else:
            raise RuntimeError(f"Shelf support has no recognized front side: {name}")
        boundary = float(center[0]) + normal_x * float(shelf_half_size[0])
        candidates.append((vertical_error, abs(float(center[0]) - float(object_center[0])),
                           boundary, name, normal_x))
    if not candidates:
        raise RuntimeError("No shelf support geometry matches the measured object height")
    selected = min(candidates)
    return selected[2:]


def shelf_withdrawal_clearance(model_guard, object_geom_id, shelf_boundary, shelf_normal_x):
    object_center = model_guard.data.geom_xpos[object_geom_id]
    object_rotation = model_guard.data.geom_xmat[object_geom_id].reshape(3, 3)
    object_half_size = model_guard.model.geom_size[object_geom_id]
    front_extent = sum(abs(float(object_rotation[0, axis])) * float(object_half_size[axis])
                       for axis in range(3))
    object_front = float(object_center[0]) + shelf_normal_x * front_extent
    return shelf_normal_x * (object_front - shelf_boundary)


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
        # Shelf reference geometry is static: derive it once with the object on
        # the shelf. A later clearance check must not re-match shelf supports
        # after the clearance lift has raised the object.
        object_geometry = mujoco.mj_name2id(model_guard.model, mujoco.mjtObj.mjOBJ_GEOM, 'thesis_object_geom')
        mujoco.mj_forward(model_guard.model, model_guard.data)
        shelf_front, shelf_geom_name, shelf_normal_x = shelf_front_for_object(model_guard, object_geometry)
        report['withdrawal_reference_shelf_geom'] = shelf_geom_name
        report['withdrawal_direction_world_x'] = shelf_normal_x
        report['required_withdrawal_m'] = max(0.005 - shelf_withdrawal_clearance(
            model_guard, object_geometry, shelf_front, shelf_normal_x), 0.0)
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
                    if (obstacle_name == 'thesis_object' and robot_name in touch_links
                            and phase in ('pregrasp', 'grasp')
                            and contact.dist < -APPROACH_PENETRATION_LIMIT_M):
                        # Touching the product on the way in is unavoidable once
                        # the jaws are around it, but a measured penetration
                        # means the approach is shoving it deeper into the shelf
                        # and it will be outside the jaws when they finally close.
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
        # The last 12 cm of the approach only have ~1.4 cm of jaw clearance on
        # each side of the 6 cm product, while a joint-space plan swings the
        # TCP about 1.9 cm sideways on the way in (measured on the 2026-11-13
        # failing run) and clips the product. The grasp phase is therefore a
        # Cartesian straight line through the reviewed TCP pose; only the
        # free-space pregrasp phase stays a joint-space plan.
        cartesian_planner = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        forward = self.create_client(GetPositionFK, '/compute_fk')
        planning_attempts = int(os.environ.get('OPENFLEX_TRAJECTORY_PLANNING_ATTEMPTS', '5'))
        if not 1 <= planning_attempts <= 20:
            raise RuntimeError('OPENFLEX_TRAJECTORY_PLANNING_ATTEMPTS must be an integer in 1-20')

        def fk_tcp_pose(joint_positions):
            request = GetPositionFK.Request()
            request.header.frame_id = 'map'
            request.fk_link_names = [plan['ik_link']]
            fk_state = RobotState()
            fk_state.joint_state.name = list(state.joint_state.name)
            overrides = dict(zip(plan['joint_names'], joint_positions))
            fk_state.joint_state.position = [overrides.get(name, position) for name, position
                in zip(fk_state.joint_state.name, state.joint_state.position)]
            fk_state.multi_dof_joint_state = deepcopy(state.multi_dof_joint_state)
            request.robot_state = fk_state
            response = self.call(forward, request)
            if response.error_code.val != 1 or not response.pose_stamped:
                raise RuntimeError('Cannot compute the planned TCP pose for the Cartesian grasp approach')
            return response.pose_stamped[0]

        grasp_waypoint = None if held_transport else fk_tcp_pose(plan['grasp_joint_positions'])

        def build_cartesian_grasp_request():
            request = GetCartesianPath.Request()
            request.header.frame_id = 'map'
            request.start_state = deepcopy(state)
            request.group_name = plan['planning_group']
            request.link_name = plan['ik_link']
            request.waypoints = [deepcopy(grasp_waypoint.pose)]
            request.max_step = 0.005
            request.jump_threshold = 0.0
            request.avoid_collisions = True
            (request.max_velocity_scaling_factor,
             request.max_acceleration_scaling_factor) = scaling_for_phase('grasp')
            return request

        def cartesian_as_motion_plan(response):
            # Every downstream check and the skill executor consume the
            # GetMotionPlan shape, so the Cartesian solution is wrapped in it
            # and only reported successful when the whole path was reached.
            motion = GetMotionPlan.Response().motion_plan_response
            motion.trajectory = response.solution
            motion.error_code.val = (1 if response.error_code.val == 1
                                     and response.fraction >= 1.0 - 1e-6 else -1)
            return motion

        def plan_and_verify(phase, build_request, verify, planning_blocked_at):
            # Joint-space planning is stochastic, so a returned path can still
            # clip the fixture even when its goal configuration is verified.
            # Replan within a bounded budget, keep every attempt as evidence,
            # and accept the phase only when one attempt passes every check.
            restored = list(state.joint_state.position)
            attempts = []
            entry = None
            for attempt in range(1, planning_attempts + 1):
                state.joint_state.position = list(restored)
                request = build_request()
                response = self.call(cartesian_planner if phase == 'grasp' else planner, request)
                response = (cartesian_as_motion_plan(response) if phase == 'grasp'
                            else response.motion_plan_response)
                entry = {'phase': phase, 'attempt': attempt,
                         'request': message_to_ordereddict(request),
                         'planning_response': message_to_ordereddict(response),
                         'trajectory_samples_valid': False, 'mujoco_environment_samples_valid': False}
                summary = {'attempt': attempt}
                if response.error_code.val != 1 or not response.trajectory.joint_trajectory.points:
                    summary.update(result='planning_failed', rejection=planning_blocked_at)
                    attempts.append(summary)
                    continue
                rejection = verify(entry, response.trajectory.joint_trajectory)
                if rejection is None:
                    summary['result'] = 'accepted'
                    attempts.append(summary)
                    entry.update(trajectory_samples_valid=True, mujoco_environment_samples_valid=True,
                                 accepted_attempt=attempt, attempts=attempts)
                    return entry
                summary.update(result='rejected', rejection=rejection,
                               rejected_sample=entry.get('rejected_sample'))
                attempts.append(summary)
                state.joint_state.position = list(restored)
            entry.update(accepted_attempt=None, attempts=attempts, planning_attempts=planning_attempts,
                         rejection=attempts[-1]['rejection'])
            return entry

        def verify_robot_path(phase):
            def verify(entry, trajectory):
                current_positions = dict(zip(state.joint_state.name, state.joint_state.position))
                samples = sample_joint_path(trajectory.joint_names,
                    [[current_positions[name] for name in trajectory.joint_names]] +
                    [point.positions for point in trajectory.points])
                for index, sample in enumerate(samples):
                    for name, position in zip(trajectory.joint_names, sample):
                        model_guard.data.qpos[model_guard.offset(name)] = position
                    contacts = simulator_contacts(phase)
                    if contacts:
                        entry.update(rejected_sample=index, mujoco_contacts=contacts)
                        return phase + '_mujoco_robot_environment_collision'
                    replacements = dict(zip(trajectory.joint_names, sample))
                    state.joint_state.position = [replacements.get(name, position) for name, position
                        in zip(state.joint_state.name, state.joint_state.position)]
                    check.robot_state = state
                    checked = self.call(self.validity, check)
                    if not checked.valid:
                        entry.update(rejected_sample=index, validity=message_to_ordereddict(checked))
                        return phase + '_full_robot_trajectory_collision'
                entry['samples'] = len(samples)
                return None
            return verify

        phases = [] if held_transport else [('pregrasp', plan['pregrasp_joint_positions']), ('grasp', plan['grasp_joint_positions'])]
        for phase, positions in phases:
            def build_joint_request(phase=phase, positions=positions):
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
                return request
            build_request = (build_cartesian_grasp_request if phase == 'grasp'
                             else build_joint_request)
            phase_report = plan_and_verify(phase, build_request, verify_robot_path(phase), phase + '_planning')
            report['phases'].append(phase_report)
            if phase_report.get('accepted_attempt') is None:
                report['blocked_at'] = phase_report['rejection']
                return report
        report['robot_only_paths_valid'] = not held_transport
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
                replacements = dict(zip(plan['joint_names'], CARRY_POSTURE))
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

        def verify_attached_path(entry, trajectory):
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
                    entry.update(rejected_sample=index, validity=message_to_ordereddict(checked))
                    return entry['phase'] + '_full_robot_trajectory_collision'
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
                    entry.update(rejected_sample=index, mujoco_contacts=contacts)
                    return entry['phase'] + '_mujoco_collision'
            entry['samples'] = len(samples)
            return None

        if 'lift_joint' in plan['joint_names']:
            measured_positions = dict(zip(state.joint_state.name, state.joint_state.position))

            def build_lift_request():
                lift_request = GetMotionPlan.Request()
                lift_request.motion_plan_request.group_name = plan['planning_group']
                lift_request.motion_plan_request.start_state = deepcopy(state)
                lift_request.motion_plan_request.allowed_planning_time = 8.0
                lift_request.motion_plan_request.num_planning_attempts = 3
                velocity, acceleration = scaling_for_phase('clearance_lift')
                lift_request.motion_plan_request.max_velocity_scaling_factor = velocity
                lift_request.motion_plan_request.max_acceleration_scaling_factor = acceleration
                lift_constraints = Constraints()
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
                return lift_request

            lift_report = plan_and_verify('clearance_lift', build_lift_request, verify_attached_path,
                                          'clearance_lift_planning')
            report['phases'].append(lift_report)
            if lift_report.get('accepted_attempt') is None:
                report['blocked_at'] = lift_report['rejection']
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
            ik_request.ik_request.avoid_collisions = True
            ik_request.ik_request.timeout.sec = 1
            lifted_positions = dict(zip(state.joint_state.name, state.joint_state.position))
            lift_constraint = JointConstraint()
            lift_constraint.joint_name, lift_constraint.position = 'lift_joint', lifted_positions['lift_joint']
            lift_constraint.tolerance_above = lift_constraint.tolerance_below = 0.025
            lift_constraint.weight = 1.0
            ik_request.ik_request.constraints.joint_constraints = [lift_constraint]
            # Prefer the reviewed 0.18 m withdrawal but fall back to the shortest
            # reachable distance that still clears the shelf by the required margin.
            candidate_withdrawals = []
            distance = 0.18
            while distance >= report['required_withdrawal_m'] - 1e-9 and distance > 0.0:
                candidate_withdrawals.append(round(distance, 4))
                distance -= 0.01
            report['withdrawal_ik_attempts'] = []
            ik_response = None
            withdrawal_distance = None
            for withdrawal in candidate_withdrawals:
                ik_request.ik_request.pose_stamped = deepcopy(retreat_fk.pose_stamped[0])
                ik_request.ik_request.pose_stamped.pose.position.x -= withdrawal
                for lift_seed in [lifted_positions['lift_joint'], lifted_positions['lift_joint'] + 0.1,
                                  lifted_positions['lift_joint'] - 0.1]:
                    seeded_positions = dict(zip(state.joint_state.name, state.joint_state.position))
                    seeded_positions['lift_joint'] = lift_seed
                    ik_request.ik_request.robot_state.joint_state.position = [seeded_positions[name]
                        for name in state.joint_state.name]
                    ik_request.ik_request.constraints.joint_constraints[0].position = lift_seed
                    ik_response = self.call(ik_client, ik_request)
                    report['withdrawal_ik_attempts'].append({'withdrawal_m': withdrawal,
                        'request': message_to_ordereddict(ik_request),
                        'response': message_to_ordereddict(ik_response)})
                    if ik_response.error_code.val == 1:
                        break
                if ik_response.error_code.val == 1:
                    withdrawal_distance = withdrawal
                    break
            if ik_response is None or ik_response.error_code.val != 1:
                report['blocked_at'] = 'shelf_withdrawal_ik'
                return report
            report['withdrawal_m'] = withdrawal_distance
            retreat_positions = dict(zip(ik_response.solution.joint_state.name,
                                        ik_response.solution.joint_state.position))

            def build_retreat_request():
                retreat_request = GetMotionPlan.Request()
                retreat_request.motion_plan_request.group_name = plan['planning_group']
                retreat_request.motion_plan_request.start_state = deepcopy(state)
                retreat_request.motion_plan_request.allowed_planning_time = 8.0
                retreat_request.motion_plan_request.num_planning_attempts = 3
                velocity, acceleration = scaling_for_phase('shelf_withdrawal')
                retreat_request.motion_plan_request.max_velocity_scaling_factor = velocity
                retreat_request.motion_plan_request.max_acceleration_scaling_factor = acceleration
                retreat_constraints = Constraints()
                for name in plan['joint_names']:
                    constraint = JointConstraint()
                    constraint.joint_name, constraint.position = name, retreat_positions[name]
                    constraint.tolerance_above = constraint.tolerance_below = 0.0005
                    constraint.weight = 1.0
                    retreat_constraints.joint_constraints.append(constraint)
                retreat_request.motion_plan_request.goal_constraints = [retreat_constraints]
                return retreat_request

            retreat_report = plan_and_verify('shelf_withdrawal', build_retreat_request, verify_attached_path,
                                             'shelf_withdrawal_planning')
            report['phases'].append(retreat_report)
            if retreat_report.get('accepted_attempt') is None:
                report['blocked_at'] = retreat_report['rejection']
                return report
            report['withdrawal_clearance_m'] = shelf_withdrawal_clearance(
                model_guard, object_geometry, shelf_front, shelf_normal_x)
            if report['withdrawal_clearance_m'] < 0.005:
                report['blocked_at'] = 'withdrawal_does_not_clear_the_shelf_front'
                return report
        def build_transport_request():
            transport_request = GetMotionPlan.Request()
            transport_request.motion_plan_request.group_name = plan['planning_group']
            transport_request.motion_plan_request.start_state = deepcopy(state)
            transport_request.motion_plan_request.allowed_planning_time = 8.0
            transport_request.motion_plan_request.num_planning_attempts = 3
            velocity, acceleration = scaling_for_phase('transport')
            transport_request.motion_plan_request.max_velocity_scaling_factor = velocity
            transport_request.motion_plan_request.max_acceleration_scaling_factor = acceleration
            transport_constraints = Constraints()
            for name, position in zip(plan['joint_names'], CARRY_POSTURE):
                constraint = JointConstraint()
                constraint.joint_name, constraint.position = name, position
                constraint.tolerance_above = constraint.tolerance_below = 0.001
                constraint.weight = 1.0
                transport_constraints.joint_constraints.append(constraint)
            transport_request.motion_plan_request.goal_constraints = [transport_constraints]
            return transport_request

        transport_report = plan_and_verify('transport', build_transport_request, verify_attached_path,
                                           'attached_transport_planning')
        report['phases'].append(transport_report)
        if transport_report.get('accepted_attempt') is None:
            report['blocked_at'] = transport_report['rejection']
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
    parser.add_argument('--model-path', type=Path,
                        default=Path(os.environ.get(
                            'OPENFLEX_SIM_SCENE_MODEL',
                            str(ROOT / 'local_dependencies/openflex_roboos/readonly_scene.xml'))))
    arguments = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=83')
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('grasp_trajectory_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    rclpy.init()
    node = TrajectoryProbe()
    report = {'grasp_execution_authorized': False}
    try:
        report = node.run(arguments.candidate_report,
                          arguments.model_path.expanduser().resolve(strict=True),
                          arguments.held_transport)
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
