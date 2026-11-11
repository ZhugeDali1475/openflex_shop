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
from geometry_msgs.msg import Pose, PoseStamped, Transform
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionIK, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
import rclpy
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose_stamped

ROOT = Path(__file__).resolve().parents[1]
for directory in ['scripts', 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge',
                  'openflex_ws/src/openflex_thesis/openflex_thesis_control',
                  'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco']:
    sys.path.insert(0, str(ROOT / directory))
from check_openflex_manipulation_safety import SafetyProbe
from openflex_roboos_bridge.navigation_guard import NavigationGuard
from openflex_roboos_bridge.scene_registry import load_scene_registry
from openflex_roboos_bridge.trajectory_guard import sample_joint_path
from openflex_thesis_control.attached_object import quaternion_conjugate, relative_pose
from openflex_thesis_mujoco.math_utils import compose_pose
from openflex_thesis_control.scene_contract import scene_readback_matches


NAMES = ['lift_joint', *[f'openarmx_right_joint{index}' for index in range(1, 8)]]
HAND = 'openarmx_right_hand'
TOUCH = {HAND, 'openarmx_right_left_finger', 'openarmx_right_right_finger'}


def joint_goal_constraints(positions):
    constraints = Constraints()
    constraints.joint_constraints = [JointConstraint(joint_name=name, position=float(positions[name]),
        tolerance_above=0.001, tolerance_below=0.001, weight=1.0) for name in NAMES]
    return constraints


def payload_contact_permitted(body_name, released):
    return not released and body_name in TOUCH


class PlaceProbe(SafetyProbe):
    def run(self, model_path, search_base=False, retreat=False):
        measured = self.probe(model_path, allow_attached=not retreat)
        diagnostic = measured['simulator_diagnostic']
        if not measured['state_valid'] or (diagnostic['attached_arm'] != 'right' if not retreat else diagnostic['attached_arm'] is not None):
            raise RuntimeError('Measured full-robot validity and requested attachment state required')
        registry = load_scene_registry(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml', model_path)
        surface = registry['affordances']['delivery_zone']['surface_center_xyz']
        release = [surface[0], surface[1], surface[2] + 0.08 + 0.015]
        report = {'trajectory_execution': False, 'physical_verification': False,
                  'model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
                  'measured_start_state': measured, 'release_center_map': release,
                  'retreat_only': retreat, 'candidates': [], 'phases': [], 'place_paths_valid': False}
        guard = NavigationGuard(model_path)
        for sample_index in range(8):
            rclpy.spin_once(self, timeout_sec=0.02)
        joints = self.fresh('joints')
        state = RobotState()
        state.joint_state = deepcopy(joints)
        odometry = self.fresh('odometry').pose.pose
        base = [odometry.position.x, odometry.position.y,
                math.atan2(2 * odometry.orientation.w * odometry.orientation.z, 1 - 2 * odometry.orientation.z ** 2)]
        state.multi_dof_joint_state.header.frame_id = 'odom'
        state.multi_dof_joint_state.joint_names = ['base_planar_joint']
        state.multi_dof_joint_state.transforms = [Transform()]
        guard.check(joints, base, base)
        object_pose = diagnostic['object_pose']
        object_address = guard.offset('thesis_object_free')
        hand = mujoco.mj_name2id(guard.model, mujoco.mjtObj.mjOBJ_BODY, HAND)
        object_body = mujoco.mj_name2id(guard.model, mujoco.mjtObj.mjOBJ_BODY, 'thesis_object')
        quaternion = guard.data.xquat[hand]
        offset, rotation = relative_pose(guard.data.xpos[hand], [*quaternion[1:], quaternion[0]],
                                        object_pose['position_m'], object_pose['orientation_xyzw'])
        attached = AttachedCollisionObject()
        attached.link_name, attached.touch_links = HAND, sorted(TOUCH)
        attached.object.header.frame_id = HAND
        attached.object.id, attached.object.operation = 'planning_only_target_box', CollisionObject.ADD
        geometry = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.07, 0.06, 0.16])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = offset
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
        attached.object.primitives, attached.object.primitive_poses = [geometry], [pose]
        state.attached_collision_objects = [] if retreat else [attached]
        if retreat:
            placed = deepcopy(attached.object)
            placed.header.frame_id, placed.id = 'map', 'released_delivery_payload'
            placed.pose.orientation.w = 1.0
            placed.primitive_poses[0].position.x, placed.primitive_poses[0].position.y, placed.primitive_poses[0].position.z = object_pose['position_m']
            placed.primitive_poses[0].orientation.x, placed.primitive_poses[0].orientation.y, placed.primitive_poses[0].orientation.z, placed.primitive_poses[0].orientation.w = object_pose['orientation_xyzw']
            update = ApplyPlanningScene.Request()
            update.scene.is_diff, update.scene.robot_state.is_diff = True, True
            update.scene.world.collision_objects = [placed]
            if not self.call(self.apply, update).success:
                raise RuntimeError('Released payload collision scene update failed')
            query = GetPlanningScene.Request()
            query.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            actual_world = self.call(self.readback, query).scene.world.collision_objects
            def to_map(stamped):
                if stamped.header.frame_id == 'map':
                    return stamped
                return do_transform_pose_stamped(stamped, self.tf_buffer.lookup_transform('map', stamped.header.frame_id, Time()))
            if not scene_readback_matches([placed], actual_world, to_map):
                raise RuntimeError('Released payload collision geometry readback failed')
            report['released_payload_world_readback_verified'] = True
        ik = self.create_client(GetPositionIK, '/compute_ik')
        planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        validity = GetStateValidity.Request()
        validity.group_name = ''

        def set_base(candidate):
            stamped = PoseStamped()
            stamped.header.frame_id = 'map'
            stamped.pose.position.x, stamped.pose.position.y = candidate[:2]
            stamped.pose.position.z = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform.translation.z
            stamped.pose.orientation.z, stamped.pose.orientation.w = math.sin(candidate[2] / 2), math.cos(candidate[2] / 2)
            transformed = do_transform_pose_stamped(stamped, self.tf_buffer.lookup_transform('odom', 'map', Time()))
            state.multi_dof_joint_state.transforms[0].translation.x = transformed.pose.position.x
            state.multi_dof_joint_state.transforms[0].translation.y = transformed.pose.position.y
            state.multi_dof_joint_state.transforms[0].translation.z = 0.0
            state.multi_dof_joint_state.transforms[0].rotation = transformed.pose.orientation
            guard.data.qpos[guard.base_offsets] = candidate
            mujoco.mj_forward(guard.model, guard.data)

        def goal_ik(target, seed_state, yaw):
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'right_arm_with_lift'
            request.ik_request.ik_link_name = HAND
            request.ik_request.robot_state = deepcopy(seed_state)
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 1
            request.ik_request.pose_stamped.header.frame_id = 'map'
            inverse_rotation = quaternion_conjugate(rotation)
            inverse_position, _ = relative_pose(offset, rotation, [0, 0, 0], [0, 0, 0, 1])
            position, orientation = compose_pose(target, [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)], inverse_position, inverse_rotation)
            destination = request.ik_request.pose_stamped.pose
            destination.position.x, destination.position.y, destination.position.z = position
            destination.orientation.x, destination.orientation.y, destination.orientation.z, destination.orientation.w = orientation
            attempts = []
            positions = dict(zip(seed_state.joint_state.name, seed_state.joint_state.position))
            for lift in [positions['lift_joint'], -0.25, -0.35, -0.45, -0.55]:
                replacements = dict(positions, lift_joint=lift)
                request.ik_request.robot_state.joint_state.position = [replacements[name] for name in seed_state.joint_state.name]
                constraint = JointConstraint(joint_name='lift_joint', position=lift,
                    tolerance_above=0.03, tolerance_below=0.03, weight=1.0)
                request.ik_request.constraints.joint_constraints = [constraint]
                response = self.call(ik, request)
                attempts.append({'lift_seed': lift, 'code': response.error_code.val})
                if response.error_code.val == 1:
                    solved = deepcopy(seed_state)
                    replacements = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
                    solved.joint_state.position = [replacements.get(name, position)
                        for name, position in zip(seed_state.joint_state.name, seed_state.joint_state.position)]
                    validity.robot_state = solved
                    checked = self.call(self.validity, validity)
                    if checked.valid:
                        return solved, attempts
            return None, attempts

        def audit(phase, trajectory):
            current = dict(zip(state.joint_state.name, state.joint_state.position))
            samples = sample_joint_path(trajectory.joint_names,
                [[current[name] for name in trajectory.joint_names]] + [point.positions for point in trajectory.points])
            for index, sample in enumerate(samples):
                replacements = dict(zip(trajectory.joint_names, sample))
                state.joint_state.position = [replacements.get(name, position)
                    for name, position in zip(state.joint_state.name, state.joint_state.position)]
                validity.robot_state = state
                checked = self.call(self.validity, validity)
                if not checked.valid:
                    phase.update(rejected_sample=index, moveit_contacts=message_to_ordereddict(checked)['contacts'])
                    return False
                for name, position in zip(trajectory.joint_names, sample):
                    guard.data.qpos[guard.offset(name)] = position
                mujoco.mj_forward(guard.model, guard.data)
                if retreat:
                    carried_position, carried_rotation = object_pose['position_m'], object_pose['orientation_xyzw']
                else:
                    quaternion = guard.data.xquat[hand]
                    carried_position, carried_rotation = compose_pose(guard.data.xpos[hand],
                        [*quaternion[1:], quaternion[0]], offset, rotation)
                guard.data.qpos[object_address:object_address + 7] = [*carried_position, carried_rotation[3], *carried_rotation[:3]]
                mujoco.mj_forward(guard.model, guard.data)
                for contact in guard.data.contact[:guard.data.ncon]:
                    if contact.dist > 0:
                        continue
                    first, second = (int(guard.model.geom_bodyid[geometry]) for geometry in (contact.geom1, contact.geom2))
                    forbidden = False
                    for robot_body, obstacle in ((first, second), (second, first)):
                        if guard.model.body_rootid[robot_body] == guard.robot:
                            name = mujoco.mj_id2name(guard.model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
                            forbidden |= obstacle in guard.environment
                            forbidden |= obstacle == 0 and not name.endswith('_wheel_link')
                            forbidden |= obstacle == object_body and not payload_contact_permitted(name, retreat)
                    if not retreat and object_body in (first, second):
                        other = second if first == object_body else first
                        forbidden |= other in guard.environment | {0}
                    if forbidden:
                        phase.update(rejected_sample=index, mujoco_contact={'bodies': [
                            mujoco.mj_id2name(guard.model, mujoco.mjtObj.mjOBJ_BODY, body) for body in (first, second)],
                            'distance_m': float(contact.dist)})
                        return False
            phase.update(trajectory_samples_valid=True, mujoco_environment_samples_valid=True, samples=len(samples))
            return True

        candidates = [base]
        if search_base:
            candidates = [[position, surface[1] + lateral, math.pi]
                          for position in [0.0, -0.10, -0.15, -0.20] for lateral in [0.0, -0.15, 0.15]]
        for candidate in candidates:
            navigation = guard.check(joints, base, candidate, payload=None if retreat else object_pose)
            entry = {'base_xyyaw': candidate, 'navigation': navigation}
            report['candidates'].append(entry)
            if not navigation['accepted']:
                continue
            set_base(candidate)
            if retreat:
                cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
                request = GetCartesianPath.Request()
                request.header.frame_id, request.group_name, request.link_name = 'map', 'right_arm_with_lift', HAND
                request.start_state = deepcopy(state)
                request.max_step, request.jump_threshold = 0.005, 2.0
                request.revolute_jump_threshold, request.prismatic_jump_threshold = 0.15, 0.015
                request.avoid_collisions = True
                request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = 0.05
                start_position = guard.data.xpos[hand].copy()
                backwards = -guard.data.xmat[hand].reshape(3, 3)[:, 2] * 0.10
                quaternion = guard.data.xquat[hand].copy()
                waypoint = Pose()
                destination = start_position + backwards
                waypoint.position.x, waypoint.position.y, waypoint.position.z = [float(value) for value in destination]
                waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w = [float(value) for value in [*quaternion[1:], quaternion[0]]]
                request.waypoints = [waypoint]
                response = self.call(cartesian, request)
                phase = {'phase': 'postplace_clearance', 'planning_response': {'trajectory': message_to_ordereddict(response.solution)},
                    'cartesian_fraction': response.fraction, 'cartesian_error_code': response.error_code.val,
                    'trajectory_samples_valid': False,
                    'mujoco_environment_samples_valid': False}
                report['phases'].append(phase)
                if response.error_code.val != 1 or response.fraction < 0.999999 or not response.solution.joint_trajectory.points:
                    report['blocked_at'] = 'postplace_clearance_cartesian'
                    return report
                if not audit(phase, response.solution.joint_trajectory):
                    report['blocked_at'] = 'postplace_clearance_collision_audit'
                    return report
                targets = [('postplace_retract', dict(zip(NAMES, [-0.10, 0, 0, 0, 1.8, 0, 0, 0])))]
            else:
                preplace = [release[0], release[1], release[2] + 0.10]
                before = None
                entry['orientation_attempts'] = []
                for object_yaw in [candidate[2] - base[2], math.pi, 0.0, math.pi / 2, -math.pi / 2]:
                    before, attempts = goal_ik(preplace, state, object_yaw)
                    entry['orientation_attempts'].append({'object_yaw': object_yaw, 'attempts': attempts})
                    if before is not None:
                        break
                if before is None:
                    continue
                lowered, entry['place_ik'] = goal_ik(release, before, object_yaw)
                if lowered is None:
                    continue
                targets = [('preplace', dict(zip(before.joint_state.name, before.joint_state.position))),
                           ('place', dict(zip(lowered.joint_state.name, lowered.joint_state.position)))]
            report['selected_base_xyyaw'] = candidate
            for phase_name, positions in targets:
                request = GetMotionPlan.Request()
                motion = request.motion_plan_request
                motion.group_name, motion.start_state = 'right_arm_with_lift', deepcopy(state)
                motion.allowed_planning_time, motion.num_planning_attempts = 8.0, 3
                motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = 0.05
                motion.goal_constraints = [joint_goal_constraints(positions)]
                response = self.call(planner, request).motion_plan_response
                phase = {'phase': phase_name, 'planning_response': message_to_ordereddict(response),
                         'trajectory_samples_valid': False, 'mujoco_environment_samples_valid': False}
                report['phases'].append(phase)
                if response.error_code.val != 1 or not response.trajectory.joint_trajectory.points:
                    report['blocked_at'] = phase_name + '_planning'
                    return report
                if not audit(phase, response.trajectory.joint_trajectory):
                    report['blocked_at'] = phase_name + '_collision_audit'
                    return report
            report['place_paths_valid'] = True
            return report
        report['blocked_at'] = 'no_valid_base_and_ik_candidate'
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--search-base', action='store_true')
    parser.add_argument('--retreat', action='store_true')
    arguments = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Only isolated simulation domain 83 is supported')
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('place_trajectory_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    rclpy.init()
    node, report = PlaceProbe(), {'trajectory_execution': False}
    try:
        report = node.run(ROOT / 'local_dependencies/openflex_roboos/readonly_scene.xml', arguments.search_base, arguments.retreat)
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        report['probe_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(output / 'result.json')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
