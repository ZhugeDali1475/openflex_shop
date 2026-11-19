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
from openflex_motion_profile import scaling_for_phase, selected_profile
from openflex_roboos_bridge.navigation_guard import NavigationGuard
from openflex_roboos_bridge.scene_registry import load_scene_registry
from openflex_roboos_bridge.trajectory_guard import sample_joint_path
from openflex_thesis_control.attached_object import quaternion_conjugate, relative_pose
from openflex_thesis_mujoco.math_utils import compose_pose
from openflex_thesis_mujoco.release_verification import reachable_release_center
from openflex_thesis_control.scene_contract import scene_readback_matches


NAMES = ['lift_joint', *[f'openarmx_right_joint{index}' for index in range(1, 8)]]
HAND = 'openarmx_right_hand'
TOUCH = {HAND, 'openarmx_right_left_finger', 'openarmx_right_right_finger'}
# The reachable release centre (RELEASE_REACH_M / SURFACE_EDGE_MARGIN_M) is shared
# with the delivery contract so the plan and the independent verification agree.


def joint_goal_constraints(positions):
    constraints = Constraints()
    constraints.joint_constraints = [JointConstraint(joint_name=name, position=float(positions[name]),
        tolerance_above=0.001, tolerance_below=0.001, weight=1.0) for name in NAMES]
    return constraints


def payload_contact_permitted(body_name, released):
    return not released and body_name in TOUCH


class PlaceProbe(SafetyProbe):
    ignore_ik_collisions = False
    assume_identity_attachment = False
    ik_anchor_current_hand = False
    ik_seed_nominal = False
    target_xyz = None
    extra_bases = None

    def run(self, model_path, search_base=False, retreat=False, destination_id='sim_delivery_top_left'):
        measured = self.probe(model_path, allow_attached=not retreat)
        diagnostic = measured['simulator_diagnostic']
        if not measured['state_valid'] or (diagnostic['attached_arm'] != 'right' if not retreat else diagnostic['attached_arm'] is not None):
            raise RuntimeError('Measured full-robot validity and requested attachment state required')
        config_path = Path(os.environ.get(
            'OPENFLEX_SIM_SCENE_CONFIG',
            str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml')))
        registry = load_scene_registry(config_path.expanduser().resolve(strict=True), model_path)
        if destination_id not in {'sim_delivery_top_left', 'sim_delivery_top_right'}:
            raise ValueError(f'Unregistered placement destination: {destination_id}')
        affordance = registry['affordances'][destination_id]
        surface = affordance['surface_center_xyz']
        surface_half_size = [value / 2 for value in
                             affordance['surface_dimensions_xy']]
        # The audited delivery base is the final waypoint of the configured
        # route, not the geometry-derived east-edge approach. Plan the placement
        # from that base while screening its approach along the route, so a
        # planning-only probe can validate the real competition delivery pose.
        station = registry['stations'].get(destination_id, {})
        route = [[float(value) for value in waypoint]
                 for waypoint in (station.get('route_waypoints') or [])]
        report = {'trajectory_execution': False, 'physical_verification': False,
                  'model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
                  'destination_id': destination_id,
                  'station_route': route,
                  'destination_body': affordance['body'],
                  'destination_surface_geom': affordance['geom'],
                  'measured_start_state': measured, 'release_center_map': None,
                  'retreat_only': retreat, 'candidates': [], 'phases': [], 'place_paths_valid': False,
                  'motion_profile': selected_profile()}
        guard = NavigationGuard(model_path)
        nominal_positions = {}
        for joint_id in range(guard.model.njnt):
            joint_name = mujoco.mj_id2name(guard.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if joint_name:
                nominal_positions[joint_name] = float(guard.model.qpos0[guard.model.jnt_qposadr[joint_id]])
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
        if self.assume_identity_attachment:
            offset, rotation = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
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
            request.ik_request.avoid_collisions = not self.ignore_ik_collisions
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
            if self.ik_seed_nominal:
                positions = {name: nominal_positions.get(name, positions[name])
                             for name in seed_state.joint_state.name}
            for lift in [positions['lift_joint'], -0.25, -0.35, -0.45, -0.55]:
                replacements = dict(positions, lift_joint=lift)
                request.ik_request.robot_state.joint_state.position = [replacements[name] for name in seed_state.joint_state.name]
                constraint = JointConstraint(joint_name='lift_joint', position=lift,
                    tolerance_above=0.03, tolerance_below=0.03, weight=1.0)
                request.ik_request.constraints.joint_constraints = [constraint]
                response = self.call(ik, request)
                record = {'lift_seed': lift, 'code': response.error_code.val}
                if response.error_code.val == 1:
                    solved = deepcopy(seed_state)
                    replacements = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
                    solved.joint_state.position = [replacements.get(name, position)
                        for name, position in zip(seed_state.joint_state.name, seed_state.joint_state.position)]
                    validity.robot_state = solved
                    checked = self.call(self.validity, validity)
                    record['state_valid'] = bool(checked.valid)
                    if not checked.valid:
                        record['contacts'] = [ordered['body_name_1'] + '|' + ordered['body_name_2']
                                              for ordered in message_to_ordereddict(checked)['contacts']]
                    attempts.append(record)
                    if checked.valid or self.ignore_ik_collisions:
                        return solved, attempts
                    continue
                attempts.append(record)
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

        # Candidate bases are (xyyaw, via): the configured delivery base is
        # reached along its audited route, while legacy geometry candidates keep
        # the historical direct-screen behaviour.
        candidates = [(base, [])]
        if route:
            candidates.append((route[-1], route[:-1]))
        if search_base:
            approach_x = surface[0] + surface_half_size[0] + 0.50
            candidates += [([approach_x + offset, surface[1] + lateral, math.pi], [])
                           for offset in (0.0, 0.10, -0.10, 0.20, -0.20)
                           for lateral in (0.0, -0.15, 0.15)]
        for explicit in (self.extra_bases or []):
            candidates.append(([float(value) for value in explicit], route[:-1] if route else []))
        for candidate, via in candidates:
            navigation = guard.check(joints, base, candidate,
                                     payload=None if retreat else object_pose,
                                     **({'via': via} if via else {}))
            if not navigation['accepted'] and route and not via:
                routed = guard.check(joints, base, candidate,
                                     payload=None if retreat else object_pose, via=route[:-1])
                if routed['accepted']:
                    navigation, via = routed, route[:-1]
            entry = {'base_xyyaw': candidate, 'navigation': navigation, 'route_via': via}
            report['candidates'].append(entry)
            if not navigation['accepted']:
                continue
            release = reachable_release_center(candidate, surface, surface_half_size)
            if self.target_xyz is not None:
                release = [float(value) for value in self.target_xyz]
            entry['release_center_map'] = release
            if release is None and not retreat:
                entry['release_rejected'] = 'surface_not_reachable_from_candidate_base'
                continue
            set_base(candidate)
            if self.ik_anchor_current_hand:
                release = [float(value) for value in guard.data.xpos[hand]]
            if retreat:
                cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
                start_position = guard.data.xpos[hand].copy()
                quaternion = guard.data.xquat[hand].copy()
                hand_z = guard.data.xmat[hand].reshape(3, 3)[:, 2]
                # The gripper grips the item from the side, so a straight pull-back
                # along the hand axis can jam on the released item or the table.
                # Try a prioritised list of clearance moves and keep the first one
                # that is planned end-to-end and collision-audited.
                away = [float(start_position[0] - surface[0]), float(start_position[1] - surface[1]), 0.0]
                magnitude = math.hypot(away[0], away[1])
                away = [value / magnitude for value in away] if magnitude > 1.0e-6 else [0.0, 0.0, 0.0]
                directions = [('up', [0.0, 0.0, 1.0]),
                              ('backward', [-float(value) for value in hand_z]),
                              ('away_from_table', away)]
                attempts, chosen, response = [], None, None

                def clearance(name, direction, distance):
                    request = GetCartesianPath.Request()
                    request.header.frame_id, request.group_name, request.link_name = 'map', 'right_arm_with_lift', HAND
                    request.start_state = deepcopy(state)
                    request.max_step, request.jump_threshold = 0.005, 2.0
                    request.revolute_jump_threshold, request.prismatic_jump_threshold = 0.15, 0.015
                    request.avoid_collisions = True
                    velocity, acceleration = scaling_for_phase('postplace_clearance')
                    request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor = velocity, acceleration
                    waypoint = Pose()
                    destination = [float(value + direction[index] * distance)
                                   for index, value in enumerate(start_position)]
                    waypoint.position.x, waypoint.position.y, waypoint.position.z = destination
                    waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w = [float(value) for value in [*quaternion[1:], quaternion[0]]]
                    request.waypoints = [waypoint]
                    result = self.call(cartesian, request)
                    return result, destination

                for name, direction in directions:
                    for distance in (0.10, 0.06, 0.03):
                        trial, destination = clearance(name, direction, distance)
                        attempts.append({'direction': name, 'distance_m': distance,
                            'waypoint_map': destination,
                            'cartesian_fraction': float(trial.fraction),
                            'cartesian_error_code': int(trial.error_code.val)})
                        if (trial.error_code.val == 1 and trial.fraction >= 0.999999
                                and trial.solution.joint_trajectory.points):
                            chosen, response = (name, distance), trial
                            break
                    if chosen is not None:
                        break
                report['retreat_clearance_attempts'] = attempts
                if chosen is None:
                    report['blocked_at'] = 'postplace_clearance_cartesian'
                    return report
                phase = {'phase': 'postplace_clearance', 'direction': chosen[0], 'distance_m': chosen[1],
                    'planning_response': {'trajectory': message_to_ordereddict(response.solution)},
                    'cartesian_fraction': response.fraction, 'cartesian_error_code': response.error_code.val,
                    'trajectory_samples_valid': False,
                    'mujoco_environment_samples_valid': False}
                report['phases'].append(phase)
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
            if release is not None:
                report['release_center_map'] = release
            for phase_name, positions in targets:
                request = GetMotionPlan.Request()
                motion = request.motion_plan_request
                motion.group_name, motion.start_state = 'right_arm_with_lift', deepcopy(state)
                motion.allowed_planning_time, motion.num_planning_attempts = 8.0, 3
                velocity, acceleration = scaling_for_phase(phase_name)
                motion.max_velocity_scaling_factor, motion.max_acceleration_scaling_factor = velocity, acceleration
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
    parser.add_argument('--ignore-ik-collisions', action='store_true',
                        help='diagnostic only: allow IK solutions that the planning scene reports as in collision')
    parser.add_argument('--assume-identity-attachment', action='store_true',
                        help='diagnostic only: ignore the measured hand->object transform when building the IK target')
    parser.add_argument('--ik-anchor-current-hand', action='store_true',
                        help='diagnostic only: ask IK to reproduce the current measured hand pose')
    parser.add_argument('--ik-seed-nominal', action='store_true',
                        help='diagnostic only: seed IK from the model nominal arm joints instead of the measured pose')
    parser.add_argument('--target-xyz', type=float, nargs=3, default=None,
                        help='diagnostic only: override the map release-centre target [x y z]')
    parser.add_argument('--base', type=float, nargs=3, action='append', default=None,
                        metavar=('X', 'Y', 'YAW'),
                        help='extra candidate placement base reached along the station route (repeatable)')
    parser.add_argument('--retreat', action='store_true')
    parser.add_argument('--destination', choices=['sim_delivery_top_left', 'sim_delivery_top_right'],
                        default='sim_delivery_top_left',
                        help='registered table geometry to plan against; this does not authorize execution')
    parser.add_argument('--model-path', type=Path,
                        default=Path(os.environ.get(
                            'OPENFLEX_SIM_SCENE_MODEL',
                            str(ROOT / 'local_dependencies/openflex_roboos/readonly_scene.xml'))))
    arguments = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Only isolated simulation domain 83 is supported')
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('place_trajectory_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    rclpy.init()
    node, report = PlaceProbe(), {'trajectory_execution': False}
    node.ignore_ik_collisions = arguments.ignore_ik_collisions
    node.assume_identity_attachment = arguments.assume_identity_attachment
    node.ik_anchor_current_hand = arguments.ik_anchor_current_hand
    node.ik_seed_nominal = arguments.ik_seed_nominal
    node.target_xyz = arguments.target_xyz
    node.extra_bases = arguments.base
    try:
        report = node.run(arguments.model_path.expanduser().resolve(strict=True),
                          arguments.search_base, arguments.retreat, arguments.destination)
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
