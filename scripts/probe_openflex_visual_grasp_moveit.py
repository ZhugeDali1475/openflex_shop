#!/usr/bin/env python3
"""Plan-only binding of the visual grasp chain to this task's MoveIt stack.

Runs in the isolated simulation domain and never executes motion.  It:

1. reads the live MuJoCo head RGB-D camera, its camera info, ``/joint_states``
   and the ``map -> head optical`` TF from the running bridge;
2. back-projects the depth frame into a target point cloud and feeds it through
   the ROS-free visual grasp chain (:mod:`visual_grasp_scene` +
   :mod:`visual_grasp_planning` + :mod:`visual_grasp_moveit`);
3. applies the resulting target collision volume *and* the static shelf boxes to
   the real MoveIt planning scene (``/apply_planning_scene``) and reads it back;
4. checks the measured start state with ``/check_state_validity``;
5. writes the perception/evaluation evidence pair.

It never calls ``/execute_kinematic_path``, never publishes a controller goal and
never authorizes grasp execution.
"""

from copy import deepcopy
from datetime import datetime
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Transform
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from rclpy.time import Time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from check_openflex_manipulation_safety import SafetyProbe  # noqa: E402
from openflex_roboos_bridge.target_observation import TargetObservation  # noqa: E402
from openflex_roboos_bridge.visual_grasp_evidence import planning_evidence_pair  # noqa: E402
from openflex_roboos_bridge.visual_grasp_input import VisualGraspPointCloudInput  # noqa: E402
from openflex_roboos_bridge.visual_grasp_moveit import build_moveit_plan  # noqa: E402
from openflex_roboos_bridge.visual_grasp_planning import GraspCandidatePose  # noqa: E402
from openflex_roboos_bridge.visual_grasp_scene import (  # noqa: E402
    build_visual_grasp_plan_request,
    load_shelf_scene,
)

PLANNING_FRAME = 'map'
CAMERA_FRAME = 'thesis_head_color_optical_frame'
# Conservative workspace window in the planning frame that isolates the observed
# product from the rest of the rendered scene.  It is a geometric filter on the
# live camera depth, not a simulator identity lookup.
TARGET_WINDOW_XYZ = ((0.95, 1.35), (-0.70, 0.70), (0.76, 1.02))
DEPTH_METERS_MIN, DEPTH_METERS_MAX = 0.15, 3.0


def depth_to_meters(message):
    encoding = message.encoding.lower()
    if encoding in ('16uc1', 'mono16'):
        values = np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width)
        return values.astype(np.float32) / 1000.0
    values = np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.width)
    return values


def backproject(depth_m, info):
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    rows, cols = np.where(np.isfinite(depth_m) & (depth_m > DEPTH_METERS_MIN)
                          & (depth_m < DEPTH_METERS_MAX))
    z = depth_m[rows, cols]
    points = np.stack([(cols - cx) * z / fx, (rows - cy) * z / fy, z], axis=1)
    return points.astype(np.float32)


def _rotation_matrix(rotation):
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def map_points(points_camera, transform):
    rotation = _rotation_matrix(transform.transform.rotation)
    translation = np.array([transform.transform.translation.x, transform.transform.translation.y,
                            transform.transform.translation.z], dtype=np.float64)
    return np.asarray(points_camera, dtype=np.float64) @ rotation.T + translation


def select_target_window(points_camera, transform):
    if len(points_camera) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    mapped = map_points(points_camera, transform)
    inside = ((mapped[:, 0] >= TARGET_WINDOW_XYZ[0][0]) & (mapped[:, 0] <= TARGET_WINDOW_XYZ[0][1])
              & (mapped[:, 1] >= TARGET_WINDOW_XYZ[1][0]) & (mapped[:, 1] <= TARGET_WINDOW_XYZ[1][1])
              & (mapped[:, 2] >= TARGET_WINDOW_XYZ[2][0]) & (mapped[:, 2] <= TARGET_WINDOW_XYZ[2][1]))
    return np.asarray(points_camera, dtype=np.float32)[inside]


class VisualGraspMoveItProbe(SafetyProbe):
    def __init__(self):
        super().__init__()
        self.subscriptions_keepalive.extend([
            self.create_subscription(CameraInfo, '/thesis_sim/head/color/camera_info',
                lambda message: self.samples.__setitem__('camera_info', (deepcopy(message), time.monotonic())),
                qos_profile_sensor_data),
            self.create_subscription(Image, '/thesis_sim/head/aligned_depth_to_color/image_raw',
                lambda message: self.samples.__setitem__('depth', (deepcopy(message), time.monotonic())),
                qos_profile_sensor_data),
        ])

    def run(self, shelf_scene_path):
        deadline = time.monotonic() + 10
        while (not {'joints', 'odometry', 'camera_info', 'depth'}.issubset(self.samples)
               and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)
        camera_info = self.fresh('camera_info')
        depth = self.fresh('depth')
        joints = self.fresh('joints')
        odometry = self.fresh('odometry')
        deadline = time.monotonic() + 5
        while not self.tf_buffer.can_transform(PLANNING_FRAME, CAMERA_FRAME, Time()):
            if time.monotonic() >= deadline:
                raise RuntimeError('Fresh head-camera TF unavailable; planning not authorized')
            rclpy.spin_once(self, timeout_sec=0.05)
        transform = self.tf_buffer.lookup_transform(PLANNING_FRAME, CAMERA_FRAME, Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        points = backproject(depth_to_meters(depth), camera_info)
        target = select_target_window(points, transform)
        if target.shape[0] < 100:
            raise RuntimeError(f'Live depth window produced only {target.shape[0]} target points')
        centroid = target.mean(axis=0)
        scene_points = points[np.random.default_rng(7).choice(
            points.shape[0], min(512, points.shape[0]), replace=False)]
        depth_sha256 = hashlib.sha256(bytes(depth.data)).hexdigest()

        observation = TargetObservation(
            perception_track_id='track_live_depth_001', task_target='red', color='red',
            bbox_px=(0, 0, depth.width, depth.height), position_camera_xyz=tuple(float(v) for v in centroid),
            position_map_xyz=tuple(float(v) for v in map_points([centroid], transform)[0]),
            depth_valid_fraction=float(
                np.count_nonzero(depth_to_meters(depth) > 0) / depth_to_meters(depth).size),
            timestamp_sec=time.time(), camera_frame=CAMERA_FRAME, map_frame=PLANNING_FRAME,
            tf_valid=True, confidence=0.5, metadata={'arm_name': 'right',
                'source': 'live_mujoco_head_depth_window_not_calibrated_perception'})
        visual_input = VisualGraspPointCloudInput(
            observation, target, scene_points, CAMERA_FRAME, observation.timestamp_sec)
        candidates = [GraspCandidatePose(
            candidate_id='live_depth_candidate_0',
            position_camera_xyz=tuple(float(v) for v in centroid),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0), score=0.0, required_width_m=0.0)]
        request = build_visual_grasp_plan_request(
            visual_input, planning_frame=PLANNING_FRAME, shelf_scene=shelf_scene_path,
            tf_parent_frame=PLANNING_FRAME, tf_child_frame=CAMERA_FRAME,
            tf_translation_xyz=(translation.x, translation.y, translation.z),
            tf_quaternion_xyzw=(rotation.x, rotation.y, rotation.z, rotation.w),
            candidates=candidates, joint_names=tuple(joints.name),
            joint_positions=tuple(float(v) for v in joints.position), target_padding_m=0.01)
        plan = build_moveit_plan(request, move_group_parameters={
            'allow_trajectory_execution': False, 'moveit_manage_controllers': False})

        apply = ApplyPlanningScene.Request()
        apply.scene.is_diff = True
        for box in plan.scene_diff:
            collision = CollisionObject()
            collision.header.frame_id = box.frame_id
            collision.id, collision.operation = box.object_id, CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type, primitive.dimensions = SolidPrimitive.BOX, list(box.dimensions_xyz)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = box.position_xyz
            pose.orientation.w = 1.0
            collision.primitives, collision.primitive_poses = [primitive], [pose]
            apply.scene.world.collision_objects.append(collision)
        applied = self.call(self.apply, apply)
        if not applied.success:
            raise RuntimeError('Visual collision scene application failed')
        query = GetPlanningScene.Request()
        query.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                       | PlanningSceneComponents.ROBOT_STATE)
        scene = self.call(self.readback, query).scene
        received_ids = {collision.id for collision in scene.world.collision_objects}
        requested_ids = {collision.id for collision in apply.scene.world.collision_objects}
        if not requested_ids.issubset(received_ids):
            raise RuntimeError('Planning scene readback missing visual collision geometry')

        state = RobotState()
        state.joint_state = deepcopy(joints)
        state.multi_dof_joint_state.header.frame_id = 'odom'
        state.multi_dof_joint_state.joint_names = ['base_planar_joint']
        base = Transform()
        base.translation.x, base.translation.y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        base.rotation = deepcopy(odometry.pose.pose.orientation)
        state.multi_dof_joint_state.transforms = [base]
        validity = GetStateValidity.Request()
        validity.group_name, validity.robot_state = '', state
        checked = self.call(self.validity, validity)

        evidence = planning_evidence_pair(
            request, request_id=datetime.now().strftime('visual_moveit_%Y%m%d_%H%M%S_%f'),
            attempt=1, frame_sha256=depth_sha256,
            frame_ref='thesis_sim/head/color/image_raw', depth_ref='thesis_sim/head/aligned_depth_to_color/image_raw')
        return {
            'trajectory_execution': False, 'robot_motion_executed': False,
            'grasp_execution_authorized': False, 'object_memory_used': False,
            'perception_not_run': True,
            'target_cloud_source': 'live_mujoco_head_depth_window',
            'candidate_source': 'fixture_pose_from_observed_centroid',
            'target_point_count': int(target.shape[0]),
            'depth_sha256': depth_sha256,
            'scene_applied': True,
            'scene_readback_ids': sorted(received_ids),
            'moveit_plan': plan.to_dict(),
            'state_valid': bool(checked.valid),
            'contacts': message_to_ordereddict(checked)['contacts'],
            'evidence': evidence,
            'remaining_requirements': ['GraspNet/segmentation front-end instead of depth window',
                                       'candidate EEF states checked by IK before any execution',
                                       'continuous path and attached-object clearance',
                                       'exclusive MoveIt execution and base lock'],
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', default=os.environ.get(
        'OPENFLEX_SIM_SCENE_CONFIG',
        str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/sim_shelf_scene.yaml')))
    arguments = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=83')
    scene_path = Path(arguments.scene).expanduser().resolve(strict=True)
    load_shelf_scene(scene_path)
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('visual_grasp_moveit_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    rclpy.init()
    node = VisualGraspMoveItProbe()
    try:
        report = node.run(scene_path)
        report['scene_sha256'] = hashlib.sha256(scene_path.read_bytes()).hexdigest()
        report['probe_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    except Exception as error:
        (output / 'result.json').write_text(json.dumps(
            {'trajectory_execution': False, 'grasp_execution_authorized': False,
             'error': str(error)}, ensure_ascii=False, indent=2) + '\n')
        raise
    finally:
        print(output / 'result.json')
        node.destroy_node()
        rclpy.shutdown()
    (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
