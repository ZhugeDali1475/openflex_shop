#!/usr/bin/env python3
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose, Transform
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningSceneComponents, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openflex_ws/src/openflex_thesis/openflex_thesis_control"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_thesis_control.scene_contract import primitive_pose, scene_readback_matches
from openflex_roboos_bridge.support_contacts import allow_wheel_floor_support


ROOT = Path(__file__).resolve().parents[1]


def collision_boxes(model_path):
    world = ET.parse(model_path).find("worldbody")
    boxes = []
    for name in ("thesis_shelf", "thesis_delivery_table"):
        body = world.find(f"body[@name='{name}']")
        if set(body.attrib) != {"name"}:
            raise ValueError("Unsupported environment parent transform")
        for geometry in body.findall("geom"):
            if geometry.get("type") != "box" or geometry.get("quat") or geometry.get("euler"):
                raise ValueError("Expected world-aligned environment boxes")
            boxes.append((geometry.get("name"), [float(value) for value in geometry.get("pos").split()],
                          [2 * float(value) for value in geometry.get("size").split()]))
    boxes.append(("openflex_grasp_safety_floor", [0.0, 0.0, -0.05], [20.0, 20.0, 0.1]))
    return boxes


class SafetyProbe(Node):
    def __init__(self):
        super().__init__("openflex_manipulation_safety_probe", start_parameter_services=False)
        self.samples = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscriptions_keepalive = [self.create_subscription(message_type, topic,
            lambda message, key=key: self.samples.__setitem__(key, (deepcopy(message), time.monotonic())), qos_profile_sensor_data)
            for key, message_type, topic in (
                ("joints", JointState, "/joint_states"), ("odometry", Odometry, "/swerve_drive_controller/odom"),
                ("diagnostic", String, "/thesis_sim/diagnostic_state"),
            )]
        self.apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.readback = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f"Unavailable planning service: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"No planning response: {client.srv_name}")
        return future.result()

    def fresh(self, key):
        message, received = self.samples.get(key, (None, 0))
        if message is None or time.monotonic() - received > 0.5:
            raise RuntimeError(f"No fresh telemetry: {key}")
        return message

    def probe(self, model_path, allow_attached=False):
        deadline = time.monotonic() + 5
        while not {'joints', 'odometry', 'diagnostic'}.issubset(self.samples) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        transform_deadline = time.monotonic() + 5
        while not (self.tf_buffer.can_transform('map', 'odom', Time())
                   and self.tf_buffer.can_transform('map', 'base_link', Time())):
            if time.monotonic() >= transform_deadline:
                raise RuntimeError('Measured map/odom/base transforms unavailable; planning not authorized')
            rclpy.spin_once(self, timeout_sec=0.05)
        joints = self.fresh("joints")
        odometry = self.fresh("odometry")
        diagnostic = json.loads(self.fresh("diagnostic").data)
        twist = odometry.twist.twist
        if (diagnostic.get("safety_stop") is not None or (diagnostic.get("attached_arm") is not None and not allow_attached)
                or math.hypot(twist.linear.x, twist.linear.y) > 0.02 or abs(twist.angular.z) > 0.03):
            raise RuntimeError("Robot must be stopped, unattached and free of safety stops")
        apply = ApplyPlanningScene.Request()
        apply.scene.is_diff = True
        apply.scene.robot_state.is_diff = True
        matrix_query = GetPlanningScene.Request()
        matrix_query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self.call(self.readback, matrix_query).scene.allowed_collision_matrix
        matrix.entry_names, rows = allow_wheel_floor_support(matrix.entry_names,
            [row.enabled for row in matrix.entry_values], 'openflex_grasp_safety_floor')
        matrix.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
        apply.scene.allowed_collision_matrix = matrix
        for name, position, dimensions in collision_boxes(model_path):
            collision = CollisionObject()
            collision.header.frame_id = "map"
            collision.id, collision.operation = name, CollisionObject.ADD
            collision.pose.orientation.w = 1.0
            primitive = SolidPrimitive()
            primitive.type, primitive.dimensions = SolidPrimitive.BOX, dimensions
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.w = 1.0
            collision.primitives, collision.primitive_poses = [primitive], [pose]
            apply.scene.world.collision_objects.append(collision)
        applied = self.call(self.apply, apply)
        if not applied.success:
            raise RuntimeError("Real planning scene application failed")
        query = GetPlanningScene.Request()
        query.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                       PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        scene = self.call(self.readback, query).scene
        readback_matrix = scene.allowed_collision_matrix
        floor_index = readback_matrix.entry_names.index('openflex_grasp_safety_floor')
        wheels = {'fl_wheel_link', 'fr_wheel_link', 'bl_wheel_link', 'br_wheel_link'}
        if any(bool(readback_matrix.entry_values[index].enabled[floor_index]) != (name in wheels)
               or bool(readback_matrix.entry_values[floor_index].enabled[index]) != (name in wheels)
               for index, name in enumerate(readback_matrix.entry_names)):
            raise RuntimeError('Only four intended wheel-floor support contact pairs may be allowed')
        requested_ids = {collision.id for collision in apply.scene.world.collision_objects}
        received_ids = {collision.id for collision in scene.world.collision_objects}
        if not requested_ids.issubset(received_ids):
            raise RuntimeError("Planning scene readback missing obstacle geometry")
        def to_map(pose):
            if pose.header.frame_id == "map":
                return pose
            from tf2_geometry_msgs import do_transform_pose_stamped

            transform = self.tf_buffer.lookup_transform("map", pose.header.frame_id, Time())
            return do_transform_pose_stamped(pose, transform)

        expected_in_map = []
        for collision in apply.scene.world.collision_objects:
            converted = deepcopy(collision)
            expected_in_map.append(converted)
        if not scene_readback_matches(expected_in_map, scene.world.collision_objects, to_map):
            raise RuntimeError("Planning scene readback geometry or measured frame transform differs")
        state = RobotState()
        state.joint_state = deepcopy(joints)
        state.multi_dof_joint_state.header.frame_id = "odom"
        state.multi_dof_joint_state.header.stamp = odometry.header.stamp
        state.multi_dof_joint_state.joint_names = ["base_planar_joint"]
        transform = Transform()
        transform.translation.x = odometry.pose.pose.position.x
        transform.translation.y = odometry.pose.pose.position.y
        transform.rotation = deepcopy(odometry.pose.pose.orientation)
        state.multi_dof_joint_state.transforms = [transform]
        validity = GetStateValidity.Request()
        validity.group_name = ""
        validity.robot_state = state
        checked = self.call(self.validity, validity)
        return {
            "trajectory_execution": False, "robot_motion_executed": False,
            "scene_applied": True, "scene_readback_ids": sorted(received_ids),
            "scene_geometry_and_frame_readback_verified": True,
            "allowed_support_contacts": [[name, 'openflex_grasp_safety_floor'] for name in sorted(wheels)],
            "scope": "measured_start_state_all_robot_links_and_scene_not_grasp_trajectory",
            "state_valid": bool(checked.valid), "contacts": message_to_ordereddict(checked)["contacts"],
            "joint_sample": message_to_ordereddict(joints), "odometry_sample": message_to_ordereddict(odometry),
            "simulator_diagnostic": diagnostic, "validity_request": message_to_ordereddict(validity),
            "planning_scene_readback": message_to_ordereddict(scene),
            "grasp_execution_authorized": False,
            "remaining_requirements": ["fresh memory and refined grasps", "TC-WAM and full-robot IK validity",
                                       "calibrated simulation box and scene readback", "every planned trajectory checked",
                                       "MoveIt exclusive control and base lock", "independent retained-object verification",
                                       "replace legacy teleport placement before full delivery"],
        }


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "83":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=83")
    output = ROOT / "local_dependencies/openflex_roboos/runs" / datetime.now().strftime("manipulation_preflight_%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    model_path = ROOT / "local_dependencies/openflex_roboos/readonly_scene.xml"
    rclpy.init()
    node = SafetyProbe()
    try:
        report = node.probe(model_path)
        report["model_sha256"] = hashlib.sha256(model_path.read_bytes()).hexdigest()
        report["passed_start_state_check"] = report["state_valid"]
    except Exception as error:
        report = {"passed_start_state_check": False, "grasp_execution_authorized": False, "error": str(error)}
        raise
    finally:
        (output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(output / "result.json")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
