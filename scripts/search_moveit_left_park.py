#!/usr/bin/env python3
"""Search a fixed left-arm park posture against real TC-WAM candidates."""

from collections import Counter
from copy import deepcopy
from itertools import product
import argparse
import json
import math
from pathlib import Path
import time
from typing import Dict, List, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from openflex_thesis_msgs.msg import AffordanceCandidateArray
from openflex_tc_wam.geometry import (
    quaternion_conjugate,
    quaternion_multiply,
    rotate_vector,
    subtract_vector,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import yaml


LEFT_JOINT_NAMES = [
    f"openarmx_left_joint{index}" for index in range(1, 8)
]
LEFT_LOWER = [-3.14, -2.60, -3.14, -2.80, -3.14, -2.20, -3.14]
LEFT_UPPER = [3.14, 2.60, 3.14, 2.80, 3.14, 2.20, 3.14]


def pose_components(pose):
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def left_margin(positions: Sequence[float]) -> float:
    margins = []
    for value, lower, upper in zip(positions, LEFT_LOWER, LEFT_UPPER):
        normalized = (float(value) - lower) / (upper - lower)
        margins.append(max(0.0, min(1.0, 2.0 * min(normalized, 1.0 - normalized))))
    return min(margins) if margins else 0.0


def candidate_joint_map(candidate) -> Dict[str, float]:
    names = [str(name) for name in candidate.joint_names]
    positions = [float(value) for value in candidate.grasp_joint_positions]
    return dict(zip(names, positions))


class LeftParkSearch(Node):
    def __init__(
        self,
        candidate_topic: str,
        validity_service: str,
        scene_service: str,
        shelf_scene_file: str,
        base_frame: str,
        candidate_timeout_sec: float,
    ):
        super().__init__("moveit_left_park_search")
        self.base_frame = base_frame
        self.candidate_timeout_sec = candidate_timeout_sec
        self.joint_state = None
        self.candidates_message = None
        self.candidates_receipt = 0.0

        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 20
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            AffordanceCandidateArray,
            candidate_topic,
            self.on_candidates,
            qos,
        )
        self.validity_client = self.create_client(
            GetStateValidity, validity_service
        )
        self.scene_client = self.create_client(ApplyPlanningScene, scene_service)
        with open(shelf_scene_file, "r", encoding="utf-8") as stream:
            self.shelf_scene = yaml.safe_load(stream) or {}

    def on_joint_state(self, message: JointState) -> None:
        self.joint_state = deepcopy(message)

    def on_candidates(self, message: AffordanceCandidateArray) -> None:
        self.candidates_message = deepcopy(message)
        self.candidates_receipt = time.monotonic()

    def wait_until_ready(self) -> None:
        deadline = time.monotonic() + self.candidate_timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.joint_state is not None
                and self.candidates_message is not None
                and self.validity_client.service_is_ready()
                and self.scene_client.service_is_ready()
            ):
                return
        missing = []
        if self.joint_state is None:
            missing.append("/joint_states")
        if self.candidates_message is None:
            missing.append("TC-WAM candidate message")
        if not self.validity_client.service_is_ready():
            missing.append("/check_state_validity")
        if not self.scene_client.service_is_ready():
            missing.append("/apply_planning_scene")
        raise RuntimeError("search prerequisites unavailable: " + ", ".join(missing))

    def select_candidates(
        self, active_arm: int, minimum_score: float, limit: int
    ) -> List[object]:
        selected = []
        for candidate in self.candidates_message.candidates:
            if int(candidate.active_arm) != active_arm:
                continue
            if not (
                candidate.evaluated
                and candidate.ik_valid
                and candidate.collision_free
            ):
                continue
            if float(candidate.total_score) < minimum_score:
                continue
            joints = candidate_joint_map(candidate)
            required = ["lift_joint", *[
                f"openarmx_right_joint{index}" for index in range(1, 8)
            ]]
            if any(name not in joints for name in required):
                continue
            selected.append(deepcopy(candidate))
        selected.sort(
            key=lambda item: (
                -float(item.total_score),
                -float(item.joint_margin),
                int(item.candidate_id),
            )
        )
        return selected[:limit]

    def scene_objects_in_candidate_base(self, candidate_pose):
        scene_frame = str(self.shelf_scene.get("frame_id", ""))
        if scene_frame != "map":
            raise ValueError(
                f"shelf scene frame must be 'map', got '{scene_frame}'"
            )
        base_position, base_rotation = pose_components(candidate_pose)
        base_inverse = quaternion_conjugate(base_rotation)
        output = []
        for specification in self.shelf_scene.get("collision_objects", []):
            if specification.get("shape") != "box":
                raise ValueError(
                    f"unsupported shelf shape: {specification.get('shape')}"
                )
            dimensions = [float(value) for value in specification["dimensions"]]
            position = tuple(float(value) for value in specification["position"])
            orientation = tuple(
                float(value)
                for value in specification.get("orientation_xyzw", [])
            )
            if len(dimensions) != 3 or len(position) != 3 or len(orientation) != 4:
                raise ValueError(
                    f"invalid shelf object: {specification.get('id', '')}"
                )
            relative_position = rotate_vector(
                base_inverse, subtract_vector(position, base_position)
            )
            relative_orientation = quaternion_multiply(
                base_inverse, orientation
            )
            collision = CollisionObject()
            collision.header.stamp = self.get_clock().now().to_msg()
            collision.header.frame_id = self.base_frame
            collision.id = str(specification["id"])
            collision.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = dimensions
            collision.primitives.append(primitive)
            pose_value = Pose()
            pose_value.position.x, pose_value.position.y, pose_value.position.z = (
                relative_position
            )
            (
                pose_value.orientation.x,
                pose_value.orientation.y,
                pose_value.orientation.z,
                pose_value.orientation.w,
            ) = relative_orientation
            collision.primitive_poses.append(pose_value)
            output.append(collision)
        if not output:
            raise ValueError("shelf scene contains no collision objects")
        return output

    def apply_scene(self, candidate_pose, timeout_sec: float) -> None:
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state = RobotState()
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = (
            self.scene_objects_in_candidate_base(candidate_pose)
        )
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("failed to apply candidate shelf scene")

    def clear_scene(self, timeout_sec: float) -> None:
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state = RobotState()
        request.scene.robot_state.is_diff = True
        for specification in self.shelf_scene.get("collision_objects", []):
            collision = CollisionObject()
            collision.header.frame_id = self.base_frame
            collision.id = str(specification["id"])
            collision.operation = CollisionObject.REMOVE
            request.scene.world.collision_objects.append(collision)
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

    def state_for(
        self, candidate, left_positions: Sequence[float], lift_position: float
    ) -> RobotState:
        positions = dict(
            zip(self.joint_state.name, self.joint_state.position)
        )
        positions.update(candidate_joint_map(candidate))
        positions["lift_joint"] = float(lift_position)
        positions.update(dict(zip(LEFT_JOINT_NAMES, left_positions)))
        state = RobotState()
        state.is_diff = False
        state.joint_state.name = sorted(positions)
        state.joint_state.position = [
            float(positions[name]) for name in state.joint_state.name
        ]
        return state

    def query(
        self,
        candidate,
        left_positions: Sequence[float],
        lift_position: float,
        group_name: str,
        timeout_sec: float,
    ) -> Tuple[bool, List[Tuple[str, str]]]:
        request = GetStateValidity.Request()
        request.robot_state = self.state_for(
            candidate, left_positions, lift_position
        )
        request.group_name = group_name
        future = self.validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            raise RuntimeError("state-validity service returned no response")
        contacts = [
            (contact.contact_body_1, contact.contact_body_2)
            for contact in response.contacts
        ]
        return bool(response.valid), contacts


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--candidate-topic", default="/thesis/tc_wam/candidates"
    )
    parser.add_argument("--group", default="right_arm_with_lift")
    parser.add_argument(
        "--validity-service", default="/check_state_validity"
    )
    parser.add_argument(
        "--scene-service", default="/apply_planning_scene"
    )
    default_scene = str(
        Path(get_package_share_directory("openflex_thesis_mujoco"))
        / "config"
        / "sim_shelf_scene.yaml"
    )
    parser.add_argument("--shelf-scene-file", default=default_scene)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--active-arm", type=int, default=2)
    parser.add_argument("--maximum-candidates", type=int, default=12)
    parser.add_argument("--minimum-candidate-score", type=float, default=0.0)
    parser.add_argument("--candidate-timeout-sec", type=float, default=30.0)
    parser.add_argument("--service-timeout-sec", type=float, default=3.0)
    parser.add_argument(
        "--lift-values", nargs="+", type=float, default=[-0.15, 0.0, 0.15]
    )
    parser.add_argument(
        "--q1-values",
        nargs="+",
        type=float,
        default=[-1.5, -1.0, -0.7, -0.4, -0.2, 0.0, 0.2, 0.5],
    )
    parser.add_argument(
        "--q2-values",
        nargs="+",
        type=float,
        default=[-1.2, -0.8, -0.5, -0.3, -0.15, 0.0],
    )
    parser.add_argument(
        "--q3-values",
        nargs="+",
        type=float,
        default=[-0.8, -0.4, 0.0, 0.4, 0.8],
    )
    parser.add_argument(
        "--q4-values",
        nargs="+",
        type=float,
        default=[1.8, 1.2, 0.6, 0.0, -0.6, -1.2, -1.8],
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = LeftParkSearch(
        args.candidate_topic,
        args.validity_service,
        args.scene_service,
        args.shelf_scene_file,
        args.base_frame,
        args.candidate_timeout_sec,
    )
    try:
        node.wait_until_ready()
        candidates = node.select_candidates(
            args.active_arm,
            args.minimum_candidate_score,
            args.maximum_candidates,
        )
        if not candidates:
            raise RuntimeError(
                "no evaluated, IK-valid, collision-free right-arm candidate"
            )
        print(
            f"CANDIDATES {len(candidates)} "
            f"source={node.candidates_message.evaluator_id} "
            f"physical={node.candidates_message.physical_evaluation}",
            flush=True,
        )
        left_spaces = [
            (q1, q2, q3, q4, 0.0, 0.0, 0.0)
            for q1, q2, q3, q4 in product(
                args.q1_values,
                args.q2_values,
                args.q3_values,
                args.q4_values,
            )
        ]
        feasible = list(left_spaces)
        coverage = {}
        contact_counts = Counter()
        started = time.monotonic()
        for candidate in candidates:
            node.apply_scene(candidate.base_pose, args.service_timeout_sec)
            for lift_position in args.lift_values:
                survivors = []
                for left_positions in feasible:
                    valid, contacts = node.query(
                        candidate,
                        left_positions,
                        lift_position,
                        args.group,
                        args.service_timeout_sec,
                    )
                    if valid:
                        survivors.append(left_positions)
                    else:
                        contact_counts.update(contacts)
                feasible = survivors
                key = f"{candidate.candidate_id}@{lift_position:.3f}"
                coverage[key] = len(feasible)
                print(
                    f"CHECK candidate={candidate.candidate_id} "
                    f"grasp={candidate.grasp_id} lift={lift_position:.3f} "
                    f"surviving_left_poses={len(feasible)}",
                    flush=True,
                )
                if not feasible:
                    break
            if not feasible:
                break
        if not feasible:
            print(
                "LEFT_PARK_NOT_FOUND contacts="
                + str(contact_counts.most_common(12)),
                flush=True,
            )
            return 1
        feasible.sort(
            key=lambda positions: (
                -left_margin(positions),
                sum(abs(value) for value in positions),
                positions,
            )
        )
        selected = feasible[0]
        result = {
            "candidate_source": node.candidates_message.evaluator_id,
            "physical_evaluation": bool(
                node.candidates_message.physical_evaluation
            ),
            "group": args.group,
            "shelf_scene_file": args.shelf_scene_file,
            "lift_values": args.lift_values,
            "candidate_count": len(candidates),
            "candidates": [
                {
                    "candidate_id": int(candidate.candidate_id),
                    "grasp_id": int(candidate.grasp_id),
                    "score": float(candidate.total_score),
                    "lift_position": float(candidate.lift_position),
                }
                for candidate in candidates
            ],
            "searched_left_poses": len(left_spaces),
            "feasible_left_poses": len(feasible),
            "coverage": coverage,
            "contact_counts": {
                " <-> ".join(contact): count
                for contact, count in contact_counts.items()
            },
            "left_park": [float(value) for value in selected],
            "left_joint_margin": left_margin(selected),
            "elapsed_sec": time.monotonic() - started,
        }
        print(
            "PARK_OK_LEFT "
            + str([round(value, 6) for value in selected]),
            flush=True,
        )
        print(
            f"park_search_checked={len(left_spaces)} "
            f"feasible={len(feasible)} "
            f"margin={result['left_joint_margin']:.6f}",
            flush=True,
        )
        rendered = json.dumps(result, indent=2, ensure_ascii=False)
        print(rendered, flush=True)
        if args.output:
            Path(args.output).write_text(rendered + "\n", encoding="utf-8")
        return 0
    except (RuntimeError, ValueError) as error:
        print(f"park_search_failed: {error}", flush=True)
        return 1
    finally:
        if node.scene_client.service_is_ready():
            try:
                node.clear_scene(args.service_timeout_sec)
            except Exception as error:
                print(f"failed_to_clear_shelf_scene: {error}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
