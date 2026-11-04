"""MuJoCo ground-truth grasp backend for the existing AnyGrasp gateway."""

from copy import deepcopy
import math

from openflex_thesis_msgs.msg import (
    AnyGraspRequest,
    GraspCandidate,
    GraspCandidateArray,
)
import rclpy
from rclpy.node import Node


class OracleAnyGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_oracle_anygrasp")
        self.declare_parameter("request_topic", "/thesis/anygrasp/request")
        self.declare_parameter("response_topic", "/thesis/anygrasp/candidates")
        self.declare_parameter("coarse_lateral_spread_m", 0.025)
        self.declare_parameter("refined_lateral_spread_m", 0.010)
        self.declare_parameter("pregrasp_distance_m", 0.12)
        self.publisher = self.create_publisher(
            GraspCandidateArray, str(self.get_parameter("response_topic").value), 10
        )
        self.create_subscription(
            AnyGraspRequest,
            str(self.get_parameter("request_topic").value),
            self.on_request,
            10,
        )
        self.get_logger().warn(
            "MUJOCO ORACLE AnyGrasp backend active; physical_perception is always false"
        )

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def on_request(self, request: AnyGraspRequest) -> None:
        output = GraspCandidateArray()
        output.header = deepcopy(request.target.header)
        output.header.stamp = self.get_clock().now().to_msg()
        output.request_id = request.request_id
        output.object_id = request.target.object_id
        output.map_version = request.target.map_version
        output.observation_sequence = request.target.observation_sequence
        output.stage = request.stage
        output.physical_perception = False
        count = max(1, int(request.top_k))
        pose = request.target.pose.pose
        yaw = self.yaw_from_quaternion(pose.orientation)
        # Object +x points from shelf to free space. A grasp moves from the
        # outward pregrasp toward the object along -object_x.
        outward = (math.cos(yaw), math.sin(yaw))
        inward = (-outward[0], -outward[1])
        lateral_axis = (-outward[1], outward[0])
        spread = float(
            self.get_parameter(
                "refined_lateral_spread_m"
                if request.stage == GraspCandidate.STAGE_REFINED
                else "coarse_lateral_spread_m"
            ).value
        )
        pregrasp_distance = float(self.get_parameter("pregrasp_distance_m").value)
        # Rotate TCP +z onto the horizontal inward approach vector.
        root_half = math.sqrt(0.5)
        tcp_orientation = (
            -inward[1] * root_half,
            inward[0] * root_half,
            0.0,
            root_half,
        )
        for index in range(count):
            centered = index - 0.5 * (count - 1)
            lateral = centered * spread
            height = 0.006 * ((index % 3) - 1)
            grasp = GraspCandidate()
            grasp.grasp_id = (
                100000 * int(request.stage)
                + 100 * (int(request.target.map_version) % 1000)
                + index
            )
            grasp.stage = request.stage
            grasp.grasp_pose.header = deepcopy(request.target.header)
            grasp.pregrasp_pose.header = deepcopy(request.target.header)
            gx = pose.position.x + lateral_axis[0] * lateral
            gy = pose.position.y + lateral_axis[1] * lateral
            gz = pose.position.z + height
            grasp.grasp_pose.pose.position.x = gx
            grasp.grasp_pose.pose.position.y = gy
            grasp.grasp_pose.pose.position.z = gz
            grasp.pregrasp_pose.pose.position.x = gx + outward[0] * pregrasp_distance
            grasp.pregrasp_pose.pose.position.y = gy + outward[1] * pregrasp_distance
            grasp.pregrasp_pose.pose.position.z = gz
            for target_pose in (grasp.grasp_pose.pose, grasp.pregrasp_pose.pose):
                target_pose.orientation.x = tcp_orientation[0]
                target_pose.orientation.y = tcp_orientation[1]
                target_pose.orientation.z = tcp_orientation[2]
                target_pose.orientation.w = tcp_orientation[3]
            grasp.approach.x = inward[0]
            grasp.approach.y = inward[1]
            grasp.width = 0.055
            grasp.quality = max(0.20, 0.98 - 0.06 * abs(centered))
            grasp.visibility_score = (
                0.98 if request.stage == GraspCandidate.STAGE_REFINED else 0.86
            )
            grasp.collision_score = 0.92
            grasp.observation_sequence = request.target.observation_sequence
            grasp.physical_perception = False
            grasp.source = "mujoco_ground_truth_oracle_NOT_PHYSICAL"
            output.grasps.append(grasp)
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OracleAnyGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

