"""Offline/diagnostic chain: coarse AnyGrasp then conditioned TC-WAM."""

from copy import deepcopy

from openflex_thesis_msgs.action import ComputeTcWam, GenerateGrasps
from openflex_thesis_msgs.msg import ObjectMemory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class AutoComputeNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_compute")
        self.declare_parameter("memory_topic", "/thesis/object_memory")
        self.declare_parameter("anygrasp_action", "/thesis/generate_grasps")
        self.declare_parameter("compute_action", "/thesis/compute_tc_wam")
        self.declare_parameter("active_arm", 2)
        self.declare_parameter("top_k", 3)
        self.declare_parameter("evaluate_robustness", True)
        self.declare_parameter("require_physical_perception", True)
        self.declare_parameter("require_physical_evaluation", True)
        self.declare_parameter("timeout_sec", 60.0)
        self.anygrasp_client = ActionClient(
            self, GenerateGrasps, str(self.get_parameter("anygrasp_action").value)
        )
        self.compute_client = ActionClient(
            self, ComputeTcWam, str(self.get_parameter("compute_action").value)
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ObjectMemory, str(self.get_parameter("memory_topic").value), self.on_memory, qos
        )
        self.pending = False
        self.pending_memory = None
        self.last_requested_version = -1

    def on_memory(self, msg: ObjectMemory) -> None:
        if self.pending or not msg.visible or not msg.grasp_authorized:
            return
        if int(msg.map_version) == self.last_requested_version:
            return
        if not self.anygrasp_client.server_is_ready() or not self.compute_client.server_is_ready():
            self.get_logger().warn("AnyGrasp/TC-WAM actions are not ready", throttle_duration_sec=2.0)
            return
        goal = GenerateGrasps.Goal()
        goal.target = deepcopy(msg)
        goal.stage = GenerateGrasps.Goal.COARSE
        goal.active_arm = int(self.get_parameter("active_arm").value)
        goal.top_k = int(self.get_parameter("top_k").value)
        goal.require_physical_perception = bool(
            self.get_parameter("require_physical_perception").value
        )
        goal.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.pending = True
        self.pending_memory = deepcopy(msg)
        self.last_requested_version = int(msg.map_version)
        future = self.anygrasp_client.send_goal_async(goal)
        future.add_done_callback(self.on_anygrasp_goal)

    def on_anygrasp_goal(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.finish_with_error("coarse AnyGrasp request rejected")
            return
        handle.get_result_async().add_done_callback(self.on_anygrasp_result)

    def on_anygrasp_result(self, future) -> None:
        result = future.result().result
        if not result.success or self.pending_memory is None:
            self.finish_with_error(f"coarse AnyGrasp failed: {result.message}")
            return
        goal = ComputeTcWam.Goal()
        goal.target = deepcopy(self.pending_memory)
        goal.grasps = deepcopy(result.grasps)
        goal.active_arm = int(self.get_parameter("active_arm").value)
        goal.evaluate_robustness = bool(self.get_parameter("evaluate_robustness").value)
        goal.require_physical_evaluation = bool(
            self.get_parameter("require_physical_evaluation").value
        )
        goal.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.compute_client.send_goal_async(goal).add_done_callback(self.on_compute_goal)

    def on_compute_goal(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.finish_with_error("conditioned TC-WAM request rejected")
            return
        handle.get_result_async().add_done_callback(self.on_compute_result)

    def on_compute_result(self, future) -> None:
        result = future.result().result
        self.pending = False
        self.pending_memory = None
        if result.success:
            self.get_logger().info(
                f"TC-WAM v{result.plan.map_version}: grasp={result.plan.grasp_id}, "
                f"lift={result.plan.lift_position:.3f}, score={result.plan.total_score:.3f}, "
                f"physical={result.plan.physical_evaluation}"
            )
        else:
            self.get_logger().error(f"TC-WAM failed: {result.message}")

    def finish_with_error(self, message: str) -> None:
        self.pending = False
        self.pending_memory = None
        self.get_logger().error(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoComputeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
