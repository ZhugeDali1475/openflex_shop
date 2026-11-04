"""Fail-closed action/topic gateway for an external AnyGrasp inference process."""

from copy import deepcopy
import time
import uuid

from openflex_thesis_msgs.action import GenerateGrasps
from openflex_thesis_msgs.msg import AnyGraspRequest, GraspCandidateArray
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from .geometry import clamp


class AnyGraspGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("anygrasp_gateway")
        self.declare_parameter("action_name", "/thesis/generate_grasps")
        self.declare_parameter("request_topic", "/thesis/anygrasp/request")
        self.declare_parameter("response_topic", "/thesis/anygrasp/candidates")
        self.declare_parameter("default_timeout_sec", 8.0)
        self.callback_group = ReentrantCallbackGroup()
        self.request_publisher = self.create_publisher(
            AnyGraspRequest, str(self.get_parameter("request_topic").value), 10
        )
        self.create_subscription(
            GraspCandidateArray,
            str(self.get_parameter("response_topic").value),
            self.on_candidates,
            10,
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            GenerateGrasps,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.responses = {}
        self.busy = False

    def goal_callback(self, request: GenerateGrasps.Goal) -> GoalResponse:
        valid = bool(
            request.target.object_id
            and request.target.header.frame_id
            and request.stage in (GenerateGrasps.Goal.COARSE, GenerateGrasps.Goal.REFINED)
            and request.top_k > 0
        )
        return GoalResponse.ACCEPT if not self.busy and valid else GoalResponse.REJECT

    def on_candidates(self, msg: GraspCandidateArray) -> None:
        if msg.request_id:
            self.responses[msg.request_id] = (deepcopy(msg), time.monotonic())

    @staticmethod
    def valid_grasp(grasp, frame_id: str, stage: int) -> bool:
        q = grasp.grasp_pose.pose.orientation
        pre_q = grasp.pregrasp_pose.pose.orientation
        return bool(
            grasp.stage == stage
            and grasp.grasp_pose.header.frame_id == frame_id
            and grasp.pregrasp_pose.header.frame_id == frame_id
            and q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5
            and pre_q.x * pre_q.x + pre_q.y * pre_q.y + pre_q.z * pre_q.z + pre_q.w * pre_q.w > 0.5
        )

    async def execute(self, goal_handle):
        self.busy = True
        request_id = uuid.uuid4().hex
        try:
            goal = goal_handle.request
            trigger = AnyGraspRequest()
            trigger.header.stamp = self.get_clock().now().to_msg()
            trigger.header.frame_id = goal.target.header.frame_id
            trigger.request_id = request_id
            trigger.target = deepcopy(goal.target)
            trigger.stage = goal.stage
            trigger.active_arm = goal.active_arm
            trigger.top_k = goal.top_k
            self.request_publisher.publish(trigger)
            timeout = goal.timeout_sec or float(self.get_parameter("default_timeout_sec").value)
            started = time.monotonic()
            while time.monotonic() - started <= timeout:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = GenerateGrasps.Result()
                    result.message = "canceled"
                    return result
                response_entry = self.responses.pop(request_id, None)
                if response_entry is None:
                    await self.sleep(0.02)
                    continue
                response, _ = response_entry
                if (
                    response.object_id != goal.target.object_id
                    or response.map_version != goal.target.map_version
                    or response.stage != goal.stage
                ):
                    return self.abort(goal_handle, 61, "AnyGrasp response contract mismatch")
                if goal.require_physical_perception and not response.physical_perception:
                    return self.abort(goal_handle, 62, "physical AnyGrasp result required")
                unique = {}
                for grasp in response.grasps:
                    if goal.require_physical_perception and not grasp.physical_perception:
                        continue
                    if self.valid_grasp(grasp, goal.target.header.frame_id, goal.stage):
                        grasp.quality = clamp(grasp.quality, 0.0, 1.0)
                        unique[int(grasp.grasp_id)] = grasp
                ordered = sorted(unique.values(), key=lambda item: (-item.quality, item.grasp_id))
                response.grasps = ordered[: int(goal.top_k)]
                if not response.grasps:
                    return self.abort(goal_handle, 63, "AnyGrasp returned no valid target grasps")
                response.header.stamp = self.get_clock().now().to_msg()
                goal_handle.succeed()
                result = GenerateGrasps.Result()
                result.success = True
                result.grasps = response
                result.message = "AnyGrasp candidates validated"
                return result
            return self.abort(goal_handle, 64, "AnyGrasp response timeout")
        finally:
            self.responses.pop(request_id, None)
            self.busy = False

    async def sleep(self, duration_sec: float) -> None:
        future = Future()
        timer = self.create_timer(duration_sec, lambda: future.set_result(True))
        try:
            await future
        finally:
            self.destroy_timer(timer)

    @staticmethod
    def abort(goal_handle, code: int, message: str):
        goal_handle.abort()
        result = GenerateGrasps.Result()
        result.error_code = code
        result.message = message
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnyGraspGatewayNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
