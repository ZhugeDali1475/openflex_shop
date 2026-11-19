"""Action contract for the fixed delivery motion owned by an external executor."""

from copy import deepcopy
import time

from openflex_thesis_msgs.action import ExecuteFixedPlace
from openflex_thesis_msgs.msg import ControlMode, FixedPlaceControl, PlaceStatus
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .async_tools import ros_sleep

LEFT_DESTINATION_ID = "sim_delivery_top_left"
_DESTINATION_ALIASES = {"delivery_zone": LEFT_DESTINATION_ID,
                        "sim_delivery_top_left": LEFT_DESTINATION_ID}


def canonical_destination_id(destination_id: str) -> str:
    try:
        return _DESTINATION_ALIASES[destination_id]
    except KeyError as error:
        raise ValueError(f"Unknown delivery destination: {destination_id}") from error


class FixedPlaceSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("fixed_place_supervisor")
        self.declare_parameter("action_name", "/thesis/execute_fixed_place")
        self.declare_parameter("place_control_topic", "/thesis/place/control")
        self.declare_parameter("place_status_topic", "/thesis/place/status")
        self.declare_parameter("control_mode_topic", "/thesis/control_mode")
        self.declare_parameter("status_timeout_sec", 0.50)
        self.declare_parameter("initial_status_grace_sec", 1.0)
        self.declare_parameter("default_timeout_sec", 30.0)
        self.callback_group = ReentrantCallbackGroup()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_publisher = self.create_publisher(
            FixedPlaceControl, str(self.get_parameter("place_control_topic").value), qos
        )
        self.create_subscription(
            PlaceStatus,
            str(self.get_parameter("place_status_topic").value),
            self.on_status,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            ControlMode,
            str(self.get_parameter("control_mode_topic").value),
            self.on_mode,
            qos,
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            ExecuteFixedPlace,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.latest_status = None
        self.status_receipt = 0.0
        self.mode = ControlMode.HOLD
        self.busy = False

    def on_status(self, msg: PlaceStatus) -> None:
        self.latest_status = deepcopy(msg)
        self.status_receipt = time.monotonic()

    def on_mode(self, msg: ControlMode) -> None:
        self.mode = int(msg.mode)

    def goal_callback(self, _request) -> GoalResponse:
        return GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT

    def command(self, goal, enabled: bool, reason: str) -> None:
        msg = FixedPlaceControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.enabled = enabled
        msg.object_id = goal.object_id
        msg.destination_id = canonical_destination_id(goal.destination_id or LEFT_DESTINATION_ID)
        msg.delivery_pose = deepcopy(goal.delivery_pose)
        msg.active_arm = goal.active_arm
        msg.reason = reason
        self.command_publisher.publish(msg)

    async def execute(self, goal_handle):
        self.busy = True
        goal = goal_handle.request
        started = time.monotonic()
        try:
            # Do not reuse a recent success from the previous delivery trial.
            self.latest_status = None
            self.status_receipt = 0.0
            if self.mode != ControlMode.PLACE:
                return self.abort(goal_handle, 60, "PLACE control mode is not active")
            self.command(goal, True, "fixed place authorized")
            timeout = goal.timeout_sec or float(self.get_parameter("default_timeout_sec").value)
            while time.monotonic() - started <= timeout:
                if goal_handle.is_cancel_requested:
                    self.command(goal, False, "place canceled")
                    goal_handle.canceled()
                    result = ExecuteFixedPlace.Result()
                    result.success = False
                    result.message = "canceled"
                    return result
                if self.mode != ControlMode.PLACE:
                    self.command(goal, False, "place mode revoked")
                    return self.abort(goal_handle, 61, "PLACE control mode revoked")
                feedback = ExecuteFixedPlace.Feedback()
                feedback.phase = "fixed_place_external_executor"
                feedback.elapsed_sec = float(time.monotonic() - started)
                goal_handle.publish_feedback(feedback)
                status = self.latest_status
                fresh = (
                    status is not None
                    and time.monotonic() - self.status_receipt
                    <= float(self.get_parameter("status_timeout_sec").value)
                )
                if fresh and status.object_id == goal.object_id:
                    expected_destination = canonical_destination_id(goal.destination_id or LEFT_DESTINATION_ID)
                    if status.destination_id != expected_destination:
                        self.command(goal, False, "place destination mismatch")
                        return self.abort(goal_handle, 64, "place status destination mismatch")
                    if status.failure_code:
                        self.command(goal, False, "place executor failed")
                        return self.abort(goal_handle, status.failure_code, status.detail)
                    if status.target_reached and status.released and status.verified:
                        self.command(goal, False, "release verified")
                        goal_handle.succeed()
                        result = ExecuteFixedPlace.Result()
                        result.success = True
                        result.release_verified = True
                        result.message = "fixed release verified"
                        return result
                if (
                    time.monotonic() - started
                    > float(self.get_parameter("initial_status_grace_sec").value)
                    and not fresh
                ):
                    self.command(goal, False, "place status stale")
                    return self.abort(goal_handle, 62, "place status stream stale")
                await ros_sleep(self, 0.05)
            self.command(goal, False, "place timeout")
            return self.abort(goal_handle, 63, "fixed place timeout")
        finally:
            self.busy = False

    @staticmethod
    def abort(goal_handle, code: int, message: str):
        goal_handle.abort()
        result = ExecuteFixedPlace.Result()
        result.success = False
        result.error_code = code
        result.message = message
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedPlaceSupervisorNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
