"""Safety supervisor for ACT base-only alignment into a bound ready region."""

from copy import deepcopy
import time

from nav_msgs.msg import Odometry
from openflex_thesis_msgs.action import ExecuteLocalAlignment
from openflex_thesis_msgs.msg import AffordanceState, ControlMode, ObjectMemory, PolicyControl
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .async_tools import ros_sleep


class LocalAlignmentSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("local_alignment_supervisor")
        self.declare_parameter("action_name", "/thesis/execute_local_alignment")
        self.declare_parameter("policy_control_topic", "/thesis/act/policy_control")
        self.declare_parameter("affordance_topic", "/thesis/tc_wam/state")
        self.declare_parameter("memory_topic", "/thesis/object_memory")
        self.declare_parameter("control_mode_topic", "/thesis/control_mode")
        self.declare_parameter("measured_velocity_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("require_physical_affordance", True)
        self.declare_parameter("stream_timeout_sec", 0.50)
        self.declare_parameter("measured_timeout_sec", 0.25)
        self.declare_parameter("ready_confirm_sec", 0.40)
        self.declare_parameter("ready_linear_mps", 0.03)
        self.declare_parameter("ready_angular_rps", 0.04)
        self.declare_parameter("default_timeout_sec", 30.0)
        self.callback_group = ReentrantCallbackGroup()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.policy_publisher = self.create_publisher(
            PolicyControl, str(self.get_parameter("policy_control_topic").value), qos
        )
        self.create_subscription(
            AffordanceState,
            str(self.get_parameter("affordance_topic").value),
            self.on_affordance,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            ObjectMemory,
            str(self.get_parameter("memory_topic").value),
            self.on_memory,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            ControlMode,
            str(self.get_parameter("control_mode_topic").value),
            self.on_control_mode,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("measured_velocity_topic").value),
            self.on_odometry,
            20,
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            ExecuteLocalAlignment,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_affordance = None
        self.latest_affordance_receipt = 0.0
        self.latest_memory = None
        self.latest_memory_receipt = 0.0
        self.latest_odometry = None
        self.latest_odometry_receipt = 0.0
        self.control_mode = ControlMode.HOLD
        self.active_episode_id = 0
        self.busy = False

    def on_affordance(self, msg: AffordanceState) -> None:
        self.latest_affordance = deepcopy(msg)
        self.latest_affordance_receipt = time.monotonic()

    def on_memory(self, msg: ObjectMemory) -> None:
        self.latest_memory = deepcopy(msg)
        self.latest_memory_receipt = time.monotonic()

    def on_control_mode(self, msg: ControlMode) -> None:
        self.control_mode = int(msg.mode)

    def on_odometry(self, msg: Odometry) -> None:
        self.latest_odometry = deepcopy(msg)
        self.latest_odometry_receipt = time.monotonic()

    def goal_callback(self, _request) -> GoalResponse:
        return GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT

    def episode_origin(self, frame_id: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id, str(self.get_parameter("base_frame").value), Time()
            )
        except TransformException as error:
            self.get_logger().error(f"cannot reset ACT episode origin: {error}")
            return None
        from geometry_msgs.msg import PoseStamped

        origin = PoseStamped()
        origin.header.stamp = self.get_clock().now().to_msg()
        origin.header.frame_id = frame_id
        origin.pose.position.x = transform.transform.translation.x
        origin.pose.position.y = transform.transform.translation.y
        origin.pose.position.z = transform.transform.translation.z
        origin.pose.orientation = deepcopy(transform.transform.rotation)
        return origin

    def publish_policy(self, goal, enabled: bool, reason: str, origin=None) -> None:
        msg = PolicyControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.enabled = enabled
        msg.reset_episode = bool(enabled)
        msg.episode_id = self.active_episode_id
        msg.object_id = goal.target.object_id
        msg.policy_variant = goal.policy_variant
        msg.map_version = goal.affordance.map_version
        msg.action_space = PolicyControl.BASE_TWIST
        msg.active_arm = goal.plan.active_arm
        msg.selected_grasp_id = goal.plan.grasp_id
        msg.selected_lift_position = goal.plan.lift_position
        msg.planning_group = goal.plan.planning_group
        if origin is not None:
            msg.episode_origin = deepcopy(origin)
        msg.target_base_pose = deepcopy(goal.plan.best_base_pose)
        msg.ready_region = deepcopy(goal.plan.ready_region)
        msg.affordance_context = list(goal.affordance.compact_state)
        msg.reason = reason
        self.policy_publisher.publish(msg)

    def current_contract_valid(self, goal) -> bool:
        now = time.monotonic()
        memory = self.latest_memory
        affordance = self.latest_affordance
        streams_fresh = bool(
            now - self.latest_memory_receipt <= float(self.get_parameter("stream_timeout_sec").value)
            and now - self.latest_affordance_receipt <= float(self.get_parameter("stream_timeout_sec").value)
        )
        return bool(
            streams_fresh
            and self.control_mode == ControlMode.ACT
            and memory is not None
            and affordance is not None
            and memory.object_id == goal.target.object_id
            and affordance.object_id == goal.target.object_id
            and memory.map_version == goal.target.map_version == affordance.map_version
            and goal.plan.object_id == goal.target.object_id
            and goal.plan.map_version == goal.target.map_version
            and affordance.ready_region_id == goal.plan.ready_region.region_id
            and affordance.selected_grasp_id == goal.plan.grasp_id
            and affordance.active_arm == goal.plan.active_arm
            and affordance.planning_group == goal.plan.planning_group
            and abs(affordance.selected_lift_position - goal.plan.lift_position) <= 1.0e-4
            and memory.visible
            and memory.grasp_authorized
            and affordance.valid
            and (
                affordance.physical_evaluation
                or not bool(self.get_parameter("require_physical_affordance").value)
            )
        )

    def measured_low_speed(self) -> bool:
        if (
            self.latest_odometry is None
            or time.monotonic() - self.latest_odometry_receipt
            > float(self.get_parameter("measured_timeout_sec").value)
        ):
            return False
        twist = self.latest_odometry.twist.twist
        linear = float(self.get_parameter("ready_linear_mps").value)
        angular = float(self.get_parameter("ready_angular_rps").value)
        return bool(
            abs(twist.linear.x) <= linear
            and abs(twist.linear.y) <= linear
            and abs(twist.angular.z) <= angular
        )

    async def execute(self, goal_handle):
        self.busy = True
        goal = goal_handle.request
        started = time.monotonic()
        ready_since = None
        try:
            if not self.current_contract_valid(goal):
                return self.abort(goal_handle, 30, "ACT alignment contract is invalid", True)
            origin = self.episode_origin(goal.target.header.frame_id)
            if origin is None:
                return self.abort(goal_handle, 31, "ACT episode origin unavailable", True)
            self.active_episode_id = int(self.get_clock().now().nanoseconds)
            self.publish_policy(goal, True, "base-only local alignment authorized", origin)
            timeout = goal.timeout_sec or float(self.get_parameter("default_timeout_sec").value)
            while time.monotonic() - started <= timeout:
                if goal_handle.is_cancel_requested:
                    self.publish_policy(goal, False, "alignment canceled")
                    goal_handle.canceled()
                    result = ExecuteLocalAlignment.Result()
                    result.recoverable = True
                    result.message = "canceled"
                    return result
                if not self.current_contract_valid(goal):
                    self.publish_policy(goal, False, "object or conditioned map invalidated")
                    return self.abort(goal_handle, 32, "alignment contract changed", True)
                affordance = self.latest_affordance
                ready_now = bool(affordance.ready and self.measured_low_speed())
                if ready_now and ready_since is None:
                    ready_since = time.monotonic()
                elif not ready_now:
                    ready_since = None
                feedback = ExecuteLocalAlignment.Feedback()
                feedback.phase = "act_base_alignment"
                feedback.elapsed_sec = float(time.monotonic() - started)
                feedback.current_score = affordance.current_score
                feedback.ready = affordance.ready
                goal_handle.publish_feedback(feedback)
                if (
                    ready_since is not None
                    and time.monotonic() - ready_since
                    >= float(self.get_parameter("ready_confirm_sec").value)
                ):
                    self.publish_policy(goal, False, "bound ready region reached")
                    goal_handle.succeed()
                    result = ExecuteLocalAlignment.Result()
                    result.success = True
                    result.ready_reached = True
                    result.final_map_version = goal.target.map_version
                    result.message = "ACT reached the selected grasp-conditioned ready region"
                    return result
                await ros_sleep(self, 0.05)
            self.publish_policy(goal, False, "alignment timeout")
            return self.abort(goal_handle, 33, "ACT alignment timeout", True)
        finally:
            if self.active_episode_id:
                self.publish_policy(goal, False, "alignment supervisor terminal cleanup")
                self.active_episode_id = 0
            self.busy = False

    @staticmethod
    def abort(goal_handle, code: int, message: str, recoverable: bool):
        goal_handle.abort()
        result = ExecuteLocalAlignment.Result()
        result.recoverable = recoverable
        result.error_code = code
        result.message = message
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalAlignmentSupervisorNode()
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
