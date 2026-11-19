"""Synthetic action dependencies for the offline mission smoke test only."""

from copy import deepcopy
import time

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.action import ExecuteLocalAlignment, ExecuteMoveItGrasp
from openflex_thesis_msgs.msg import FixedPlaceControl, PlaceStatus
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .async_tools import ros_sleep


class MockDependenciesNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_dependencies")
        self.declare_parameter("nav2_action", "/navigate_to_pose")
        self.declare_parameter(
            "local_alignment_action", "/thesis/execute_local_alignment"
        )
        self.declare_parameter("moveit_grasp_action", "/thesis/execute_moveit_grasp")
        self.declare_parameter("odometry_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("place_control_topic", "/thesis/place/control")
        self.declare_parameter("place_status_topic", "/thesis/place/status")
        self.declare_parameter("response_delay_sec", 0.20)
        self.callback_group = ReentrantCallbackGroup()
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("nav2_action").value),
            execute_callback=self.execute_navigation,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.alignment_server = ActionServer(
            self,
            ExecuteLocalAlignment,
            str(self.get_parameter("local_alignment_action").value),
            execute_callback=self.execute_local_alignment,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.moveit_server = ActionServer(
            self,
            ExecuteMoveItGrasp,
            str(self.get_parameter("moveit_grasp_action").value),
            execute_callback=self.execute_moveit_grasp,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odometry_topic").value), 20
        )
        self.place_publisher = self.create_publisher(
            PlaceStatus, str(self.get_parameter("place_status_topic").value), 20
        )
        self.create_subscription(
            FixedPlaceControl,
            str(self.get_parameter("place_control_topic").value),
            self.on_place_control,
            latched_qos,
        )
        self.pending_place = None
        self.place_ready_at = 0.0
        self.create_timer(0.05, self.on_timer)
        self.get_logger().warn("SYNTHETIC dependencies active; never use these logs as thesis evidence")

    async def execute_navigation(self, goal_handle):
        await ros_sleep(self, float(self.get_parameter("response_delay_sec").value))
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()

    async def execute_local_alignment(self, goal_handle):
        await ros_sleep(self, float(self.get_parameter("response_delay_sec").value))
        result = ExecuteLocalAlignment.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.recoverable = True
            result.message = "synthetic alignment canceled"
            return result
        goal_handle.succeed()
        result.success = True
        result.ready_reached = True
        result.final_map_version = goal_handle.request.target.map_version
        result.message = "synthetic ACT base alignment reached the bound ready region"
        return result

    async def execute_moveit_grasp(self, goal_handle):
        await ros_sleep(self, float(self.get_parameter("response_delay_sec").value))
        result = ExecuteMoveItGrasp.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.message = "synthetic MoveIt grasp canceled"
            return result
        goal = goal_handle.request
        goal_handle.succeed()
        result.success = True
        result.reposition_required = False
        result.grasp_verified = True
        result.transport_safe = True
        result.executed_grasp_id = (
            goal.refined_grasps.grasps[0].grasp_id
            if goal.refined_grasps.grasps
            else goal.coarse_plan.grasp_id
        )
        result.executed_lift_position = goal.coarse_plan.lift_position
        result.message = "synthetic MoveIt lift-arm grasp and transport motion completed"
        return result

    def on_place_control(self, msg: FixedPlaceControl) -> None:
        if msg.enabled:
            self.pending_place = deepcopy(msg)
            self.place_ready_at = time.monotonic() + float(
                self.get_parameter("response_delay_sec").value
            )
        else:
            self.pending_place = None

    def on_timer(self) -> None:
        now = self.get_clock().now()
        odometry = Odometry()
        odometry.header.stamp = now.to_msg()
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        self.odom_publisher.publish(odometry)
        if self.pending_place is not None and time.monotonic() >= self.place_ready_at:
            status = PlaceStatus()
            status.header.stamp = now.to_msg()
            status.object_id = self.pending_place.object_id
            status.destination_id = self.pending_place.destination_id
            status.target_reached = True
            status.released = True
            status.verified = True
            status.detail = "synthetic smoke-test release"
            self.place_publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockDependenciesNode()
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
