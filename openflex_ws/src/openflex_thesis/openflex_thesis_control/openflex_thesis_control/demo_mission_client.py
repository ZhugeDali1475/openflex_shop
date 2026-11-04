"""Send one deterministic goal to the offline mission smoke test."""

from openflex_thesis_msgs.action import RunPickDeliver
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class DemoMissionClient(Node):
    def __init__(self) -> None:
        super().__init__("demo_mission_client")
        self.client = ActionClient(self, RunPickDeliver, "/thesis/run_pick_deliver")
        self.sent = False
        self.create_timer(0.2, self.try_send)

    def try_send(self) -> None:
        if self.sent or not self.client.server_is_ready():
            return
        self.sent = True
        goal = RunPickDeliver.Goal()
        goal.task_id = "OFFLINE_SMOKE_TEST_DO_NOT_USE_AS_EVIDENCE"
        goal.object_id = "thesis_item_01"
        goal.pickup_observation_pose.header.frame_id = "map"
        goal.pickup_observation_pose.pose.orientation.w = 1.0
        goal.delivery_pose.header.frame_id = "map"
        goal.delivery_pose.pose.position.x = 0.10
        goal.delivery_pose.pose.orientation.w = 1.0
        goal.active_arm = RunPickDeliver.Goal.ARM_RIGHT
        goal.policy_variant = "tc_wam_ready_region_act_smoke_test"
        goal.coarse_top_k = 5
        goal.refined_top_k = 3
        goal.max_recovery_attempts = 1
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal)

    def on_feedback(self, feedback) -> None:
        msg = feedback.feedback
        self.get_logger().info(f"phase={msg.phase} detail={msg.detail}")

    def on_goal(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("OFFLINE_MISSION_REJECTED")
            rclpy.shutdown()
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future) -> None:
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f"OFFLINE_MISSION_SUCCESS duration={result.total_duration_sec:.3f}s"
            )
        else:
            self.get_logger().error(
                f"OFFLINE_MISSION_FAILED code={result.error_code} message={result.message}"
            )
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoMissionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
