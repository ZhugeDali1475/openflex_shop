"""Send one parameterized pick-deliver goal to the thesis simulator stack."""

from openflex_thesis_msgs.action import RunPickDeliver
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class SimMissionClient(Node):
    def __init__(self) -> None:
        super().__init__("thesis_mujoco_mission_client")
        self.declare_parameter("task_id", "MUJOCO_TRIAL_001")
        self.declare_parameter("object_id", "thesis_item_01")
        self.declare_parameter("active_arm", 2)
        self.declare_parameter("pickup_observation_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("pickup_navigation_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("delivery_pose", [-0.85, -0.80, 0.0])
        self.declare_parameter("coarse_top_k", 5)
        self.declare_parameter("refined_top_k", 5)
        self.declare_parameter("max_recovery_attempts", 2)
        self.client = ActionClient(self, RunPickDeliver, "/thesis/run_pick_deliver")
        self.sent = False
        self.create_timer(0.25, self.try_send)

    def try_send(self) -> None:
        if self.sent or not self.client.server_is_ready():
            return
        self.sent = True
        pickup = [float(value) for value in self.get_parameter("pickup_observation_pose").value]
        pickup_navigation = [float(value) for value in self.get_parameter("pickup_navigation_pose").value]
        delivery = [float(value) for value in self.get_parameter("delivery_pose").value]
        if len(pickup) != 3 or len(delivery) != 3:
            raise ValueError("pickup_observation_pose and delivery_pose need [x, y, yaw]")
        if len(pickup_navigation) != 3:
            raise ValueError("pickup_navigation_pose needs [x, y, yaw]")
        goal = RunPickDeliver.Goal()
        goal.task_id = str(self.get_parameter("task_id").value)
        goal.object_id = str(self.get_parameter("object_id").value)
        goal.pickup_observation_pose.header.frame_id = "map"
        goal.pickup_observation_pose.pose.position.x = pickup_navigation[0]
        goal.pickup_observation_pose.pose.position.y = pickup_navigation[1]
        goal.pickup_observation_pose.pose.orientation.z = __import__("math").sin(0.5 * pickup_navigation[2])
        goal.pickup_observation_pose.pose.orientation.w = __import__("math").cos(0.5 * pickup_navigation[2])
        goal.delivery_pose.header.frame_id = "map"
        goal.delivery_pose.pose.position.x = delivery[0]
        goal.delivery_pose.pose.position.y = delivery[1]
        goal.delivery_pose.pose.orientation.z = __import__("math").sin(0.5 * delivery[2])
        goal.delivery_pose.pose.orientation.w = __import__("math").cos(0.5 * delivery[2])
        goal.active_arm = int(self.get_parameter("active_arm").value)
        goal.policy_variant = "scripted_local_baseline_NOT_ACT"
        goal.coarse_top_k = int(self.get_parameter("coarse_top_k").value)
        goal.refined_top_k = int(self.get_parameter("refined_top_k").value)
        goal.max_recovery_attempts = int(
            self.get_parameter("max_recovery_attempts").value
        )
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal)

    def on_feedback(self, wrapper) -> None:
        feedback = wrapper.feedback
        self.get_logger().info(f"phase={feedback.phase} detail={feedback.detail}")

    def on_goal(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MUJOCO_MISSION_REJECTED")
            rclpy.shutdown()
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future) -> None:
        result = future.result().result
        self.get_logger().info("RECOVERY_TASK_RESULT " + json.dumps({
            "task_id": str(self.get_parameter("task_id").value), "success": bool(result.success),
            "error_code": int(result.error_code), "duration_sec": float(result.total_duration_sec),
            "recovery_attempts": int(result.recovery_attempts), "message": result.message,
        }))
        if result.success:
            self.get_logger().info(
                f"MUJOCO_MISSION_SUCCESS duration={result.total_duration_sec:.3f}s"
            )
        else:
            self.get_logger().error(
                f"MUJOCO_MISSION_FAILED code={result.error_code} message={result.message}"
            )
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimMissionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
