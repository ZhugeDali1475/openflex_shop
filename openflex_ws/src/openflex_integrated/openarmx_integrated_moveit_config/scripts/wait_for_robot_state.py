#!/usr/bin/env python3

"""Exit after the active robot publishes joint states and odom -> base_link."""

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class RobotStateReadiness(Node):
    def __init__(self):
        super().__init__("wait_for_robot_state")
        self.declare_parameter("require_controllers", True)
        self._require_controllers = bool(
            self.get_parameter("require_controllers").value
        )
        self._have_joint_states = False
        self._have_odom_tf = False
        self._controllers_ready = not self._require_controllers
        self._controller_future = None
        self._last_status_log_ns = 0
        self._required_controllers = {
            "swerve_drive_controller",
            "left_forward_position_controller",
            "right_forward_position_controller",
            "head_forward_position_controller",
            "lift_state_controller",
        }
        self._controller_list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )
        self._timer = self.create_timer(0.1, self._check_ready)
        self.get_logger().info(
            "Waiting for active chassis/arm/lift controllers, /joint_states, and TF "
            "odom -> base_link before starting MoveIt"
        )

    def _on_joint_states(self, message: JointState) -> None:
        if not self._have_joint_states:
            self.get_logger().info(
                f"Joint states are ready ({len(message.name)} joints)"
            )
        self._have_joint_states = True

    def _check_ready(self) -> None:
        if self._require_controllers:
            self._poll_controllers()
        if not self._have_odom_tf:
            try:
                self._tf_buffer.lookup_transform(
                    "odom",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception:
                pass
            else:
                self._have_odom_tf = True
                self.get_logger().info("TF odom -> base_link is ready")

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_status_log_ns >= 2_000_000_000:
            controller_status = (
                "ready"
                if self._controllers_ready
                else ("waiting" if self._require_controllers else "not_required")
            )
            self.get_logger().info(
                "Robot state readiness: "
                f"controllers={controller_status}, "
                f"joint_states={'ready' if self._have_joint_states else 'waiting'}, "
                f"odom_tf={'ready' if self._have_odom_tf else 'waiting'}"
            )
            self._last_status_log_ns = now_ns

        if not (
            self._controllers_ready
            and self._have_joint_states
            and self._have_odom_tf
        ):
            return
        self.get_logger().info("Robot state is ready")
        self._timer.cancel()
        self.destroy_node()
        raise SystemExit(0)

    def _poll_controllers(self) -> None:
        future = self._controller_future
        if future is not None and future.done():
            self._controller_future = None
            try:
                response = future.result()
            except Exception:
                response = None
            if response is not None:
                states = {
                    controller.name: controller.state
                    for controller in response.controller
                }
                ready = all(
                    states.get(name) == "active"
                    for name in self._required_controllers
                )
                if ready and not self._controllers_ready:
                    self.get_logger().info("Required position controllers are active")
                self._controllers_ready = ready
        if (
            self._controller_future is None
            and self._controller_list_client.service_is_ready()
        ):
            self._controller_future = self._controller_list_client.call_async(
                ListControllers.Request()
            )


def main() -> int:
    rclpy.init()
    node = RobotStateReadiness()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
