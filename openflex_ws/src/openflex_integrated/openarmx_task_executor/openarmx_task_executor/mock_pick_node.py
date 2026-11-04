"""Clearly separated mock PickObject server for navigation-chain validation."""

import time

import rclpy
from openarmx_task_msgs.action import PickObject
from openarmx_task_msgs.msg import TaskError
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from .manipulation_contract import (
    normalize_arm_name,
    require_detection_id,
    validate_detection_arm,
)


class MockPickNode(Node):
    """Return a deterministic attached-object result without moving hardware."""

    def __init__(self) -> None:
        super().__init__("competition_mock_pick")
        self.declare_parameter("action_name", "/competition/mock_pick_object")
        self.declare_parameter("delay_sec", 0.50)
        self.declare_parameter("succeed", True)
        self.delay_sec = max(0.0, float(self.get_parameter("delay_sec").value))
        self.succeed = bool(self.get_parameter("succeed").value)
        self._active_goal = False
        self._server = ActionServer(
            self,
            PickObject,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().warn(
            "MOCK PICK ONLY: no real detection, SAM2, GraspNetBaseline, MoveIt "
            "grasp execution, gripper, or attachment verification is performed"
        )

    def goal_callback(self, request: PickObject.Goal) -> GoalResponse:
        if self._active_goal:
            return GoalResponse.REJECT
        try:
            arm_name = normalize_arm_name(request.arm_name)
            require_detection_id(request.detection_id)
            validate_detection_arm(request.detection_id, arm_name)
        except ValueError as exc:
            self.get_logger().error(f"Rejecting invalid mock pick goal: {exc}")
            return GoalResponse.REJECT
        self._active_goal = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _sleep(self, seconds: float) -> None:
        future = Future()
        timer_holder = []

        def wake() -> None:
            if timer_holder:
                timer_holder[0].cancel()
            if not future.done():
                future.set_result(None)

        timer_holder.append(self.create_timer(max(0.001, seconds), wake))
        try:
            await future
        finally:
            timer_holder[0].cancel()
            self.destroy_timer(timer_holder[0])

    async def execute_callback(self, goal_handle):
        result = PickObject.Result()
        started = time.monotonic()
        try:
            while time.monotonic() - started < self.delay_sec:
                if goal_handle.is_cancel_requested:
                    result.error_code = TaskError.CANCELLED
                    result.message = "mock pick cancelled"
                    goal_handle.canceled()
                    return result
                feedback = PickObject.Feedback()
                feedback.phase = "MOCK_GRASP"
                feedback.progress = min(
                    0.99,
                    (time.monotonic() - started) / max(self.delay_sec, 0.001),
                )
                goal_handle.publish_feedback(feedback)
                await self._sleep(min(0.05, max(0.001, self.delay_sec)))

            result.success = self.succeed
            result.object_attached = self.succeed
            result.error_code = TaskError.OK if self.succeed else TaskError.PICK_FAILED
            result.message = (
                "mock pick success: object attachment assumed"
                if self.succeed
                else "mock pick configured to fail"
            )
            if self.succeed:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            self._active_goal = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPickNode()
    executor = MultiThreadedExecutor(num_threads=2)
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


if __name__ == "__main__":
    main()
