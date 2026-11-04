"""ReadOrder server: head motion, fresh RGB capture, shared Qwen, validation."""

from __future__ import annotations

import copy
import math
import os
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from openarmx_task_msgs.action import ReadOrder
from openarmx_task_msgs.msg import OrderLine, TaskError
from openflex_grasp_interfaces.action import QueryVlm
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from .competition_config import CompetitionConfig
from .head_pitch_motion import next_position_step, position_reached
from .receipt_recognition import (
    build_receipt_prompt,
    parse_receipt_response,
    validate_order_items,
)
from .shelf_inventory_db import ShelfInventoryDatabase


YAW_JOINT = "openarmx_head_yaw_joint"
PITCH_JOINT = "openarmx_head_pitch_joint"


class ShoppingTicketReaderNode(Node):
    def __init__(self) -> None:
        super().__init__("shopping_ticket_reader")
        share = get_package_share_directory("openarmx_task_executor")
        self.declare_parameter(
            "competition_config", os.path.join(share, "config", "competition.yaml")
        )
        self.declare_parameter("database_path", "")
        self.declare_parameter("action_name", "/read_delivery_order")
        self.declare_parameter("vlm_action_name", "/openflex_grasp/query_vlm")
        self.declare_parameter("head_image_topic", "/vision/color/image_raw")
        self.declare_parameter(
            "head_command_topic", "/head_forward_position_controller/commands"
        )
        self.declare_parameter("pitch_target_rad", math.radians(-70.0))
        self.declare_parameter("pitch_min_rad", -1.5708)
        self.declare_parameter("pitch_max_rad", 1.5708)
        self.declare_parameter("head_rate_hz", 20.0)
        self.declare_parameter("head_speed_rad_s", 0.5)
        self.declare_parameter("head_tolerance_rad", math.radians(1.5))
        self.declare_parameter("head_settle_sec", 0.3)
        self.declare_parameter("head_motion_timeout_sec", 8.0)
        self.declare_parameter("camera_wait_timeout_sec", 3.0)
        self.declare_parameter("expected_image_width", 1280)
        self.declare_parameter("expected_image_height", 720)
        self.declare_parameter("max_new_tokens", 256)
        self.declare_parameter("max_quantity", 9)

        config = CompetitionConfig.from_yaml(
            str(self.get_parameter("competition_config").value)
        )
        database_path = str(self.get_parameter("database_path").value).strip()
        database_path = database_path or config.inventory.database_path
        if not database_path:
            raise RuntimeError("shopping-ticket reader requires a database_path")
        self._database = ShelfInventoryDatabase(database_path)

        self._callbacks = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._latest_image: Optional[Image] = None
        self._image_sequence = 0
        self._vlm_goal_handle = None

        self._head_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("head_command_topic").value),
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            20,
            callback_group=self._callbacks,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("head_image_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
            callback_group=self._callbacks,
        )
        self._vlm_client = ActionClient(
            self,
            QueryVlm,
            str(self.get_parameter("vlm_action_name").value),
            callback_group=self._callbacks,
        )
        self._action = ActionServer(
            self,
            ReadOrder,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._callbacks,
        )
        self.get_logger().info(
            f"ReadOrder ready; SKU catalog={self._database.path}, target pitch=-70 deg"
        )

    def _joint_state_callback(self, message: JointState) -> None:
        with self._state_lock:
            for index, name in enumerate(message.name):
                if index < len(message.position):
                    self._joint_positions[str(name)] = float(message.position[index])

    def _image_callback(self, message: Image) -> None:
        with self._state_lock:
            self._latest_image = message
            self._image_sequence += 1

    def _goal(self, goal: ReadOrder.Goal):
        if not str(goal.station_id).strip():
            return GoalResponse.REJECT
        if int(goal.expected_distinct_skus) < 0:
            return GoalResponse.REJECT
        if not 0.0 <= float(goal.min_confidence) <= 1.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, _goal) -> CancelResponse:
        with self._state_lock:
            vlm_handle = self._vlm_goal_handle
        if vlm_handle is not None:
            vlm_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    @staticmethod
    def _feedback(handle, phase: str, raw_text: str = "") -> None:
        feedback = ReadOrder.Feedback()
        feedback.phase = phase
        feedback.raw_text = raw_text
        feedback.confidence = 0.0
        handle.publish_feedback(feedback)

    def _head_state(self) -> tuple[float, float]:
        with self._state_lock:
            if YAW_JOINT not in self._joint_positions:
                raise RuntimeError(f"joint state is missing {YAW_JOINT}")
            if PITCH_JOINT not in self._joint_positions:
                raise RuntimeError(f"joint state is missing {PITCH_JOINT}")
            return (
                float(self._joint_positions[YAW_JOINT]),
                float(self._joint_positions[PITCH_JOINT]),
            )

    def _publish_head(self, yaw: float, pitch: float) -> None:
        message = Float64MultiArray()
        message.data = [float(yaw), float(pitch)]
        self._head_pub.publish(message)

    def _move_pitch(
        self,
        yaw: float,
        start_pitch: float,
        target_pitch: float,
        handle,
        *,
        honor_cancel: bool,
    ) -> None:
        lower = float(self.get_parameter("pitch_min_rad").value)
        upper = float(self.get_parameter("pitch_max_rad").value)
        target_pitch = min(upper, max(lower, float(target_pitch)))
        rate_hz = max(1.0, float(self.get_parameter("head_rate_hz").value))
        period = 1.0 / rate_hz
        max_step = float(self.get_parameter("head_speed_rad_s").value) / rate_hz
        tolerance = float(self.get_parameter("head_tolerance_rad").value)
        settle_sec = max(0.0, float(self.get_parameter("head_settle_sec").value))
        deadline = time.monotonic() + float(
            self.get_parameter("head_motion_timeout_sec").value
        )
        command = float(start_pitch)
        stable_since: Optional[float] = None
        while time.monotonic() < deadline:
            if honor_cancel and handle.is_cancel_requested:
                raise RuntimeError("shopping-ticket read cancelled")
            command = next_position_step(command, target_pitch, max_step)
            self._publish_head(yaw, command)
            _, actual = self._head_state()
            if position_reached(actual, target_pitch, tolerance):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= settle_sec:
                    self._publish_head(yaw, target_pitch)
                    return
            else:
                stable_since = None
            time.sleep(period)
        raise RuntimeError(
            f"head pitch failed to reach {target_pitch:.4f} rad before safety timeout"
        )

    def _fresh_image(self, after_sequence: int, handle) -> Image:
        deadline = time.monotonic() + float(
            self.get_parameter("camera_wait_timeout_sec").value
        )
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                raise RuntimeError("shopping-ticket read cancelled")
            with self._state_lock:
                sequence = self._image_sequence
                image = self._latest_image
            if image is not None and sequence > after_sequence:
                width = int(self.get_parameter("expected_image_width").value)
                height = int(self.get_parameter("expected_image_height").value)
                if int(image.width) != width or int(image.height) != height:
                    raise RuntimeError(
                        f"head RGB profile is {image.width}x{image.height}; "
                        f"expected {width}x{height}"
                    )
                return copy.deepcopy(image)
            time.sleep(0.02)
        raise RuntimeError("no fresh head RGB frame arrived after pitch settled")

    def _wait_future(self, future, handle, *, cancel_downstream: bool = False):
        cancel_sent = False
        while not future.done():
            if handle.is_cancel_requested and cancel_downstream and not cancel_sent:
                with self._state_lock:
                    vlm_handle = self._vlm_goal_handle
                if vlm_handle is not None:
                    vlm_handle.cancel_goal_async()
                cancel_sent = True
            time.sleep(0.02)
        return future.result()

    def _query_feedback(self, handle, feedback_message) -> None:
        self._feedback(handle, str(feedback_message.feedback.phase))

    def _query_vlm(self, image: Image, prompt: str, handle):
        if not self._vlm_client.server_is_ready():
            raise RuntimeError("shared Qwen QueryVlm action is unavailable")
        goal = QueryVlm.Goal()
        goal.image = image
        goal.prompt = prompt
        goal.max_new_tokens = int(self.get_parameter("max_new_tokens").value)
        future = self._vlm_client.send_goal_async(
            goal, feedback_callback=lambda message: self._query_feedback(handle, message)
        )
        vlm_handle = self._wait_future(future, handle)
        if not vlm_handle.accepted:
            raise RuntimeError("shared Qwen worker rejected QueryVlm request")
        with self._state_lock:
            self._vlm_goal_handle = vlm_handle
        return vlm_handle.get_result_async()

    def _execute(self, handle):
        result = ReadOrder.Result()
        acquired = self._request_lock.acquire(blocking=False)
        original_state: Optional[tuple[float, float]] = None
        restored = False
        error_code = TaskError.ORDER_READ_FAILED
        try:
            if not acquired:
                raise RuntimeError("another shopping-ticket request is already active")
            if self._head_pub.get_subscription_count() <= 0:
                raise RuntimeError("head position controller is unavailable")
            if not self._vlm_client.server_is_ready():
                raise RuntimeError("shared Qwen QueryVlm action is unavailable")
            allowed_skus = set(self._database.list_catalog_skus())
            if not allowed_skus:
                raise RuntimeError("SKU catalog is empty")
            original_state = self._head_state()
            yaw, original_pitch = original_state
            target = float(self.get_parameter("pitch_target_rad").value)

            self._feedback(handle, "LOWERING_HEAD")
            error_code = TaskError.SAFETY_STOP
            self._move_pitch(
                yaw, original_pitch, target, handle, honor_cancel=True
            )
            with self._state_lock:
                settled_sequence = self._image_sequence
            self._feedback(handle, "WAITING_FRESH_FRAME")
            error_code = TaskError.CAMERA_UNAVAILABLE
            image = self._fresh_image(settled_sequence, handle)

            self._feedback(handle, "SUBMITTING_QWEN")
            error_code = TaskError.MODEL_UNAVAILABLE
            query_future = self._query_vlm(
                image, build_receipt_prompt(allowed_skus), handle
            )

            self._feedback(handle, "RESTORING_HEAD")
            error_code = TaskError.SAFETY_STOP
            _, current_pitch = self._head_state()
            self._move_pitch(
                yaw, current_pitch, original_pitch, handle, honor_cancel=False
            )
            restored = True

            query_response = self._wait_future(
                query_future, handle, cancel_downstream=True
            )
            if handle.is_cancel_requested:
                raise RuntimeError("shopping-ticket read cancelled")
            if (
                query_response.status != GoalStatus.STATUS_SUCCEEDED
                or not query_response.result.success
            ):
                raise RuntimeError(
                    query_response.result.message or "shared Qwen inference failed"
                )

            self._feedback(handle, "VALIDATING_ORDER", query_response.result.text)
            error_code = TaskError.ORDER_INVALID
            items = validate_order_items(
                parse_receipt_response(query_response.result.text, allowed_skus),
                expected_distinct_skus=int(handle.request.expected_distinct_skus),
                max_quantity=int(self.get_parameter("max_quantity").value),
            )
            confidence = 1.0
            if confidence < float(handle.request.min_confidence):
                raise RuntimeError("validated order confidence is below requested minimum")
            for item in items:
                line = OrderLine()
                line.sku = str(item["sku"])
                line.display_name = line.sku
                line.specification = ""
                line.quantity = int(item["quantity"])
                line.confidence = confidence
                result.lines.append(line)
            result.success = True
            result.error_code = TaskError.OK
            result.message = "ok"
            handle.succeed()
        except Exception as error:
            if original_state is not None and not restored:
                try:
                    yaw, original_pitch = original_state
                    _, current_pitch = self._head_state()
                    self._feedback(handle, "RESTORING_HEAD")
                    self._move_pitch(
                        yaw,
                        current_pitch,
                        original_pitch,
                        handle,
                        honor_cancel=False,
                    )
                except Exception as restore_error:
                    error = RuntimeError(f"{error}; head restore failed: {restore_error}")
                    error_code = TaskError.SAFETY_STOP
            result.success = False
            result.lines = []
            result.error_code = (
                TaskError.CANCELLED if handle.is_cancel_requested else error_code
            )
            result.message = str(error)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            with self._state_lock:
                self._vlm_goal_handle = None
            if acquired:
                self._request_lock.release()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShoppingTicketReaderNode()
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


if __name__ == "__main__":
    main()
