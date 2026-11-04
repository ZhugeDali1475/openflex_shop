from __future__ import annotations

import copy
import json
import math
import time
from typing import Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from openarmx_integrated_moveit_msgs.action import PlanPregrasp
except ImportError:  # pragma: no cover - handled at node startup.
    PlanPregrasp = None


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off", ""}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def parse_arm(value) -> int:
    text = str(value).strip().lower()
    if text in {"left", "l", "0"}:
        return 0
    if text in {"right", "r", "1"}:
        return 1
    raise ValueError("arm must be left/right or 0/1")


def parse_max_dof(value) -> int:
    text = str(value).strip().lower()
    if text == "auto":
        return 0
    try:
        dof = int(text)
    except ValueError as error:
        raise ValueError("max_dof must be auto, 7, 8, or 11") from error
    if dof not in {0, 7, 8, 11}:
        raise ValueError("max_dof must be auto, 7, 8, or 11")
    return dof


def parse_base_yaw_mode(value) -> int:
    text = str(value).strip().lower()
    if text in {"current", "current_state", "0"}:
        return 0
    if text in {"explicit", "1"}:
        return 1
    raise ValueError("base_yaw_mode must be current/current_state or explicit")


def normalize_quaternion_xyzw(quaternion: Iterable[float]) -> tuple[float, float, float, float]:
    q = tuple(float(value) for value in quaternion)
    if len(q) != 4:
        raise ValueError("quaternion must have four values")
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-9 or not all(math.isfinite(value) for value in q):
        raise ValueError("quaternion must be finite and non-zero")
    return tuple(value / norm for value in q)


def multiply_quaternions_xyzw(
    left: Iterable[float],
    right: Iterable[float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = normalize_quaternion_xyzw(left)
    x2, y2, z2, w2 = normalize_quaternion_xyzw(right)
    return normalize_quaternion_xyzw((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def _cross(left: tuple[float, float, float], right: tuple[float, float, float]):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def rotate_vector_by_quaternion(
    vector: Iterable[float],
    quaternion: Iterable[float],
) -> tuple[float, float, float]:
    x, y, z = (float(value) for value in vector)
    qx, qy, qz, qw = normalize_quaternion_xyzw(quaternion)
    qvec = (qx, qy, qz)
    uv = _cross(qvec, (x, y, z))
    uuv = _cross(qvec, uv)
    return (
        x + 2.0 * (qw * uv[0] + uuv[0]),
        y + 2.0 * (qw * uv[1] + uuv[1]),
        z + 2.0 * (qw * uv[2] + uuv[2]),
    )


def transform_pose_fields(
    position: Iterable[float],
    orientation_xyzw: Iterable[float],
    translation: Iterable[float],
    rotation_xyzw: Iterable[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    rotated = rotate_vector_by_quaternion(position, rotation_xyzw)
    tx, ty, tz = (float(value) for value in translation)
    return (
        (rotated[0] + tx, rotated[1] + ty, rotated[2] + tz),
        multiply_quaternions_xyzw(rotation_xyzw, orientation_xyzw),
    )


def yaw_only_quaternion_xyzw(
    quaternion: Iterable[float],
) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = normalize_quaternion_xyzw(quaternion)
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def transform_pose_fields_via_planar_base(
    position: Iterable[float],
    orientation_xyzw: Iterable[float],
    base_from_source_translation: Iterable[float],
    base_from_source_rotation_xyzw: Iterable[float],
    target_from_base_translation: Iterable[float],
    target_from_base_rotation_xyzw: Iterable[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Transform through a planar base without changing navigation TF."""
    base_position, base_orientation = transform_pose_fields(
        position,
        orientation_xyzw,
        base_from_source_translation,
        base_from_source_rotation_xyzw,
    )
    target_x, target_y, _ = (
        float(value) for value in target_from_base_translation
    )
    return transform_pose_fields(
        base_position,
        base_orientation,
        (target_x, target_y, 0.0),
        yaw_only_quaternion_xyzw(target_from_base_rotation_xyzw),
    )


def transform_pose_via_planar_base(
    tf_buffer,
    message: PoseStamped,
    *,
    target_frame: str,
    lookup_time: Time,
    timeout: Duration,
    base_frame: str = "base_link",
) -> PoseStamped:
    source_frame = message.header.frame_id
    if source_frame == target_frame:
        return copy.deepcopy(message)

    if target_frame == "odom":
        base_from_source = tf_buffer.lookup_transform(
            base_frame,
            source_frame,
            lookup_time,
            timeout=timeout,
        )
        target_from_base = tf_buffer.lookup_transform(
            target_frame,
            base_frame,
            lookup_time,
            timeout=timeout,
        )
        base_t = base_from_source.transform.translation
        base_q = base_from_source.transform.rotation
        target_t = target_from_base.transform.translation
        target_q = target_from_base.transform.rotation
        position, orientation = transform_pose_fields_via_planar_base(
            (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ),
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ),
            (base_t.x, base_t.y, base_t.z),
            (base_q.x, base_q.y, base_q.z, base_q.w),
            (target_t.x, target_t.y, target_t.z),
            (target_q.x, target_q.y, target_q.z, target_q.w),
        )
    else:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            lookup_time,
            timeout=timeout,
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        position, orientation = transform_pose_fields(
            (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ),
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ),
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    transformed = PoseStamped()
    transformed.header.stamp = message.header.stamp
    transformed.header.frame_id = target_frame
    (
        transformed.pose.position.x,
        transformed.pose.position.y,
        transformed.pose.position.z,
    ) = position
    (
        transformed.pose.orientation.x,
        transformed.pose.orientation.y,
    ) = orientation[:2]
    (
        transformed.pose.orientation.z,
        transformed.pose.orientation.w,
    ) = orientation[2:]
    return transformed


class ProductMoveItBridgeNode(Node):
    def __init__(self):
        super().__init__("product_moveit_bridge")
        if PlanPregrasp is None:
            raise RuntimeError(
                "openarmx_integrated_moveit_msgs is unavailable; build/source the MoveIt workspace first"
            )

        self.declare_parameter("arm", "right")
        self.declare_parameter("pregrasp_topic", "/product_perception/pregrasp_pose")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("plan_action", "/openflex/plan_pregrasp")
        self.declare_parameter("execute", False)
        self.declare_parameter("max_dof", "auto")
        self.declare_parameter("base_yaw_mode", "current")
        self.declare_parameter("desired_base_yaw_odom", 0.0)
        self.declare_parameter("base_yaw_tolerance", 0.0)
        self.declare_parameter("base_yaw_samples", 0)
        self.declare_parameter("front_distance", 0.0)
        self.declare_parameter("velocity_scale", 0.0)
        self.declare_parameter("acceleration_scale", 0.0)
        self.declare_parameter("planning_timeout_sec", 0.0)
        self.declare_parameter("use_latest_transform", True)
        self.declare_parameter("transform_timeout_s", 1.0)
        self.declare_parameter("action_wait_timeout_s", 5.0)
        self.declare_parameter("min_goal_interval_s", 2.0)
        self.declare_parameter("send_once", False)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._action_name = str(self.get_parameter("plan_action").value)
        self._goal_active = False
        self._last_goal_time = 0.0
        self._sent_count = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(self, PlanPregrasp, self._action_name)

        pregrasp_topic = str(self.get_parameter("pregrasp_topic").value)
        self.create_subscription(PoseStamped, pregrasp_topic, self._on_pregrasp_pose, 1)
        self.odom_pose_pub = self.create_publisher(
            PoseStamped,
            "/product_perception/pregrasp_pose_odom",
            1,
        )
        self.request_pub = self.create_publisher(
            String,
            "/product_perception/moveit_request",
            10,
        )
        self.result_pub = self.create_publisher(
            String,
            "/product_perception/moveit_result",
            10,
        )
        self.get_logger().info(
            f"Product MoveIt bridge is ready: {pregrasp_topic} -> {self._action_name}, "
            f"target_frame={self._target_frame}, execute={self._bool_param('execute')}"
        )

    def _bool_param(self, name: str) -> bool:
        return parse_bool(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _on_pregrasp_pose(self, message: PoseStamped):
        if self._goal_active:
            self.get_logger().warning("Ignoring pregrasp pose while a MoveIt goal is active")
            return
        if self._bool_param("send_once") and self._sent_count > 0:
            return
        now = time.monotonic()
        min_interval = max(0.0, self._float_param("min_goal_interval_s"))
        if now - self._last_goal_time < min_interval:
            return
        if not message.header.frame_id:
            self._publish_result({"accepted": False, "message": "pregrasp pose has empty frame_id"}, warn=True)
            return

        try:
            odom_pose = self._transform_pose(message)
            goal = self._build_goal(odom_pose)
        except Exception as error:
            self._publish_result({"accepted": False, "message": str(error)}, warn=True)
            return

        self.odom_pose_pub.publish(odom_pose)
        request_payload = {
            "arm": str(self.get_parameter("arm").value),
            "action": self._action_name,
            "execute": bool(goal.execute),
            "max_dof": int(goal.max_dof),
            "tcp_pregrasp_pose": self._pose_to_dict(odom_pose),
        }
        self._publish(self.request_pub, request_payload)
        self._send_goal(goal)

    def _transform_pose(self, message: PoseStamped):
        lookup_time = (
            Time()
            if self._bool_param("use_latest_transform")
            else Time.from_msg(message.header.stamp)
        )
        try:
            return transform_pose_via_planar_base(
                self._tf_buffer,
                message,
                target_frame=self._target_frame,
                lookup_time=lookup_time,
                timeout=Duration(
                    seconds=max(0.0, self._float_param("transform_timeout_s"))
                ),
            )
        except TransformException as error:
            raise RuntimeError(
                f"failed to transform {message.header.frame_id} to {self._target_frame}: {error}"
            ) from error

    def _build_goal(self, odom_pose: PoseStamped):
        goal = PlanPregrasp.Goal()
        goal.arm = parse_arm(self.get_parameter("arm").value)
        goal.max_dof = parse_max_dof(self.get_parameter("max_dof").value)
        goal.tcp_pregrasp_pose = odom_pose
        goal.base_yaw_mode = parse_base_yaw_mode(self.get_parameter("base_yaw_mode").value)
        goal.desired_base_yaw_odom = self._float_param("desired_base_yaw_odom")
        goal.base_yaw_tolerance = self._float_param("base_yaw_tolerance")
        goal.base_yaw_samples = self._int_param("base_yaw_samples")
        goal.front_distance = self._float_param("front_distance")
        goal.velocity_scale = self._float_param("velocity_scale")
        goal.acceleration_scale = self._float_param("acceleration_scale")
        goal.planning_timeout_sec = self._float_param("planning_timeout_sec")
        goal.execute = self._bool_param("execute")
        return goal

    def _send_goal(self, goal):
        wait_timeout = max(0.0, self._float_param("action_wait_timeout_s"))
        if not self._action_client.wait_for_server(timeout_sec=wait_timeout):
            self._publish_result(
                {
                    "accepted": False,
                    "message": f"MoveIt action server {self._action_name} is not available",
                },
                warn=True,
            )
            return

        self._goal_active = True
        self._last_goal_time = time.monotonic()
        self._sent_count += 1
        future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._goal_active = False
            self._publish_result({"accepted": False, "message": str(error)}, warn=True)
            return

        if not goal_handle.accepted:
            self._goal_active = False
            self._publish_result({"accepted": False, "message": "MoveIt goal rejected"}, warn=True)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_active = False
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:
            self._publish_result({"accepted": True, "message": str(error)}, warn=True)
            return

        payload = {
            "accepted": True,
            "status": int(wrapped.status),
            "moveit_error_code": int(result.error_code.val),
            "message": result.message,
            "initial_route": int(result.initial_route),
            "selected_stage": int(result.selected_stage),
            "base_goal_odom": self._pose_to_dict(result.base_goal_odom),
            "lift_goal": float(result.lift_goal),
            "arm_goal": {
                "name": list(result.arm_goal.name),
                "position": [float(value) for value in result.arm_goal.position],
            },
            "planning_time_s": float(result.planning_time),
        }
        self._publish_result(payload, warn=result.error_code.val != 1)

    def _on_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"MoveIt feedback: phase={feedback.phase}, "
            f"generated={feedback.candidates_generated}, "
            f"valid={feedback.candidates_valid}, planned={feedback.candidates_planned}"
        )

    def _publish_result(self, payload: dict, warn: bool = False):
        self._publish(self.result_pub, payload)
        text = json.dumps(payload, ensure_ascii=False)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    @staticmethod
    def _publish(publisher, payload: dict):
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(message)

    @staticmethod
    def _pose_to_dict(message: PoseStamped):
        return {
            "frame_id": message.header.frame_id,
            "position": {
                "x": float(message.pose.position.x),
                "y": float(message.pose.position.y),
                "z": float(message.pose.position.z),
            },
            "orientation": {
                "x": float(message.pose.orientation.x),
                "y": float(message.pose.orientation.y),
                "z": float(message.pose.orientation.z),
                "w": float(message.pose.orientation.w),
            },
        }


def main(args=None):
    rclpy.init(args=args)
    node = ProductMoveItBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
