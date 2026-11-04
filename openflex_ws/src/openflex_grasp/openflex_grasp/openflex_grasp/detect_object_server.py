"""Real DetectObject server: selected 2D box -> SAM2 -> same-frame RGB-D pose/cache.

This node deliberately has no GraspNet import. Grasp candidate fitting starts only
after PickObject consumes the internal detection snapshot.
"""

from __future__ import annotations

import json
import struct
import threading
import time
from uuid import uuid4

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String

from openarmx_task_msgs.action import DetectObject
from openarmx_task_msgs.msg import TaskError
from openflex_grasp_interfaces.action import QueryVlm
from openflex_grasp_interfaces.srv import GetDetectionSnapshot, ReverifyDetection

from .detection_store import DetectionSnapshot, DetectionStore, DetectionStoreError
from .backend_owner import BackendOwner
from .perception.instances import build_instances, scene_points, select_instance
from .perception.pipeline import ProductPerceptionPipeline, SegmentationResult
from .perception.rgbd import CameraIntrinsics, FrozenRgbdFrame
from .perception.router import RgbdRouter
from .paths import QWEN_MODEL, SAM_MODEL


def _stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _task_error(error: Exception) -> int:
    text = str(error).lower()
    if "stale" in text or "camera" in text or "frame" in text:
        return TaskError.CAMERA_UNAVAILABLE
    if "expired" in text:
        return TaskError.DETECTION_EXPIRED
    if "qwen" in text or "model" in text or "sam" in text:
        return TaskError.MODEL_UNAVAILABLE
    return TaskError.PERCEPTION_FAILED


class DetectObjectServer(Node):
    def __init__(self):
        super().__init__("openflex_detect_object")
        self._owner = BackendOwner("detect_object")
        self.declare_parameter("qwen_model", str(QWEN_MODEL))
        self.declare_parameter("sam_model", str(SAM_MODEL))
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("provider", "qwen_local")
        self.declare_parameter("detection_ttl_s", 30.0)
        self.declare_parameter("frame_max_age_s", 0.75)
        self.declare_parameter("max_product_instances", 8)
        self.declare_parameter("instance_min_points", 100)
        self.declare_parameter("debug_max_product_points", 8000)
        self.declare_parameter("debug_max_scene_points", 12000)
        self.declare_parameter("right_camera_serial", "")
        self.declare_parameter("left_camera_serial", "")
        self.declare_parameter("right_rgb_topic", "/cam_right/color/image")
        self.declare_parameter("right_depth_topic", "/cam_right/depth/image")
        self.declare_parameter("right_camera_info_topic", "/cam_right/depth/camera_info")
        self.declare_parameter("left_rgb_topic", "/cam_left/color/image")
        self.declare_parameter("left_depth_topic", "/cam_left/depth/image")
        self.declare_parameter("left_camera_info_topic", "/cam_left/depth/camera_info")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("sync_slop_s", 0.12)

        self._bridge = CvBridge()
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._debug_publishers = {
            "rgb": self.create_publisher(Image, "/openflex_grasp/debug/rgb", debug_qos),
            "overlay": self.create_publisher(Image, "/openflex_grasp/debug/overlay", debug_qos),
            "mask": self.create_publisher(Image, "/openflex_grasp/debug/mask", debug_qos),
            "product_points": self.create_publisher(
                PointCloud2, "/openflex_grasp/debug/product_points", debug_qos
            ),
            "scene_points": self.create_publisher(
                PointCloud2, "/openflex_grasp/debug/scene_points", debug_qos
            ),
            "result": self.create_publisher(
                String, "/openflex_grasp/debug/detection_result", debug_qos
            ),
        }
        self._router = RgbdRouter()
        self._store = DetectionStore()
        self._busy = threading.Lock()
        self._pipeline = ProductPerceptionPipeline(
            str(self.get_parameter("qwen_model").value),
            str(self.get_parameter("sam_model").value),
            str(self.get_parameter("device").value),
        )
        self._make_sync("right")
        self._make_sync("left")
        self._action = ActionServer(
            self, DetectObject, "/detect_object", execute_callback=self._execute,
            goal_callback=self._goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self._vlm_action = ActionServer(
            self,
            QueryVlm,
            "/openflex_grasp/query_vlm",
            execute_callback=self._execute_vlm,
            goal_callback=self._vlm_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self._snapshot = self.create_service(
            GetDetectionSnapshot, "/openflex_grasp/get_detection_snapshot", self._get_snapshot
        )
        self._reverify = self.create_service(
            ReverifyDetection, "/openflex_grasp/reverify_detection", self._reverify_detection
        )
        self.get_logger().info("DetectObject server ready: Qwen+SAM2 shared worker, both wrists subscribed")

    @staticmethod
    def _vlm_goal(goal: QueryVlm.Goal):
        if not str(goal.prompt).strip():
            return GoalResponse.REJECT
        if int(goal.image.width) <= 0 or int(goal.image.height) <= 0:
            return GoalResponse.REJECT
        if not goal.image.data:
            return GoalResponse.REJECT
        if int(goal.max_new_tokens) > 2048:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _vlm_feedback(handle, phase: str):
        message = QueryVlm.Feedback()
        message.phase = phase
        handle.publish_feedback(message)

    def _execute_vlm(self, handle):
        result = QueryVlm.Result()
        acquired = False
        try:
            self._vlm_feedback(handle, "WAITING_FOR_QWEN")
            while not acquired:
                if handle.is_cancel_requested:
                    raise RuntimeError("VLM request cancelled")
                acquired = self._busy.acquire(timeout=0.1)
            if handle.is_cancel_requested:
                raise RuntimeError("VLM request cancelled")
            self._vlm_feedback(handle, "QWEN_INFERENCE")
            rgb = self._bridge.imgmsg_to_cv2(
                handle.request.image, desired_encoding="rgb8"
            )
            text = self._pipeline.box_provider.generate_text(
                np.ascontiguousarray(rgb),
                str(handle.request.prompt),
                max_new_tokens=int(handle.request.max_new_tokens) or 256,
            )
            if handle.is_cancel_requested:
                raise RuntimeError("VLM request cancelled")
            result.success = True
            result.text = text
            result.model = str(self.get_parameter("qwen_model").value)
            result.error_code = TaskError.OK
            result.message = "ok"
            handle.succeed()
        except Exception as error:
            result.success = False
            result.text = ""
            result.model = str(self.get_parameter("qwen_model").value)
            result.error_code = _task_error(error)
            result.message = str(error)
            if handle.is_cancel_requested or "cancelled" in str(error).lower():
                handle.canceled()
            else:
                handle.abort()
        finally:
            if acquired:
                self._busy.release()
        return result

    def _make_sync(self, arm: str):
        rgb = message_filters.Subscriber(
            self, Image, str(self.get_parameter(f"{arm}_rgb_topic").value), qos_profile=qos_profile_sensor_data
        )
        depth = message_filters.Subscriber(
            self, Image, str(self.get_parameter(f"{arm}_depth_topic").value), qos_profile=qos_profile_sensor_data
        )
        info = message_filters.Subscriber(
            self, CameraInfo, str(self.get_parameter(f"{arm}_camera_info_topic").value), qos_profile=qos_profile_sensor_data
        )
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, depth, info], queue_size=10,
            slop=float(self.get_parameter("sync_slop_s").value), allow_headerless=False,
        )
        sync.registerCallback(lambda a, b, c: self._on_frame(arm, a, b, c))
        setattr(self, f"_{arm}_sync", sync)

    def _on_frame(self, arm: str, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        try:
            bgr = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            serial = str(self.get_parameter(f"{arm}_camera_serial").value).strip() or f"{arm}-unconfigured"
            self._router.update(FrozenRgbdFrame(
                rgb=rgb, depth=depth,
                intrinsics=CameraIntrinsics(float(info_msg.k[0]), float(info_msg.k[4]), float(info_msg.k[2]), float(info_msg.k[5]), int(info_msg.width), int(info_msg.height)),
                arm_name=arm, camera_serial=serial,
                frame_id=depth_msg.header.frame_id, stamp_ns=_stamp_ns(depth_msg.header),
                depth_scale=float(self.get_parameter("depth_scale").value),
            ))
        except Exception as error:
            self.get_logger().warn(f"discarded invalid {arm} RGB-D frame: {error}")

    def _goal(self, goal: DetectObject.Goal):
        if str(goal.arm_name).strip().lower() not in {"left", "right"}:
            return GoalResponse.REJECT
        if not (str(goal.sku_name).strip() or str(goal.object_id).strip()):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _feedback(self, handle, phase: str, confidence: float = 0.0):
        message = DetectObject.Feedback()
        message.phase = phase
        message.confidence = float(confidence)
        handle.publish_feedback(message)

    def _execute(self, handle):
        result = DetectObject.Result()
        goal = handle.request
        target = str(goal.sku_name).strip() or str(goal.object_id).strip()
        started = time.monotonic()
        timeout_s = max(0.1, float(goal.timeout_sec) if goal.timeout_sec > 0 else 30.0)
        acquired = False
        try:
            if not self._busy.acquire(blocking=False):
                raise RuntimeError("perception worker is busy; requests are serialized")
            acquired = True
            arm = str(goal.arm_name).strip().lower()
            self._publish_detection_status(
                state="running",
                message="正在冻结腕部 RGB-D 图像",
                target=target,
                arm_name=arm,
            )
            self._feedback(handle, "FREEZE_RGBD")
            frame = self._router.freeze(arm, max_age_s=float(self.get_parameter("frame_max_age_s").value))
            if handle.is_cancel_requested:
                raise RuntimeError("detect request cancelled")
            self._feedback(handle, "QWEN_DETECT")
            provider = str(self.get_parameter("provider").value)
            if provider != "qwen_local":
                raise RuntimeError("only qwen_local is owned by this worker; select OWLv2 through its scene adapter")
            detections = self._pipeline.locate_all(
                frame.rgb,
                target,
                qwen_grounding_prompt=str(goal.qwen_grounding_prompt),
                max_boxes=int(self.get_parameter("max_product_instances").value),
            )
            if time.monotonic() - started > timeout_s:
                raise RuntimeError("Qwen detection timed out")
            self._feedback(handle, "SAM2_SEGMENT")
            segmented = [
                SegmentationResult(detection, *self._pipeline.segment_box(frame.rgb, detection.box_xyxy))
                for detection in detections
            ]
            self._feedback(handle, "BUILD_POINTCLOUD")
            instances = build_instances(
                frame, segmented, min_points=int(self.get_parameter("instance_min_points").value)
            )
            selected = select_instance(instances)
            all_scene = scene_points(frame)
            if all_scene.shape[0] == 0:
                raise RuntimeError("same-frame scene point cloud has no valid depth")
            center = np.asarray(selected.quality["center_camera_m"], dtype=np.float64)
            pose = PoseStamped()
            pose.header.frame_id = frame.frame_id
            pose.header.stamp.sec = frame.stamp_ns // 1_000_000_000
            pose.header.stamp.nanosec = frame.stamp_ns % 1_000_000_000
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center.tolist()
            pose.pose.orientation.w = 1.0
            confidence = float(selected.quality["score"])
            if confidence < float(goal.min_confidence):
                raise RuntimeError(f"best instance confidence {confidence:.3f} below requested {goal.min_confidence:.3f}")
            detection_id = str(uuid4())
            self._store.put(DetectionSnapshot(
                detection_id=detection_id, object_id=target, arm_name=arm,
                frame=frame, provider="qwen_local", model=str(self.get_parameter("qwen_model").value),
                request_id=detection_id, box_xyxy=selected.result.detection.box_xyxy,
                mask=selected.mask, product_points=selected.points, scene_points=all_scene,
                object_center=center, confidence=confidence,
                expires_at=time.monotonic() + float(self.get_parameter("detection_ttl_s").value),
            ))
            try:
                self._publish_debug_detection(
                    detection_id=detection_id,
                    object_id=target,
                    arm_name=arm,
                    provider=provider,
                    frame=frame,
                    box_xyxy=selected.result.detection.box_xyxy,
                    mask=selected.mask,
                    product_points=selected.points,
                    scene_points=all_scene,
                    object_center=center,
                    confidence=confidence,
                )
            except Exception as debug_error:
                self.get_logger().warning(
                    f"Detection succeeded but debug visualization publish failed: {debug_error}"
                )
            result.found, result.object_pose, result.confidence = True, pose, confidence
            result.detection_id, result.error_code, result.message = detection_id, TaskError.OK, "ok"
            self._feedback(handle, "DETECTION_READY", confidence)
            handle.succeed()
        except Exception as error:
            result.found = False
            result.confidence = 0.0
            result.detection_id = ""
            result.error_code = _task_error(error)
            result.message = str(error)
            self._publish_detection_status(
                state="error",
                message=str(error),
                target=target,
                arm_name=str(goal.arm_name).strip().lower(),
            )
            if handle.is_cancel_requested or "cancelled" in str(error).lower():
                handle.canceled()
            else:
                handle.abort()
        finally:
            if acquired:
                self._busy.release()
        return result

    def _get_snapshot(self, request, response):
        try:
            snapshot = self._store.get(request.detection_id, consume=bool(request.consume))
            response.success, response.error_code, response.message = True, TaskError.OK, "ok"
            response.detection_id, response.arm_name, response.object_id = snapshot.detection_id, snapshot.arm_name, snapshot.object_id
            response.provider, response.model, response.request_id = snapshot.provider, snapshot.model, snapshot.request_id
            response.camera_serial, response.frame_id = snapshot.frame.camera_serial, snapshot.frame.frame_id
            response.stamp.sec = snapshot.frame.stamp_ns // 1_000_000_000
            response.stamp.nanosec = snapshot.frame.stamp_ns % 1_000_000_000
            response.object_pose = PoseStamped()
            response.object_pose.header.frame_id = snapshot.frame.frame_id
            response.object_pose.header.stamp = response.stamp
            response.object_pose.pose.position.x, response.object_pose.pose.position.y, response.object_pose.pose.position.z = snapshot.object_center.tolist()
            response.object_pose.pose.orientation.w = 1.0
            response.box_xyxy = list(snapshot.box_xyxy)
            response.mask = self._image(snapshot.mask, snapshot.frame)
            response.product_points = self._cloud(snapshot.product_points, snapshot.frame)
            response.scene_points = self._cloud(snapshot.scene_points, snapshot.frame)
        except DetectionStoreError as error:
            response.success, response.error_code, response.message = False, _task_error(error), str(error)
        return response

    def _reverify_detection(self, request, response):
        acquired = False
        try:
            if not self._busy.acquire(blocking=False):
                raise RuntimeError("perception worker is busy; reverification cannot start")
            acquired = True
            snapshot = self._store.get(request.detection_id, consume=False)
            latest = self._router.freeze(
                snapshot.arm_name,
                max_age_s=float(self.get_parameter("frame_max_age_s").value),
            )
            if latest.camera_serial != snapshot.frame.camera_serial:
                raise RuntimeError("camera serial changed since detection")
            if latest.frame_id != snapshot.frame.frame_id:
                raise RuntimeError("camera optical frame changed since detection")
            new_mask, _score = self._pipeline.segment_box(latest.rgb, snapshot.box_xyxy)
            from .perception.masks import clean_mask
            from .perception.pointcloud import product_points
            new_mask = clean_mask(new_mask)
            new_points = product_points(
                latest.rgb, latest.depth, new_mask, latest.intrinsics,
                depth_scale=latest.depth_scale,
            )
            if new_points.shape[0] < int(self.get_parameter("instance_min_points").value):
                raise RuntimeError("reverification has insufficient valid product depth")
            old_xyz = np.asarray(snapshot.product_points[:, :3], dtype=np.float64)
            new_xyz = np.asarray(new_points[:, :3], dtype=np.float64)
            old_center = np.median(old_xyz, axis=0)
            new_center = np.median(new_xyz, axis=0)
            center_shift = float(np.linalg.norm(new_center - old_center))
            old_size = np.maximum(np.quantile(old_xyz, 0.9, axis=0) - np.quantile(old_xyz, 0.1, axis=0), 1e-4)
            new_size = np.maximum(np.quantile(new_xyz, 0.9, axis=0) - np.quantile(new_xyz, 0.1, axis=0), 1e-4)
            size_change = float(np.max(np.abs(new_size - old_size) / old_size))
            old_mask = np.asarray(snapshot.mask, dtype=bool)
            new_mask = np.asarray(new_mask, dtype=bool)
            intersection = int(np.count_nonzero(old_mask & new_mask))
            union = int(np.count_nonzero(old_mask | new_mask))
            mask_iou = float(intersection / union) if union else 0.0
            old_depth_ratio = len(old_xyz) / max(1, int(np.count_nonzero(old_mask)))
            new_depth_ratio = len(new_xyz) / max(1, int(np.count_nonzero(new_mask)))
            valid_depth_ratio = float(min(1.0, new_depth_ratio / max(old_depth_ratio, 1e-6)))
            response.center_shift_m = center_shift
            response.size_change_ratio = size_change
            response.mask_iou = mask_iou
            response.valid_depth_ratio = valid_depth_ratio
            failures = []
            if center_shift > float(request.max_center_shift_m):
                failures.append(f"center shift {center_shift:.4f}m")
            if size_change > float(request.max_size_change_ratio):
                failures.append(f"size change {size_change:.3f}")
            if mask_iou < float(request.min_mask_iou):
                failures.append(f"mask IoU {mask_iou:.3f}")
            if valid_depth_ratio < float(request.min_valid_depth_ratio):
                failures.append(f"valid depth retention {valid_depth_ratio:.3f}")
            if failures:
                response.success = False
                response.error_code = TaskError.DETECTION_MISMATCH
                response.message = "; ".join(failures)
            else:
                response.success = True
                response.error_code = TaskError.OK
                response.message = (
                    f"reverified: center={center_shift:.4f}m size={size_change:.3f} "
                    f"iou={mask_iou:.3f} depth={valid_depth_ratio:.3f}"
                )
        except Exception as error:
            response.success = False
            response.error_code = _task_error(error)
            response.message = str(error)
        finally:
            if acquired:
                self._busy.release()
        return response

    def _image(self, mask, frame):
        message = self._bridge.cv2_to_imgmsg(mask.astype(np.uint8) * 255, encoding="mono8")
        message.header.frame_id = frame.frame_id
        message.header.stamp.sec = frame.stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = frame.stamp_ns % 1_000_000_000
        return message

    def _rgb_image(self, rgb, frame):
        message = self._bridge.cv2_to_imgmsg(
            np.ascontiguousarray(rgb), encoding="rgb8"
        )
        message.header.frame_id = frame.frame_id
        message.header.stamp.sec = frame.stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = frame.stamp_ns % 1_000_000_000
        return message

    def _publish_debug_detection(
        self,
        *,
        detection_id,
        object_id,
        arm_name,
        provider,
        frame,
        box_xyxy,
        mask,
        product_points,
        scene_points,
        object_center,
        confidence,
    ):
        rgb = np.ascontiguousarray(frame.rgb)
        overlay = rgb.copy()
        selected = np.asarray(mask, dtype=bool)
        tint = np.zeros_like(overlay)
        tint[..., 1] = 210
        tint[..., 2] = 245
        overlay[selected] = (
            overlay[selected].astype(np.float32) * 0.48
            + tint[selected].astype(np.float32) * 0.52
        ).astype(np.uint8)
        x1, y1, x2, y2 = (int(value) for value in box_xyxy)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (20, 255, 80), 3)
        self._debug_publishers["rgb"].publish(self._rgb_image(rgb, frame))
        self._debug_publishers["overlay"].publish(self._rgb_image(overlay, frame))
        self._debug_publishers["mask"].publish(self._image(mask, frame))
        self._debug_publishers["product_points"].publish(
            self._cloud(
                self._debug_sample(
                    product_points,
                    int(self.get_parameter("debug_max_product_points").value),
                ),
                frame,
            )
        )
        self._debug_publishers["scene_points"].publish(
            self._cloud(
                self._debug_sample(
                    scene_points,
                    int(self.get_parameter("debug_max_scene_points").value),
                ),
                frame,
            )
        )
        payload = {
            "valid": True,
            "message": "检测、分割和 RGB-D 点云已就绪",
            "target": object_id,
            "arm_name": arm_name,
            "provider": provider,
            "detection_id": detection_id,
            "frame_id": frame.frame_id,
            "stamp_ns": frame.stamp_ns,
            "camera_serial": frame.camera_serial,
            "box_xyxy": [int(value) for value in box_xyxy],
            "confidence": float(confidence),
            "object_pose": {
                "frame_id": frame.frame_id,
                "position": [float(value) for value in object_center],
                "orientation": [0.0, 0.0, 0.0, 1.0],
            },
        }
        self._debug_publishers["result"].publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_detection_status(self, *, state, message, target, arm_name):
        payload = {
            "state": str(state),
            "valid": state != "error",
            "message": str(message),
            "target": str(target),
            "arm_name": str(arm_name),
        }
        self._debug_publishers["result"].publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    @staticmethod
    def _debug_sample(points, limit):
        points = np.asarray(points)
        if points.shape[0] <= max(1, limit):
            return points
        indices = np.linspace(0, points.shape[0] - 1, max(1, limit), dtype=np.int64)
        return points[indices]

    @staticmethod
    def _cloud(points, frame):
        message = PointCloud2()
        message.header.frame_id = frame.frame_id
        message.header.stamp.sec = frame.stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = frame.stamp_ns % 1_000_000_000
        message.height, message.width, message.is_dense = 1, int(points.shape[0]), True
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.point_step, message.row_step = 16, 16 * message.width
        packed = bytearray(message.row_step)
        for index, point in enumerate(points):
            r, g, b = (int(value) for value in point[3:6])
            struct.pack_into("<fffI", packed, 16 * index, point[0], point[1], point[2], (r << 16) | (g << 8) | b)
        message.data = bytes(packed)
        return message

    def destroy_node(self):
        self._owner.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
