import json
import struct
import threading
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String

from .paths import QWEN_MODEL, SAM_MODEL
from .perception.masks import clean_mask
from .perception.pipeline import ProductPerceptionPipeline
from .perception.pointcloud import (
    product_instance_graspability,
    product_points,
    semantic_instance_selection_key,
)
from .perception.rgbd import CameraIntrinsics
from .perception.visualization import render_overlay


class ProductPointCloudNode(Node):
    def __init__(self):
        super().__init__("product_pointcloud")
        self.declare_parameter("qwen_model", str(QWEN_MODEL))
        self.declare_parameter("sam_model", str(SAM_MODEL))
        self.declare_parameter("device", "cuda")
        self.declare_parameter("target", "water bottle")
        self.declare_parameter("rgb_topic", "/cam_left/color/image")
        self.declare_parameter("depth_topic", "/cam_left/depth/image")
        self.declare_parameter("camera_info_topic", "/cam_left/depth/camera_info")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.07)
        self.declare_parameter("max_depth_m", 1.5)
        self.declare_parameter("max_depth_spread_m", 0.25)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("scene_point_stride", 2)
        self.declare_parameter("sync_slop_s", 0.12)
        self.declare_parameter("max_product_instances", 8)
        self.declare_parameter("instance_min_points", 100)

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest = None
        self._busy = False
        self._target = str(self.get_parameter("target").value).strip()
        self._pending = bool(self._target)

        self.get_logger().info("Loading Qwen3.5 and SAM2 models; first startup can take a minute")
        self.segmenter = ProductPerceptionPipeline(
            str(self.get_parameter("qwen_model").value),
            str(self.get_parameter("sam_model").value),
            str(self.get_parameter("device").value),
        )
        self.get_logger().info("Models loaded; waiting for synchronized RGB-D frames")

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        rgb_sub = message_filters.Subscriber(self, Image, rgb_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        info_sub = message_filters.Subscriber(self, CameraInfo, info_topic, qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=10,
            slop=float(self.get_parameter("sync_slop_s").value),
            allow_headerless=False,
        )
        self._sync.registerCallback(self._on_frame)

        self.create_subscription(String, "/product_perception/target", self._on_target, 10)
        self.mask_pub = self.create_publisher(Image, "/product_perception/mask", 1)
        self.overlay_pub = self.create_publisher(Image, "/product_perception/overlay", 1)
        self.points_pub = self.create_publisher(PointCloud2, "/product_perception/points", 1)
        self.scene_points_pub = self.create_publisher(PointCloud2, "/product_perception/scene_points", 1)
        self.result_pub = self.create_publisher(String, "/product_perception/result", 10)

    def _on_target(self, message: String):
        target = message.data.strip()
        if not target:
            self.get_logger().warning("Ignoring empty product target")
            return
        with self._lock:
            self._target = target
            self._pending = True
        self.get_logger().info(f"Queued product target: {target}")
        self._start_if_ready()

    def _on_frame(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        with self._lock:
            self._latest = (rgb_msg, depth_msg, info_msg)
        self._start_if_ready()

    def _start_if_ready(self):
        with self._lock:
            if self._busy or not self._pending or self._latest is None:
                return
            self._busy = True
            self._pending = False
            target = self._target
            snapshot = self._latest
        threading.Thread(target=self._process, args=(target, snapshot), daemon=True).start()

    def _process(self, target, snapshot):
        started = time.monotonic()
        rgb_msg, depth_msg, info_msg = snapshot
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            segmentation_results = self.segmenter.infer_all(rgb, target)
            max_instances = max(
                1, int(self.get_parameter("max_product_instances").value)
            )
            segmentation_results = segmentation_results[:max_instances]
            intrinsics = CameraIntrinsics(
                fx=float(info_msg.k[0]),
                fy=float(info_msg.k[4]),
                cx=float(info_msg.k[2]),
                cy=float(info_msg.k[5]),
                width=int(info_msg.width),
                height=int(info_msg.height),
            )
            instances = []
            min_points = max(1, int(self.get_parameter("instance_min_points").value))
            for index, result in enumerate(segmentation_results):
                instance_mask = clean_mask(result.mask)
                instance_points = product_points(
                    rgb,
                    depth,
                    instance_mask,
                    intrinsics,
                    depth_scale=float(self.get_parameter("depth_scale").value),
                    min_depth_m=float(self.get_parameter("min_depth_m").value),
                    max_depth_m=float(self.get_parameter("max_depth_m").value),
                    max_depth_spread_m=float(
                        self.get_parameter("max_depth_spread_m").value
                    ),
                    stride=max(1, int(self.get_parameter("point_stride").value)),
                )
                graspability = product_instance_graspability(
                    instance_mask,
                    instance_points,
                    result.detection.box_xyxy,
                    rgb.shape,
                    detection_confidence=result.detection.confidence,
                    segmentation_score=result.sam_score,
                    min_points=min_points,
                )
                instances.append(
                    {
                        "index": index,
                        "result": result,
                        "mask": instance_mask,
                        "points": instance_points,
                        "graspability": graspability,
                    }
                )

            valid_instances = [
                instance for instance in instances if instance["graspability"]["valid"]
            ]
            if not valid_instances:
                reasons = [
                    instance["graspability"]["reason"] for instance in instances
                ]
                raise RuntimeError(
                    "no product instance has enough valid depth points: "
                    + "; ".join(reasons)
                )
            selected = max(
                valid_instances,
                key=lambda instance: semantic_instance_selection_key(
                    instance["result"].detection.confidence,
                    instance["result"].sam_score,
                    instance["graspability"]["score"],
                ),
            )
            selected_index = int(selected["index"])
            result = selected["result"]
            mask = selected["mask"]
            points = selected["points"]
            scene_points = product_points(
                rgb,
                depth,
                np.ones(depth.shape[:2], dtype=bool),
                intrinsics,
                depth_scale=float(self.get_parameter("depth_scale").value),
                min_depth_m=float(self.get_parameter("min_depth_m").value),
                max_depth_m=float(self.get_parameter("max_depth_m").value),
                max_depth_spread_m=0.0,
                stride=max(1, int(self.get_parameter("scene_point_stride").value)),
            )
            overlay = render_overlay(
                rgb,
                mask,
                result.detection.box_xyxy,
                f"{result.detection.label} [selected]",
            )
            for instance in instances:
                if instance["index"] == selected_index:
                    continue
                x1, y1, x2, y2 = instance["result"].detection.box_xyxy
                semantic_score = instance["result"].detection.confidence
                geometry_score = instance["graspability"]["score"]
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 190, 0), 2)
                cv2.putText(
                    overlay,
                    f"candidate {instance['index']}: sem {semantic_score:.2f} geo {geometry_score:.2f}",
                    (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 190, 0),
                    1,
                    cv2.LINE_AA,
                )
            self._publish_images(mask, overlay, rgb_msg.header)
            self.scene_points_pub.publish(self._pointcloud_message(scene_points, depth_msg.header))
            self.points_pub.publish(self._pointcloud_message(points, depth_msg.header))
            payload = {
                "target": target,
                "label": result.detection.label,
                "bbox_xyxy": list(result.detection.box_xyxy),
                "grounding_confidence": result.detection.confidence,
                "sam_score": result.sam_score,
                "mask_pixels": int(mask.sum()),
                "point_count": int(points.shape[0]),
                "scene_point_count": int(scene_points.shape[0]),
                "instance_count": len(instances),
                "selected_instance_index": selected_index,
                "instance_selection_policy": "semantic_then_segmentation_then_geometry",
                "instances": [
                    {
                        "index": int(instance["index"]),
                        "label": instance["result"].detection.label,
                        "bbox_xyxy": list(
                            instance["result"].detection.box_xyxy
                        ),
                        "selected": instance["index"] == selected_index,
                        **instance["graspability"],
                    }
                    for instance in instances
                ],
                "frame_id": depth_msg.header.frame_id,
                "elapsed_s": round(time.monotonic() - started, 3),
            }
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False)
            self.result_pub.publish(message)
            self.get_logger().info(message.data)
        except Exception as error:
            self.get_logger().error(f"Product inference failed for {target!r}: {error}")
        finally:
            with self._lock:
                self._busy = False
            self._start_if_ready()

    def _publish_images(self, mask, overlay, header):
        mask_msg = self.bridge.cv2_to_imgmsg(mask.astype(np.uint8) * 255, encoding="mono8")
        mask_msg.header = header
        self.mask_pub.publish(mask_msg)
        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay_bgr, encoding="bgr8")
        overlay_msg.header = header
        self.overlay_pub.publish(overlay_msg)

    @staticmethod
    def _pointcloud_message(points: np.ndarray, header) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        packed = bytearray(cloud.row_step)
        for index, point in enumerate(points):
            r, g, b = (int(value) for value in point[3:6])
            rgb_bits = (r << 16) | (g << 8) | b
            struct.pack_into("<fffI", packed, index * 16, point[0], point[1], point[2], rgb_bits)
        cloud.data = bytes(packed)
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = ProductPointCloudNode()
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
