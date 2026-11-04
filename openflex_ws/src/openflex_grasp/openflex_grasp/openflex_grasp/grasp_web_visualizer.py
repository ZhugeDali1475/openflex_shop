from __future__ import annotations

import copy
import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Path as NavPath
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String


def _pointcloud_xyz(message: PointCloud2) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PointCloud2 does not contain x/y/z fields")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [fields[name].offset for name in ("x", "y", "z")],
        "itemsize": message.point_step,
    })
    count = int(message.width * message.height)
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((records["x"], records["y"], records["z"]))
    return points[np.isfinite(points).all(axis=1)].astype(np.float32, copy=False)


def _downsample(points: np.ndarray, limit: int) -> tuple[np.ndarray, int]:
    original_count = int(points.shape[0])
    if original_count <= limit:
        return points, original_count
    indices = np.linspace(0, original_count - 1, max(1, int(limit)), dtype=np.int64)
    return points[indices], original_count


class GraspWebVisualizerNode(Node):
    def __init__(self):
        super().__init__("grasp_web_visualizer")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("open_browser", True)
        self.declare_parameter("browser_delay_s", 1.0)
        self.declare_parameter("product", "")
        self.declare_parameter("rgb_topic", "/openflex_grasp/debug/rgb")
        self.declare_parameter("overlay_topic", "/openflex_grasp/debug/overlay")
        self.declare_parameter("mask_topic", "/openflex_grasp/debug/mask")
        self.declare_parameter("product_cloud_topic", "/openflex_grasp/debug/product_points")
        self.declare_parameter("scene_cloud_topic", "/openflex_grasp/debug/scene_points")
        self.declare_parameter("candidate_topic", "/product_perception/grasp_candidates")
        self.declare_parameter("grasp_result_topic", "/openflex_grasp/debug/grasp_result")
        self.declare_parameter("segmentation_result_topic", "/openflex_grasp/debug/detection_result")
        self.declare_parameter(
            "execution_result_topic",
            "/openflex_grasp/debug/execution_result",
        )
        self.declare_parameter("path_topic", "/openflex_grasp/debug/grasp_path")
        self.declare_parameter("max_product_points", 5000)
        self.declare_parameter("max_scene_points", 8000)

        self._lock = threading.RLock()
        self._bridge = CvBridge()
        self._web_root = Path(get_package_share_directory("openflex_grasp")) / "web"
        self._versions = {
            "rgb": 0,
            "overlay": 0,
            "mask": 0,
            "product_cloud": 0,
            "scene_cloud": 0,
            "candidates": 0,
            "path": 0,
        }
        self._images: dict[str, tuple[bytes, str]] = {}
        self._clouds = {
            "product_cloud": {"frame_id": "", "points": [], "count": 0},
            "scene_cloud": {"frame_id": "", "points": [], "count": 0},
        }
        self._candidates: dict = {
            "candidate_count": 0,
            "shown_count": 0,
            "candidates": [],
        }
        self._path: dict = {"frame_id": "", "poses": []}
        self._status = {
            "target": str(self.get_parameter("product").value),
            "segmentation": {"state": "waiting", "message": ""},
            "grasp": {"state": "waiting", "message": ""},
            "execution": {"state": "waiting", "message": ""},
            "updated_at": time.time(),
        }
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            lambda message: self._on_image("rgb", message, ".jpg", "image/jpeg"),
            debug_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("overlay_topic").value),
            lambda message: self._on_image("overlay", message, ".jpg", "image/jpeg"),
            debug_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("mask_topic").value),
            lambda message: self._on_image("mask", message, ".png", "image/png"),
            debug_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("product_cloud_topic").value),
            lambda message: self._on_cloud("product_cloud", message, "max_product_points"),
            debug_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("scene_cloud_topic").value),
            lambda message: self._on_cloud("scene_cloud", message, "max_scene_points"),
            debug_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("candidate_topic").value),
            self._on_candidates,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("segmentation_result_topic").value),
            lambda message: self._on_json_status("segmentation", message),
            debug_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("grasp_result_topic").value),
            lambda message: self._on_json_status("grasp", message),
            debug_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("execution_result_topic").value),
            lambda message: self._on_json_status("execution", message),
            debug_qos,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("path_topic").value),
            self._on_path,
            debug_qos,
        )

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._server = ThreadingHTTPServer((host, port), self._handler_class())
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="grasp-web-http",
            daemon=True,
        )
        self._server_thread.start()
        display_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
        self._url = f"http://{display_host}:{port}/"
        self.get_logger().info(f"Grasp visualization webpage: {self._url}")
        if self._as_bool("open_browser"):
            threading.Timer(
                max(0.2, float(self.get_parameter("browser_delay_s").value)),
                self._open_browser,
            ).start()

    def _as_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        return value is True or str(value).lower() in {"1", "true", "yes", "on"}

    def _on_image(self, name: str, message: Image, extension: str, content_type: str):
        try:
            if name == "mask":
                image = self._bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
                ok, encoded = cv2.imencode(extension, image)
            else:
                image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                ok, encoded = cv2.imencode(
                    extension,
                    image,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 88] if extension == ".jpg" else [],
                )
            if not ok:
                raise RuntimeError("OpenCV image encoding failed")
            with self._lock:
                self._images[name] = (encoded.tobytes(), content_type)
                self._versions[name] += 1
                self._status["updated_at"] = time.time()
        except Exception as error:
            self.get_logger().warning(f"Web visualizer image update failed for {name}: {error}")

    def _on_cloud(self, name: str, message: PointCloud2, limit_parameter: str):
        try:
            points = _pointcloud_xyz(message)
            points, original_count = _downsample(
                points,
                int(self.get_parameter(limit_parameter).value),
            )
            payload = {
                "frame_id": message.header.frame_id,
                "points": np.round(points, 5).tolist(),
                "count": original_count,
            }
            with self._lock:
                self._clouds[name] = payload
                self._versions[name] += 1
                self._status["updated_at"] = time.time()
        except Exception as error:
            self.get_logger().warning(f"Web visualizer cloud update failed for {name}: {error}")

    def _on_candidates(self, message: String):
        try:
            payload = json.loads(message.data)
            with self._lock:
                self._candidates = payload
                self._versions["candidates"] += 1
                self._status["updated_at"] = time.time()
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Invalid candidate diagnostic JSON: {error}")

    def _on_json_status(self, name: str, message: String):
        try:
            payload = json.loads(message.data)
            state = str(payload.get("state", "")).strip() or (
                "ok" if payload.get("valid", payload.get("accepted", True)) else "error"
            )
            if name == "execution":
                state = "ok" if payload.get("accepted", False) else "error"
                stage = payload.get("stage", "")
                if stage and stage not in {"complete", "plan_only_complete", "disarmed"} and state == "ok":
                    state = "running"
            summary = payload.get("message", payload.get("reason", ""))
            with self._lock:
                self._status[name] = {
                    "state": state,
                    "message": str(summary),
                    "payload": payload,
                }
                if name == "segmentation":
                    self._status["target"] = str(payload.get("target", self._status["target"]))
                    if state == "running":
                        self._status["grasp"] = {"state": "waiting", "message": "等待本轮 GraspNet"}
                        self._status["execution"] = {"state": "waiting", "message": "等待本轮规划"}
                        self._path = {"frame_id": "", "poses": []}
                        self._versions["path"] += 1
                self._status["updated_at"] = time.time()
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Invalid {name} JSON: {error}")

    def _on_path(self, message: NavPath):
        poses = []
        for pose in message.poses:
            poses.append({
                "position": [
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(pose.pose.position.z),
                ],
                "orientation": [
                    float(pose.pose.orientation.x),
                    float(pose.pose.orientation.y),
                    float(pose.pose.orientation.z),
                    float(pose.pose.orientation.w),
                ],
            })
        with self._lock:
            self._path = {"frame_id": message.header.frame_id, "poses": poses}
            self._versions["path"] += 1
            self._status["updated_at"] = time.time()

    def _snapshot(self):
        with self._lock:
            return {
                "url": self._url,
                "versions": dict(self._versions),
                "clouds": {
                    name: {
                        "frame_id": value["frame_id"],
                        "count": value["count"],
                    }
                    for name, value in self._clouds.items()
                },
                "candidates": {
                    key: value
                    for key, value in self._candidates.items()
                    if key != "candidates"
                },
                "path": self._path,
                "status": copy.deepcopy(self._status),
            }

    def _json_response(self, payload):
        return json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8"

    def _handler_class(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                node._handle_http(self)

            def log_message(self, format_string, *args):
                return

        return Handler

    def _handle_http(self, handler: BaseHTTPRequestHandler):
        path = urlparse(handler.path).path
        if path == "/api/state":
            body, content_type = self._json_response(self._snapshot())
            self._write_http(handler, 200, body, content_type)
            return
        if path == "/api/candidates":
            with self._lock:
                payload = copy.deepcopy(self._candidates)
            body, content_type = self._json_response(payload)
            self._write_http(handler, 200, body, content_type)
            return
        if path in ("/api/cloud/product", "/api/cloud/scene"):
            name = "product_cloud" if path.endswith("product") else "scene_cloud"
            with self._lock:
                payload = copy.deepcopy(self._clouds[name])
            body, content_type = self._json_response(payload)
            self._write_http(handler, 200, body, content_type)
            return
        image_names = {
            "/api/image/rgb.jpg": "rgb",
            "/api/image/overlay.jpg": "overlay",
            "/api/image/mask.png": "mask",
        }
        if path in image_names:
            with self._lock:
                image = self._images.get(image_names[path])
            if image is None:
                self._write_http(handler, 404, b"image not available", "text/plain; charset=utf-8")
            else:
                self._write_http(handler, 200, image[0], image[1])
            return
        if path in ("/", "/index.html", "/app.js", "/style.css"):
            filename = "index.html" if path in ("/", "/index.html") else path[1:]
            content_types = {
                "index.html": "text/html; charset=utf-8",
                "app.js": "text/javascript; charset=utf-8",
                "style.css": "text/css; charset=utf-8",
            }
            file_path = self._web_root / filename
            if not file_path.is_file():
                self._write_http(handler, 404, b"web asset not found", "text/plain; charset=utf-8")
            else:
                self._write_http(handler, 200, file_path.read_bytes(), content_types[filename])
            return
        self._write_http(handler, 404, b"not found", "text/plain; charset=utf-8")

    @staticmethod
    def _write_http(handler, status: int, body: bytes, content_type: str):
        handler.send_response(status)
        handler.send_header("Content-Type", content_type)
        handler.send_header("Content-Length", str(len(body)))
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.end_headers()
        handler.wfile.write(body)

    def _open_browser(self):
        try:
            subprocess.Popen(
                ["xdg-open", self._url],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as error:
            self.get_logger().warning(f"Could not open browser automatically: {error}")

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspWebVisualizerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
