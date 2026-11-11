from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from nav_msgs.msg import Odometry
from openflex_thesis_msgs.msg import (
    AffordanceState, ControlMode, GraspStatus, MissionState, ObjectMemory,
    ObjectObservation, PlaceStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from .snapshot import SnapshotCache, TOPICS, encode_image
from .scene_registry import load_scene_registry


MESSAGE_TYPES = {
    "odometry": Odometry, "joint_states": JointState, "grasp_status": GraspStatus,
    "place_status": PlaceStatus, "control_mode": ControlMode, "mission_state": MissionState,
    "object_observation": ObjectObservation, "object_memory": ObjectMemory,
    "affordance": AffordanceState, "camera": Image,
    "simulation_diagnostic": String,
}
LATCHED_KEYS = {"object_observation", "object_memory", "control_mode", "mission_state", "affordance"}


class SnapshotNode(Node):
    def __init__(self):
        super().__init__("openflex_roboos_readonly", start_parameter_services=False)
        self.declare_parameter("snapshot_port", 18552)
        self.declare_parameter("scene_config", "")
        self.declare_parameter("scene_model", "")
        self.cache = SnapshotCache(lambda: self.get_clock().now().nanoseconds)
        self.refresh_scene_registry()
        self.create_timer(1.0, self.refresh_scene_registry)
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.listeners = [self.create_subscription(
            message_type, TOPICS[key], partial(self.receive, key),
            latched_qos if key in LATCHED_KEYS else qos_profile_sensor_data,
        ) for key, message_type in MESSAGE_TYPES.items()]
        self.http = ThreadingHTTPServer(
            ("127.0.0.1", self.get_parameter("snapshot_port").value), make_handler(self.cache))
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info("Read-only snapshots on loopback; no motion publishers, services or action clients")

    def refresh_scene_registry(self):
        try:
            registry = load_scene_registry(self.get_parameter("scene_config").value,
                                           self.get_parameter("scene_model").value)
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
            registry = {"status": "unavailable", "execution_enabled": False, "error": str(error)}
        self.cache.set_registry(registry)

    def receive(self, key, message):
        try:
            if key == 'simulation_diagnostic':
                data = json.loads(message.data)
                if not isinstance(data, dict) or data.get('source') != 'mujoco_diagnostic_only_NOT_PHYSICAL':
                    raise ValueError('Unexpected diagnostic provenance')
                source = data['stamp']
                stamp = source['sec'] * 1_000_000_000 + source['nanosec']
                data['header'] = {'stamp': source, 'frame_id': 'map'}
            else:
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if key == "camera":
                data = encode_image(message)
                data["header"] = message_to_ordereddict(message.header)
            elif key != 'simulation_diagnostic':
                data = message_to_ordereddict(message)
            self.cache.update(key, data, stamp)
        except (ValueError, TypeError, OverflowError, KeyError) as error:
            self.cache.invalidate(key, error)

    def destroy_node(self):
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(timeout=3)
        return super().destroy_node()


def make_handler(cache):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            allowed_hosts = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
            if self.headers.get("Host") not in allowed_hosts or self.headers.get("Origin"):
                self.send_error(403)
                return
            if self.path not in ("/state", "/observation"):
                self.send_error(404)
                return
            payload = json.dumps(cache.snapshot(self.path == "/observation"), allow_nan=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format, *args):
            return

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SnapshotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
