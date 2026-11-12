from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.msg import ControlMode
from openflex_thesis_msgs.srv import SetControlMode
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .navigation_guard import NavigationGuard, angle_difference
from .action_journal import GoalAcceptanceUnknown, record_terminal, send_audited_goal
from .operation_store import OperationStore
from .scene_registry import load_scene_registry
from .telemetry_wait import TelemetryUnavailable, wait_for_fresh_stop


class NavigationNode(Node):
    def __init__(self):
        if os.environ.get("ROS_DOMAIN_ID") != "83" or os.environ.get("OPENFLEX_ENABLE_SIM_NAV") != "1":
            raise RuntimeError("Navigation requires explicit simulation opt-in and isolated ROS domain 83")
        super().__init__("openflex_roboos_navigation", start_parameter_services=False)
        self.declare_parameter("scene_model", "")
        self.declare_parameter("scene_config", "")
        self.declare_parameter("journal", "")
        self.guard = NavigationGuard(self.get_parameter("scene_model").value)
        self.registry = load_scene_registry(self.get_parameter("scene_config").value,
                                           self.get_parameter("scene_model").value)
        self.store = OperationStore(self.get_parameter("journal").value)
        self.sample_lock = threading.Lock()
        self.samples = {}
        self.group = ReentrantCallbackGroup()
        self.mode_client = self.create_client(SetControlMode, "/thesis/set_control_mode", callback_group=self.group)
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self.group)
        self.listeners = [
            self.create_subscription(Odometry, "/swerve_drive_controller/odom",
                                     lambda message: self.receive("odometry", message), qos_profile_sensor_data),
            self.create_subscription(JointState, "/joint_states",
                                     lambda message: self.receive("joints", message), qos_profile_sensor_data),
            self.create_subscription(String, "/thesis_sim/diagnostic_state", self.diagnostic, qos_profile_sensor_data),
        ]
        self.manipulation = None
        if os.environ.get('OPENFLEX_ENABLE_SIM_MANIPULATION') == '1':
            from .manipulation_skill import ManipulationSkill

            self.manipulation = ManipulationSkill(self)
        self.placement = None
        if os.environ.get('OPENFLEX_ENABLE_SIM_PLACE') == '1':
            from .place_skill import PlaceSkill

            self.placement = PlaceSkill(self)
        self.http = ThreadingHTTPServer(("127.0.0.1", 18553), handler_for(self))
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()

    def receive(self, key, message):
        with self.sample_lock:
            self.samples[key] = (deepcopy(message), time.monotonic())

    def diagnostic(self, message):
        try:
            self.receive("diagnostic", json.loads(message.data))
        except ValueError:
            with self.sample_lock:
                self.samples.pop("diagnostic", None)

    def sample(self, key):
        with self.sample_lock:
            message, received = deepcopy(self.samples.get(key, (None, 0)))
        if message is None or time.monotonic() - received > 0.5:
            receipt_age = time.monotonic() - received if message is not None else None
            raise TelemetryUnavailable(f"Missing/stale telemetry: {key}; receipt_age_seconds={receipt_age}")
        if key == "diagnostic":
            stamp = message["stamp"]
            source = stamp["sec"] * 1_000_000_000 + stamp["nanosec"]
        else:
            source = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - source) / 1e9
        if not -0.1 <= age <= 0.5:
            raise TelemetryUnavailable(f"Telemetry clock mismatch: {key}")
        return message

    @staticmethod
    def yaw(pose):
        quaternion = pose.orientation
        return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                          1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))

    def base_pose(self):
        pose = self.sample("odometry").pose.pose
        return [pose.position.x, pose.position.y, self.yaw(pose)]

    def healthy(self, allow_attached=False):
        forbidden = {"mission_orchestrator", "thesis_mujoco_mission_client", "scripted_local_alignment_baseline",
                     "controller_manager", "robot_state_publisher"}
        if forbidden.intersection(self.get_node_names()):
            raise RuntimeError("Competing scheduler, policy or hardware controller detected")
        publishers = self.get_publishers_info_by_topic("/thesis/cmd_vel_selected")
        if len(publishers) != 1 or publishers[0].node_name != "velocity_arbiter":
            raise RuntimeError("Selected velocity must have exactly one existing arbiter publisher")
        diagnostic = self.sample("diagnostic")
        if diagnostic.get("source") != "mujoco_diagnostic_only_NOT_PHYSICAL":
            raise RuntimeError("Unexpected simulator provenance")
        if diagnostic.get("safety_stop") is not None:
            raise RuntimeError("Existing MuJoCo safety stop is active; do not reset automatically")
        if diagnostic.get("attached_arm") is not None and not allow_attached:
            raise RuntimeError("Transport with a held object is not validated for this navigation stage")
        self.sample("odometry")
        return diagnostic

    @staticmethod
    def await_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("ROS response deadline exceeded")
            time.sleep(0.01)
        return future.result()

    def set_mode(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=2):
            raise RuntimeError("Control arbiter unavailable")
        request = SetControlMode.Request()
        request.requested_mode = mode
        request.requester = "openflex_roboos_navigation"
        request.reason = "serialized simulation skill; no hardware authorization"
        response = self.await_future(self.mode_client.call_async(request), 3)
        if not response.accepted or response.current_mode != mode:
            raise RuntimeError(f"Control mode rejected: {response.message}")
        self.wait_control_ack(mode, response.epoch)
        return {"mode": int(response.current_mode), "epoch": int(response.epoch)}

    def wait_control_ack(self, mode, epoch, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                acknowledgement = self.sample("diagnostic").get("control_ack") or {}
            except TelemetryUnavailable:
                time.sleep(0.01)
                continue
            valid = (acknowledgement.get("mode") == mode and acknowledgement.get("epoch") == epoch
                and acknowledgement.get("fresh") is True and acknowledgement.get("inputs_healthy") is True)
            if mode == ControlMode.MOVEIT:
                valid = valid and acknowledgement.get("arm_control_valid") is True
            if mode == ControlMode.PLACE:
                valid = valid and acknowledgement.get("place_control_valid") is True
            if valid:
                return
            time.sleep(0.01)
        raise TimeoutError(f"Simulator has not acknowledged control mode {mode}, epoch {epoch}")

    def require_control_ack(self, mode, epoch):
        acknowledgement = self.sample('diagnostic').get('control_ack') or {}
        valid = (epoch is not None and acknowledgement.get('mode') == mode
                 and acknowledgement.get('epoch') == epoch and acknowledgement.get('fresh') is True
                 and acknowledgement.get('inputs_healthy') is True)
        if mode == ControlMode.MOVEIT:
            valid = valid and acknowledgement.get('arm_control_valid') is True
        elif mode == ControlMode.PLACE:
            valid = valid and acknowledgement.get('place_control_valid') is True
        if not valid:
            raise RuntimeError(f'Executor control acknowledgement revoked: mode={mode}, epoch={epoch}')
        return acknowledgement

    def stopped(self, timeout=4):
        def stationary():
            twist = self.sample("odometry").twist.twist
            return math.hypot(twist.linear.x, twist.linear.y) <= 0.02 and abs(twist.angular.z) <= 0.03

        return wait_for_fresh_stop(stationary, timeout)

    def submit(self, arguments):
        if set(arguments) != {"target_id", "request_id", "timeout_sec"}:
            raise ValueError("Expected target_id, request_id and timeout_sec")
        target_id = arguments["target_id"]
        if target_id not in self.registry["stations"]:
            raise ValueError("Unknown registered station")
        if isinstance(arguments['timeout_sec'], bool) or not isinstance(arguments['timeout_sec'], (int, float)):
            raise ValueError('timeout_sec must be a JSON number')
        timeout = float(arguments["timeout_sec"])
        if not math.isfinite(timeout) or not 0.1 <= timeout <= 90:
            raise ValueError("timeout_sec must be 0.1-90")
        arguments = dict(arguments, timeout_sec=timeout)
        record, accepted = self.store.begin(arguments["request_id"], "navigate_to_target", arguments)
        if accepted:
            threading.Thread(target=self.execute, args=(arguments,), daemon=True).start()
        return record

    def execute(self, arguments, target_override=None, finalize=True, allow_attached=False):
        request_id = arguments["request_id"]
        handle, result_future = None, None
        status, detail, success, released = "failed", "", False, False
        deadline = time.monotonic() + float(arguments["timeout_sec"])
        try:
            carrying = self.sample('diagnostic').get('attached_arm') is not None
            if carrying:
                if not allow_attached and arguments.get('target_id') != 'delivery_zone':
                    raise RuntimeError('Held navigation is restricted to the verified delivery approach')
                if self.manipulation is None or not self.manipulation.retained():
                    raise RuntimeError('Fresh independent right-hand retention required for transport')
                allow_attached = True
            self.healthy(allow_attached=allow_attached)
            self.store.event(request_id, "hold_before_preflight", self.set_mode(ControlMode.HOLD))
            if not self.stopped():
                raise RuntimeError("Base stop was not confirmed")
            target = target_override if target_override is not None else self.registry["stations"][arguments["target_id"]]["approach_candidate_xyyaw"]
            payload = self.sample('diagnostic')['object_pose'] if carrying else None
            check = self.guard.check(self.sample("joints"), self.base_pose(), target, payload=payload)
            self.store.event(request_id, "offline_geometry_preflight", check)
            if not check["accepted"]:
                raise RuntimeError("Sampled path intersects robot/environment geometry")
            if not self.action.wait_for_server(timeout_sec=2):
                raise RuntimeError("Existing simulator navigation action unavailable")
            self.store.event(request_id, "control_acquired", self.set_mode(ControlMode.NAV2))
            for waypoint in check["waypoints"]:
                self.healthy(allow_attached=allow_attached)
                if carrying and not self.manipulation.retained():
                    raise RuntimeError('Object retention lost during transport')
                if self.store.cancel_event.is_set():
                    status = "canceled"
                    raise RuntimeError("Cancellation requested")
                if time.monotonic() >= deadline:
                    status = "timed_out"
                    raise TimeoutError("Skill deadline exceeded")
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = "map"
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x, goal.pose.pose.position.y = waypoint[:2]
                goal.pose.pose.orientation.z = math.sin(waypoint[2] / 2)
                goal.pose.pose.orientation.w = math.cos(waypoint[2] / 2)
                try:
                    handle = send_audited_goal(self, request_id, self.action, goal,
                                              '/navigate_to_pose', 'navigation_waypoint')
                except GoalAcceptanceUnknown:
                    status = "unknown"
                    raise
                result_future = handle.get_result_async()
                while not result_future.done():
                    self.healthy(allow_attached=allow_attached)
                    if carrying and not self.manipulation.retained():
                        raise RuntimeError('Object retention lost during transport')
                    if self.store.cancel_event.is_set():
                        status = "canceled"
                        raise RuntimeError("Cancellation requested")
                    if time.monotonic() >= deadline:
                        status = "timed_out"
                        raise TimeoutError("Skill deadline exceeded")
                    time.sleep(0.02)
                result = result_future.result()
                record_terminal(self.store, request_id, handle, result)
                self.store.event(request_id, "action_result", {"status": int(result.status), "waypoint": waypoint})
                if result.status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f"Navigation action did not succeed: {result.status}")
                handle, result_future = None, None
            self.healthy(allow_attached=allow_attached)
            if carrying and not self.manipulation.retained():
                raise RuntimeError('Object retention lost at transport goal')
            actual = self.base_pose()
            if math.dist(actual[:2], target[:2]) > 0.06 or abs(angle_difference(target[2], actual[2])) > 0.07:
                raise RuntimeError("Reported action success disagrees with measured pose")
            status, success = "succeeded", True
            self.store.event(request_id, "measured_goal", {"expected": target, "actual": actual})
        except Exception as error:
            detail = str(error)
        finally:
            try:
                cancel_future = handle.cancel_goal_async() if handle is not None else None
                self.store.event(request_id, "terminal_hold", self.set_mode(ControlMode.HOLD))
                if cancel_future is not None:
                    self.await_future(cancel_future, 3)
                    terminal = self.await_future(result_future, 3)
                    record_terminal(self.store, request_id, handle, terminal)
                    if terminal.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                        raise RuntimeError("Action terminal state unknown")
                released = self.stopped()
                if not released:
                    raise RuntimeError("Final base stop not confirmed")
            except Exception as error:
                status, success = "unknown", False
                detail += f"; stop/action confirmation failed: {error}"
            result = dict(success=success and released, detail=detail, stopped_confirmed=released,
                          backend="existing_collision_unaware_sim_navigation_NOT_Nav2")
            if finalize:
                self.store.finish(request_id, status, **result)
            else:
                return dict(status=status, **result)

    def destroy_node(self):
        if self.store.active_id is not None:
            self.store.cancel_event.set()
            deadline = time.monotonic() + 10
            while self.store.active_id is not None and time.monotonic() < deadline:
                time.sleep(0.05)
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(timeout=2)
        return super().destroy_node()


def handler_for(node):
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if self.headers.get("Host") not in ("127.0.0.1:18553", "localhost:18553") or self.headers.get("Origin"):
                self.send_error(403)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("Expected a small JSON request")
                self.connection.settimeout(3)
                arguments = json.loads(self.rfile.read(length))
                if self.path == "/navigate":
                    result = node.submit(arguments)
                elif self.path == '/grasp' and node.manipulation is not None:
                    result = node.manipulation.submit(arguments)
                elif self.path == '/place' and node.placement is not None:
                    result = node.placement.submit(arguments)
                elif self.path == "/status" and set(arguments) == {"request_id"}:
                    result = node.store.read(arguments["request_id"])
                elif self.path == "/cancel" and set(arguments) == {"request_id"}:
                    result = node.store.cancel(arguments["request_id"])
                else:
                    raise ValueError("Unknown route or arguments")
            except (ValueError, TypeError, KeyError, OSError) as error:
                result = {"status": "rejected", "success": False, "error": str(error)}
            payload = json.dumps(result, ensure_ascii=False, allow_nan=False).encode()
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
    node = NavigationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
