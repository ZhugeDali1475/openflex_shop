"""ROS truth backend and virtual referee for a full supermarket match."""

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import random
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from openarmx_task_executor.station_db import StationDatabase
from openarmx_task_msgs.action import (
    DetectObject,
    DockToStation,
    ExecuteCompetitionPhase,
    InspectRetailScene,
    PickObject,
    PlaceObject,
    ReadOrder,
    SetPerceptionMode,
    VerifyPlacement,
)
from openarmx_task_msgs.msg import CompetitionWorkItem, OrderLine, TaskError
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .sim_world import (
    PHASE_RESTOCKING,
    PHASE_RETURN_MISPLACED,
    CompetitionWorld,
)
from .detection_store import TruthDetectionStore


class SupermarketSimulationNode(Node):
    """Supplies every external capability required by the production coordinator."""

    def __init__(self) -> None:
        super().__init__("openarmx_supermarket_sim")
        share = get_package_share_directory("openarmx_supermarket_sim")
        executor_share = get_package_share_directory("openarmx_task_executor")
        defaults = {
            "random_seed": 20260729,
            "auto_run": False,
            "exit_on_complete": True,
            "auto_judge_pause_sec": 1.0,
            "time_scale": 0.02,
            "navigation_speed_mps": 0.50,
            "minimum_action_delay_sec": 0.02,
            "collision_probability": 0.0,
            "drop_probability": 0.0,
            "out_of_bounds_probability": 0.0,
            "world_config": os.path.join(share, "config", "world.yaml"),
            "station_config": os.path.join(executor_share, "config", "stations.yaml"),
            "status_topic": "/sim_competition/status",
            "report_path": "/tmp/openarmx_supermarket_sim_report.json",
            "truth_detection_ttl_sec": 30.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        world_config = str(self.get_parameter("world_config").value) or defaults[
            "world_config"
        ]
        station_config = str(self.get_parameter("station_config").value) or defaults[
            "station_config"
        ]
        seed = int(self.get_parameter("random_seed").value)
        self.world = CompetitionWorld(world_config, seed)
        self.stations = StationDatabase.from_yaml(station_config)
        self.random = random.Random(seed + 991)
        self.callback_group = ReentrantCallbackGroup()
        self.base_pose = [0.0, 0.0, 0.0]
        self.base_twist = [0.0, 0.0, 0.0]
        self.current_station = "start"
        self.current_mode = SetPerceptionMode.Goal.IDLE
        self.navigation_active = False
        self.held_arm = ""
        self.detections = TruthDetectionStore(
            float(self.get_parameter("truth_detection_ttl_sec").value)
        )
        self.referee_phase = 0
        self.phase_goal_active = False
        self.phase_failed = False
        self.run_id = f"sim-{self.world.seed}"
        self.last_phase_finished_wall = 0.0
        self.started_wall = 0.0
        self.phase_durations = {}
        self.phase_started_wall = 0.0
        self.shutdown_timer = None

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            self._latched_qos(),
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom_safe", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self._publish_odom)
        self.create_timer(0.25, self._publish_status)

        self._create_capability_servers()
        self.phase_client = ActionClient(
            self,
            ExecuteCompetitionPhase,
            "/competition/execute_phase",
            callback_group=self.callback_group,
        )
        self.coordinator_reset_client = self.create_client(
            Trigger, "/competition/reset_run", callback_group=self.callback_group
        )
        self.create_service(
            Trigger,
            "/sim_competition/reset",
            self._reset_service,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/sim_competition/start_next_phase",
            self._start_next_service,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/sim_competition/inject_collision",
            self._inject_collision,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/sim_competition/inject_drop",
            self._inject_drop,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "/sim_competition/inject_out_of_bounds",
            self._inject_out_of_bounds,
            callback_group=self.callback_group,
        )
        self.create_timer(0.20, self._auto_referee)
        self._publish_status()
        self.get_logger().warn(
            "TRUTH-MODE SIMULATION: OCR, semantic inspection and manipulation use "
            "hidden simulator state; results are not physical perception evidence"
        )
        self.get_logger().info(
            f"simulation seed={self.world.seed}, run_id={self.run_id}, "
            f"order={[entry.sku for entry in self.world.order]}"
        )

    @staticmethod
    def _latched_qos() -> QoSProfile:
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def _action_server(self, action_type, name: str, execute_callback, goal_callback=None):
        return ActionServer(
            self,
            action_type,
            name,
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )

    def _create_capability_servers(self) -> None:
        self.action_servers = [
            self._action_server(
                NavigateToPose,
                "/navigate_to_pose",
                self._navigate,
                self._navigation_goal,
            ),
            self._action_server(DockToStation, "/dock_to_station", self._dock),
            self._action_server(
                DetectObject,
                "/detect_object",
                self._detect,
                self._detect_goal,
            ),
            self._action_server(
                PickObject,
                "/pick_object",
                self._pick,
                self._pick_goal,
            ),
            self._action_server(PlaceObject, "/place_object", self._place),
            self._action_server(ReadOrder, "/read_delivery_order", self._read_order),
            self._action_server(
                InspectRetailScene,
                "/inspect_retail_scene",
                self._inspect,
            ),
            self._action_server(
                VerifyPlacement,
                "/verify_placement",
                self._verify,
            ),
            self._action_server(
                SetPerceptionMode,
                "/set_perception_mode",
                self._set_perception_mode,
            ),
        ]

    def _navigation_goal(self, _request) -> GoalResponse:
        if self.navigation_active:
            return GoalResponse.REJECT
        self.navigation_active = True
        return GoalResponse.ACCEPT

    def _detect_goal(self, request: DetectObject.Goal) -> GoalResponse:
        try:
            self.detections.validate_detect_request(
                request.object_id,
                request.station_id,
                request.arm_name,
            )
        except ValueError as exc:
            self.get_logger().error(f"Rejecting invalid truth detect goal: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _pick_goal(self, request: PickObject.Goal) -> GoalResponse:
        try:
            self.detections.validate(
                request.detection_id,
                request.object_id,
                self.current_station,
                request.arm_name,
            )
        except ValueError as exc:
            self.get_logger().error(f"Rejecting invalid truth pick goal: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _sleep(self, seconds: float) -> None:
        future = Future()
        timer_holder = []

        def wake() -> None:
            if not future.done():
                future.set_result(None)
            timer_holder[0].cancel()

        timer_holder.append(
            self.create_timer(
                max(0.001, seconds), wake, callback_group=self.callback_group
            )
        )
        await future

    def _scaled_delay(self, nominal_sec: float) -> float:
        return max(
            float(self.get_parameter("minimum_action_delay_sec").value),
            nominal_sec * float(self.get_parameter("time_scale").value),
        )

    @staticmethod
    def _yaw(pose) -> float:
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _station_near(self, x: float, y: float) -> str:
        nearest = ""
        distance = math.inf
        for station_id, station in self.stations.items():
            value = math.hypot(station.nav_pose.x - x, station.nav_pose.y - y)
            if value < distance:
                nearest = station_id
                distance = value
        return nearest if distance <= 0.10 else "unregistered_pose"

    async def _navigate(self, goal_handle):
        result = NavigateToPose.Result()
        target = goal_handle.request.pose.pose
        dx = float(target.position.x) - self.base_pose[0]
        dy = float(target.position.y) - self.base_pose[1]
        distance = math.hypot(dx, dy)
        speed = max(0.05, float(self.get_parameter("navigation_speed_mps").value))
        duration = self._scaled_delay(distance / speed)
        if distance > 1.0e-6:
            self.base_twist[0] = distance / max(duration, 0.01)
        try:
            elapsed = 0.0
            while elapsed < duration:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                step = min(0.05, duration - elapsed)
                await self._sleep(step)
                elapsed += step
            self.base_pose = [
                float(target.position.x),
                float(target.position.y),
                self._yaw(target),
            ]
            self.current_station = self._station_near(self.base_pose[0], self.base_pose[1])
            xmin, xmax, ymin, ymax = self.world.map_bounds
            outside = not (
                xmin <= self.base_pose[0] <= xmax and ymin <= self.base_pose[1] <= ymax
            )
            if outside or self.random.random() < float(
                self.get_parameter("out_of_bounds_probability").value
            ):
                self.world.record_out_of_bounds()
            if self.random.random() < float(
                self.get_parameter("collision_probability").value
            ):
                self.world.record_collision()
            goal_handle.succeed()
            return result
        finally:
            self.base_twist = [0.0, 0.0, 0.0]
            self.navigation_active = False

    async def _dock(self, goal_handle):
        request = goal_handle.request
        result = DockToStation.Result()
        await self._sleep(self._scaled_delay(1.0))
        if self.stations.get(request.station_id) is None:
            result.error_code = TaskError.STATION_NOT_FOUND
            result.message = "unknown simulated station"
            goal_handle.abort()
            return result
        self.current_station = request.station_id
        result.success = True
        result.error_code = TaskError.OK
        result.message = "truth docking completed"
        goal_handle.succeed()
        return result

    async def _detect(self, goal_handle):
        request = goal_handle.request
        result = DetectObject.Result()
        await self._sleep(self._scaled_delay(0.8))
        found = self.world.detect(request.object_id, request.station_id)
        result.found = found
        result.confidence = 0.99 if found else 0.0
        result.detection_id = (
            self.detections.create(
                request.object_id,
                request.station_id,
                request.arm_name,
            )
            if found
            else ""
        )
        result.object_pose.header.stamp = self.get_clock().now().to_msg()
        result.object_pose.header.frame_id = "base_link"
        result.object_pose.pose.position.x = 0.55
        result.object_pose.pose.position.z = 0.80
        result.object_pose.pose.orientation.w = 1.0
        result.error_code = TaskError.OK if found else TaskError.OBJECT_NOT_FOUND
        result.message = "truth object pose" if found else "object is not at station"
        if found:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _pick(self, goal_handle):
        request = goal_handle.request
        result = PickObject.Result()
        try:
            self.detections.consume(
                request.detection_id,
                request.object_id,
                self.current_station,
                request.arm_name,
            )
        except ValueError as exc:
            result.error_code = TaskError.INVALID_REQUEST
            result.message = str(exc)
            goal_handle.abort()
            return result
        await self._sleep(self._scaled_delay(2.0))
        success = self.world.pick(request.object_id, self.current_station)
        if success and self.random.random() < float(
            self.get_parameter("drop_probability").value
        ):
            self.world.drop_held()
            success = False
        result.success = success
        result.object_attached = success
        result.error_code = TaskError.OK if success else TaskError.PICK_FAILED
        result.message = "truth grasp attached" if success else "truth grasp failed"
        self.held_arm = request.arm_name if success else ""
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _place(self, goal_handle):
        request = goal_handle.request
        result = PlaceObject.Result()
        await self._sleep(self._scaled_delay(2.0))
        success = self.world.place(request.object_id, self.current_station)
        result.success = success
        result.object_released = success
        result.error_code = TaskError.OK if success else TaskError.PLACE_FAILED
        result.message = "truth placement released" if success else "no held object"
        if success:
            self.held_arm = ""
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _read_order(self, goal_handle):
        result = ReadOrder.Result()
        await self._sleep(self._scaled_delay(1.5))
        for entry in self.world.order:
            line = OrderLine()
            line.sku = entry.sku
            line.display_name = entry.display_name
            line.specification = entry.specification
            line.quantity = 1
            line.confidence = 0.99
            result.lines.append(line)
        result.success = True
        result.error_code = TaskError.OK
        result.message = "truth order generated from hidden match state"
        goal_handle.succeed()
        return result

    async def _inspect(self, goal_handle):
        request = goal_handle.request
        result = InspectRetailScene.Result()
        await self._sleep(self._scaled_delay(1.2))
        phase = (
            PHASE_RESTOCKING
            if request.mode == InspectRetailScene.Goal.MISSING_STOCK
            else PHASE_RETURN_MISPLACED
        )
        for work in self.world.work_visible(phase, request.station_id):
            item = CompetitionWorkItem()
            item.kind = (
                CompetitionWorkItem.RESTOCK
                if phase == PHASE_RESTOCKING
                else CompetitionWorkItem.RETURN_MISPLACED
            )
            item.work_id = work.work_id
            item.object_id = work.object_id
            item.sku = work.sku
            item.source_station = work.source_station
            item.destination_station = work.destination_station
            item.destination_slot = work.destination_slot
            item.confidence = 0.99
            item.evidence_id = f"truth:{work.work_id}:{request.station_id}"
            result.work_items.append(item)
        result.success = True
        result.error_code = TaskError.OK
        result.message = "truth semantic inspection"
        goal_handle.succeed()
        return result

    async def _verify(self, goal_handle):
        request = goal_handle.request
        result = VerifyPlacement.Result()
        await self._sleep(self._scaled_delay(0.8))
        success = self.world.verify(
            request.object_id, request.station_id, request.slot_id
        )
        result.success = success
        result.stable = success
        result.position_error = 0.005 if success else 1.0
        result.confidence = 0.99 if success else 0.0
        result.error_code = TaskError.OK if success else TaskError.PLACEMENT_VERIFY_FAILED
        result.message = "truth placement verified" if success else "wrong station or slot"
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _set_perception_mode(self, goal_handle):
        result = SetPerceptionMode.Result()
        await self._sleep(self._scaled_delay(0.3))
        self.current_mode = int(goal_handle.request.mode)
        result.success = True
        result.active_mode = self.current_mode
        result.error_code = TaskError.OK
        result.message = "truth perception mode ready"
        goal_handle.succeed()
        return result

    def _reset_service(self, _request, response):
        if self.phase_goal_active:
            response.success = False
            response.message = "cannot reset while a simulated phase is active"
            return response
        snapshot = self.world.reset()
        self.random.seed(self.world.seed + 991)
        self.base_pose = [0.0, 0.0, 0.0]
        self.base_twist = [0.0, 0.0, 0.0]
        self.current_station = "start"
        self.held_arm = ""
        self.detections.clear()
        self.referee_phase = 0
        self.phase_failed = False
        self.phase_durations = {}
        self.started_wall = 0.0
        self.run_id = f"sim-{self.world.seed}"
        if self.coordinator_reset_client.service_is_ready():
            self.coordinator_reset_client.call_async(Trigger.Request())
        response.success = True
        response.message = json.dumps(
            {"run_id": self.run_id, "seed": snapshot["seed"]}, sort_keys=True
        )
        self._publish_status()
        return response

    def _start_next_service(self, _request, response):
        success, message = self._start_next_phase()
        response.success = success
        response.message = message
        return response

    def _start_next_phase(self):
        if self.phase_goal_active:
            return False, "a simulated phase is already active"
        if self.phase_failed:
            return False, "simulation is failure-latched; reset the match"
        if self.referee_phase >= PHASE_RETURN_MISPLACED:
            return False, "all simulated phases are complete"
        if not self.phase_client.server_is_ready():
            return False, "competition coordinator action is not ready"
        phase = self.referee_phase + 1
        goal = ExecuteCompetitionPhase.Goal()
        goal.run_id = self.run_id
        goal.phase = phase
        goal.dry_run = False
        goal.timeout_sec = 420.0 if phase < 3 else 360.0
        self.phase_goal_active = True
        self.phase_started_wall = time.monotonic()
        if self.started_wall == 0.0:
            self.started_wall = self.phase_started_wall
        future = self.phase_client.send_goal_async(goal)
        future.add_done_callback(self._phase_goal_response)
        self.get_logger().info(f"virtual referee started phase {phase}")
        return True, f"phase {phase} start command sent"

    def _phase_goal_response(self, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                self.phase_goal_active = False
                self.phase_failed = True
                self.get_logger().error("competition coordinator rejected referee goal")
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(self._phase_result)
        except Exception as error:
            self.phase_goal_active = False
            self.phase_failed = True
            self.get_logger().error(f"failed to send phase goal: {error}")

    def _phase_result(self, future) -> None:
        self.phase_goal_active = False
        wrapped = future.result()
        result = wrapped.result
        duration = time.monotonic() - self.phase_started_wall
        attempted_phase = self.referee_phase + 1
        self.phase_durations[str(attempted_phase)] = duration
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.referee_phase = attempted_phase
            self.last_phase_finished_wall = time.monotonic()
            self.get_logger().info(
                f"virtual referee accepted phase {attempted_phase}; "
                f"score={self.world.score}"
            )
            if self.referee_phase == PHASE_RETURN_MISPLACED:
                self._write_report()
                if bool(self.get_parameter("auto_run").value) and bool(
                    self.get_parameter("exit_on_complete").value
                ):
                    self.shutdown_timer = self.create_timer(0.5, self._shutdown_after_match)
        else:
            self.phase_failed = True
            self.get_logger().error(
                f"phase {attempted_phase} failed: code={result.error_code}, "
                f"message={result.message}"
            )
        self._publish_status()

    def _shutdown_after_match(self) -> None:
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
        self.get_logger().info("automatic match complete; shutting down simulation")
        if rclpy.ok():
            rclpy.shutdown()

    def _auto_referee(self) -> None:
        if not bool(self.get_parameter("auto_run").value):
            return
        if self.phase_goal_active or self.phase_failed or self.referee_phase >= 3:
            return
        pause = float(self.get_parameter("auto_judge_pause_sec").value)
        if self.referee_phase == 0 or time.monotonic() - self.last_phase_finished_wall >= pause:
            self._start_next_phase()

    def _inject_collision(self, _request, response):
        self.world.record_collision()
        response.success = True
        response.message = "collision penalty injected (-5)"
        self._publish_status()
        return response

    def _inject_drop(self, _request, response):
        dropped = self.world.drop_held()
        if not dropped:
            self.world.drops += 1
        response.success = True
        response.message = "drop penalty injected (-5)"
        self._publish_status()
        return response

    def _inject_out_of_bounds(self, _request, response):
        self.world.record_out_of_bounds()
        response.success = True
        response.message = "out-of-bounds penalty injected (-10)"
        self._publish_status()
        return response

    def _status(self) -> dict:
        snapshot = self.world.snapshot()
        snapshot.update(
            {
                "run_id": self.run_id,
                "referee_phase": self.referee_phase,
                "phase_goal_active": self.phase_goal_active,
                "phase_failed": self.phase_failed,
                "perception_mode": self.current_mode,
                "current_station": self.current_station,
                "base_pose": list(self.base_pose),
                "phase_durations_wall_sec": dict(self.phase_durations),
                "elapsed_wall_sec": (
                    0.0 if self.started_wall == 0.0 else time.monotonic() - self.started_wall
                ),
                "truth_mode": True,
            }
        )
        return snapshot

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(self._status(), ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(message)

    def _write_report(self) -> None:
        path = Path(str(self.get_parameter("report_path").value)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(self._status(), ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        self.get_logger().info(f"simulation report written to {path}")

    def _publish_odom(self) -> None:
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = deepcopy(stamp)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.base_pose[0]
        odom.pose.pose.position.y = self.base_pose[1]
        half = 0.5 * self.base_pose[2]
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = self.base_twist[0]
        odom.twist.twist.linear.y = self.base_twist[1]
        odom.twist.twist.angular.z = self.base_twist[2]
        self.odom_pub.publish(odom)

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = deepcopy(stamp)
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = deepcopy(stamp)
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.translation.x = self.base_pose[0]
        odom_to_base.transform.translation.y = self.base_pose[1]
        odom_to_base.transform.rotation.z = math.sin(half)
        odom_to_base.transform.rotation.w = math.cos(half)
        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupermarketSimulationNode()
    executor = MultiThreadedExecutor(num_threads=12)
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
