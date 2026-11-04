"""Referee-gated coordinator for the three supermarket competition phases."""

import math
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger

from openarmx_task_msgs.action import (
    DockToStation,
    ExecuteCompetitionPhase,
    ExecuteTransport,
    InspectRetailScene,
    ReadOrder,
    SetPerceptionMode,
    VerifyPlacement,
)
from openarmx_task_msgs.msg import CompetitionWorkItem, TaskError

from .competition_config import (
    PHASE_PICKING,
    PHASE_RESTOCKING,
    PHASE_RETURN_MISPLACED,
    CompetitionConfig,
    PhasePolicy,
)
from .competition_state import CompetitionRunState
from .station_db import StationDatabase
from .shelf_inventory_db import ShelfInventoryDatabase, InventorySlot


class CompetitionCoordinatorNode(Node):
    """Runs one autonomous phase per referee-authorized action goal."""

    def __init__(self) -> None:
        super().__init__("openarmx_competition_coordinator")
        package_share = get_package_share_directory("openarmx_task_executor")
        self.declare_parameter(
            "competition_config", os.path.join(package_share, "config", "competition.yaml")
        )
        self.declare_parameter(
            "station_config", os.path.join(package_share, "config", "stations.yaml")
        )
        self.declare_parameter("station_mission", "")
        self.declare_parameter("action_name", "/competition/execute_phase")
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("dock_action_name", "/dock_to_station")
        self.declare_parameter("transport_action_name", "/execute_transport")
        self.declare_parameter("read_order_action_name", "/read_delivery_order")
        self.declare_parameter("inspect_action_name", "/inspect_retail_scene")
        self.declare_parameter("verify_action_name", "/verify_placement")
        self.declare_parameter("perception_mode_action_name", "/set_perception_mode")
        self.declare_parameter("odom_topic", "/odom_safe")
        self.declare_parameter("server_wait_sec", 3.0)
        self.declare_parameter("base_stop_linear_speed", 0.02)
        self.declare_parameter("base_stop_angular_speed", 0.03)
        self.declare_parameter("base_stop_hold_sec", 0.5)
        self.declare_parameter("odom_timeout_sec", 0.5)

        self.config = CompetitionConfig.from_yaml(
            str(self.get_parameter("competition_config").value)
        )
        self.inventory: Optional[ShelfInventoryDatabase] = None
        if self.config.inventory.enabled:
            self.inventory = ShelfInventoryDatabase(self.config.inventory.database_path)
            self.get_logger().info(
                f"Loaded read-only shelf inventory database: {self.inventory.path}"
            )
        self.stations = StationDatabase.from_yaml(
            str(self.get_parameter("station_config").value),
            str(self.get_parameter("station_mission").value),
        )
        self._validate_station_references()
        self.state = CompetitionRunState()
        self.phase_elapsed_sec = 0.0

        self.server_wait_sec = float(self.get_parameter("server_wait_sec").value)
        self.base_stop_linear_speed = float(
            self.get_parameter("base_stop_linear_speed").value
        )
        self.base_stop_angular_speed = float(
            self.get_parameter("base_stop_angular_speed").value
        )
        self.base_stop_hold_sec = float(self.get_parameter("base_stop_hold_sec").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.last_odom_time = 0.0
        self.last_linear_speed = math.inf
        self.last_angular_speed = math.inf
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("nav_action_name").value)
        )
        self.dock_client = ActionClient(
            self, DockToStation, str(self.get_parameter("dock_action_name").value)
        )
        self.transport_client = ActionClient(
            self,
            ExecuteTransport,
            str(self.get_parameter("transport_action_name").value),
        )
        self.read_order_client = ActionClient(
            self, ReadOrder, str(self.get_parameter("read_order_action_name").value)
        )
        self.inspect_client = ActionClient(
            self,
            InspectRetailScene,
            str(self.get_parameter("inspect_action_name").value),
        )
        self.verify_client = ActionClient(
            self,
            VerifyPlacement,
            str(self.get_parameter("verify_action_name").value),
        )
        self.perception_mode_client = ActionClient(
            self,
            SetPerceptionMode,
            str(self.get_parameter("perception_mode_action_name").value),
        )
        self.server = ActionServer(
            self,
            ExecuteCompetitionPhase,
            str(self.get_parameter("action_name").value),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.reset_service = self.create_service(
            Trigger, "/competition/reset_run", self._reset_run
        )
        self.get_logger().info(
            "Competition coordinator ready; expected phase=1, control=autonomous"
        )

    def _reset_run(self, _request, response):
        if self.state.active_phase:
            response.success = False
            response.message = "cannot reset while a competition phase is active"
            return response
        self.state.reset()
        self.phase_elapsed_sec = 0.0
        response.success = True
        response.message = "competition run reset; expected phase=1"
        self.get_logger().warn(response.message)
        return response

    def _validate_station_references(self) -> None:
        required = set()
        for policy in self.config.phases.values():
            required.add(policy.entry_station)
            required.add(policy.judging_station)
            required.update(policy.scan_tour)
        for route in self.config.sku_routes.values():
            required.add(route.source_station)
            required.add(route.destination_station)
        if self.config.inventory.enabled:
            required.update(self.config.inventory.shelf_face_stations.values())
            required.add(self.config.inventory.default_destination_station)
        missing = sorted(station for station in required if self.stations.get(station) is None)
        if missing:
            raise ValueError(f"competition config references unknown stations: {missing}")

    def _odom_callback(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        self.last_linear_speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
        self.last_angular_speed = abs(angular.z)
        self.last_odom_time = time.monotonic()

    def goal_callback(self, request: ExecuteCompetitionPhase.Goal) -> GoalResponse:
        if not self.state.can_start(str(request.run_id), int(request.phase)):
            self.get_logger().error(
                f"Refusing run={request.run_id!r} phase={request.phase}; "
                f"expected={self.state.expected_phase}, failed={self.state.failed_phase}"
            )
            return GoalResponse.REJECT
        self.state.start(str(request.run_id), int(request.phase))
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _sleep(self, seconds: float) -> None:
        future = Future()
        timers = []

        def wake() -> None:
            if not future.done():
                future.set_result(None)
            timers[0].cancel()

        timers.append(self.create_timer(max(0.001, seconds), wake))
        await future

    def _cancelled(self, goal_handle) -> bool:
        return bool(goal_handle.is_cancel_requested)

    def _feedback(
        self,
        goal_handle,
        state: str,
        completed: int,
        required: int,
        active_work_id: str = "",
        progress: Optional[float] = None,
    ) -> None:
        feedback = ExecuteCompetitionPhase.Feedback()
        feedback.state = state
        feedback.active_work_id = active_work_id
        feedback.required_items = int(required)
        feedback.completed_items = int(completed)
        feedback.progress = float(
            progress if progress is not None else completed / max(1, required)
        )
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        success: bool,
        code: int,
        required: int,
        completed: int,
        waiting: bool,
        message: str,
    ) -> ExecuteCompetitionPhase.Result:
        result = ExecuteCompetitionPhase.Result()
        result.success = success
        result.error_code = int(code)
        result.required_items = int(required)
        result.completed_items = int(completed)
        result.waiting_for_referee = waiting
        result.message = message
        return result

    async def _wait_for_server(self, client, goal_handle, deadline: float) -> bool:
        wait_deadline = min(deadline, time.monotonic() + self.server_wait_sec)
        while not client.server_is_ready():
            if self._cancelled(goal_handle) or time.monotonic() >= wait_deadline:
                return False
            client.wait_for_server(timeout_sec=0.2)
            await self._sleep(0.02)
        return True

    async def _call_action(self, client, request, goal_handle, deadline: float):
        if not await self._wait_for_server(client, goal_handle, deadline):
            return None
        child_handle = await client.send_goal_async(request)
        if not child_handle.accepted:
            return None
        future = child_handle.get_result_async()
        while not future.done():
            if self._cancelled(goal_handle) or time.monotonic() >= deadline:
                await child_handle.cancel_goal_async()
                return None
            await self._sleep(0.05)
        wrapped = future.result()
        return wrapped.status, wrapped.result

    async def _wait_for_base_stop(self, goal_handle, deadline: float) -> bool:
        stable_since = None
        while not self._cancelled(goal_handle) and time.monotonic() < deadline:
            now = time.monotonic()
            fresh = now - self.last_odom_time <= self.odom_timeout_sec
            stopped = (
                self.last_linear_speed <= self.base_stop_linear_speed
                and self.last_angular_speed <= self.base_stop_angular_speed
            )
            if fresh and stopped:
                stable_since = stable_since or now
                if now - stable_since >= self.base_stop_hold_sec:
                    return True
            else:
                stable_since = None
            await self._sleep(0.05)
        return False

    async def _navigate(self, station_id: str, goal_handle, deadline: float) -> int:
        station = self.stations.get(station_id)
        if station is None:
            return TaskError.STATION_NOT_FOUND
        request = NavigateToPose.Goal()
        request.pose = station.nav_pose.to_msg(self.get_clock().now().to_msg())
        response = await self._call_action(self.nav_client, request, goal_handle, deadline)
        if response is None:
            return (
                TaskError.CANCELLED
                if self._cancelled(goal_handle)
                else TaskError.NAVIGATION_UNAVAILABLE
            )
        status, _ = response
        if status != GoalStatus.STATUS_SUCCEEDED:
            return TaskError.NAVIGATION_FAILED
        if not await self._wait_for_base_stop(goal_handle, deadline):
            return TaskError.BASE_NOT_STOPPED
        return TaskError.OK

    async def _dock(self, station_id: str, goal_handle, deadline: float) -> int:
        station = self.stations.get(station_id)
        if station is None:
            return TaskError.STATION_NOT_FOUND
        if not station.docking_enabled:
            return TaskError.OK
        request = DockToStation.Goal()
        request.station_id = station.station_id
        request.marker_id = station.marker_id
        request.position_tolerance = station.position_tolerance
        request.yaw_tolerance = station.yaw_tolerance
        request.timeout_sec = max(0.0, deadline - time.monotonic())
        if station.target_marker_pose is not None:
            request.use_target_marker_pose = True
            request.target_marker_pose = station.target_marker_pose.to_msg(
                self.get_clock().now().to_msg()
            ).pose
        response = await self._call_action(self.dock_client, request, goal_handle, deadline)
        if response is None:
            return TaskError.DOCKING_UNAVAILABLE
        status, payload = response
        if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
            return TaskError.DOCKING_FAILED
        if not await self._wait_for_base_stop(goal_handle, deadline):
            return TaskError.BASE_NOT_STOPPED
        return TaskError.OK

    async def _arrive(self, station_id: str, dock: bool, goal_handle, deadline: float) -> int:
        code = await self._navigate(station_id, goal_handle, deadline)
        if code == TaskError.OK and dock:
            code = await self._dock(station_id, goal_handle, deadline)
        return code

    async def _set_perception_mode(
        self, mode: int, reason: str, goal_handle, deadline: float
    ) -> int:
        if not self.config.perception_mode_control:
            return TaskError.OK
        request = SetPerceptionMode.Goal()
        request.mode = int(mode)
        request.reason = reason
        request.timeout_sec = max(0.0, deadline - time.monotonic())
        response = await self._call_action(
            self.perception_mode_client, request, goal_handle, deadline
        )
        if response is None:
            return TaskError.DETECTION_UNAVAILABLE
        status, payload = response
        return (
            TaskError.OK
            if status == GoalStatus.STATUS_SUCCEEDED and payload.success
            else TaskError.DETECTION_UNAVAILABLE
        )

    async def _preflight(self, phase: int, goal_handle, deadline: float) -> int:
        clients = [(self.nav_client, TaskError.NAVIGATION_UNAVAILABLE)]
        if phase == PHASE_PICKING:
            clients.append((self.read_order_client, TaskError.ORDER_READ_FAILED))
            entry = self.config.phases[phase].entry_station
            if self.stations.get(entry).docking_enabled:
                clients.append((self.dock_client, TaskError.DOCKING_UNAVAILABLE))
        else:
            clients.append((self.inspect_client, TaskError.INSPECTION_FAILED))
        clients.append((self.transport_client, TaskError.MOVEIT_UNAVAILABLE))
        if self.config.phases[phase].verify_placement:
            clients.append((self.verify_client, TaskError.PLACEMENT_VERIFY_FAILED))
        if self.config.perception_mode_control:
            clients.append((self.perception_mode_client, TaskError.DETECTION_UNAVAILABLE))
        for client, code in clients:
            if not await self._wait_for_server(client, goal_handle, deadline):
                return TaskError.CANCELLED if self._cancelled(goal_handle) else code
        return TaskError.OK

    async def _read_order(self, policy: PhasePolicy, goal_handle, deadline: float):
        request = ReadOrder.Goal()
        request.station_id = policy.entry_station
        request.expected_distinct_skus = policy.required_items
        request.min_confidence = self.config.min_order_confidence
        request.timeout_sec = max(0.0, deadline - time.monotonic())
        response = await self._call_action(self.read_order_client, request, goal_handle, deadline)
        if response is None:
            return [], TaskError.ORDER_READ_FAILED
        status, payload = response
        if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
            return [], TaskError.ORDER_READ_FAILED
        distinct = {line.sku for line in payload.lines}
        total = sum(int(line.quantity) for line in payload.lines)
        if len(distinct) != policy.required_items or total != policy.required_items:
            return [], TaskError.ORDER_INVALID
        work: List[CompetitionWorkItem] = []
        for line in payload.lines:
            if not line.sku or line.confidence < self.config.min_order_confidence:
                return [], TaskError.ORDER_INVALID
            configured_route = self.config.try_route_for(line.sku, line.specification)
            inventory_slot: Optional[InventorySlot] = None
            inventory_station = ""
            if self.inventory is not None:
                inventory_slot = self.inventory.select_sku_slot(
                    line.sku, self.config.inventory.shelf_face_stations.keys()
                )
                if inventory_slot is not None:
                    inventory_station = self.config.inventory.station_for(
                        inventory_slot.shelf_id, inventory_slot.face
                    ) or ""
            use_inventory = (
                inventory_slot is not None
                and bool(inventory_station)
                and (
                    self.config.inventory.prefer_inventory
                    or configured_route is None
                    or not configured_route.source_slot
                )
            )
            if use_inventory:
                source_station = inventory_station
                source_slot = inventory_slot.slot_id
                destination_station = (
                    configured_route.destination_station
                    if configured_route is not None
                    else self.config.inventory.default_destination_station
                )
                object_id = (
                    configured_route.object_id
                    if configured_route is not None
                    else line.sku
                )
                destination_slot = (
                    configured_route.destination_slot
                    if configured_route is not None
                    else ""
                )
            elif configured_route is not None:
                route = configured_route
                source_station = route.source_station
                source_slot = route.source_slot
                destination_station = route.destination_station
                destination_slot = route.destination_slot
                object_id = route.object_id
            else:
                self.get_logger().error(
                    f"no route or inventory location for SKU {line.sku!r}"
                )
                return [], TaskError.WORK_PLAN_INCOMPLETE
            for index in range(int(line.quantity)):
                item = CompetitionWorkItem()
                item.kind = CompetitionWorkItem.PICK_ORDER
                item.work_id = f"pick:{line.sku}:{index + 1}"
                item.object_id = object_id
                item.sku = line.sku
                item.specification = line.specification
                item.source_station = source_station
                item.source_slot = source_slot
                item.destination_station = destination_station
                item.destination_slot = destination_slot
                if inventory_slot is not None and use_inventory:
                    item.observed_pose.header.frame_id = "map"
                    item.observed_pose.pose.position.x = inventory_slot.world_x
                    item.observed_pose.pose.position.y = inventory_slot.world_y
                    item.observed_pose.pose.position.z = inventory_slot.world_z
                    item.observed_pose.pose.orientation.w = 1.0
                item.confidence = line.confidence
                item.evidence_id = f"order:{line.sku}:{index + 1}"
                work.append(item)
        return work, TaskError.OK

    async def _scan(self, policy: PhasePolicy, goal_handle, deadline: float):
        mode = (
            InspectRetailScene.Goal.MISSING_STOCK
            if policy.phase == PHASE_RESTOCKING
            else InspectRetailScene.Goal.MISPLACED_ITEMS
        )
        by_key: Dict[str, CompetitionWorkItem] = {}
        for index, station_id in enumerate(policy.scan_tour):
            self._feedback(
                goal_handle,
                "DREAM_PATROL_NAVIGATION",
                0,
                policy.required_items,
                progress=0.05 + 0.25 * index / max(1, len(policy.scan_tour)),
            )
            code = await self._arrive(station_id, False, goal_handle, deadline)
            if code != TaskError.OK:
                return [], code
            request = InspectRetailScene.Goal()
            request.mode = mode
            request.station_id = station_id
            request.max_results = policy.required_items
            request.use_dream_memory = policy.use_dream
            request.min_confidence = self.config.min_inspection_confidence
            request.timeout_sec = max(0.0, deadline - time.monotonic())
            response = await self._call_action(
                self.inspect_client, request, goal_handle, deadline
            )
            if response is None:
                return [], TaskError.INSPECTION_FAILED
            status, payload = response
            if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
                return [], TaskError.INSPECTION_FAILED
            for item in payload.work_items:
                key = item.work_id or item.evidence_id
                if not key or not item.source_station or not item.destination_station:
                    continue
                previous = by_key.get(key)
                if previous is None or item.confidence > previous.confidence:
                    by_key[key] = item
        ranked = sorted(by_key.values(), key=lambda item: item.confidence, reverse=True)
        selected: List[CompetitionWorkItem] = []
        used_destinations = set()
        for item in ranked:
            destination_key = (item.destination_station, item.destination_slot)
            if destination_key in used_destinations:
                continue
            selected.append(item)
            used_destinations.add(destination_key)
            if len(selected) == policy.required_items:
                break
        if len(selected) != policy.required_items:
            return selected, TaskError.WORK_PLAN_INCOMPLETE
        return selected, TaskError.OK

    async def _transport(
        self, item: CompetitionWorkItem, goal_handle, deadline: float
    ) -> int:
        if (
            self.stations.get(item.source_station) is None
            or self.stations.get(item.destination_station) is None
        ):
            return TaskError.STATION_NOT_FOUND
        request = ExecuteTransport.Goal()
        request.task_id = item.work_id
        request.object_id = item.object_id or item.sku
        request.source_station = item.source_station
        request.destination_station = item.destination_station
        request.source_slot = item.source_slot
        request.destination_slot = item.destination_slot
        if item.source_slot:
            try:
                _, _, level, _ = ShelfInventoryDatabase.parse_slot_id(item.source_slot)
            except ValueError:
                level = -1
            if level >= 0:
                request.use_source_level = True
                request.source_level = int(level)
        request.use_nominal_object_pose = bool(item.observed_pose.header.frame_id)
        if request.use_nominal_object_pose:
            request.nominal_object_pose = item.observed_pose
        request.max_retries = self.config.max_transport_retries
        request.timeout_sec = max(0.0, deadline - time.monotonic())
        response = await self._call_action(
            self.transport_client, request, goal_handle, deadline
        )
        if response is None:
            return (
                TaskError.CANCELLED
                if self._cancelled(goal_handle)
                else TaskError.MOVEIT_UNAVAILABLE
            )
        status, payload = response
        if status != GoalStatus.STATUS_SUCCEEDED or not payload.success:
            return (
                int(payload.error_code)
                if payload is not None
                else TaskError.INTERNAL_ERROR
            )
        return TaskError.OK

    async def _verify(
        self, item: CompetitionWorkItem, goal_handle, deadline: float
    ) -> int:
        request = VerifyPlacement.Goal()
        request.object_id = item.object_id or item.sku
        request.sku = item.sku
        request.station_id = item.destination_station
        request.slot_id = item.destination_slot
        request.position_tolerance = 0.03
        request.timeout_sec = max(0.0, deadline - time.monotonic())
        response = await self._call_action(
            self.verify_client, request, goal_handle, deadline
        )
        if response is None:
            return TaskError.PLACEMENT_VERIFY_FAILED
        status, payload = response
        if (
            status != GoalStatus.STATUS_SUCCEEDED
            or not payload.success
            or not payload.stable
        ):
            return TaskError.PLACEMENT_VERIFY_FAILED
        return TaskError.OK

    async def _run_phase(
        self, policy: PhasePolicy, goal_handle, deadline: float
    ) -> Tuple[int, int]:
        completed = 0
        self._feedback(
            goal_handle,
            "PREFLIGHT",
            completed,
            policy.required_items,
            progress=0.01,
        )
        code = await self._preflight(policy.phase, goal_handle, deadline)
        if code != TaskError.OK:
            return code, completed

        if policy.phase == PHASE_PICKING:
            self._feedback(
                goal_handle,
                "NAVIGATE_TO_ORDER",
                completed,
                policy.required_items,
                progress=0.04,
            )
            code = await self._arrive(policy.entry_station, True, goal_handle, deadline)
            if code != TaskError.OK:
                return code, completed
            code = await self._set_perception_mode(
                SetPerceptionMode.Goal.TARGET_REFINE,
                "read delivery order",
                goal_handle,
                deadline,
            )
            if code != TaskError.OK:
                return code, completed
            self._feedback(
                goal_handle,
                "READ_AND_VALIDATE_ORDER",
                completed,
                policy.required_items,
                progress=0.10,
            )
            work_items, code = await self._read_order(policy, goal_handle, deadline)
        else:
            code = await self._set_perception_mode(
                SetPerceptionMode.Goal.DREAM_OBSERVE,
                "semantic shelf patrol",
                goal_handle,
                deadline,
            )
            if code != TaskError.OK:
                return code, completed
            work_items, code = await self._scan(policy, goal_handle, deadline)
        if code != TaskError.OK:
            return code, completed

        code = await self._set_perception_mode(
            SetPerceptionMode.Goal.LOCAL_GRASP,
            "execute validated retail work items",
            goal_handle,
            deadline,
        )
        if code != TaskError.OK:
            return code, completed
        for item in work_items:
            self._feedback(
                goal_handle,
                "TRANSPORT_ITEM",
                completed,
                policy.required_items,
                item.work_id,
                0.35 + 0.50 * completed / max(1, policy.required_items),
            )
            code = await self._transport(item, goal_handle, deadline)
            if code != TaskError.OK:
                return code, completed
            if policy.verify_placement:
                self._feedback(
                    goal_handle,
                    "VERIFY_PLACEMENT",
                    completed,
                    policy.required_items,
                    item.work_id,
                )
                code = await self._verify(item, goal_handle, deadline)
                if code != TaskError.OK:
                    return code, completed
            completed += 1

        code = await self._set_perception_mode(
            SetPerceptionMode.Goal.IDLE, "phase complete", goal_handle, deadline
        )
        if code != TaskError.OK:
            return code, completed
        self._feedback(
            goal_handle,
            "NAVIGATE_TO_JUDGING_AREA",
            completed,
            policy.required_items,
            progress=0.92,
        )
        code = await self._arrive(policy.judging_station, False, goal_handle, deadline)
        return code, completed

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        phase = int(request.phase)
        policy = self.config.phases[phase]
        started = time.monotonic()
        requested_timeout = float(request.timeout_sec)
        budget = (
            policy.timeout_sec
            if requested_timeout <= 0.0
            else min(policy.timeout_sec, requested_timeout)
        )
        total_remaining = self.config.total_timeout_sec - self.phase_elapsed_sec
        deadline = started + min(budget, total_remaining)
        completed = 0
        try:
            if request.dry_run:
                code = await self._preflight(phase, goal_handle, deadline)
                self.state.release_dry_run()
                if code == TaskError.OK:
                    goal_handle.succeed()
                    return self._result(
                        True,
                        code,
                        policy.required_items,
                        0,
                        False,
                        "dry-run preflight passed; no motion was commanded",
                    )
                goal_handle.abort()
                return self._result(
                    False,
                    code,
                    policy.required_items,
                    0,
                    False,
                    "dry-run preflight failed",
                )

            code, completed = await self._run_phase(policy, goal_handle, deadline)
            self.phase_elapsed_sec += time.monotonic() - started
            if code == TaskError.OK and completed == policy.required_items:
                self.state.complete()
                self._feedback(
                    goal_handle,
                    "WAITING_FOR_REFEREE",
                    completed,
                    policy.required_items,
                    progress=1.0,
                )
                goal_handle.succeed()
                waiting = phase != PHASE_RETURN_MISPLACED
                message = (
                    f"phase {phase} completed in judging area; wait for referee"
                    if waiting
                    else "all three phases completed at finish"
                )
                return self._result(
                    True,
                    TaskError.OK,
                    policy.required_items,
                    completed,
                    waiting,
                    message,
                )
            if self._cancelled(goal_handle):
                code = TaskError.CANCELLED
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.state.fail()
            return self._result(
                False,
                code,
                policy.required_items,
                completed,
                False,
                f"phase {phase} failed and is latched; "
                "referee-authorized reset/restart is required",
            )
        except Exception as error:
            self.get_logger().error(f"Unhandled competition coordinator failure: {error}")
            if self.state.active_phase:
                self.state.fail()
            goal_handle.abort()
            return self._result(
                False,
                TaskError.INTERNAL_ERROR,
                policy.required_items,
                completed,
                False,
                str(error),
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionCoordinatorNode()
    executor = MultiThreadedExecutor(num_threads=6)
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
