"""Final thesis mission: Nav2 -> AnyGrasp/TC-WAM -> ACT -> AnyGrasp/MoveIt."""

from copy import deepcopy
import math
import time
from typing import Optional, Tuple

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.action import (
    ComputeTcWam,
    ExecuteFixedPlace,
    ExecuteLocalAlignment,
    ExecuteMoveItGrasp,
    GenerateGrasps,
    RunPickDeliver,
)
from openflex_thesis_msgs.msg import ControlMode, MissionState, ObjectMemory
from openflex_thesis_msgs.srv import SetControlMode
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from .async_tools import ros_sleep
from .recovery_adapter import RecoveryMissionMixin


class MissionOrchestratorNode(RecoveryMissionMixin, Node):
    def __init__(self) -> None:
        super().__init__("mission_orchestrator")
        self.declare_parameter("mission_action", "/thesis/run_pick_deliver")
        self.declare_parameter("enable_recovery_research", False)
        self.declare_parameter("recovery_local_only", False)
        self.declare_parameter("recovery_forced_action", "")
        self.declare_parameter("recovery_method", "heuristic_memory")
        self.declare_parameter("recovery_model_path", "")
        self.declare_parameter("recovery_log_path", "")
        self.declare_parameter("recovery_scenario_id", "development")
        self.declare_parameter("recovery_partition", "development")
        self.declare_parameter("recovery_evidence_class", "unclassified")
        self.declare_parameter("recovery_seed", 7)
        self.declare_parameter("recovery_budget_sec", 300.0)
        self.declare_parameter("recovery_grasp_reserve_sec", 45.0)
        self.declare_parameter("reobserve_cost_sec", 140.0)
        self.declare_parameter("switch_grasp_cost_sec", 5.0)
        self.declare_parameter("reposition_cost_sec", 140.0)
        self.declare_parameter("reobserve_window_sec", 0.75)
        self.declare_parameter("recovery_min_base_shift_m", 0.08)
        self.declare_parameter("nav2_action", "/navigate_to_pose")
        self.declare_parameter("anygrasp_action", "/thesis/generate_grasps")
        self.declare_parameter("tc_wam_action", "/thesis/compute_tc_wam")
        self.declare_parameter("local_alignment_action", "/thesis/execute_local_alignment")
        self.declare_parameter("moveit_grasp_action", "/thesis/execute_moveit_grasp")
        self.declare_parameter("fixed_place_action", "/thesis/execute_fixed_place")
        self.declare_parameter("set_mode_service", "/thesis/set_control_mode")
        self.declare_parameter("memory_topic", "/thesis/object_memory")
        self.declare_parameter("measured_velocity_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("mission_state_topic", "/thesis/mission_state")
        self.declare_parameter("require_physical_affordance", True)
        self.declare_parameter("require_physical_perception", True)
        self.declare_parameter("observation_timeout_sec", 5.0)
        self.declare_parameter("memory_stream_timeout_sec", 0.5)
        self.declare_parameter("stop_timeout_sec", 3.0)
        self.declare_parameter("stopped_linear_mps", 0.02)
        self.declare_parameter("stopped_angular_rps", 0.03)
        self.declare_parameter("measured_timeout_sec", 0.25)
        self.declare_parameter("stop_confirm_sec", 0.50)
        self.declare_parameter("server_wait_timeout_sec", 5.0)
        self.declare_parameter("anygrasp_timeout_sec", 10.0)
        self.declare_parameter("tc_wam_timeout_sec", 120.0)
        self.declare_parameter("local_alignment_timeout_sec", 30.0)
        self.declare_parameter("moveit_grasp_timeout_sec", 45.0)
        self.declare_parameter("fixed_place_timeout_sec", 30.0)
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("nav2_action").value),
            callback_group=self.callback_group,
        )
        self.anygrasp_client = ActionClient(
            self,
            GenerateGrasps,
            str(self.get_parameter("anygrasp_action").value),
            callback_group=self.callback_group,
        )
        self.tc_wam_client = ActionClient(
            self,
            ComputeTcWam,
            str(self.get_parameter("tc_wam_action").value),
            callback_group=self.callback_group,
        )
        self.alignment_client = ActionClient(
            self,
            ExecuteLocalAlignment,
            str(self.get_parameter("local_alignment_action").value),
            callback_group=self.callback_group,
        )
        self.moveit_client = ActionClient(
            self,
            ExecuteMoveItGrasp,
            str(self.get_parameter("moveit_grasp_action").value),
            callback_group=self.callback_group,
        )
        self.place_client = ActionClient(
            self,
            ExecuteFixedPlace,
            str(self.get_parameter("fixed_place_action").value),
            callback_group=self.callback_group,
        )
        self.mode_client = self.create_client(
            SetControlMode,
            str(self.get_parameter("set_mode_service").value),
            callback_group=self.callback_group,
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_publisher = self.create_publisher(
            MissionState, str(self.get_parameter("mission_state_topic").value), qos
        )
        self.create_subscription(
            ObjectMemory,
            str(self.get_parameter("memory_topic").value),
            self.on_memory,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("measured_velocity_topic").value),
            self.on_odometry,
            20,
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            RunPickDeliver,
            str(self.get_parameter("mission_action").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        self.latest_memory: Optional[ObjectMemory] = None
        self.latest_memory_receipt = 0.0
        self.latest_observation_receipt = 0.0
        self.latest_odometry: Optional[Odometry] = None
        self.latest_odometry_receipt = 0.0
        self.recovery_tf_buffer = Buffer()
        self.recovery_tf_listener = TransformListener(self.recovery_tf_buffer, self)
        self.busy = False
        self.current_phase = MissionState.IDLE
        self.current_attempt = 0
        self.current_grasp_id = 0
        self.current_ready_region_id = ""

    def on_memory(self, msg: ObjectMemory) -> None:
        if self.latest_memory is None or msg.observation_sequence != self.latest_memory.observation_sequence:
            self.latest_observation_receipt = time.monotonic()
        self.latest_memory = deepcopy(msg)
        self.latest_memory_receipt = time.monotonic()

    def on_odometry(self, msg: Odometry) -> None:
        if getattr(self, "record_recovery_distance", False) and self.latest_odometry is not None:
            previous = self.latest_odometry.pose.pose.position
            current = msg.pose.pose.position
            distance = math.hypot(current.x - previous.x, current.y - previous.y)
            self.recovery_distance_m += distance
        self.latest_odometry = deepcopy(msg)
        self.latest_odometry_receipt = time.monotonic()

    def goal_callback(self, request: RunPickDeliver.Goal) -> GoalResponse:
        valid = bool(
            request.task_id
            and request.object_id
            and request.destination_id in ("", "delivery_zone", "sim_delivery_top_left")
            and request.active_arm in (request.ARM_LEFT, request.ARM_RIGHT)
        )
        return GoalResponse.ACCEPT if not self.busy and valid else GoalResponse.REJECT

    def publish_phase(
        self,
        goal_handle,
        phase: int,
        detail: str,
        score: float = 0.0,
        error_code: int = 0,
    ) -> None:
        self.current_phase = phase
        request = goal_handle.request
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = request.task_id
        msg.object_id = request.object_id
        msg.phase = phase
        msg.attempt = self.current_attempt
        msg.map_version = self.latest_memory.map_version if self.latest_memory else 0
        msg.grasp_id = self.current_grasp_id
        msg.ready_region_id = self.current_ready_region_id
        msg.current_affordance_score = score
        msg.detail = detail
        msg.error_code = error_code
        self.state_publisher.publish(msg)
        feedback = RunPickDeliver.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        feedback.current_affordance_score = score
        feedback.recovery_attempts = self.current_attempt
        goal_handle.publish_feedback(feedback)

    async def set_mode(self, mode: int, reason: str) -> Tuple[bool, str]:
        wait = float(self.get_parameter("server_wait_timeout_sec").value)
        if not self.mode_client.wait_for_service(timeout_sec=wait):
            return False, "control-mode service unavailable"
        request = SetControlMode.Request()
        request.requested_mode = mode
        request.requester = self.get_name()
        request.reason = reason
        response = await self.mode_client.call_async(request)
        return bool(response and response.accepted), response.message if response else "no response"

    async def navigate(self, pose) -> Tuple[bool, str]:
        wait = float(self.get_parameter("server_wait_timeout_sec").value)
        if not self.nav_client.wait_for_server(timeout_sec=wait):
            return False, "Nav2 action server unavailable"
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(pose)
        handle = await self.nav_client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return False, "Nav2 goal rejected"
        wrapper = await handle.get_result_async()
        return wrapper.status == GoalStatus.STATUS_SUCCEEDED, f"Nav2 status={wrapper.status}"

    async def wait_for_stop(self, goal_handle) -> bool:
        timeout = float(self.get_parameter("stop_timeout_sec").value)
        required = float(self.get_parameter("stop_confirm_sec").value)
        started = time.monotonic()
        stopped_since = None
        while time.monotonic() - started <= timeout:
            if goal_handle.is_cancel_requested:
                return False
            fresh = bool(
                self.latest_odometry is not None
                and time.monotonic() - self.latest_odometry_receipt
                <= float(self.get_parameter("measured_timeout_sec").value)
            )
            if fresh:
                twist = self.latest_odometry.twist.twist
                stopped = bool(
                    abs(twist.linear.x) <= float(self.get_parameter("stopped_linear_mps").value)
                    and abs(twist.linear.y) <= float(self.get_parameter("stopped_linear_mps").value)
                    and abs(twist.angular.z) <= float(self.get_parameter("stopped_angular_rps").value)
                )
                if stopped and stopped_since is None:
                    stopped_since = time.monotonic()
                elif not stopped:
                    stopped_since = None
                if stopped_since is not None and time.monotonic() - stopped_since >= required:
                    return True
            else:
                stopped_since = None
            await ros_sleep(self, 0.05)
        return False

    async def await_skill(self, handle, timeout_sec):
        if not (bool(self.get_parameter("enable_recovery_research").value)
                or bool(self.get_parameter("recovery_local_only").value)):
            return (await handle.get_result_async()).result
        deadline = time.monotonic() + timeout_sec
        local_deadline = getattr(self, "local_deadline", None)
        if local_deadline is not None:
            deadline = min(deadline, local_deadline)
        future = handle.get_result_async()
        while not future.done():
            if time.monotonic() >= deadline:
                handle.cancel_goal_async()
                await self.set_mode(ControlMode.HOLD, "skill deadline exceeded; no further recovery authorized")
                raise TimeoutError("skill execution exceeded its time budget")
            await ros_sleep(self, 0.05)
        return future.result().result

    async def wait_for_object(self, goal_handle) -> Optional[ObjectMemory]:
        started = time.monotonic()
        timeout = float(self.get_parameter("observation_timeout_sec").value)
        while time.monotonic() - started <= timeout:
            if goal_handle.is_cancel_requested:
                return None
            memory = self.latest_memory
            if (
                memory is not None
                and time.monotonic() - self.latest_memory_receipt
                <= float(self.get_parameter("memory_stream_timeout_sec").value)
                and memory.object_id == goal_handle.request.object_id
                and memory.visible
                and memory.grasp_authorized
            ):
                return deepcopy(memory)
            await ros_sleep(self, 0.05)
        return None

    async def generate_grasps(self, goal_handle, memory, stage: int, top_k: int):
        wait = float(self.get_parameter("server_wait_timeout_sec").value)
        if not self.anygrasp_client.wait_for_server(timeout_sec=wait):
            return None, "AnyGrasp action server unavailable"
        goal = GenerateGrasps.Goal()
        goal.target = deepcopy(memory)
        goal.stage = stage
        goal.active_arm = goal_handle.request.active_arm
        goal.top_k = max(1, top_k)
        goal.require_physical_perception = bool(
            self.get_parameter("require_physical_perception").value
        )
        goal.timeout_sec = float(self.get_parameter("anygrasp_timeout_sec").value)
        handle = await self.anygrasp_client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return None, "AnyGrasp goal rejected"
        result = await self.await_skill(handle, goal.timeout_sec)
        return (result.grasps if result.success else None), result.message

    async def compute_tc_wam(self, goal_handle, memory, grasps, preferred_candidate=None, base_pose=None):
        self.last_tc_failure_code = None
        wait = float(self.get_parameter("server_wait_timeout_sec").value)
        if not self.tc_wam_client.wait_for_server(timeout_sec=wait):
            return None, "TC-WAM action server unavailable"
        goal = ComputeTcWam.Goal()
        goal.target = deepcopy(memory)
        goal.grasps = deepcopy(grasps)
        goal.select_candidate = preferred_candidate is not None
        goal.preferred_candidate_id = int(preferred_candidate or 0)
        goal.restrict_base = base_pose is not None
        if base_pose is not None:
            goal.requested_base_pose = deepcopy(base_pose)
        goal.active_arm = goal_handle.request.active_arm
        goal.evaluate_robustness = True
        goal.require_physical_evaluation = bool(
            self.get_parameter("require_physical_affordance").value
        )
        goal.timeout_sec = float(self.get_parameter("tc_wam_timeout_sec").value)
        handle = await self.tc_wam_client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return None, "TC-WAM goal rejected"
        result = await self.await_skill(handle, goal.timeout_sec)
        self.last_tc_failure_code = int(result.error_code)
        return (result if result.success else None), result.message

    async def align_base(self, goal_handle, memory, tc_wam):
        wait = float(self.get_parameter("server_wait_timeout_sec").value)
        if not self.alignment_client.wait_for_server(timeout_sec=wait):
            return None, "ACT alignment action server unavailable"
        goal = ExecuteLocalAlignment.Goal()
        goal.target = deepcopy(memory)
        goal.affordance = deepcopy(tc_wam.affordance_state)
        goal.plan = deepcopy(tc_wam.plan)
        goal.policy_variant = goal_handle.request.policy_variant or "tc_wam_ready_region_act"
        goal.timeout_sec = float(self.get_parameter("local_alignment_timeout_sec").value)
        handle = await self.alignment_client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return None, "ACT alignment goal rejected"
        result = await self.await_skill(handle, goal.timeout_sec)
        return result, result.message

    async def execute_moveit(self, goal_handle, memory, plan, refined):
        wait = float(self.get_parameter("server_wait_timeout_sec").value)
        if not self.moveit_client.wait_for_server(timeout_sec=wait):
            return None, "MoveIt grasp action server unavailable"
        goal = ExecuteMoveItGrasp.Goal()
        goal.target = deepcopy(memory)
        goal.coarse_plan = deepcopy(plan)
        goal.refined_grasps = deepcopy(refined)
        goal.active_arm = goal_handle.request.active_arm
        goal.require_physical_perception = bool(
            self.get_parameter("require_physical_perception").value
        )
        goal.timeout_sec = float(self.get_parameter("moveit_grasp_timeout_sec").value)
        handle = await self.moveit_client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return None, "MoveIt grasp goal rejected"
        result = await self.await_skill(handle, goal.timeout_sec)
        return result, result.message

    async def execute_place(self, goal_handle):
        wait = float(self.get_parameter("server_wait_timeout_sec").value)
        if not self.place_client.wait_for_server(timeout_sec=wait):
            return None, "fixed-place action server unavailable"
        request = goal_handle.request
        goal = ExecuteFixedPlace.Goal()
        goal.object_id = request.object_id
        goal.destination_id = request.destination_id
        goal.delivery_pose = deepcopy(request.delivery_pose)
        goal.active_arm = request.active_arm
        goal.timeout_sec = float(self.get_parameter("fixed_place_timeout_sec").value)
        handle = await self.place_client.send_goal_async(goal)
        if handle is None or not handle.accepted:
            return None, "fixed-place goal rejected"
        result = await self.await_skill(handle, goal.timeout_sec)
        return result, result.message

    async def fail(self, goal_handle, started: float, code: int, detail: str):
        journal = getattr(self, "recovery_journal", None)
        if journal:
            journal.finish(None if code == 99 else False, detail)
            if self.local_deadline is not None:
                journal.write("local_terminal", success=False, error_code=code)
            if self.record_recovery_distance:
                journal.write("base_motion", distance_m=self.recovery_distance_m)
            self.record_recovery_distance = False
            journal.write("task_terminal", success=False, error_code=code, message=detail)
        await self.set_mode(ControlMode.HOLD, "mission failure")
        self.publish_phase(goal_handle, MissionState.FAILED, detail, error_code=code)
        goal_handle.abort()
        result = RunPickDeliver.Result()
        result.terminal_phase = self.current_phase
        result.error_code = code
        result.message = detail
        result.total_duration_sec = float(time.monotonic() - started)
        result.recovery_attempts = self.current_attempt
        return result

    async def execute(self, goal_handle):
        self.busy = True
        self.current_attempt = 0
        self.current_grasp_id = 0
        self.current_ready_region_id = ""
        self.recovery_journal = None
        self.local_deadline = None
        self.record_recovery_distance = False
        started = time.monotonic()
        request = goal_handle.request
        try:
            accepted, detail = await self.set_mode(ControlMode.NAV2, "coarse navigation to observation zone")
            if not accepted:
                return await self.fail(goal_handle, started, 40, detail)
            self.publish_phase(
                goal_handle, MissionState.NAVIGATE_TO_PICKUP, "Nav2 to shelf observation/handoff zone"
            )
            success, detail = await self.navigate(request.pickup_observation_pose)
            if not success:
                return await self.fail(goal_handle, started, 41, detail)
            await self.set_mode(ControlMode.HOLD, "Nav2 handoff complete")
            self.publish_phase(goal_handle, MissionState.WAIT_FOR_STOP, "confirm measured base stop")
            if not await self.wait_for_stop(goal_handle):
                return await self.fail(goal_handle, started, 42, "base did not settle after Nav2")

            retained_grasps = None
            if (bool(self.get_parameter("enable_recovery_research").value)
                    or bool(self.get_parameter("recovery_local_only").value)):
                self.local_deadline = time.monotonic() + float(self.get_parameter("recovery_budget_sec").value)
            moveit_result = None
            research = bool(self.get_parameter("enable_recovery_research").value)
            if research:
                moveit_result, code, detail = await self.run_recovery_pick(goal_handle)
                if moveit_result is None:
                    return await self.fail(goal_handle, started, code, detail)
            max_attempts = int(request.max_recovery_attempts)
            while not research and self.current_attempt <= max_attempts:
                self.publish_phase(
                    goal_handle, MissionState.OBSERVE_OBJECT, "wait for fresh target observation"
                )
                memory = await self.wait_for_object(goal_handle)
                if memory is None:
                    return await self.fail(goal_handle, started, 43, "fresh target unavailable")
                if retained_grasps is None or retained_grasps.map_version != memory.map_version:
                    self.publish_phase(
                        goal_handle,
                        MissionState.COARSE_ANYGRASP,
                        "first AnyGrasp Top-K from observation zone",
                    )
                    map_grasps, detail = await self.generate_grasps(
                        goal_handle,
                        memory,
                        GenerateGrasps.Goal.COARSE,
                        int(request.coarse_top_k or 3),
                    )
                    if map_grasps is None:
                        return await self.fail(goal_handle, started, 44, detail)
                else:
                    map_grasps = retained_grasps
                self.publish_phase(
                    goal_handle,
                    MissionState.COMPUTE_TC_WAM,
                    "jointly score base, lift+arm IK stratum and grasp",
                )
                tc_wam, detail = await self.compute_tc_wam(goal_handle, memory, map_grasps)
                if tc_wam is None:
                    return await self.fail(goal_handle, started, 45, detail)
                self.current_grasp_id = tc_wam.plan.grasp_id
                self.current_ready_region_id = tc_wam.plan.ready_region.region_id
                accepted, detail = await self.set_mode(
                    ControlMode.ACT, "conditioned TC-WAM ready; ACT owns base twist only"
                )
                if not accepted:
                    return await self.fail(goal_handle, started, 46, detail)
                self.publish_phase(
                    goal_handle,
                    MissionState.ACT_ALIGN,
                    "ACT base-only alignment into bound ready region",
                    tc_wam.affordance_state.current_score,
                )
                alignment, detail = await self.align_base(goal_handle, memory, tc_wam)
                await self.set_mode(ControlMode.HOLD, "ACT block canceled; lock base")
                if alignment is None or not alignment.success:
                    recoverable = bool(alignment is not None and alignment.recoverable)
                    if not recoverable or self.current_attempt >= max_attempts:
                        return await self.fail(goal_handle, started, 47, detail)
                    self.current_attempt += 1
                    retained_grasps = None
                    self.publish_phase(
                        goal_handle, MissionState.RECOVERY, "reobserve after ACT alignment failure"
                    )
                    continue
                self.publish_phase(
                    goal_handle,
                    MissionState.WAIT_FOR_ALIGNMENT_STOP,
                    "ready reached; cancel ACT and confirm locked base",
                )
                if not await self.wait_for_stop(goal_handle):
                    return await self.fail(goal_handle, started, 48, "base did not stop after ACT")
                refined_memory = await self.wait_for_object(goal_handle)
                if refined_memory is None:
                    return await self.fail(goal_handle, started, 49, "near-field target unavailable")
                if refined_memory.map_version != tc_wam.plan.map_version:
                    if self.current_attempt >= max_attempts:
                        return await self.fail(goal_handle, started, 50, "object changed after alignment")
                    self.current_attempt += 1
                    retained_grasps = None
                    self.publish_phase(
                        goal_handle, MissionState.RECOVERY, "map version changed; discard bound ready set"
                    )
                    continue
                self.publish_phase(
                    goal_handle,
                    MissionState.REFINED_ANYGRASP,
                    "second AnyGrasp updates grasp from near-field point cloud",
                )
                refined, detail = await self.generate_grasps(
                    goal_handle,
                    refined_memory,
                    GenerateGrasps.Goal.REFINED,
                    int(request.refined_top_k or 5),
                )
                if refined is None:
                    return await self.fail(goal_handle, started, 51, detail)
                accepted, detail = await self.set_mode(
                    ControlMode.MOVEIT, "base locked; MoveIt owns lift, arm and gripper"
                )
                if not accepted:
                    return await self.fail(goal_handle, started, 52, detail)
                self.publish_phase(
                    goal_handle,
                    MissionState.MOVEIT_GRASP,
                    "validate refined grasps at current base, then jointly execute lift+arm",
                )
                moveit_result, detail = await self.execute_moveit(
                    goal_handle, refined_memory, tc_wam.plan, refined
                )
                await self.set_mode(ControlMode.HOLD, "MoveIt phase ended; retain base lock")
                if moveit_result is not None and moveit_result.success:
                    self.current_grasp_id = moveit_result.executed_grasp_id
                    break
                reposition = bool(moveit_result is not None and moveit_result.reposition_required)
                if not reposition or self.current_attempt >= max_attempts:
                    return await self.fail(goal_handle, started, 53, detail)
                self.current_attempt += 1
                retained_grasps = refined
                self.publish_phase(
                    goal_handle,
                    MissionState.RECOVERY,
                    "refined grasps unreachable; recompute local TC-WAM and realign",
                )

            if moveit_result is None or not moveit_result.transport_safe:
                return await self.fail(goal_handle, started, 54, "grasp not verified in transport pose")
            self.publish_phase(
                goal_handle, MissionState.VERIFY_GRASP, "independent grasp verification passed"
            )
            self.publish_phase(
                goal_handle, MissionState.TRANSPORT_SAFE, "object attached; lift+arm in transport pose"
            )
            self.local_deadline = None
            self.record_recovery_distance = False
            if self.recovery_journal:
                self.recovery_journal.write("base_motion", distance_m=self.recovery_distance_m)
            if bool(self.get_parameter("recovery_local_only").value):
                self.publish_phase(goal_handle, MissionState.SUCCEEDED, "local grasp benchmark completed; delivery excluded")
                goal_handle.succeed()
                result = RunPickDeliver.Result()
                result.success = True
                result.terminal_phase = MissionState.SUCCEEDED
                result.message = "local verified grasp only; delivery excluded"
                result.total_duration_sec = float(time.monotonic() - started)
                result.recovery_attempts = self.current_attempt
                if self.recovery_journal:
                    self.recovery_journal.write("task_terminal", success=True, duration_sec=result.total_duration_sec)
                return result
            if not await self.wait_for_stop(goal_handle):
                return await self.fail(goal_handle, started, 55, "transport handoff base stop lost")
            accepted, detail = await self.set_mode(ControlMode.NAV2, "transport object to delivery")
            if not accepted:
                return await self.fail(goal_handle, started, 56, detail)
            self.publish_phase(
                goal_handle, MissionState.NAVIGATE_TO_DELIVERY, "Nav2 transports grasped object"
            )
            success, detail = await self.navigate(request.delivery_pose)
            if not success:
                return await self.fail(goal_handle, started, 57, detail)
            await self.set_mode(ControlMode.HOLD, "delivery navigation complete")
            if not await self.wait_for_stop(goal_handle):
                return await self.fail(goal_handle, started, 58, "delivery stop not confirmed")
            accepted, detail = await self.set_mode(ControlMode.PLACE, "fixed delivery action authorized")
            if not accepted:
                return await self.fail(goal_handle, started, 59, detail)
            self.publish_phase(goal_handle, MissionState.FIXED_PLACE, "execute fixed delivery action")
            place_result, detail = await self.execute_place(goal_handle)
            await self.set_mode(ControlMode.HOLD, "mission terminal hold")
            if place_result is None or not place_result.success or not place_result.release_verified:
                return await self.fail(goal_handle, started, 60, detail)
            self.publish_phase(goal_handle, MissionState.SUCCEEDED, "final layered mission completed")
            goal_handle.succeed()
            result = RunPickDeliver.Result()
            result.success = True
            result.terminal_phase = MissionState.SUCCEEDED
            result.message = "Nav2 + AnyGrasp-conditioned TC-WAM + ACT + MoveIt completed"
            result.total_duration_sec = float(time.monotonic() - started)
            result.recovery_attempts = self.current_attempt
            if self.recovery_journal:
                self.recovery_journal.write("task_terminal", success=True, duration_sec=result.total_duration_sec)
            return result
        except TimeoutError as error:
            return await self.fail(goal_handle, started, 91, str(error))
        except Exception as error:
            self.get_logger().error(f"mission raised: {error}")
            return await self.fail(goal_handle, started, 99, str(error))
        finally:
            self.busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=8)
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
