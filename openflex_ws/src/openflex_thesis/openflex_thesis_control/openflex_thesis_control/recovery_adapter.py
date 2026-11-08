"""ROS-message adaptation for the task-local recovery experiment."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import Pose
from rclpy.time import Time
from tf2_ros import TransformException

from openflex_care_agent.experience import (
    FailureMemory, FailureRecord, RecoveryJournal, RecoveryOption, RecoverySelector,
)
from openflex_thesis_msgs.action import GenerateGrasps
from openflex_thesis_msgs.msg import ControlMode, MissionState

from .async_tools import ros_sleep


def position(pose):
    return (float(pose.position.x), float(pose.position.y), float(pose.position.z))


def yaw(pose):
    orientation = pose.orientation
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def grasp_key(base, grasp, lift):
    orientation = grasp.grasp_pose.pose.orientation
    sign = -1.0 if orientation.w < 0.0 else 1.0
    base_yaw = yaw(base)
    return (float(base.position.x), float(base.position.y), math.sin(base_yaw), math.cos(base_yaw),
            *position(grasp.grasp_pose.pose),
            sign * orientation.x, sign * orientation.y, sign * orientation.z, sign * orientation.w, float(lift))


class RecoveryMissionMixin:
    def initialize_recovery(self, goal_handle):
        self.failure_memory = FailureMemory()
        self.attempted_keys = set()
        self.recovery_distance_m = 0.0
        self.record_recovery_distance = False
        self.recovery_selector = RecoverySelector(
            str(self.get_parameter("recovery_method").value),
            str(self.get_parameter("recovery_model_path").value),
            int(self.get_parameter("recovery_seed").value),
            forced_action=str(self.get_parameter("recovery_forced_action").value),
        )
        self.recovery_journal = RecoveryJournal(
            str(self.get_parameter("recovery_log_path").value), goal_handle.request.task_id,
            {"method": self.recovery_selector.method,
             "scenario_id": str(self.get_parameter("recovery_scenario_id").value),
             "partition": str(self.get_parameter("recovery_partition").value),
             "seed": int(self.get_parameter("recovery_seed").value),
             "forced_action": str(self.get_parameter("recovery_forced_action").value),
             "evidence_class": str(self.get_parameter("recovery_evidence_class").value)},
        )
        self.recovery_journal.write("task_start", budget_sec=float(self.get_parameter("recovery_budget_sec").value))

    def base_pose(self):
        if self.latest_memory is None or self.latest_odometry is None:
            return None
        frame = self.latest_memory.header.frame_id
        child = self.latest_odometry.child_frame_id or "base_link"
        try:
            transform = self.recovery_tf_buffer.lookup_transform(frame, child, Time()).transform
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = deepcopy(transform.rotation)
        return pose

    def recovery_features(self, memory, candidate, remaining, distance, duration):
        return {
            "observation_age": max(0.0, time.monotonic() - self.latest_observation_receipt),
            "confidence": float(memory.confidence),
            "target_shift": 0.0, "base_distance": distance, "base_angle": 0.0,
            "candidate_count": 0.0, "score": float(candidate.total_score) if candidate else 0.0,
            "joint_margin": float(candidate.joint_margin) if candidate else 0.0,
            "robustness": float(candidate.robustness) if candidate else 0.0,
            "grasp_quality": float(candidate.grasp_quality) if candidate else 0.0,
            "remaining_sec": remaining, "attempt": float(self.current_attempt),
            "repeat_count": 0.0, "duration_sec": duration, "distance_m": distance,
        }

    def recovery_options(self, memory, tc_wam, refined, remaining, reason, stage):
        base = self.base_pose()
        if base is None:
            return [], {}
        target = position(memory.pose.pose)
        candidates = list(tc_wam.scored_candidates.candidates) if tc_wam else []
        feasible = [candidate for candidate in candidates if candidate.evaluated
                    and candidate.ground_free and candidate.ik_valid
                    and candidate.collision_free and not candidate.rejection_reason]
        best = max(feasible, key=lambda candidate: candidate.total_score, default=None)
        features = self.recovery_features(memory, best, remaining, 0.0,
                                          float(self.get_parameter("reobserve_cost_sec").value))
        features["candidate_count"] = float(len(feasible))
        if refined is not None and tc_wam is not None:
            preferred = max(refined.grasps, key=lambda grasp: (grasp.quality, -grasp.grasp_id), default=None)
            if preferred is not None:
                features["repeat_count"] = self.failure_memory.repeats(memory.object_id, target,
                    grasp_key(base, preferred, tc_wam.plan.lift_position))
        options = [RecoveryOption("reobserve", (), features)]
        payloads = {"reobserve": None}
        if tc_wam is not None and tc_wam.plan.map_version != memory.map_version:
            return options, payloads
        if refined is not None and tc_wam is not None and stage in ("validation", "pregrasp", "grasp", "verification"):
            available = []
            for grasp in refined.grasps:
                key = grasp_key(base, grasp, tc_wam.plan.lift_position)
                repeat = self.failure_memory.repeats(memory.object_id, target, key)
                if tuple(round(value, 5) for value in key) not in self.attempted_keys:
                    available.append((grasp, key, repeat))
            if available:
                grasp, key, repeat = max(available, key=lambda pair: pair[0].quality
                    - (0.15 * pair[2] if self.recovery_selector.uses_memory else 0.0))
                choice = self.recovery_features(memory, best, remaining, 0.0,
                                                float(self.get_parameter("switch_grasp_cost_sec").value))
                choice["candidate_count"] = float(len(available))
                choice["grasp_quality"] = float(grasp.quality)
                choice["repeat_count"] = float(repeat)
                options.append(RecoveryOption("switch_grasp", key, choice))
                payloads["switch_grasp"] = grasp
        alternatives = []
        for candidate in feasible:
            distance = math.dist(position(base)[:2], position(candidate.base_pose)[:2])
            if distance < float(self.get_parameter("recovery_min_base_shift_m").value):
                continue
            grasp = next((item for item in self.recovery_grasps.grasps
                          if item.grasp_id == candidate.grasp_id), None)
            if grasp is None:
                continue
            key = grasp_key(candidate.base_pose, grasp, candidate.lift_position)
            repeat = self.failure_memory.repeats(memory.object_id, target, key)
            ranking = candidate.total_score - (0.15 * repeat if self.recovery_selector.uses_memory else 0.0)
            alternatives.append((ranking, candidate, key, repeat, distance))
        if alternatives:
            _, candidate, key, repeat, distance = max(alternatives, key=lambda item: item[0])
            choice = self.recovery_features(memory, candidate, remaining, distance,
                                            float(self.get_parameter("reposition_cost_sec").value))
            choice["candidate_count"] = float(len(alternatives))
            choice["repeat_count"] = float(repeat)
            angle = yaw(candidate.base_pose) - yaw(base)
            choice["base_angle"] = abs(math.atan2(math.sin(angle), math.cos(angle)))
            options.append(RecoveryOption("reposition", key, choice))
            payloads["reposition"] = candidate
        return options, payloads

    async def refresh_observation(self, goal_handle):
        sequence = self.latest_memory.observation_sequence if self.latest_memory else 0
        started = time.monotonic()
        window = float(self.get_parameter("reobserve_window_sec").value)
        while time.monotonic() - started < window:
            if goal_handle.is_cancel_requested:
                return None
            await ros_sleep(self, 0.05)
        memory = await self.wait_for_object(goal_handle)
        return memory if memory and memory.observation_sequence > sequence else None

    async def recovery_align(self, goal_handle, memory, tc_wam):
        accepted, detail = await self.set_mode(ControlMode.ACT, "recovery candidate alignment")
        if not accepted:
            return False, False, detail
        self.publish_phase(goal_handle, MissionState.ACT_ALIGN, "align to selected recovery region")
        alignment, detail = await self.align_base(goal_handle, memory, tc_wam)
        held, _ = await self.set_mode(ControlMode.HOLD, "recovery alignment ended")
        stopped = held and await self.wait_for_stop(goal_handle)
        return bool(alignment and alignment.success and stopped), bool(stopped and alignment and alignment.recoverable), detail

    async def run_recovery_pick(self, goal_handle):
        self.initialize_recovery(goal_handle)
        started = time.monotonic()
        budget = float(self.get_parameter("recovery_budget_sec").value)
        reserve = float(self.get_parameter("recovery_grasp_reserve_sec").value)
        tc_wam = None
        refined = None
        next_action = "reobserve"
        next_payload = None
        self.recovery_grasps = None
        original_refined = None
        alternative_context = None
        previous_target = None
        while True:
            remaining = budget - (time.monotonic() - started)
            if goal_handle.is_cancel_requested:
                self.recovery_journal.finish(None, "cancelled")
                return None, 90, "recovery cancelled"
            if remaining < reserve:
                self.recovery_journal.finish(False, "budget_exhausted")
                return None, 91, "local recovery budget exhausted"
            memory = (await self.refresh_observation(goal_handle) if self.current_attempt and next_action == "reobserve"
                      else await self.wait_for_object(goal_handle))
            if memory is None:
                self.recovery_journal.finish(False, "fresh_observation_unavailable")
                return None, 43, "fresh observation unavailable"
            if self.base_pose() is None:
                self.recovery_journal.finish(None, "base_transform_unavailable")
                return None, 94, "base pose cannot be expressed in the target frame"
            if previous_target is None:
                previous_target = position(memory.pose.pose)
            reason, stage = "", ""
            if next_action == "switch_grasp":
                if tc_wam is None or memory.map_version != tc_wam.plan.map_version:
                    reason, stage = "target_changed", "observation"
                else:
                    refined = deepcopy(original_refined)
                    refined.grasps = [deepcopy(next_payload)]
            else:
                self.publish_phase(goal_handle, MissionState.COARSE_ANYGRASP, "generate recovery grasp candidates")
                if next_action == "reobserve" or self.recovery_grasps is None:
                    self.recovery_grasps, detail = await self.generate_grasps(
                        goal_handle, memory, GenerateGrasps.Goal.COARSE, int(goal_handle.request.coarse_top_k or 3))
                if self.recovery_grasps is None:
                    self.recovery_journal.finish(None, "grasp_generation_failed")
                    return None, 44, detail
                self.publish_phase(goal_handle, MissionState.COMPUTE_TC_WAM, "evaluate recovery candidates")
                restricted = (next_payload.base_pose if next_action == "reposition" else
                              self.base_pose() if self.current_attempt else None)
                tc_wam, detail = await self.compute_tc_wam(goal_handle, memory, self.recovery_grasps,
                                                         base_pose=restricted)
                if tc_wam is not None:
                    if not self.current_attempt:
                        alternative_context = deepcopy(tc_wam)
                    self.current_grasp_id = tc_wam.plan.grasp_id
                    self.current_ready_region_id = tc_wam.plan.ready_region.region_id
                    candidates = tc_wam.scored_candidates.candidates
                    feasible = [item for item in candidates if not item.rejection_reason]
                    self.recovery_journal.write("capability", total=len(candidates), feasible=len(feasible),
                        feasible_base_count=len(set((round(item.base_pose.position.x, 3),
                            round(item.base_pose.position.y, 3)) for item in feasible)))
                if tc_wam is None:
                    if self.last_tc_failure_code != 14:
                        self.recovery_journal.finish(None, "capability_service_failure")
                        return None, 45, detail
                    reason, stage = "no_candidate", "capability"
                else:
                    if self.current_attempt and next_action == "reobserve":
                        success, recoverable = True, True
                    else:
                        success, recoverable, detail = await self.recovery_align(goal_handle, memory, tc_wam)
                    if not success:
                        if not recoverable:
                            self.recovery_journal.finish(None, "unsafe_alignment")
                            return None, 47, detail
                        reason, stage = "alignment", "align"
                    else:
                        memory = await self.wait_for_object(goal_handle)
                        if memory is None:
                            reason, stage = "stale_observation", "observation"
                            memory = self.latest_memory
                        elif memory.map_version != tc_wam.plan.map_version:
                            reason, stage = "target_changed", "observation"
                        else:
                            self.publish_phase(goal_handle, MissionState.REFINED_ANYGRASP, "near-field recovery observation")
                            refined, detail = await self.generate_grasps(
                                goal_handle, memory, GenerateGrasps.Goal.REFINED,
                                int(goal_handle.request.refined_top_k or 3))
                            if refined is None:
                                self.recovery_journal.finish(None, "grasp_generation_failed")
                                return None, 44, detail
                            else:
                                original_refined = deepcopy(refined)
            if not reason:
                accepted, detail = await self.set_mode(ControlMode.MOVEIT, "recovery grasp owns lift arm and gripper")
                if not accepted:
                    self.recovery_journal.finish(None, "control_guard_rejected")
                    return None, 52, detail
                self.publish_phase(goal_handle, MissionState.MOVEIT_GRASP, "fixed MoveIt grasp attempt")
                result, detail = await self.execute_moveit(goal_handle, memory, tc_wam.plan, refined)
                if result is not None:
                    self.recovery_journal.write("skill_result", error_code=int(result.error_code),
                        failure_stage=result.failure_stage, failure_reason=result.failure_reason,
                        recovery_safe=bool(result.recovery_safe), grasp_id=int(result.executed_grasp_id),
                        tested_grasp_ids=list(result.tested_grasp_ids),
                        grasp_failure_reasons=list(result.grasp_failure_reasons))
                held, _ = await self.set_mode(ControlMode.HOLD, "recovery grasp ended")
                if result and result.success and result.grasp_verified and result.transport_safe:
                    self.recovery_journal.finish(True, "grasp_verified")
                    self.recovery_journal.write("local_terminal", success=True, duration_sec=time.monotonic() - started)
                    self.current_grasp_id = result.executed_grasp_id
                    return result, 0, "verified"
                if not held or result is None or not result.recovery_safe or not await self.wait_for_stop(goal_handle):
                    infrastructure = result is None or result.error_code in (70, 71, 72, 73, 82, 99)
                    self.recovery_journal.finish(None if infrastructure else False, "unsafe_or_unverified")
                    return None, 53, detail
                reason, stage = result.failure_reason, result.failure_stage
                base = self.base_pose()
                if base is None:
                    self.recovery_journal.finish(None, "base_transform_unavailable")
                    return None, 94, "base transform lost after grasp attempt"
                for grasp_id, failure in zip(result.tested_grasp_ids, result.grasp_failure_reasons):
                    grasp = next((item for item in refined.grasps if item.grasp_id == grasp_id), None)
                    if grasp is not None and failure != "feasible_not_executed":
                        self.attempted_keys.add(tuple(round(value, 5) for value in
                            grasp_key(base, grasp, tc_wam.plan.lift_position)))
                        self.failure_memory.add(FailureRecord(memory.object_id, memory.map_version,
                            position(memory.pose.pose), grasp_key(base, grasp, tc_wam.plan.lift_position), failure))
                grasp = next((item for item in refined.grasps if item.grasp_id == result.executed_grasp_id), None)
                if grasp:
                    self.attempted_keys.add(tuple(round(value, 5) for value in
                        grasp_key(base, grasp, tc_wam.plan.lift_position)))
                    self.failure_memory.add(FailureRecord(memory.object_id, memory.map_version,
                        position(memory.pose.pose), grasp_key(base, grasp, tc_wam.plan.lift_position), reason))
            self.recovery_journal.finish(False, reason)
            if self.current_attempt >= goal_handle.request.max_recovery_attempts or memory is None:
                return None, 92, "recovery attempt limit reached: " + reason
            remaining = budget - (time.monotonic() - started)
            if original_refined is not None and stage in ("validation", "pregrasp", "grasp", "verification"):
                updated, _ = await self.compute_tc_wam(goal_handle, memory, original_refined)
                if updated is not None:
                    alternative_context = updated
                    self.recovery_grasps = deepcopy(original_refined)
                    self.recovery_journal.write("recovery_candidate_evaluation", feasible=sum(
                        not item.rejection_reason for item in updated.scored_candidates.candidates))
            remaining = budget - (time.monotonic() - started)
            option_context = tc_wam if tc_wam is not None else alternative_context
            if option_context is not None and alternative_context is not None:
                option_context = deepcopy(option_context)
                option_context.scored_candidates = deepcopy(alternative_context.scored_candidates)
            options, payloads = self.recovery_options(memory, option_context, original_refined, remaining, reason, stage)
            if previous_target is not None:
                for option in options:
                    option.features["target_shift"] = math.dist(previous_target, position(memory.pose.pose))
            previous_target = position(memory.pose.pose)
            self.recovery_journal.write("failure", reason=reason, stage=stage,
                map_version=int(memory.map_version), options=[
                    {"action": option.action, "features": dict(option.features), "key": option.key} for option in options])
            self.recovery_journal.write("failure_context", object_id=memory.object_id,
                target=position(memory.pose.pose), base=position(self.base_pose()),
                grasp_id=int(self.current_grasp_id), observation_sequence=int(memory.observation_sequence))
            selected = self.recovery_selector.choose(options, reason, stage, remaining, reserve)
            if selected is None:
                return None, 93, "no budget-feasible recovery option"
            if selected.action == "reobserve":
                self.attempted_keys.clear()
            self.current_attempt += 1
            propensity = 1.0 / len([option for option in options if option.safe
                and option.features["duration_sec"] + reserve <= remaining]) if self.recovery_selector.method == "random" else None
            if self.recovery_selector.forced_action:
                propensity = 1.0
            self.record_recovery_distance = True
            self.recovery_journal.begin(selected, reason, stage, propensity)
            next_action, next_payload = selected.action, payloads[selected.action]
            self.publish_phase(goal_handle, MissionState.RECOVERY, "selected recovery: " + next_action)
