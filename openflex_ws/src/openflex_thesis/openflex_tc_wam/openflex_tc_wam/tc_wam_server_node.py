"""AnyGrasp-conditioned TC-WAM over base, lift stratum and grasp pose."""

from copy import deepcopy
import math
from typing import Dict, Optional, Sequence, Set, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from openflex_thesis_msgs.action import ComputeTcWam, EvaluateCandidates
from openflex_thesis_msgs.msg import (
    AffordanceCandidate,
    AffordanceCandidateArray,
    AffordanceState,
    GraspCandidate,
    ManipulationPlan,
    ReadyRegion,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .candidate_generation import CandidateGeneratorConfig, generate_candidates
from .geometry import (
    PlanarPose,
    clamp,
    normalize_angle,
    planar_distance,
    quaternion_to_yaw,
    relative_planar,
    yaw_to_quaternion,
)
from .scoring import (
    CandidateMetrics,
    CandidateScore,
    ScoreWeights,
    conditioned_ready_candidate_ids,
    project_score_grid,
    score_candidate,
    select_best,
)


class TcWamServerNode(Node):
    """Select (b*, h*, g*) and publish a g*/h*-conditioned ready set."""

    def __init__(self) -> None:
        super().__init__("tc_wam_server")
        self.declare_parameter("evaluator_action", "/thesis/evaluate_candidates")
        self.declare_parameter("compute_action", "/thesis/compute_tc_wam")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scored_topic", "/thesis/tc_wam/candidates")
        self.declare_parameter("grid_topic", "/thesis/tc_wam/score_grid")
        self.declare_parameter("marker_topic", "/thesis/tc_wam/markers")
        self.declare_parameter("best_pose_topic", "/thesis/tc_wam/best_pose")
        self.declare_parameter("state_topic", "/thesis/tc_wam/state")
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("distance_min_m", 0.60)
        self.declare_parameter("distance_max_m", 1.00)
        self.declare_parameter("distance_step_m", 0.10)
        self.declare_parameter("lateral_min_m", -0.30)
        self.declare_parameter("lateral_max_m", 0.30)
        self.declare_parameter("lateral_step_m", 0.10)
        self.declare_parameter("yaw_offsets_rad", [0.0])
        # These are IK redundancy strata, not separately commanded lift targets.
        self.declare_parameter("lift_seed_positions_m", [-0.45, -0.15, 0.15])
        self.declare_parameter("left_planning_group", "left_arm_with_lift")
        self.declare_parameter("right_planning_group", "right_arm_with_lift")
        self.declare_parameter("left_ik_link", "openarmx_left_hand_tcp")
        self.declare_parameter("right_ik_link", "openarmx_right_hand_tcp")
        self.declare_parameter("reachability_threshold", 0.34)
        self.declare_parameter("ready_absolute_threshold", 0.65)
        self.declare_parameter("ready_relative_ratio", 0.90)
        self.declare_parameter("ready_neighbor_distance_m", 0.11)
        self.declare_parameter("ready_neighbor_yaw_rad", 0.20)
        self.declare_parameter("ready_lift_tolerance_m", 0.02)
        self.declare_parameter("ready_distance_tolerance_m", 0.12)
        self.declare_parameter("ready_yaw_tolerance_rad", 0.18)
        self.declare_parameter("maximum_local_move_m", 1.25)
        self.declare_parameter("weight_reachability", 0.22)
        self.declare_parameter("weight_joint_margin", 0.18)
        self.declare_parameter("weight_robustness", 0.25)
        self.declare_parameter("weight_grasp_quality", 0.15)
        self.declare_parameter("weight_visibility", 0.10)
        self.declare_parameter("weight_move", 0.10)
        self.declare_parameter("require_navigation_gate", False)
        self.declare_parameter("evaluator_wait_timeout_sec", 5.0)
        self.declare_parameter("state_publish_rate_hz", 20.0)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.generator_config = CandidateGeneratorConfig(
            distance_min_m=float(self.get_parameter("distance_min_m").value),
            distance_max_m=float(self.get_parameter("distance_max_m").value),
            distance_step_m=float(self.get_parameter("distance_step_m").value),
            lateral_min_m=float(self.get_parameter("lateral_min_m").value),
            lateral_max_m=float(self.get_parameter("lateral_max_m").value),
            lateral_step_m=float(self.get_parameter("lateral_step_m").value),
            yaw_offsets_rad=tuple(float(v) for v in self.get_parameter("yaw_offsets_rad").value),
        )
        self.generator_config.validate()
        self.lift_seeds = tuple(
            float(value) for value in self.get_parameter("lift_seed_positions_m").value
        )
        if not self.lift_seeds:
            raise ValueError("at least one lift seed is required")
        self.weights = ScoreWeights(
            reachability=float(self.get_parameter("weight_reachability").value),
            joint_margin=float(self.get_parameter("weight_joint_margin").value),
            robustness=float(self.get_parameter("weight_robustness").value),
            grasp_quality=float(self.get_parameter("weight_grasp_quality").value),
            visibility=float(self.get_parameter("weight_visibility").value),
            move=float(self.get_parameter("weight_move").value),
        ).normalized()
        self.callback_group = ReentrantCallbackGroup()
        self.evaluator = ActionClient(
            self,
            EvaluateCandidates,
            str(self.get_parameter("evaluator_action").value),
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            ComputeTcWam,
            str(self.get_parameter("compute_action").value),
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scored_publisher = self.create_publisher(
            AffordanceCandidateArray, str(self.get_parameter("scored_topic").value), qos
        )
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("grid_topic").value), qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), qos
        )
        self.best_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("best_pose_topic").value), qos
        )
        self.state_publisher = self.create_publisher(
            AffordanceState, str(self.get_parameter("state_topic").value), qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.busy = False
        self.active_context = None
        rate = float(self.get_parameter("state_publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("state_publish_rate_hz must be positive")
        self.create_timer(1.0 / rate, self.publish_dynamic_state)

    def planning_contract(self, arm: int) -> Tuple[str, str]:
        if arm == AffordanceCandidate.ARM_LEFT:
            return (
                str(self.get_parameter("left_planning_group").value),
                str(self.get_parameter("left_ik_link").value),
            )
        return (
            str(self.get_parameter("right_planning_group").value),
            str(self.get_parameter("right_ik_link").value),
        )

    def goal_callback(self, request: ComputeTcWam.Goal) -> GoalResponse:
        valid_arm = request.active_arm in (
            AffordanceCandidate.ARM_LEFT,
            AffordanceCandidate.ARM_RIGHT,
        )
        grasps_match = bool(
            request.grasps.grasps
            and request.grasps.object_id == request.target.object_id
            and request.grasps.map_version == request.target.map_version
        )
        return GoalResponse.ACCEPT if not self.busy and valid_arm and grasps_match else GoalResponse.REJECT

    def current_base_pose(self, world_frame: str) -> Optional[PlanarPose]:
        try:
            transform = self.tf_buffer.lookup_transform(world_frame, self.base_frame, Time())
        except TransformException as error:
            self.get_logger().warn(
                f"Cannot transform {world_frame} <- {self.base_frame}: {error}",
                throttle_duration_sec=2.0,
            )
            return None
        rotation = transform.transform.rotation
        return PlanarPose(
            transform.transform.translation.x,
            transform.transform.translation.y,
            quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def raw_candidates(
        self, goal: ComputeTcWam.Goal, current_base: Optional[PlanarPose]
    ) -> AffordanceCandidateArray:
        target_pose = goal.target.pose.pose
        orientation = target_pose.orientation
        object_pose = PlanarPose(
            target_pose.position.x,
            target_pose.position.y,
            quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
        )
        base_poses = generate_candidates(object_pose, self.generator_config)
        planning_group, ik_link = self.planning_contract(int(goal.active_arm))
        output = AffordanceCandidateArray()
        output.header = deepcopy(goal.target.header)
        output.object_id = goal.target.object_id
        output.map_version = goal.target.map_version
        output.grasp_request_id = goal.grasps.request_id
        output.evaluator_id = "pending"
        max_move = float(self.get_parameter("maximum_local_move_m").value)
        identifier = 0
        for pose in base_poses:
            for grasp in goal.grasps.grasps:
                for lift_seed in self.lift_seeds:
                    candidate = AffordanceCandidate()
                    candidate.candidate_id = identifier
                    identifier += 1
                    candidate.base_pose.position.x = pose.x
                    candidate.base_pose.position.y = pose.y
                    candidate.base_pose.orientation.x, candidate.base_pose.orientation.y, \
                        candidate.base_pose.orientation.z, candidate.base_pose.orientation.w = yaw_to_quaternion(pose.yaw)
                    candidate.active_arm = goal.active_arm
                    candidate.grasp_id = grasp.grasp_id
                    candidate.lift_seed = lift_seed
                    candidate.lift_position = lift_seed
                    candidate.planning_group = planning_group
                    candidate.ik_link = ik_link
                    candidate.ground_free = True
                    candidate.path_valid = True
                    candidate.grasp_quality = clamp(grasp.quality, 0.0, 1.0)
                    candidate.visibility_score = clamp(grasp.visibility_score, 0.0, 1.0)
                    candidate.move_score = (
                        clamp(1.0 - planar_distance(current_base, pose) / max_move, 0.0, 1.0)
                        if current_base is not None and max_move > 0.0
                        else 0.5
                    )
                    output.candidates.append(candidate)
        return output

    @staticmethod
    def metrics(candidate: AffordanceCandidate) -> CandidateMetrics:
        orientation = candidate.base_pose.orientation
        return CandidateMetrics(
            candidate_id=int(candidate.candidate_id),
            x=candidate.base_pose.position.x,
            y=candidate.base_pose.position.y,
            yaw=quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            ),
            evaluated=candidate.evaluated,
            ground_free=candidate.ground_free,
            path_valid=candidate.path_valid,
            ik_valid=candidate.ik_valid,
            collision_free=candidate.collision_free,
            reachability=candidate.reachability,
            joint_margin=candidate.joint_margin,
            robustness=candidate.robustness,
            move_score=candidate.move_score,
            grasp_id=int(candidate.grasp_id),
            lift_seed=float(candidate.lift_seed),
            lift_position=float(candidate.lift_position),
            planning_group=candidate.planning_group,
            grasp_quality=candidate.grasp_quality,
            visibility_score=candidate.visibility_score,
        )

    async def execute(self, goal_handle):
        self.busy = True
        try:
            request = goal_handle.request
            if request.require_physical_evaluation and not request.grasps.physical_perception:
                return self.fail(goal_handle, 9, "physical AnyGrasp perception required")
            feedback = ComputeTcWam.Feedback()
            feedback.phase = "expand_base_lift_grasp_candidates"
            goal_handle.publish_feedback(feedback)
            current_base = self.current_base_pose(request.target.header.frame_id)
            raw = self.raw_candidates(request, current_base)
            feedback.total_candidates = len(raw.candidates)
            goal_handle.publish_feedback(feedback)
            if not self.evaluator.wait_for_server(
                timeout_sec=float(self.get_parameter("evaluator_wait_timeout_sec").value)
            ):
                return self.fail(goal_handle, 10, "candidate evaluator action is unavailable")
            evaluation_goal = EvaluateCandidates.Goal()
            evaluation_goal.target = request.target
            evaluation_goal.grasps = request.grasps
            evaluation_goal.candidates = raw
            evaluation_goal.active_arm = request.active_arm
            evaluation_goal.evaluate_robustness = request.evaluate_robustness
            evaluation_goal.timeout_sec = request.timeout_sec
            feedback.phase = "moveit_joint_lift_arm_evaluation"
            goal_handle.publish_feedback(feedback)
            evaluator_handle = await self.evaluator.send_goal_async(evaluation_goal)
            if evaluator_handle is None or not evaluator_handle.accepted:
                return self.fail(goal_handle, 11, "candidate evaluator rejected the request")
            evaluation = (await evaluator_handle.get_result_async()).result
            if not evaluation.success:
                return self.fail(goal_handle, 12, f"candidate evaluation failed: {evaluation.message}")
            evaluated = evaluation.evaluated_candidates
            if request.require_physical_evaluation and not evaluated.physical_evaluation:
                return self.fail(goal_handle, 13, "physical evaluation required; synthetic results rejected")
            if goal_handle.is_cancel_requested:
                evaluator_handle.cancel_goal_async()
                goal_handle.canceled()
                result = ComputeTcWam.Result()
                result.message = "canceled"
                return result
            scores: Dict[int, CandidateScore] = {}
            metrics: Dict[int, CandidateMetrics] = {}
            for candidate in evaluated.candidates:
                metric = self.metrics(candidate)
                score = score_candidate(
                    metric,
                    self.weights,
                    float(self.get_parameter("reachability_threshold").value),
                    bool(self.get_parameter("require_navigation_gate").value),
                )
                candidate.total_score = score.score
                candidate.rejection_reason = score.rejection_reason
                metrics[int(candidate.candidate_id)] = metric
                scores[int(candidate.candidate_id)] = score
            best_score = select_best(list(scores.values()))
            if best_score is None:
                self.publish_map(evaluated, scores, set(), None)
                return self.fail(goal_handle, 14, "no feasible (base, lift, grasp) candidate")
            ready_ids = set(
                conditioned_ready_candidate_ids(
                    list(metrics.values()),
                    list(scores.values()),
                    best_score.candidate_id,
                    float(self.get_parameter("ready_absolute_threshold").value),
                    float(self.get_parameter("ready_relative_ratio").value),
                    float(self.get_parameter("ready_neighbor_distance_m").value),
                    float(self.get_parameter("ready_neighbor_yaw_rad").value),
                    lift_solution_tolerance_m=float(
                        self.get_parameter("ready_lift_tolerance_m").value
                    ),
                )
            )
            by_id = {int(item.candidate_id): item for item in evaluated.candidates}
            best = by_id[best_score.candidate_id]
            for candidate in evaluated.candidates:
                candidate.ready_member = int(candidate.candidate_id) in ready_ids
            region = self.build_ready_region(evaluated, best, ready_ids, best_score.score)
            plan = self.build_plan(request, evaluated, best, region)
            state = self.build_state(request, evaluated, best, ready_ids, region, current_base)
            self.publish_map(evaluated, scores, ready_ids, best)
            self.state_publisher.publish(state)
            self.active_context = (
                deepcopy(request),
                deepcopy(evaluated),
                deepcopy(best),
                set(ready_ids),
                deepcopy(region),
            )
            goal_handle.succeed()
            result = ComputeTcWam.Result()
            result.success = True
            result.scored_candidates = evaluated
            result.affordance_state = state
            result.plan = plan
            result.message = "AnyGrasp-conditioned TC-WAM computed"
            return result
        except Exception as error:
            self.get_logger().error(f"TC-WAM computation raised: {error}")
            return self.fail(goal_handle, 99, str(error))
        finally:
            self.busy = False

    @staticmethod
    def fail(goal_handle, code: int, message: str) -> ComputeTcWam.Result:
        goal_handle.abort()
        result = ComputeTcWam.Result()
        result.error_code = code
        result.message = message
        return result

    def build_ready_region(
        self,
        evaluated: AffordanceCandidateArray,
        best: AffordanceCandidate,
        ready_ids: Set[int],
        best_score: float,
    ) -> ReadyRegion:
        region = ReadyRegion()
        region.header = deepcopy(evaluated.header)
        region.object_id = evaluated.object_id
        region.map_version = evaluated.map_version
        region.active_arm = best.active_arm
        region.grasp_id = best.grasp_id
        region.planning_group = best.planning_group
        region.lift_position = best.lift_position
        region.score_threshold = max(
            float(self.get_parameter("ready_absolute_threshold").value),
            float(self.get_parameter("ready_relative_ratio").value) * best_score,
        )
        region.best_base_pose = deepcopy(best.base_pose)
        region.position_tolerance_m = float(
            self.get_parameter("ready_distance_tolerance_m").value
        )
        region.yaw_tolerance_rad = float(self.get_parameter("ready_yaw_tolerance_rad").value)
        lift_mm = int(round(1000.0 * best.lift_seed))
        region.region_id = (
            f"v{evaluated.map_version}:a{best.active_arm}:g{best.grasp_id}:h{lift_mm}"
        )
        for candidate in evaluated.candidates:
            if int(candidate.candidate_id) in ready_ids:
                region.member_candidate_ids.append(candidate.candidate_id)
                region.member_base_poses.append(deepcopy(candidate.base_pose))
        return region

    @staticmethod
    def find_grasp(request: ComputeTcWam.Goal, grasp_id: int) -> GraspCandidate:
        for grasp in request.grasps.grasps:
            if int(grasp.grasp_id) == grasp_id:
                return deepcopy(grasp)
        raise ValueError(f"selected grasp {grasp_id} is absent from request")

    def build_plan(
        self,
        request: ComputeTcWam.Goal,
        evaluated: AffordanceCandidateArray,
        best: AffordanceCandidate,
        region: ReadyRegion,
    ) -> ManipulationPlan:
        plan = ManipulationPlan()
        plan.header = deepcopy(evaluated.header)
        plan.object_id = evaluated.object_id
        plan.map_version = evaluated.map_version
        plan.active_arm = best.active_arm
        plan.candidate_id = best.candidate_id
        plan.grasp_id = best.grasp_id
        plan.planning_group = best.planning_group
        plan.ik_link = best.ik_link
        plan.best_base_pose.header = deepcopy(evaluated.header)
        plan.best_base_pose.pose = deepcopy(best.base_pose)
        plan.selected_grasp = self.find_grasp(request, int(best.grasp_id))
        plan.lift_position = best.lift_position
        plan.joint_names = list(best.joint_names)
        plan.pregrasp_joint_positions = list(best.pregrasp_joint_positions)
        plan.grasp_joint_positions = list(best.grasp_joint_positions)
        plan.total_score = best.total_score
        plan.reachability = best.reachability
        plan.joint_margin = best.joint_margin
        plan.robustness = best.robustness
        plan.grasp_quality = best.grasp_quality
        plan.physical_evaluation = evaluated.physical_evaluation
        plan.ready_region = deepcopy(region)
        return plan

    @staticmethod
    def planar_from_pose(pose) -> PlanarPose:
        q = pose.orientation
        return PlanarPose(
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )

    def build_state(
        self,
        request: ComputeTcWam.Goal,
        evaluated: AffordanceCandidateArray,
        best: AffordanceCandidate,
        ready_ids: Set[int],
        region: ReadyRegion,
        current_base: Optional[PlanarPose],
    ) -> AffordanceState:
        state = AffordanceState()
        state.header = deepcopy(evaluated.header)
        state.header.stamp = self.get_clock().now().to_msg()
        state.object_id = evaluated.object_id
        state.map_version = evaluated.map_version
        state.active_arm = best.active_arm
        state.selected_grasp_id = best.grasp_id
        state.selected_lift_position = best.lift_position
        state.planning_group = best.planning_group
        state.ready_region_id = region.region_id
        state.ready_region = deepcopy(region)
        state.best_base_pose.header = deepcopy(evaluated.header)
        state.best_base_pose.pose = deepcopy(best.base_pose)
        state.best_score = best.total_score
        state.reachability = best.reachability
        state.joint_margin = best.joint_margin
        state.robustness = best.robustness
        state.physical_evaluation = evaluated.physical_evaluation
        state.object_visible = request.target.visible
        state.grasp_authorized = request.target.grasp_authorized
        target_pose = request.target.pose.pose
        target_world = self.planar_from_pose(target_pose)
        best_world = self.planar_from_pose(best.base_pose)
        target_relative = PlanarPose(0.0, 0.0, 0.0)
        best_delta = PlanarPose(0.0, 0.0, 0.0)
        current_score = 0.0
        ready_distance = float("inf")
        ready = False
        if current_base is not None:
            target_relative = relative_planar(current_base, target_world)
            best_delta = relative_planar(current_base, best_world)
            conditioned = [
                item
                for item in evaluated.candidates
                if item.grasp_id == best.grasp_id
                and item.planning_group == best.planning_group
                and abs(item.lift_seed - best.lift_seed) <= 1.0e-4
            ]
            if conditioned:
                nearest = min(
                    conditioned,
                    key=lambda item: math.hypot(
                        item.base_pose.position.x - current_base.x,
                        item.base_pose.position.y - current_base.y,
                    ),
                )
                nearest_world = self.planar_from_pose(nearest.base_pose)
                ready_distance = planar_distance(current_base, nearest_world)
                yaw_error = abs(normalize_angle(nearest_world.yaw - current_base.yaw))
                current_score = nearest.total_score
                ready = bool(
                    int(nearest.candidate_id) in ready_ids
                    and ready_distance <= region.position_tolerance_m
                    and yaw_error <= region.yaw_tolerance_rad
                    and request.target.grasp_authorized
                )
        state.current_score = current_score
        state.ready = ready
        state.valid = bool(
            current_base is not None
            and request.target.visible
            and request.target.grasp_authorized
            and (evaluated.physical_evaluation or not request.require_physical_evaluation)
        )
        state.target_in_base.position.x = target_relative.x
        state.target_in_base.position.y = target_relative.y
        state.target_in_base.position.z = target_pose.position.z
        state.target_in_base.orientation.x, state.target_in_base.orientation.y, \
            state.target_in_base.orientation.z, state.target_in_base.orientation.w = yaw_to_quaternion(target_relative.yaw)
        state.preferred_base_delta.x = best_delta.x
        state.preferred_base_delta.y = best_delta.y
        state.preferred_base_delta.theta = best_delta.yaw
        state.compact_state = [
            target_relative.x,
            target_relative.y,
            target_pose.position.z,
            math.sin(target_relative.yaw),
            math.cos(target_relative.yaw),
            best_delta.x,
            best_delta.y,
            math.sin(best_delta.yaw),
            math.cos(best_delta.yaw),
            current_score,
            best.total_score,
            best.reachability,
            best.joint_margin,
            best.robustness,
            best.grasp_quality,
            best.lift_position,
            ready_distance if math.isfinite(ready_distance) else 9.0,
            1.0 if ready else 0.0,
        ]
        return state

    def publish_dynamic_state(self) -> None:
        context = self.active_context
        if context is None:
            return
        request, evaluated, best, ready_ids, region = context
        current = self.current_base_pose(evaluated.header.frame_id)
        self.state_publisher.publish(
            self.build_state(request, evaluated, best, ready_ids, region, current)
        )

    def publish_map(
        self,
        candidates: AffordanceCandidateArray,
        scores: Dict[int, CandidateScore],
        ready_ids: Set[int],
        best: Optional[AffordanceCandidate],
    ) -> None:
        self.scored_publisher.publish(candidates)
        conditioned = []
        if best is not None:
            conditioned = [
                candidate
                for candidate in candidates.candidates
                if candidate.grasp_id == best.grasp_id
                and candidate.planning_group == best.planning_group
                and abs(candidate.lift_seed - best.lift_seed) <= 1.0e-4
            ]
        points = [
            (
                item.base_pose.position.x,
                item.base_pose.position.y,
                item.total_score,
                scores[int(item.candidate_id)].feasible,
            )
            for item in conditioned
        ]
        projection = project_score_grid(points, float(self.get_parameter("grid_resolution").value))
        grid = OccupancyGrid()
        grid.header = deepcopy(candidates.header)
        grid.info.map_load_time = deepcopy(candidates.header.stamp)
        grid.info.resolution = projection.resolution
        grid.info.width = projection.width
        grid.info.height = projection.height
        grid.info.origin.position.x = projection.origin_x
        grid.info.origin.position.y = projection.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = list(projection.data)
        self.grid_publisher.publish(grid)
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        resolution = float(self.get_parameter("grid_resolution").value)
        for item in conditioned:
            identifier = int(item.candidate_id)
            score = scores[identifier]
            marker = Marker()
            marker.header = deepcopy(candidates.header)
            marker.ns = "conditioned_tc_wam"
            marker.id = identifier
            marker.type = Marker.CUBE if identifier in ready_ids else Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = deepcopy(item.base_pose)
            marker.pose.position.z = 0.03
            marker.scale.x = resolution * 0.8 if identifier in ready_ids else 0.16
            marker.scale.y = resolution * 0.8 if identifier in ready_ids else 0.025
            marker.scale.z = 0.025
            if not score.feasible:
                marker.color.r = marker.color.g = marker.color.b = 0.05
            elif score.score <= 0.5:
                marker.color.r = 1.0
                marker.color.g = 2.0 * score.score
            else:
                marker.color.r = 2.0 * (1.0 - score.score)
                marker.color.g = 1.0
            marker.color.a = 0.9
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)
        if best is not None:
            best_pose = PoseStamped()
            best_pose.header = deepcopy(candidates.header)
            best_pose.pose = deepcopy(best.base_pose)
            self.best_publisher.publish(best_pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcWamServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
