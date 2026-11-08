from copy import deepcopy
from types import SimpleNamespace

from geometry_msgs.msg import Pose
from openflex_care_agent.experience import FailureMemory, FailureRecord, RecoverySelector
from openflex_thesis_msgs.action import ComputeTcWam
from openflex_thesis_msgs.msg import AffordanceCandidate, GraspCandidate, GraspCandidateArray, ObjectMemory

from openflex_thesis_control.recovery_adapter import RecoveryMissionMixin, grasp_key


class Adapter(RecoveryMissionMixin):
    def __init__(self):
        self.failure_memory = FailureMemory()
        self.recovery_selector = RecoverySelector()
        self.attempted_keys = set()
        self.current_attempt = 0
        self.latest_observation_receipt = 0.0
        self.base = Pose()
        self.base.orientation.w = 1.0

    def get_parameter(self, name):
        return SimpleNamespace(value={"reobserve_cost_sec": 35.0, "switch_grasp_cost_sec": 5.0,
                                      "reposition_cost_sec": 35.0, "recovery_min_base_shift_m": 0.08}[name])

    def base_pose(self):
        return deepcopy(self.base)


def context():
    adapter = Adapter()
    memory = ObjectMemory()
    memory.object_id = "item"
    memory.map_version = 1
    memory.confidence = 0.9
    result = ComputeTcWam.Result()
    result.plan.map_version = memory.map_version
    candidate = AffordanceCandidate()
    candidate.candidate_id = 3
    candidate.grasp_id = 7
    candidate.evaluated = candidate.ground_free = candidate.ik_valid = candidate.collision_free = True
    candidate.base_pose.position.x = 0.2
    candidate.base_pose.orientation.w = 1.0
    candidate.total_score = 0.8
    result.scored_candidates.candidates = [candidate]
    grasp = GraspCandidate()
    grasp.grasp_id = 7
    grasp.quality = 0.8
    grasp.grasp_pose.pose.orientation.w = 1.0
    refined = GraspCandidateArray()
    refined.grasps = [grasp]
    adapter.recovery_grasps = refined
    return adapter, memory, result, refined


def test_concrete_reposition_candidate_is_not_silently_lost():
    adapter, memory, result, refined = context()
    options, payloads = adapter.recovery_options(memory, result, refined, 300, "ik_collision", "validation")
    assert {option.action for option in options} == {"reobserve", "switch_grasp", "reposition"}
    assert payloads["reposition"].candidate_id == 3


def test_checked_or_executed_candidate_cannot_be_offered_as_new():
    adapter, memory, result, refined = context()
    key = grasp_key(adapter.base, refined.grasps[0], result.plan.lift_position)
    adapter.attempted_keys.add(tuple(round(value, 5) for value in key))
    options, _ = adapter.recovery_options(memory, result, refined, 300, "ik_collision", "validation")
    assert "switch_grasp" not in [option.action for option in options]


def test_no_capability_result_only_allows_reobservation():
    adapter, memory, _, _ = context()
    options, _ = adapter.recovery_options(memory, None, None, 300, "no_candidate", "capability")
    assert [option.action for option in options] == ["reobserve"]


def test_stale_capability_never_authorizes_candidate_reuse():
    adapter, memory, result, refined = context()
    memory.map_version += 1
    options, _ = adapter.recovery_options(memory, result, refined, 300, "target_changed", "verification")
    assert [option.action for option in options] == ["reobserve"]


def test_planar_candidate_and_tf_base_height_share_memory_identity():
    adapter, _, result, refined = context()
    ground_base = deepcopy(adapter.base)
    actual_base = deepcopy(adapter.base)
    actual_base.position.z = 0.17
    assert grasp_key(ground_base, refined.grasps[0], result.plan.lift_position) == grasp_key(
        actual_base, refined.grasps[0], result.plan.lift_position)


def test_equivalent_quaternion_signs_share_grasp_identity():
    adapter, _, result, refined = context()
    reversed_grasp = deepcopy(refined.grasps[0])
    reversed_grasp.grasp_pose.pose.orientation.w = -1.0
    assert grasp_key(adapter.base, refined.grasps[0], result.plan.lift_position) == grasp_key(
        adapter.base, reversed_grasp, result.plan.lift_position)


def test_reobserve_penalty_tracks_the_candidate_fixed_execution_would_repeat():
    adapter, memory, result, refined = context()
    unused = deepcopy(refined.grasps[0])
    unused.grasp_id = 8
    unused.quality = 0.1
    unused.grasp_pose.pose.position.x = 1.0
    refined.grasps.append(unused)
    key = grasp_key(adapter.base, refined.grasps[0], result.plan.lift_position)
    adapter.failure_memory.add(FailureRecord(memory.object_id, 1, (0.0, 0.0, 0.0), key, "ik_collision"))
    options, _ = adapter.recovery_options(memory, result, refined, 300, "ik_collision", "validation")
    reobserve = next(option for option in options if option.action == "reobserve")
    assert reobserve.features["repeat_count"] == 1
