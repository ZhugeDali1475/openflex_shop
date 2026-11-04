from openflex_tc_wam.scoring import (
    CandidateMetrics,
    ScoreWeights,
    conditioned_ready_candidate_ids,
    ready_candidate_ids,
    score_candidate,
    select_best,
)


def candidate(identifier, robustness=0.8, collision_free=True):
    return CandidateMetrics(
        identifier, 0.0, 0.0, 0.0, True, True, True, True,
        collision_free, 0.8, 0.7, robustness, 0.9,
        grasp_id=7, lift_seed=0.4, planning_group="right_arm_with_lift",
        lift_position=0.4, grasp_quality=0.9, visibility_score=0.8,
    )


def test_hard_collision_gate_cannot_be_compensated():
    result = score_candidate(candidate(1, collision_free=False), ScoreWeights(), 0.3)
    assert not result.feasible
    assert result.score == 0.0
    assert result.rejection_reason == "collision"


def test_best_and_ready_region_are_deterministic():
    scores = [
        score_candidate(candidate(1, robustness=0.5), ScoreWeights(), 0.3),
        score_candidate(candidate(2, robustness=1.0), ScoreWeights(), 0.3),
    ]
    assert select_best(scores).candidate_id == 2
    assert ready_candidate_ids(scores, 0.6, 0.95) == [2]


def test_ready_region_cannot_mix_grasps_or_disconnected_islands():
    metrics = [
        candidate(1),
        CandidateMetrics(**{**candidate(2).__dict__, "candidate_id": 2, "x": 0.1}),
        CandidateMetrics(**{**candidate(3).__dict__, "candidate_id": 3, "x": 0.2, "grasp_id": 8}),
        CandidateMetrics(**{**candidate(4).__dict__, "candidate_id": 4, "x": 1.0}),
        CandidateMetrics(**{**candidate(5).__dict__, "candidate_id": 5, "x": 0.05, "lift_position": 0.5}),
    ]
    scores = [score_candidate(item, ScoreWeights(), 0.3) for item in metrics]
    ready = conditioned_ready_candidate_ids(
        metrics, scores, 1, 0.5, 0.8, neighbor_distance_m=0.11, neighbor_yaw_rad=0.1
    )
    assert ready == [1, 2]
