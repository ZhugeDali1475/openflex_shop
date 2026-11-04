import math

from openflex_tc_wam.candidate_generation import CandidateGeneratorConfig, generate_candidates
from openflex_tc_wam.geometry import PlanarPose, normalize_angle


def test_candidate_count_and_facing_direction():
    config = CandidateGeneratorConfig(
        distance_min_m=0.6,
        distance_max_m=0.8,
        distance_step_m=0.2,
        lateral_min_m=0.0,
        lateral_max_m=0.0,
        lateral_step_m=0.1,
        yaw_offsets_rad=(0.0,),
    )
    result = generate_candidates(PlanarPose(1.0, 2.0, 0.0), config)
    assert len(result) == 2
    assert result[0].x == 1.6
    assert result[0].y == 2.0
    assert abs(normalize_angle(result[0].yaw - math.pi)) < 1.0e-9
