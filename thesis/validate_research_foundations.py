"""Constructed mathematical checks, not training or robot performance evidence."""

import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "openflex_ws/src/openflex_thesis/openflex_tc_wam"
sys.path.insert(0, str(PACKAGE))
from openflex_tc_wam.scoring import (
    CandidateMetrics, ScoreWeights, conditioned_ready_candidate_ids, score_candidate,
)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(frame, pose):
    cosine = math.cos(frame[2])
    sine = math.sin(frame[2])
    return (frame[0] + cosine * pose[0] - sine * pose[1],
            frame[1] + sine * pose[0] + cosine * pose[1], wrap_angle(frame[2] + pose[2]))


def local_pose(origin, current):
    delta_x = current[0] - origin[0]
    delta_y = current[1] - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (cosine * delta_x + sine * delta_y, -sine * delta_x + cosine * delta_y,
            wrap_angle(current[2] - origin[2]))


def evaluate():
    origin = (1.1, -2.3, 0.7)
    current = (1.8, -0.9, -0.4)
    frame_change = (-8.3, 4.2, 1.3)
    before = local_pose(origin, current)
    after = local_pose(compose_pose(frame_change, origin), compose_pose(frame_change, current))
    errors = [abs(before[index] - after[index]) for index in (0, 1)]
    errors.append(abs(wrap_angle(before[2] - after[2])))
    if max(errors) > 1.0e-12:
        raise AssertionError("Common-frame invariance failed")
    first = CandidateMetrics(1, 0.0, 0.0, 0.0, True, True, True, True, True,
                             1.0, 0.8, 1.0, 0.9, grasp_id=7, lift_seed=0.15,
                             lift_position=0.16, planning_group="right_arm_with_lift",
                             grasp_quality=0.9, visibility_score=0.8)
    second = replace(first, candidate_id=2, x=0.1)
    candidates = [first, second]
    scores = [score_candidate(item, ScoreWeights(), 0.34) for item in candidates]
    component = conditioned_ready_candidate_ids(candidates, scores, 1, 0.65, 0.9, 0.11, 0.2)
    if component != [1, 2]:
        raise AssertionError("Geometric adjacency example changed")
    angle_left = math.pi - 1.0e-6
    angle_right = -math.pi + 1.0e-6
    circular_distance = abs(wrap_angle(angle_left - angle_right))
    sine_cosine_distance = math.hypot(math.sin(angle_left) - math.sin(angle_right),
                                     math.cos(angle_left) - math.cos(angle_right))
    source = PACKAGE / "openflex_tc_wam/scoring.py"
    return {
        "evidence_type": "constructed_offline_mathematical_checks_NOT_robot_or_training_experiments",
        "gauge_invariance": {"before": before, "after": after, "max_absolute_error": max(errors),
                             "assumption": "Same constant SE(2) transform applied to origin and current pose"},
        "velocity_aliasing": {"velocities_at_both_times": [0, 0, 0],
                              "local_positions": [[0, 0, 0], [0.5, 0, 0]],
                              "conclusion": "Instantaneous velocity alone does not determine relative displacement; images or history may disambiguate."},
        "yaw_boundary": {"scalar_distance": abs(angle_left - angle_right),
                         "circular_distance": circular_distance, "sine_cosine_distance": sine_cosine_distance,
                         "warning": "Sine-cosine adds one dimension; training benefit is not established."},
        "geometric_adjacency_not_motion_certificate": {
            "component": component, "endpoint_x": [0.0, 0.1], "endpoint_y": [0.0, 0.0],
            "barrier_x_interval": [0.045, 0.055], "barrier_y_interval": [-1.0, 1.0],
            "straight_segment_midpoint": [0.05, 0.0], "midpoint_is_blocked": True,
            "conclusion": "The adjacency rule has no swept-path or IK-branch certificate. Separate motion validation remains necessary.",
            "source": str(source.relative_to(ROOT)), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        },
        "limitations": "No measured success rate, generalization gain, speed-up, or novelty claim follows from these checks.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    with arguments.output.open("x") as stream:
        json.dump(evaluate(), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(arguments.output)


if __name__ == "__main__":
    main()
