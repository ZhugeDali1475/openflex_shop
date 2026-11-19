"""Offline grasp-plan acceptance record for the single red fixture object."""

from dataclasses import dataclass
import math
from typing import Mapping, Tuple

from .competition_registry import TARGET_OBJECT_ID


class GraspPlanError(ValueError):
    pass


@dataclass(frozen=True)
class GraspCandidate:
    candidate_id: str
    object_id: str
    object_dimensions_m: Tuple[float, float, float]
    mass_kg: float
    shelf_id: str
    face: str
    level: int
    pregrasp_joint_positions: Tuple[float, ...]
    grasp_joint_positions: Tuple[float, ...]


def evaluate_grasp_candidate(candidate: GraspCandidate, *, full_robot_collision_free: bool,
                             ik_valid: bool, tc_wam_valid: bool,
                             non_active_arm_clear: bool) -> Mapping[str, object]:
    if candidate.object_id != TARGET_OBJECT_ID:
        raise GraspPlanError("candidate is not the registered single red target")
    if len(candidate.object_dimensions_m) != 3 or any(
            not math.isfinite(value) or value <= 0 for value in candidate.object_dimensions_m):
        raise GraspPlanError("object dimensions must be finite and positive")
    if not math.isfinite(candidate.mass_kg) or candidate.mass_kg <= 0:
        raise GraspPlanError("object mass must be finite and positive")
    if not candidate.shelf_id or candidate.face not in {"neg", "pos"} or candidate.level < 0:
        raise GraspPlanError("candidate must identify a valid shelf face and level")
    checks = {
        "full_robot_collision_free": full_robot_collision_free,
        "ik_valid": ik_valid,
        "tc_wam_valid": tc_wam_valid,
        "non_active_arm_clear": non_active_arm_clear,
    }
    if any(value is not True for value in checks.values()):
        failed = [key for key, value in checks.items() if value is not True]
        return {"candidate_id": candidate.candidate_id, "object_id": candidate.object_id,
                "graspable": False, "failed_checks": failed,
                "grasp_execution_authorized": False}
    return {"candidate_id": candidate.candidate_id, "object_id": candidate.object_id,
            "graspable": True, "failed_checks": [],
            "grasp_execution_authorized": True,
            "planning_only": True,
            "physical_execution_validated": False}
