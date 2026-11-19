"""Offline grasp-plan acceptance record for the single red fixture object."""

from dataclasses import dataclass
import math
from typing import Mapping, Tuple

from .competition_registry import TARGET_OBJECT_ID
from .target_observation import TargetObservation, TargetObservationError


class GraspPlanError(ValueError):
    pass


def validate_visual_grasp_input(observation: TargetObservation) -> Mapping[str, object]:
    """Validate the only input accepted by a future pure-vision grasp backend.

    This is intentionally separate from the legacy simulator-object planner.
    It proves the boundary at the API layer without pretending that the current
    MuJoCo planning probe is an RGB-D grasp model.
    """
    if not isinstance(observation, TargetObservation):
        raise GraspPlanError("visual grasp requires a TargetObservation, not object_id")
    try:
        observation.validate_for_grasp()
    except TargetObservationError as error:
        raise GraspPlanError(str(error)) from error
    return {
        "perception_track_id": observation.perception_track_id,
        "task_target": observation.task_target,
        "position_map_xyz": list(observation.position_map_xyz),
        "source": observation.source,
        "physical_verification": False,
        "oracle_input": False,
    }


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
