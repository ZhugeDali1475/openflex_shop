"""Bind the visual grasp-planning request to the separated evidence schema.

The planning request already carries no simulator identity.  This module writes
it into the perception layer and keeps the simulation-evaluation layer separate
and truth-free until an evaluator links it after execution.
"""

from __future__ import annotations

from typing import Mapping, Optional

from .evidence_schema import (
    EvidenceSchemaError,
    PerceptionEvidence,
    SimulationEvaluationEvidence,
)
from .visual_grasp_planning import VisualGraspPlanRequest, VisualGraspPlanningError


class VisualGraspEvidenceError(VisualGraspPlanningError):
    """Raised when planning evidence would mix perception and simulator truth."""


def planning_perception_evidence(request: VisualGraspPlanRequest, *, request_id: str,
                                 attempt: int, frame_sha256: str, frame_ref: str,
                                 depth_ref: str) -> PerceptionEvidence:
    if not isinstance(request, VisualGraspPlanRequest):
        raise VisualGraspEvidenceError("planning evidence requires a VisualGraspPlanRequest")
    record = {
        "boundary": "visual_grasp_planning",
        "perception_track_id": request.observation.perception_track_id,
        "task_target": request.observation.task_target,
        "planning_frame": request.planning_frame,
        "candidate_count": len(request.candidates),
        "visual_grasp_plan": request.to_planning_payload(),
    }
    evidence = PerceptionEvidence(
        request_id=request_id, attempt=attempt, frame_sha256=frame_sha256,
        frame_ref=frame_ref, depth_ref=depth_ref, record=record)
    try:
        evidence.to_dict()
    except EvidenceSchemaError as error:
        raise VisualGraspEvidenceError(str(error)) from error
    return evidence


def planning_evaluation_evidence(request_id: str, attempt: int, *,
                                 mujoco_object_id: Optional[str] = None,
                                 physical_verification: bool = False,
                                 source: str = "pending_post_execution_evaluator"
                                 ) -> SimulationEvaluationEvidence:
    evidence = SimulationEvaluationEvidence(
        request_id=request_id, attempt=attempt, mujoco_object_id=mujoco_object_id,
        physical_verification=bool(physical_verification), source=source)
    try:
        evidence.to_dict()
    except EvidenceSchemaError as error:
        raise VisualGraspEvidenceError(str(error)) from error
    return evidence


def planning_evidence_pair(request: VisualGraspPlanRequest, *, request_id: str, attempt: int,
                           frame_sha256: str, frame_ref: str, depth_ref: str
                           ) -> Mapping[str, Mapping[str, object]]:
    """Return the perception/evaluation pair; perception never carries truth."""

    perception = planning_perception_evidence(
        request, request_id=request_id, attempt=attempt, frame_sha256=frame_sha256,
        frame_ref=frame_ref, depth_ref=depth_ref)
    evaluation = planning_evaluation_evidence(request_id, attempt)
    try:
        return {"perception": perception.to_dict(), "evaluation": evaluation.to_dict()}
    except EvidenceSchemaError as error:
        raise VisualGraspEvidenceError(str(error)) from error
