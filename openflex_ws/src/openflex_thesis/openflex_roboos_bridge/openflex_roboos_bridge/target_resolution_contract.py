"""Fail-closed visual candidate to grasp-authorization contract.

Vision/model output is only a candidate.  The contract requires unique
identity, depth/pose confirmation, and an explicit authorization record before
grasp planning may be requested.  It never calls a detector or ROS.
"""

from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

from .competition_registry import TARGET_OBJECT_ID


class TargetResolutionError(ValueError):
    pass


@dataclass(frozen=True)
class VisualCandidate:
    candidate_id: str
    color: str
    confidence: float
    depth_m: float | None
    pose_xyzyaw: Tuple[float, float, float, float] | None
    visible: bool = True
    resolved_object_id: str | None = None


@dataclass(frozen=True)
class GraspAuthorization:
    object_id: str
    candidate_id: str
    evidence: Mapping[str, object]


@dataclass(frozen=True)
class RecognitionAudit:
    model_candidates: Tuple[Mapping[str, object], ...]
    oracle_truth: Mapping[str, object]

    def validate(self) -> None:
        if "model_output" in self.oracle_truth:
            raise TargetResolutionError("oracle truth must remain separate from model output")


def resolve_unique_red_candidate(candidates: Sequence[VisualCandidate], *, min_confidence: float = 0.8) -> VisualCandidate:
    if not isinstance(min_confidence, (int, float)) or not 0.0 <= min_confidence <= 1.0:
        raise TargetResolutionError("min_confidence must be in [0, 1]")
    eligible = [item for item in candidates if item.visible and item.color.casefold() == "red"
                and item.confidence >= min_confidence]
    if len(eligible) != 1:
        raise TargetResolutionError("red target is absent, ambiguous, occluded, or below confidence")
    return eligible[0]


def confirm_unique_instance(candidate: VisualCandidate, *, registry_object_id: str = TARGET_OBJECT_ID,
                            identity_confirmed: bool) -> str:
    if identity_confirmed is not True:
        raise TargetResolutionError("unique object identity confirmation is required")
    if candidate.resolved_object_id != registry_object_id:
        raise TargetResolutionError("candidate does not map to the registered single red object")
    return registry_object_id


def authorize_grasp(candidate: VisualCandidate, *, identity_confirmed: bool,
                    depth_confirmed: bool, pose_confirmed: bool) -> GraspAuthorization:
    object_id = confirm_unique_instance(candidate, identity_confirmed=identity_confirmed)
    if depth_confirmed is not True or pose_confirmed is not True:
        raise TargetResolutionError("depth and pose confirmation are required before grasp authorization")
    if candidate.depth_m is None or candidate.pose_xyzyaw is None:
        raise TargetResolutionError("candidate lacks depth or pose data")
    return GraspAuthorization(object_id, candidate.candidate_id, {
        "identity_confirmed": True, "depth_confirmed": True, "pose_confirmed": True,
        "source": "candidate_confirmation_not_oracle_truth"})
