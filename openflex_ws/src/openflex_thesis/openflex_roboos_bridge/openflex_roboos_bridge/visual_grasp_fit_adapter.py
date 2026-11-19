"""ROS-free transport adapter between visual point clouds and the FitGrasp backend.

This closes the middle of the pure-vision chain:

    TargetObservation -> VisualGraspPointCloudInput -> FitGraspRequestPayload
        -> (future FitGrasp server / GraspNet worker) -> FitGraspResult
        -> GraspCandidatePose -> VisualGraspPlanRequest

Everything stays in the camera optical frame and carries no simulator identity.
The adapter does not call a ROS action, load a model, or execute motion; it only
shapes a request payload and validates a result.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple

import numpy as np

from .visual_grasp_input import VisualGraspPointCloudInput
from .visual_grasp_planning import GraspCandidatePose, VisualGraspPlanningError


class VisualGraspFitError(VisualGraspPlanningError):
    """Raised when a FitGrasp request/result cannot stay on the visual boundary."""


def _finite_vector(values, length: int, name: str) -> Tuple[float, ...]:
    sequence = tuple(values)
    if len(sequence) != length:
        raise VisualGraspFitError(f"{name} must have {length} components")
    result = tuple(float(value) for value in sequence)
    if not all(math.isfinite(value) for value in result):
        raise VisualGraspFitError(f"{name} must be finite")
    return result


def _unit_quaternion(values, name: str) -> Tuple[float, float, float, float]:
    quaternion = _finite_vector(values, 4, name)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-9 or abs(norm - 1.0) > 0.01:
        raise VisualGraspFitError(f"{name} must be a unit quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


@dataclass(frozen=True)
class FitGraspRequestPayload:
    """ROS-free analogue of ``FitGrasp.Goal`` built from a visual capture."""

    detection_id: str
    arm_name: str
    grasp_strategy: str
    frame_id: str
    timestamp_sec: float
    product_xyz: np.ndarray
    scene_xyz: np.ndarray
    timeout_sec: float
    source: str = "rgbd_visual_observation"

    @classmethod
    def from_visual_input(cls, visual_input: VisualGraspPointCloudInput, *,
                          grasp_strategy: str = "auto",
                          timeout_sec: float = 5.0) -> "FitGraspRequestPayload":
        if not isinstance(visual_input, VisualGraspPointCloudInput):
            raise VisualGraspFitError(
                "fit request requires a VisualGraspPointCloudInput, not object_id")
        strategy = str(grasp_strategy).strip()
        if not strategy:
            raise VisualGraspFitError("grasp_strategy is required")
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0:
            raise VisualGraspFitError("timeout_sec must be finite and positive")
        return cls(
            detection_id=visual_input.observation.perception_track_id,
            arm_name=str(visual_input.observation.metadata["arm_name"]).strip().lower(),
            grasp_strategy=strategy,
            frame_id=visual_input.frame_id,
            timestamp_sec=float(visual_input.timestamp_sec),
            product_xyz=visual_input.product_xyz.copy(),
            scene_xyz=visual_input.scene_xyz.copy(),
            timeout_sec=timeout,
            source=visual_input.observation.source,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "detection_id": self.detection_id,
            "arm_name": self.arm_name,
            "grasp_strategy": self.grasp_strategy,
            "frame_id": self.frame_id,
            "timestamp_sec": self.timestamp_sec,
            "product_xyz": self.product_xyz,
            "scene_xyz": self.scene_xyz,
            "timeout_sec": self.timeout_sec,
            "source": self.source,
            "oracle_input": False,
            "physical_verification": False,
        }


@dataclass(frozen=True)
class FitGraspResult:
    """ROS-free analogue of the ``FitGrasp`` result, in the camera frame."""

    success: bool
    frame_id: str
    timestamp_sec: float
    grasp_position_camera_xyz: Tuple[float, float, float]
    grasp_quaternion_xyzw: Tuple[float, float, float, float]
    required_width_m: float
    score: float
    candidate_count: int
    pregrasp_position_camera_xyz: Optional[Tuple[float, float, float]] = None
    pregrasp_quaternion_xyzw: Optional[Tuple[float, float, float, float]] = None
    constrained_candidate_count: int = 0
    error_code: int = 0
    message: str = ""
    source: str = "graspnet_visual_fit"


def grasp_candidates_from_result(result: FitGraspResult, visual_input: VisualGraspPointCloudInput,
                                 *, candidate_id: Optional[str] = None,
                                 timestamp_tolerance_sec: float = 1e-3
                                 ) -> Tuple[GraspCandidatePose, ...]:
    """Validate a FitGrasp result and convert it to planning candidates."""

    if not isinstance(result, FitGraspResult):
        raise VisualGraspFitError("expected a FitGraspResult")
    if not isinstance(visual_input, VisualGraspPointCloudInput):
        raise VisualGraspFitError(
            "result conversion requires a VisualGraspPointCloudInput, not object_id")
    if not bool(result.success):
        raise VisualGraspFitError(
            f"FitGrasp reported failure (error_code={int(result.error_code)}): {result.message}")
    if str(result.source).lower().startswith("oracle") or "mujoco" in str(result.source).lower():
        raise VisualGraspFitError("oracle/MuJoCo fit results are not valid visual candidates")
    if str(result.frame_id).strip() != visual_input.frame_id:
        raise VisualGraspFitError("FitGrasp result frame must match the visual camera frame")
    if abs(float(result.timestamp_sec) - float(visual_input.timestamp_sec)) > float(timestamp_tolerance_sec):
        raise VisualGraspFitError("FitGrasp result must describe the same frame as the point clouds")
    if int(result.candidate_count) < 1:
        raise VisualGraspFitError("FitGrasp reported no candidates")
    width = float(result.required_width_m)
    if not math.isfinite(width) or width < 0:
        raise VisualGraspFitError("required_width_m must be finite and non-negative")
    score = float(result.score)
    if not math.isfinite(score):
        raise VisualGraspFitError("grasp score must be finite")
    identifier = str(candidate_id or f"{visual_input.observation.perception_track_id}_0").strip()
    candidate = GraspCandidatePose(
        candidate_id=identifier,
        position_camera_xyz=_finite_vector(
            result.grasp_position_camera_xyz, 3, "grasp position"),
        quaternion_xyzw=_unit_quaternion(
            result.grasp_quaternion_xyzw, "grasp orientation"),
        score=score,
        required_width_m=width,
    )
    return (candidate,)
