"""ROS-free input contract for adapting RGB-D observations to FitGrasp.

This module intentionally stops before ROS action transport.  It accepts only a
validated :class:`TargetObservation` and point clouds derived from the same
camera frame.  It never creates or infers a MuJoCo/object-memory identity.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np

from .target_observation import TargetObservation, TargetObservationError


class VisualGraspInputError(ValueError):
    """Raised when visual point-cloud input cannot be sent to a grasp fitter."""


def _xyz(points: np.ndarray, name: str, minimum_points: int) -> np.ndarray:
    values = np.asarray(points)
    if values.ndim != 2 or values.shape[1] < 3:
        raise VisualGraspInputError(f"{name} must have shape (N, >=3)")
    if values.shape[0] < int(minimum_points):
        raise VisualGraspInputError(
            f"{name} has {values.shape[0]} points; at least {minimum_points} are required"
        )
    xyz = np.array(values[:, :3], dtype=np.float32, order="C", copy=True)
    if not np.isfinite(xyz).all():
        raise VisualGraspInputError(f"{name} contains non-finite XYZ values")
    return np.ascontiguousarray(xyz)


@dataclass(frozen=True)
class VisualGraspPointCloudInput:
    """Same-frame visual point clouds ready for a future FitGrasp adapter.

    ``product_xyz`` and ``scene_xyz`` are camera-frame coordinates.  The class
    deliberately has no ``object_id`` field; the only identity is the validated
    visual ``perception_track_id`` inside ``observation``.
    """

    observation: TargetObservation
    product_xyz: np.ndarray
    scene_xyz: np.ndarray
    frame_id: str
    timestamp_sec: float
    minimum_points: int = 100

    def __post_init__(self) -> None:
        if not isinstance(self.observation, TargetObservation):
            raise VisualGraspInputError("observation must be a TargetObservation")
        try:
            self.observation.validate_for_grasp()
        except TargetObservationError as error:
            raise VisualGraspInputError(str(error)) from error
        if not self.frame_id or self.frame_id != self.observation.camera_frame:
            raise VisualGraspInputError(
                "point clouds must use the observation camera_frame"
            )
        arm_name = str(self.observation.metadata.get("arm_name", "")).strip().lower()
        if arm_name not in {"left", "right"}:
            raise VisualGraspInputError("observation metadata must identify left or right arm")
        if not math.isfinite(float(self.timestamp_sec)):
            raise VisualGraspInputError("timestamp_sec must be finite")
        if abs(float(self.timestamp_sec) - float(self.observation.timestamp_sec)) > 1e-3:
            raise VisualGraspInputError("point-cloud and observation timestamps differ")
        if int(self.minimum_points) < 1:
            raise VisualGraspInputError("minimum_points must be positive")
        product = _xyz(self.product_xyz, "product_xyz", int(self.minimum_points))
        scene = _xyz(self.scene_xyz, "scene_xyz", int(self.minimum_points))
        product.setflags(write=False)
        scene.setflags(write=False)
        object.__setattr__(self, "product_xyz", product)
        object.__setattr__(self, "scene_xyz", scene)

    def to_fit_grasp_payload(self) -> Mapping[str, object]:
        """Serialize a ROS-free payload for a separately implemented adapter.

        The payload is not a ROS ``FitGrasp.Goal`` and cannot execute a grasp.
        """

        return {
            "perception_track_id": self.observation.perception_track_id,
            "arm_name": str(self.observation.metadata["arm_name"]).strip().lower(),
            "frame_id": self.frame_id,
            "timestamp_sec": float(self.timestamp_sec),
            "product_xyz": self.product_xyz.copy(),
            "scene_xyz": self.scene_xyz.copy(),
            "source": self.observation.source,
            "oracle_input": False,
            "physical_verification": False,
        }
