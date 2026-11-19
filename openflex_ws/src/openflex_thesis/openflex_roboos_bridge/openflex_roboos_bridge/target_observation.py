"""Explicit boundary between visual observations and simulator identities.

This module is deliberately ROS-free.  A visual observation is the only
object identity accepted by the pure-vision search/grasp path.  A MuJoCo
object id can be attached by an evaluator after the run, but is rejected as
an online observation field.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping, Optional, Tuple


class TargetObservationError(ValueError):
    """Raised when an observation is not safe to authorize for grasping."""


@dataclass(frozen=True)
class TargetObservation:
    perception_track_id: str
    task_target: str
    color: Optional[str]
    bbox_px: Tuple[int, int, int, int]
    position_camera_xyz: Tuple[float, float, float]
    position_map_xyz: Tuple[float, float, float]
    depth_valid_fraction: float
    timestamp_sec: float
    camera_frame: str
    map_frame: str
    tf_valid: bool
    confidence: float
    source: str = "rgbd_visual_observation"
    pointcloud_ref: Optional[str] = None
    mask_ref: Optional[str] = None
    metadata: Mapping[str, object] = field(default_factory=dict)

    def validate_for_grasp(self) -> None:
        if not self.perception_track_id or self.perception_track_id.startswith("mujoco_"):
            raise TargetObservationError("a visual perception_track_id is required")
        if not self.task_target:
            raise TargetObservationError("task_target is required")
        if len(self.bbox_px) != 4:
            raise TargetObservationError("bbox_px must contain four values")
        for vector_name, vector in (("position_camera_xyz", self.position_camera_xyz),
                                    ("position_map_xyz", self.position_map_xyz)):
            if len(vector) != 3 or not all(math.isfinite(float(value)) for value in vector):
                raise TargetObservationError(f"{vector_name} must be finite 3D coordinates")
        if not 0.0 < float(self.depth_valid_fraction) <= 1.0:
            raise TargetObservationError("depth_valid_fraction must be in (0, 1]")
        if not math.isfinite(float(self.timestamp_sec)):
            raise TargetObservationError("timestamp_sec must be finite")
        if not self.camera_frame or not self.map_frame or self.tf_valid is not True:
            raise TargetObservationError("valid camera/map frames and TF are required")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise TargetObservationError("confidence must be in [0, 1]")
        if self.source.startswith("oracle") or "mujoco" in self.source.lower():
            raise TargetObservationError("oracle/MuJoCo observations are not valid visual grasp input")

    def to_dict(self) -> dict:
        return {
            "perception_track_id": self.perception_track_id,
            "task_target": self.task_target,
            "color": self.color,
            "bbox_px": list(self.bbox_px),
            "position_camera_xyz": list(self.position_camera_xyz),
            "position_map_xyz": list(self.position_map_xyz),
            "depth_valid_fraction": self.depth_valid_fraction,
            "timestamp_sec": self.timestamp_sec,
            "camera_frame": self.camera_frame,
            "map_frame": self.map_frame,
            "tf_valid": self.tf_valid,
            "confidence": self.confidence,
            "source": self.source,
            "pointcloud_ref": self.pointcloud_ref,
            "mask_ref": self.mask_ref,
            "metadata": dict(self.metadata),
            "mujoco_object_id": None,
            "physical_verification": False,
        }


def observation_from_candidate(candidate, *, task_target: str, timestamp_sec: float,
                               camera_frame: str, map_frame: str,
                               position_camera_xyz, depth_valid_fraction: float = 1.0,
                               track_id: Optional[str] = None, tf_valid: bool = True,
                               source: str = "rgbd_visual_observation") -> TargetObservation:
    """Build and validate an observation from an image-derived candidate."""
    pose = candidate.pose_xyzyaw
    if pose is None:
        raise TargetObservationError("candidate has no RGB-D map position")
    x0, y0, x1, y1 = candidate.bbox_px
    observation = TargetObservation(
        perception_track_id=track_id or f"track_{candidate.candidate_id.replace('[', '_').replace(']', '_').replace('#', '_')}",
        task_target=task_target, color=candidate.color, bbox_px=(x0, y0, x1, y1),
        position_camera_xyz=tuple(float(value) for value in position_camera_xyz),
        position_map_xyz=tuple(float(value) for value in pose[:3]),
        depth_valid_fraction=float(depth_valid_fraction), timestamp_sec=float(timestamp_sec),
        camera_frame=camera_frame, map_frame=map_frame, tf_valid=tf_valid,
        confidence=float(candidate.confidence), source=source)
    observation.validate_for_grasp()
    return observation
