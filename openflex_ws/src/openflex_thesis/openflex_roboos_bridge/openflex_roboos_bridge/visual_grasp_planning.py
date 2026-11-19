"""ROS-free visual grasp planning-request contract.

This module implements the pure-data boundary recommended in the phase-A audit
(section 11): a grasp candidate fitted from RGB-D point clouds is transformed
from the camera optical frame into the MoveIt planning frame and paired with an
independent target collision volume plus the static shelf obstacles.  The
resulting request contains robot measured state, candidate poses and the
collision scene only.

It never reads ``ObjectMemory``, never emits a simulator ``object_id`` and never
calls MoveIt or executes motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence, Tuple

import numpy as np

from .target_observation import TargetObservation, TargetObservationError
from .visual_grasp_input import VisualGraspPointCloudInput


class VisualGraspPlanningError(ValueError):
    """Raised when visual candidates cannot form a safe planning request."""


_SIM_IDENTITY_KEYS = frozenset({
    "object_id", "mujoco_object_id", "mujoco_id", "truth_object_id",
    "oracle_object_id", "selected_object_id",
})


def _finite_triplet(values, name: str) -> Tuple[float, float, float]:
    sequence = tuple(values)
    if len(sequence) != 3:
        raise VisualGraspPlanningError(f"{name} must have three components")
    result = tuple(float(value) for value in sequence)
    if not all(math.isfinite(value) for value in result):
        raise VisualGraspPlanningError(f"{name} must be finite")
    return result  # type: ignore[return-value]


def _unit_quaternion(values, name: str) -> Tuple[float, float, float, float]:
    sequence = tuple(values)
    if len(sequence) != 4:
        raise VisualGraspPlanningError(f"{name} must have four components")
    quaternion = tuple(float(value) for value in sequence)
    if not all(math.isfinite(value) for value in quaternion):
        raise VisualGraspPlanningError(f"{name} must be finite")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-9 or abs(norm - 1.0) > 0.01:
        raise VisualGraspPlanningError(f"{name} must be a unit quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _rotation_matrix(quaternion_xyzw) -> Tuple[Tuple[float, float, float], ...]:
    x, y, z, w = _unit_quaternion(quaternion_xyzw, "quaternion")
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _quaternion_multiply(left, right) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _assert_no_simulator_identity(value, path: str = "payload") -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            if str(key) in _SIM_IDENTITY_KEYS and item is not None:
                raise VisualGraspPlanningError(
                    f"planning request leaks simulator identity at {path}.{key}"
                )
            _assert_no_simulator_identity(item, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _assert_no_simulator_identity(item, f"{path}[{index}]")


@dataclass(frozen=True)
class FrameTransform:
    """Pose of the camera optical frame expressed in the planning frame."""

    translation_xyz: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "translation_xyz",
                           _finite_triplet(self.translation_xyz, "transform translation"))
        object.__setattr__(self, "quaternion_xyzw",
                           _unit_quaternion(self.quaternion_xyzw, "transform orientation"))

    def transform_point(self, point_camera_xyz: Sequence[float]) -> Tuple[float, float, float]:
        rotation = _rotation_matrix(self.quaternion_xyzw)
        x, y, z = _finite_triplet(point_camera_xyz, "point")
        return tuple(
            self.translation_xyz[row] + rotation[row][0] * x
            + rotation[row][1] * y + rotation[row][2] * z
            for row in range(3)
        )  # type: ignore[return-value]


@dataclass(frozen=True)
class Aabb:
    """Axis-aligned bounding box in a single named frame."""

    min_xyz: Tuple[float, float, float]
    max_xyz: Tuple[float, float, float]

    def __post_init__(self) -> None:
        low = _finite_triplet(self.min_xyz, "aabb min")
        high = _finite_triplet(self.max_xyz, "aabb max")
        if any(high[axis] <= low[axis] for axis in range(3)):
            raise VisualGraspPlanningError("aabb max must exceed min on every axis")
        object.__setattr__(self, "min_xyz", low)
        object.__setattr__(self, "max_xyz", high)

    def to_dict(self) -> Mapping[str, object]:
        return {"min_xyz": list(self.min_xyz), "max_xyz": list(self.max_xyz)}


def aabb_from_pointcloud(points, *, padding_m: float = 0.0,
                         minimum_points: int = 1) -> Aabb:
    """Conservative target volume derived from a segmented product cloud."""

    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < 3:
        raise VisualGraspPlanningError("point cloud must have shape (N, >=3)")
    if values.shape[0] < int(minimum_points):
        raise VisualGraspPlanningError(
            f"point cloud needs at least {minimum_points} points")
    xyz = values[:, :3]
    if not np.isfinite(xyz).all():
        raise VisualGraspPlanningError("point cloud contains non-finite XYZ")
    padding = float(padding_m)
    if not math.isfinite(padding) or padding < 0:
        raise VisualGraspPlanningError("padding_m must be finite and non-negative")
    low = xyz.min(axis=0) - padding
    high = xyz.max(axis=0) + padding
    return Aabb(tuple(float(value) for value in low),
                tuple(float(value) for value in high))


def transform_aabb(box: Aabb, transform: FrameTransform) -> Aabb:
    """Bounding AABB of a box's eight corners after a frame transform."""

    if not isinstance(box, Aabb) or not isinstance(transform, FrameTransform):
        raise VisualGraspPlanningError("transform_aabb requires an Aabb and FrameTransform")
    corners = np.asarray([
        transform.transform_point((x, y, z))
        for x in (box.min_xyz[0], box.max_xyz[0])
        for y in (box.min_xyz[1], box.max_xyz[1])
        for z in (box.min_xyz[2], box.max_xyz[2])
    ], dtype=np.float64)
    low = corners.min(axis=0)
    high = corners.max(axis=0)
    return Aabb(tuple(float(value) for value in low), tuple(float(value) for value in high))


@dataclass(frozen=True)
class GraspCandidatePose:
    """A fitted grasp pose in the camera optical frame; never a simulator identity."""

    candidate_id: str
    position_camera_xyz: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]
    score: float = 0.0
    required_width_m: float = 0.0

    def __post_init__(self) -> None:
        candidate_id = str(self.candidate_id).strip()
        lowered = candidate_id.lower()
        if not candidate_id:
            raise VisualGraspPlanningError("candidate_id is required")
        if lowered.startswith(("mujoco", "thesis_item")) or "object_id" in lowered:
            raise VisualGraspPlanningError("candidate_id must not carry a simulator identity")
        position = _finite_triplet(self.position_camera_xyz, "candidate position")
        quaternion = _unit_quaternion(self.quaternion_xyzw, "candidate orientation")
        score = float(self.score)
        width = float(self.required_width_m)
        if not math.isfinite(score):
            raise VisualGraspPlanningError("candidate score must be finite")
        if not math.isfinite(width) or width < 0:
            raise VisualGraspPlanningError("required_width_m must be finite and non-negative")
        object.__setattr__(self, "candidate_id", candidate_id)
        object.__setattr__(self, "position_camera_xyz", position)
        object.__setattr__(self, "quaternion_xyzw", quaternion)
        object.__setattr__(self, "score", score)
        object.__setattr__(self, "required_width_m", width)


@dataclass(frozen=True)
class VisualGraspPlanRequest:
    """Pure-data MoveIt planning input built only from visual evidence."""

    observation: TargetObservation
    planning_frame: str
    camera_frame: str
    camera_to_planning: FrameTransform
    target_aabb_planning: Aabb
    static_obstacles_planning: Tuple[Aabb, ...]
    candidates: Tuple[GraspCandidatePose, ...]
    joint_names: Tuple[str, ...]
    joint_positions: Tuple[float, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.observation, TargetObservation):
            raise VisualGraspPlanningError("planning request requires a TargetObservation")
        try:
            self.observation.validate_for_grasp()
        except TargetObservationError as error:
            raise VisualGraspPlanningError(str(error)) from error
        planning_frame = str(self.planning_frame).strip()
        if not planning_frame:
            raise VisualGraspPlanningError("planning_frame is required")
        camera_frame = str(self.camera_frame).strip()
        if camera_frame != self.observation.camera_frame:
            raise VisualGraspPlanningError(
                "point-cloud camera_frame must match the observation camera_frame")
        if not isinstance(self.camera_to_planning, FrameTransform):
            raise VisualGraspPlanningError("camera_to_planning must be a FrameTransform")
        if not isinstance(self.target_aabb_planning, Aabb):
            raise VisualGraspPlanningError("target_aabb_planning must be an Aabb")
        obstacles = tuple(self.static_obstacles_planning)
        if not obstacles or not all(isinstance(box, Aabb) for box in obstacles):
            raise VisualGraspPlanningError(
                "planning requires the static shelf collision geometry")
        candidates = tuple(self.candidates)
        if not candidates or not all(isinstance(item, GraspCandidatePose) for item in candidates):
            raise VisualGraspPlanningError("at least one fitted visual candidate is required")
        names = tuple(str(name).strip() for name in self.joint_names)
        if not names or any(not name for name in names):
            raise VisualGraspPlanningError("joint_names must be non-empty")
        if len(set(names)) != len(names):
            raise VisualGraspPlanningError("joint_names must be unique")
        positions = tuple(float(value) for value in self.joint_positions)
        if len(positions) != len(names) or not all(math.isfinite(value) for value in positions):
            raise VisualGraspPlanningError("joint_positions must match joint_names and be finite")
        object.__setattr__(self, "planning_frame", planning_frame)
        object.__setattr__(self, "camera_frame", camera_frame)
        object.__setattr__(self, "static_obstacles_planning", obstacles)
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "joint_positions", positions)

    @classmethod
    def from_visual_input(cls, visual_input: VisualGraspPointCloudInput, *,
                          planning_frame: str, camera_to_planning: FrameTransform,
                          static_obstacles_planning: Sequence[Aabb],
                          candidates: Sequence[GraspCandidatePose],
                          joint_names: Sequence[str], joint_positions: Sequence[float],
                          target_padding_m: float = 0.0) -> "VisualGraspPlanRequest":
        if not isinstance(visual_input, VisualGraspPointCloudInput):
            raise VisualGraspPlanningError(
                "planning requires a VisualGraspPointCloudInput, not object_id")
        camera_aabb = aabb_from_pointcloud(
            visual_input.product_xyz, padding_m=target_padding_m,
            minimum_points=visual_input.minimum_points)
        return cls(
            observation=visual_input.observation,
            planning_frame=planning_frame,
            camera_frame=visual_input.frame_id,
            camera_to_planning=camera_to_planning,
            target_aabb_planning=transform_aabb(camera_aabb, camera_to_planning),
            static_obstacles_planning=tuple(static_obstacles_planning),
            candidates=tuple(candidates),
            joint_names=tuple(joint_names),
            joint_positions=tuple(joint_positions),
        )

    def to_planning_payload(self) -> Mapping[str, object]:
        """Serialize a MoveIt-ready request that contains no simulator identity."""

        rotation = _rotation_matrix(self.camera_to_planning.quaternion_xyzw)
        translation = self.camera_to_planning.translation_xyz
        candidates = []
        for candidate in self.candidates:
            x, y, z = candidate.position_camera_xyz
            position = tuple(
                translation[row] + rotation[row][0] * x
                + rotation[row][1] * y + rotation[row][2] * z
                for row in range(3)
            )
            orientation = _quaternion_multiply(
                self.camera_to_planning.quaternion_xyzw, candidate.quaternion_xyzw)
            candidates.append({
                "candidate_id": candidate.candidate_id,
                "position_planning_xyz": list(position),
                "quaternion_xyzw_planning": list(orientation),
                "score": candidate.score,
                "required_width_m": candidate.required_width_m,
            })
        payload = {
            "planning_request": True,
            "perception_track_id": self.observation.perception_track_id,
            "task_target": self.observation.task_target,
            "planning_frame": self.planning_frame,
            "camera_frame": self.camera_frame,
            "robot_state": {
                "joint_names": list(self.joint_names),
                "joint_positions": list(self.joint_positions),
            },
            "candidates": candidates,
            "collision_scene": {
                "target_aabb": self.target_aabb_planning.to_dict(),
                "static_obstacles": [box.to_dict() for box in self.static_obstacles_planning],
            },
            "oracle_input": False,
            "object_memory_used": False,
            "physical_verification": False,
        }
        _assert_no_simulator_identity(payload)
        return payload
