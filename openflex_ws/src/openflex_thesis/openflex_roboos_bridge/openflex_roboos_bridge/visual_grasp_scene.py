"""ROS-free binding of the static shelf scene and head-camera TF.

The MuJoCo bridge publishes ``map -> thesis_head_color_optical_frame`` live from
the simulated body pose, and the MoveIt evaluator loads shelf collision geometry
from a YAML scene whose ``frame_id`` must equal the planning frame.  This module
mirrors both contracts without ROS: it parses the shelf YAML into conservative
AABBs, checks frame agreement, and produces a :class:`VisualGraspPlanRequest`
from a measured camera TF reading.

It never reads ``ObjectMemory`` and never executes motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence, Tuple, Union

import yaml

from .visual_grasp_input import VisualGraspPointCloudInput
from .visual_grasp_planning import (
    Aabb,
    FrameTransform,
    GraspCandidatePose,
    VisualGraspPlanRequest,
    VisualGraspPlanningError,
    transform_aabb,
)


class ShelfSceneError(VisualGraspPlanningError):
    """Raised when the static shelf scene or camera TF cannot bind safely."""


@dataclass(frozen=True)
class ShelfScene:
    """Static shelf collision geometry expressed in a single frame."""

    frame_id: str
    calibrated: bool
    object_ids: Tuple[str, ...]
    obstacles: Tuple[Aabb, ...]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "frame_id": self.frame_id,
            "calibrated": bool(self.calibrated),
            "collision_object_ids": list(self.object_ids),
            "collision_object_count": len(self.obstacles),
            "calibration_note": "digital scene only; not physical shelf calibration",
        }


def _finite_vector(values, length: int, name: str, *, positive: bool = False
                   ) -> Tuple[float, ...]:
    if values is None:
        raise ShelfSceneError(f"{name} is required")
    sequence = tuple(values)
    if len(sequence) != length:
        raise ShelfSceneError(f"{name} must have {length} components")
    result = tuple(float(value) for value in sequence)
    if not all(math.isfinite(value) for value in result):
        raise ShelfSceneError(f"{name} must be finite")
    if positive and any(value <= 0 for value in result):
        raise ShelfSceneError(f"{name} must be positive")
    return result


def _box_aabb(position, dimensions, orientation_xyzw, name: str) -> Aabb:
    half = tuple(value / 2.0 for value in dimensions)
    local = Aabb(tuple(-value for value in half), half)
    try:
        transform = FrameTransform(tuple(position), tuple(orientation_xyzw))
        return transform_aabb(local, transform)
    except VisualGraspPlanningError as error:
        raise ShelfSceneError(f"{name}: {error}") from error


def load_shelf_scene(source: Union[str, Path, Mapping]) -> ShelfScene:
    """Parse a shelf-scene YAML (path or mapping) into conservative AABBs."""

    if isinstance(source, (str, Path)):
        path = Path(source)
        if not path.is_file():
            raise ShelfSceneError(f"shelf scene file not found: {path}")
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
    elif isinstance(source, Mapping):
        document = source
    else:
        raise ShelfSceneError("shelf scene source must be a path or mapping")
    if not isinstance(document, Mapping):
        raise ShelfSceneError("shelf scene must be a mapping")
    frame_id = str(document.get("frame_id", "")).strip()
    if not frame_id:
        raise ShelfSceneError("shelf scene must declare a frame_id")
    calibrated = bool(document.get("calibrated", False))
    specifications = document.get("collision_objects") or []
    if not specifications:
        raise ShelfSceneError("shelf scene contains no collision objects")
    identifiers = []
    obstacles = []
    for specification in specifications:
        if not isinstance(specification, Mapping):
            raise ShelfSceneError("each shelf collision object must be a mapping")
        identifier = str(specification.get("id", "")).strip()
        if not identifier:
            raise ShelfSceneError("shelf collision object requires an id")
        if specification.get("shape") != "box":
            raise ShelfSceneError("only box shelf collision objects are supported")
        name = f"shelf object '{identifier}'"
        dimensions = _finite_vector(
            specification.get("dimensions"), 3, f"{name}: dimensions", positive=True)
        position = _finite_vector(specification.get("position"), 3, f"{name}: position")
        orientation = _finite_vector(
            specification.get("orientation_xyzw"), 4, f"{name}: orientation")
        obstacles.append(_box_aabb(position, dimensions, orientation, name))
        identifiers.append(identifier)
    if len(set(identifiers)) != len(identifiers):
        raise ShelfSceneError("shelf collision object ids must be unique")
    return ShelfScene(frame_id=frame_id, calibrated=calibrated,
                      object_ids=tuple(identifiers), obstacles=tuple(obstacles))


def frame_transform_from_tf(*, planning_frame: str, camera_frame: str,
                            parent_frame: str, child_frame: str,
                            translation_xyz: Sequence[float],
                            quaternion_xyzw: Sequence[float],
                            ) -> FrameTransform:
    """Build the camera->planning transform from a measured TF reading.

    The TF must be exactly ``planning_frame -> camera_frame`` (matching the
    MuJoCo bridge's ``map -> thesis_head_color_optical_frame`` broadcast).
    """

    if str(parent_frame).strip() != str(planning_frame).strip():
        raise ShelfSceneError("camera TF parent_frame must be the planning frame")
    if str(child_frame).strip() != str(camera_frame).strip():
        raise ShelfSceneError("camera TF child_frame must be the observation camera frame")
    try:
        return FrameTransform(tuple(translation_xyz), tuple(quaternion_xyzw))
    except VisualGraspPlanningError as error:
        raise ShelfSceneError(str(error)) from error


def build_visual_grasp_plan_request(
        visual_input: VisualGraspPointCloudInput, *,
        planning_frame: str, shelf_scene: Union[ShelfScene, str, Path, Mapping],
        tf_parent_frame: str, tf_child_frame: str,
        tf_translation_xyz: Sequence[float], tf_quaternion_xyzw: Sequence[float],
        candidates: Sequence[GraspCandidatePose], joint_names: Sequence[str],
        joint_positions: Sequence[float], target_padding_m: float = 0.0,
) -> VisualGraspPlanRequest:
    """Combine a visual capture, measured TF and shelf scene into a plan request."""

    if not isinstance(visual_input, VisualGraspPointCloudInput):
        raise ShelfSceneError("planning requires a VisualGraspPointCloudInput, not object_id")
    scene = shelf_scene if isinstance(shelf_scene, ShelfScene) else load_shelf_scene(shelf_scene)
    frame = str(planning_frame).strip()
    if scene.frame_id != frame:
        raise ShelfSceneError(
            f"shelf scene frame '{scene.frame_id}' must match planning frame '{frame}'")
    if frame != visual_input.observation.map_frame:
        raise ShelfSceneError("planning frame must match the observation map_frame")
    transform = frame_transform_from_tf(
        planning_frame=frame, camera_frame=visual_input.frame_id,
        parent_frame=tf_parent_frame, child_frame=tf_child_frame,
        translation_xyz=tf_translation_xyz, quaternion_xyzw=tf_quaternion_xyzw)
    try:
        return VisualGraspPlanRequest.from_visual_input(
            visual_input, planning_frame=frame, camera_to_planning=transform,
            static_obstacles_planning=scene.obstacles, candidates=tuple(candidates),
            joint_names=tuple(joint_names), joint_positions=tuple(joint_positions),
            target_padding_m=target_padding_m)
    except VisualGraspPlanningError as error:
        raise ShelfSceneError(str(error)) from error
