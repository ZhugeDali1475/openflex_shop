"""ROS-free binding of a visual grasp request to the MoveIt planning interface.

The phase-A audit (section 11) requires the visual point-cloud grasp chain to
reach this task's MoveIt planner without borrowing the ``ObjectMemory``
identity.  :mod:`visual_grasp_planning` already produces a truth-free
:class:`~openflex_roboos_bridge.visual_grasp_planning.VisualGraspPlanRequest`;
this module turns that request into the two payloads the MoveIt services need:

* a planning-scene world diff that carries the independent target collision
  volume plus the static shelf obstacles in the planning frame;
* one robot state per visual candidate, holding only measured joint positions.

It is deliberately plan-only: the produced payload pins ``plan_only`` and
``trajectory_execution`` false and refuses a MoveIt parameter set that would
allow execution.  It never reads ``ObjectMemory``, never calls MoveIt itself and
never executes motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Mapping, Optional, Sequence, Tuple

from .visual_grasp_planning import (
    Aabb,
    VisualGraspPlanRequest,
    VisualGraspPlanningError,
)


class MoveItTransportError(VisualGraspPlanningError):
    """Raised when a visual request cannot bind to the MoveIt planning interface."""


_SAFE_IDENTIFIER = re.compile(r"[^0-9A-Za-z_]+")


def _finite_triplet(values, name: str) -> Tuple[float, float, float]:
    sequence = tuple(values)
    if len(sequence) != 3:
        raise MoveItTransportError(f"{name} must have three components")
    result = tuple(float(value) for value in sequence)
    if not all(math.isfinite(value) for value in result):
        raise MoveItTransportError(f"{name} must be finite")
    return result  # type: ignore[return-value]


def collision_object_id(perception_track_id: str) -> str:
    """Return a stable, identity-free planning-scene id for the visual target."""

    sanitized = _SAFE_IDENTIFIER.sub("_", str(perception_track_id).strip()).strip("_")
    if not sanitized:
        raise MoveItTransportError("perception_track_id cannot form a collision id")
    return f"visual_target_{sanitized}"


@dataclass(frozen=True)
class CollisionBox:
    """A world-aligned box ready to become a ``CollisionObject`` primitive."""

    object_id: str
    frame_id: str
    position_xyz: Tuple[float, float, float]
    dimensions_xyz: Tuple[float, float, float]

    def __post_init__(self) -> None:
        identifier = str(self.object_id).strip()
        frame = str(self.frame_id).strip()
        if not identifier or not frame:
            raise MoveItTransportError("collision box requires an id and a frame")
        position = _finite_triplet(self.position_xyz, "collision box position")
        dimensions = _finite_triplet(self.dimensions_xyz, "collision box dimensions")
        if any(size <= 0 for size in dimensions):
            raise MoveItTransportError("collision box dimensions must be positive")
        object.__setattr__(self, "object_id", identifier)
        object.__setattr__(self, "frame_id", frame)
        object.__setattr__(self, "position_xyz", position)
        object.__setattr__(self, "dimensions_xyz", dimensions)

    @classmethod
    def from_aabb(cls, box: Aabb, *, object_id: str, frame_id: str) -> "CollisionBox":
        if not isinstance(box, Aabb):
            raise MoveItTransportError("from_aabb requires an Aabb")
        low, high = box.min_xyz, box.max_xyz
        return cls(
            object_id=object_id,
            frame_id=frame_id,
            position_xyz=tuple((low[axis] + high[axis]) / 2.0 for axis in range(3)),
            dimensions_xyz=tuple(high[axis] - low[axis] for axis in range(3)),
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "id": self.object_id,
            "frame_id": self.frame_id,
            "shape": "box",
            "position_xyz": list(self.position_xyz),
            "dimensions_xyz": list(self.dimensions_xyz),
        }


@dataclass(frozen=True)
class CandidateState:
    """Measured-joint robot state paired with one visual candidate pose."""

    candidate_id: str
    joint_names: Tuple[str, ...]
    joint_positions: Tuple[float, ...]
    position_planning_xyz: Tuple[float, float, float]
    quaternion_xyzw_planning: Tuple[float, float, float, float]

    def __post_init__(self) -> None:
        identifier = str(self.candidate_id).strip()
        if not identifier:
            raise MoveItTransportError("candidate state requires a candidate_id")
        names = tuple(str(name).strip() for name in self.joint_names)
        positions = tuple(float(value) for value in self.joint_positions)
        if not names or len(names) != len(positions):
            raise MoveItTransportError("candidate state joint names and positions must align")
        if not all(math.isfinite(value) for value in positions):
            raise MoveItTransportError("candidate state joint positions must be finite")
        position = _finite_triplet(self.position_planning_xyz, "candidate planning position")
        quaternion = tuple(float(value) for value in self.quaternion_xyzw_planning)
        if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
            raise MoveItTransportError("candidate planning quaternion must be four finite values")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-9 or abs(norm - 1.0) > 0.01:
            raise MoveItTransportError("candidate planning quaternion must be unit length")
        object.__setattr__(self, "candidate_id", identifier)
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "joint_positions", positions)
        object.__setattr__(self, "position_planning_xyz", position)
        object.__setattr__(self, "quaternion_xyzw_planning",
                           tuple(value / norm for value in quaternion))

    def to_dict(self) -> Mapping[str, object]:
        return {
            "candidate_id": self.candidate_id,
            "joint_names": list(self.joint_names),
            "joint_positions": list(self.joint_positions),
            "position_planning_xyz": list(self.position_planning_xyz),
            "quaternion_xyzw_planning": list(self.quaternion_xyzw_planning),
        }


@dataclass(frozen=True)
class VisualGraspMoveItPlan:
    """Plan-only MoveIt request assembled from a visual grasp request."""

    planning_frame: str
    scene_diff: Tuple[CollisionBox, ...]
    candidate_states: Tuple[CandidateState, ...]
    plan_only: bool = True
    trajectory_execution: bool = False

    def __post_init__(self) -> None:
        frame = str(self.planning_frame).strip()
        if not frame:
            raise MoveItTransportError("planning frame is required")
        boxes = tuple(self.scene_diff)
        if not boxes or not all(isinstance(box, CollisionBox) for box in boxes):
            raise MoveItTransportError("the scene diff needs at least the target volume")
        if any(box.frame_id != frame for box in boxes):
            raise MoveItTransportError("every scene box must use the planning frame")
        states = tuple(self.candidate_states)
        if not states or not all(isinstance(state, CandidateState) for state in states):
            raise MoveItTransportError("at least one candidate robot state is required")
        if self.plan_only is not True or self.trajectory_execution is not False:
            raise MoveItTransportError("visual grasp transport is plan-only by contract")
        object.__setattr__(self, "planning_frame", frame)
        object.__setattr__(self, "scene_diff", boxes)
        object.__setattr__(self, "candidate_states", states)

    def to_dict(self) -> Mapping[str, object]:
        return {
            "planning_frame": self.planning_frame,
            "plan_only": True,
            "trajectory_execution": False,
            "robot_motion_executed": False,
            "object_memory_used": False,
            "scene_diff": [box.to_dict() for box in self.scene_diff],
            "candidate_states": [state.to_dict() for state in self.candidate_states],
        }


def assert_plan_only_parameters(parameters: Mapping[str, object]) -> None:
    """Refuse a MoveIt parameter set that would allow trajectory execution."""

    if not isinstance(parameters, Mapping):
        raise MoveItTransportError("move_group parameters must be a mapping")
    if parameters.get("allow_trajectory_execution") not in (False, None, "false", "False"):
        raise MoveItTransportError("MoveIt must run with allow_trajectory_execution=False")
    if parameters.get("moveit_manage_controllers") not in (False, None, "false", "False"):
        raise MoveItTransportError("MoveIt must not manage controllers in a visual planning run")


def build_moveit_plan(request: VisualGraspPlanRequest, *,
                      move_group_parameters: Optional[Mapping[str, object]] = None
                      ) -> VisualGraspMoveItPlan:
    """Convert a plan request into a plan-only MoveIt transport payload."""

    if not isinstance(request, VisualGraspPlanRequest):
        raise MoveItTransportError("a VisualGraspPlanRequest is required")
    if move_group_parameters is not None:
        assert_plan_only_parameters(move_group_parameters)
    frame = request.planning_frame
    boxes = [CollisionBox.from_aabb(
        request.target_aabb_planning,
        object_id=collision_object_id(request.observation.perception_track_id),
        frame_id=frame)]
    boxes.extend(
        CollisionBox.from_aabb(box, object_id=f"visual_shelf_{index:02d}", frame_id=frame)
        for index, box in enumerate(request.static_obstacles_planning))
    payload = request.to_planning_payload()
    states = []
    for candidate in payload["candidates"]:
        states.append(CandidateState(
            candidate_id=candidate["candidate_id"],
            joint_names=request.joint_names,
            joint_positions=request.joint_positions,
            position_planning_xyz=tuple(candidate["position_planning_xyz"]),
            quaternion_xyzw_planning=tuple(candidate["quaternion_xyzw_planning"]),
        ))
    return VisualGraspMoveItPlan(
        planning_frame=frame, scene_diff=tuple(boxes), candidate_states=tuple(states))


def visual_grasp_moveit_payload(request: VisualGraspPlanRequest, *,
                                move_group_parameters: Optional[Mapping[str, object]] = None
                                ) -> Mapping[str, object]:
    """Convenience wrapper returning the serialized plan-only payload."""

    return build_moveit_plan(request, move_group_parameters=move_group_parameters).to_dict()
