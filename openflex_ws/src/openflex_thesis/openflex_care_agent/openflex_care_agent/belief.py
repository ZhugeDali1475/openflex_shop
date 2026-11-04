"""Structured belief state for shelf pick-and-place."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from math import isfinite
from typing import Optional, Tuple

from .failure import FailureRecord


class TaskPhase(str, Enum):
    INIT = "init"
    PARSE_TASK = "parse_task"
    NAVIGATE_OBSERVE = "navigate_observe"
    ACQUIRE_TARGET = "acquire_target"
    EVALUATE_CAPABILITY = "evaluate_capability"
    ALIGN_TO_READY = "align_to_ready"
    GRASP = "grasp"
    VERIFY_GRASP = "verify_grasp"
    NAVIGATE_DELIVERY = "navigate_delivery"
    PLACE = "place"
    VERIFY_PLACE = "verify_place"
    RECOVERY = "recovery"
    DONE = "done"
    HOLD = "hold"
    FAILED = "failed"


@dataclass(frozen=True)
class TaskSpec:
    instruction: str
    object_id: str
    total_time_budget_sec: float

    def __post_init__(self) -> None:
        if not self.instruction.strip():
            raise ValueError("instruction must not be empty")
        if not self.object_id.strip():
            raise ValueError("object_id must not be empty")
        if not isfinite(self.total_time_budget_sec) or self.total_time_budget_sec <= 0.0:
            raise ValueError("total_time_budget_sec must be finite and positive")


@dataclass(frozen=True)
class ObjectBelief:
    object_id: str
    visible: bool
    grasp_authorized: bool
    confidence: float
    map_version: int
    age_sec: float
    position_m: Optional[Tuple[float, float, float]] = None
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if not self.object_id.strip():
            raise ValueError("object_id must not be empty")
        if not isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be within [0, 1]")
        if self.map_version < 0:
            raise ValueError("map_version must be non-negative")
        if not isfinite(self.age_sec) or self.age_sec < 0.0:
            raise ValueError("age_sec must be finite and non-negative")
        if self.position_m is not None:
            if len(self.position_m) != 3 or not all(
                isfinite(value) for value in self.position_m
            ):
                raise ValueError("position_m must contain three finite values")
        if not isfinite(self.yaw_rad):
            raise ValueError("yaw_rad must be finite")


@dataclass(frozen=True)
class RobotContext:
    camera_fresh: bool
    odometry_fresh: bool
    joint_state_fresh: bool
    localization_valid: bool
    base_stopped: bool
    base_available: bool
    manipulation_available: bool
    perception_ready: bool
    moveit_ready: bool
    remaining_time_sec: float
    map_version: int

    def __post_init__(self) -> None:
        if not isfinite(self.remaining_time_sec) or self.remaining_time_sec < 0.0:
            raise ValueError("remaining_time_sec must be finite and non-negative")
        if self.map_version < 0:
            raise ValueError("map_version must be non-negative")


@dataclass(frozen=True)
class BeliefState:
    task: TaskSpec
    phase: TaskPhase
    target: ObjectBelief
    robot: RobotContext
    capability_valid: bool = False
    ready_region_valid: bool = False
    object_attached: bool = False
    delivery_target_valid: bool = False
    failure_history: Tuple[FailureRecord, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.target.object_id != self.task.object_id:
            raise ValueError("target object does not match the task object")

    @property
    def map_consistent(self) -> bool:
        return self.target.map_version == self.robot.map_version

    @property
    def latest_failure(self) -> Optional[FailureRecord]:
        return self.failure_history[-1] if self.failure_history else None

    def with_failure(self, record: FailureRecord) -> "BeliefState":
        return replace(
            self,
            phase=TaskPhase.RECOVERY,
            failure_history=(*self.failure_history, record),
            capability_valid=False,
            ready_region_valid=False,
        )

    def with_phase(self, phase: TaskPhase) -> "BeliefState":
        return replace(self, phase=phase)

    def attempts_for(self, failure_code) -> int:
        return sum(
            record.failure_code == failure_code for record in self.failure_history
        )
