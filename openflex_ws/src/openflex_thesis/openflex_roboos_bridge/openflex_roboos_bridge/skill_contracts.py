"""Pure-Python contracts for the two-level OpenFlex Skill design.

This module intentionally has no ROS, MuJoCo, MCP or network dependency.  It
defines the data/transition boundary that those adapters must obey later.
"""

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Any, Mapping, Optional, Tuple


class SkillLayer(str, Enum):
    ATOMIC = "atomic"
    COMPOSITE = "composite"


class SkillStatus(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    UNKNOWN = "unknown"


class CancellationSemantics(str, Enum):
    READ_ONLY = "read_only"
    BOUNDED_NO_MOTION = "bounded_no_motion"
    CANCEL_AND_CONFIRM_STOP = "cancel_and_confirm_stop"
    STOP_CONFIRMATION_OPERATION = "stop_confirmation_operation"


TERMINAL_STATUSES = frozenset({
    SkillStatus.SUCCEEDED,
    SkillStatus.FAILED,
    SkillStatus.CANCELLED,
    SkillStatus.UNKNOWN,
})

MOTION_SKILLS = frozenset({
    "navigate", "align_base", "execute_grasp", "navigate_with_payload",
    "execute_place", "retreat",
})


def _commands_motion(name: str) -> bool:
    """True when a composite step commands motion, directly or via a nested composite."""
    return name in MOTION_SKILLS or any(skill.name == name for skill in COMPOSITE_SKILLS)


_ALLOWED_TRANSITIONS = {
    None: frozenset({SkillStatus.ACCEPTED, SkillStatus.UNKNOWN}),
    SkillStatus.ACCEPTED: frozenset({SkillStatus.RUNNING, SkillStatus.FAILED,
                                     SkillStatus.CANCELLED, SkillStatus.UNKNOWN}),
    SkillStatus.RUNNING: frozenset({SkillStatus.SUCCEEDED, SkillStatus.FAILED,
                                    SkillStatus.CANCELLED, SkillStatus.UNKNOWN}),
    SkillStatus.SUCCEEDED: frozenset(),
    SkillStatus.FAILED: frozenset(),
    SkillStatus.CANCELLED: frozenset(),
    SkillStatus.UNKNOWN: frozenset(),
}


def validate_status_transition(previous: Optional[SkillStatus], current: SkillStatus) -> None:
    """Raise ``ValueError`` if a persisted skill status jumps illegally."""

    try:
        previous = None if previous is None else SkillStatus(previous)
        current = SkillStatus(current)
    except ValueError as error:
        raise ValueError("unknown skill status") from error
    if current not in _ALLOWED_TRANSITIONS[previous]:
        previous_name = "initial" if previous is None else previous.value
        raise ValueError(f"illegal skill status transition: {previous_name} -> {current.value}")


def _finite_timeout(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("timeout_sec must be a JSON number")
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout_sec must be finite and positive")
    return timeout


def _optional_id(value: Any, field_name: str) -> Optional[str]:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string when supplied")
    return value


@dataclass(frozen=True)
class SkillRequest:
    """Validated request envelope shared by atomic and composite skills."""

    request_id: str
    skill_name: str
    arguments: Mapping[str, Any] = field(default_factory=dict)
    timeout_sec: float = 600.0
    object_id: Optional[str] = None
    station_id: Optional[str] = None
    affordance_id: Optional[str] = None
    control_mode: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.request_id, str) or not self.request_id.strip():
            raise ValueError("request_id must be a non-empty string")
        if not isinstance(self.skill_name, str) or not self.skill_name.strip():
            raise ValueError("skill_name must be a non-empty string")
        if not isinstance(self.arguments, Mapping):
            raise ValueError("arguments must be a mapping")
        object.__setattr__(self, "timeout_sec", _finite_timeout(self.timeout_sec))
        object.__setattr__(self, "object_id", _optional_id(self.object_id, "object_id"))
        object.__setattr__(self, "station_id", _optional_id(self.station_id, "station_id"))
        object.__setattr__(self, "affordance_id", _optional_id(self.affordance_id, "affordance_id"))
        object.__setattr__(self, "control_mode", _optional_id(self.control_mode, "control_mode"))


@dataclass(frozen=True)
class SkillResult:
    """Evidence-bearing result envelope; ``succeeded`` is never inferred."""

    request_id: str
    skill_name: str
    status: SkillStatus
    success: Optional[bool] = None
    source: str = ""
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    evidence_refs: Tuple[str, ...] = ()
    child_request_ids: Tuple[str, ...] = ()
    facts: Mapping[str, Any] = field(default_factory=dict)
    error_code: Optional[str] = None
    message: str = ""
    safety_verified: bool = False
    freshness_verified: bool = False
    collision_verified: Optional[bool] = None
    stopped_confirmed: Optional[bool] = None
    independent_verification_ref: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.request_id, str) or not self.request_id.strip():
            raise ValueError("request_id must be a non-empty string")
        if not isinstance(self.skill_name, str) or not self.skill_name.strip():
            raise ValueError("skill_name must be a non-empty string")
        if not isinstance(self.facts, Mapping):
            raise ValueError("facts must be a mapping")
        status = SkillStatus(self.status)
        object.__setattr__(self, "status", status)
        if self.success is True and status is not SkillStatus.SUCCEEDED:
            raise ValueError("success=true requires status=succeeded")
        if status is SkillStatus.SUCCEEDED and self.success is not True:
            raise ValueError("succeeded requires explicit success=true")
        if status in TERMINAL_STATUSES and (not isinstance(self.source, str) or not self.source.strip()):
            raise ValueError("terminal result requires a non-empty source")
        for name, value in (("started_at", self.started_at), ("completed_at", self.completed_at)):
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                      or not math.isfinite(float(value))):
                raise ValueError(f"{name} must be finite when supplied")
        if self.completed_at is not None and self.started_at is not None:
            if float(self.completed_at) < float(self.started_at):
                raise ValueError("completed_at cannot precede started_at")
        if not all(isinstance(item, str) and item for item in self.evidence_refs):
            raise ValueError("evidence_refs must contain non-empty strings")
        if not all(isinstance(item, str) and item for item in self.child_request_ids):
            raise ValueError("child_request_ids must contain non-empty strings")
        if self.independent_verification_ref is not None and (
                not isinstance(self.independent_verification_ref, str)
                or not self.independent_verification_ref.strip()):
            raise ValueError("independent_verification_ref must be a non-empty string")

    @property
    def terminal(self) -> bool:
        return self.status in TERMINAL_STATUSES

    @property
    def completion_verified(self) -> bool:
        motion_stopped = self.skill_name not in MOTION_SKILLS or self.stopped_confirmed is True
        return (self.status is SkillStatus.SUCCEEDED and self.success is True
                and self.safety_verified and self.freshness_verified
                and bool(self.evidence_refs) and bool(self.independent_verification_ref)
                and motion_stopped)


@dataclass(frozen=True)
class AtomicSkillSpec:
    name: str
    version: str
    required_arguments: Tuple[str, ...] = ()
    allowed_values: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)
    preconditions: Tuple[str, ...] = ()
    postconditions: Tuple[str, ...] = ()
    allowed_control_modes: Tuple[str, ...] = ()
    cancellation_semantics: CancellationSemantics = CancellationSemantics.BOUNDED_NO_MOTION
    min_timeout_sec: float = 0.1
    max_timeout_sec: float = 600.0
    destructive: bool = False
    exposed_to_model: bool = False

    layer: SkillLayer = field(default=SkillLayer.ATOMIC, init=False)

    def validate(self, request: SkillRequest) -> None:
        if request.skill_name != self.name:
            raise ValueError(f"request skill mismatch: expected {self.name}")
        missing = [key for key in self.required_arguments
                   if key not in request.arguments and getattr(request, key, None) is None]
        if missing:
            raise ValueError(f"missing required skill arguments: {', '.join(missing)}")
        timeout = _finite_timeout(request.timeout_sec)
        if not self.min_timeout_sec <= timeout <= self.max_timeout_sec:
            raise ValueError("timeout_sec is outside the skill's registered bounds")
        if self.allowed_control_modes and request.control_mode not in self.allowed_control_modes:
            raise ValueError("control_mode is not allowed for this skill")
        for key, allowed in self.allowed_values.items():
            value = request.arguments.get(key, getattr(request, key, None))
            if value is not None and value not in allowed:
                raise ValueError(f"{key} is not an allowed registered value")


@dataclass(frozen=True)
class CompositeStep:
    skill_name: str
    depends_on: Tuple[str, ...] = ()
    requires_facts: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CompositeSkillSpec:
    name: str
    version: str
    steps: Tuple[CompositeStep, ...]
    max_steps: int
    total_timeout_sec: float = 1200.0
    max_observations: int = 3
    max_replans: int = 2
    max_motion_steps: int = 4
    stop_on_non_success: bool = True
    exposed_to_model: bool = True

    layer: SkillLayer = field(default=SkillLayer.COMPOSITE, init=False)

    def validate(self) -> None:
        if not self.name or not self.version:
            raise ValueError("composite skill name/version are required")
        if not isinstance(self.max_steps, int) or self.max_steps <= 0:
            raise ValueError("max_steps must be a positive integer")
        _finite_timeout(self.total_timeout_sec)
        if self.max_observations < 0 or self.max_replans < 0:
            raise ValueError("observation and replan budgets cannot be negative")
        if not isinstance(self.max_motion_steps, int) or isinstance(self.max_motion_steps, bool) \
                or self.max_motion_steps < 0:
            raise ValueError("motion budget must be a non-negative integer")
        if self.stop_on_non_success is not True:
            raise ValueError("composite skills must stop on failed/cancelled/unknown child results")
        if not self.steps or len(self.steps) > self.max_steps:
            raise ValueError("composite steps must be non-empty and within max_steps")
        names = [step.skill_name for step in self.steps]
        if len(set(names)) != len(names):
            raise ValueError("composite step names must be unique")
        known = set(names)
        for step in self.steps:
            if step.skill_name in {"reset_scene", "reset_simulation", "teleport_object"}:
                raise ValueError("scene reset/teleport is not an allowed composite task step")
            if step.skill_name in step.depends_on:
                raise ValueError("a composite step cannot depend on itself")
            unknown = set(step.depends_on) - known
            if unknown:
                raise ValueError(f"composite step depends on unknown steps: {sorted(unknown)}")
        # A small DAG check prevents an accidental cyclic execution plan.
        visiting, visited = set(), set()

        def visit(name: str) -> None:
            if name in visiting:
                raise ValueError("composite skill steps contain a cycle")
            if name in visited:
                return
            visiting.add(name)
            step = next(item for item in self.steps if item.skill_name == name)
            for dependency in step.depends_on:
                visit(dependency)
            visiting.remove(name)
            visited.add(name)

        for name in names:
            visit(name)


class CompositeExecution:
    """Deterministic, single-flight executor state for a composite Skill.

    This is deliberately an in-memory pure-Python state machine.  It does not
    invoke ROS or reset MuJoCo.  A future adapter must persist the snapshot and
    make the physical reset/stop operation produce the referenced evidence.
    """

    def __init__(self, spec: CompositeSkillSpec, request_id: str, started_at: Optional[float] = None):
        spec.validate()
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("composite request_id must be a non-empty string")
        self.spec = spec
        self.request_id = request_id
        self.started_at = time.monotonic() if started_at is None else float(started_at)
        if not math.isfinite(self.started_at):
            raise ValueError("started_at must be finite")
        self.step_status = {step.skill_name: None for step in spec.steps}
        self.step_results = {}
        self.active_step: Optional[str] = None
        self.halted_reason: Optional[str] = None
        self.requires_stop_confirmation = False
        self.requires_reset = False
        self.observation_count = 0
        self.replan_count = 0
        self.motion_count = 0
        self.reset_count = 0
        self.stop_confirmation_ref: Optional[str] = None
        self.reset_evidence_refs = []

    @property
    def terminal(self) -> bool:
        return self.requires_reset or self.halted_reason is not None

    @property
    def succeeded(self) -> bool:
        return (not self.halted_reason and self.requires_reset
                and all(status is SkillStatus.SUCCEEDED for status in self.step_status.values()))

    def _step(self, name: str) -> CompositeStep:
        for step in self.spec.steps:
            if step.skill_name == name:
                return step
        raise ValueError(f"unknown composite step: {name}")

    def _check_deadline(self, now: Optional[float]) -> None:
        current = time.monotonic() if now is None else float(now)
        if not math.isfinite(current):
            raise ValueError("current time must be finite")
        if not self.terminal and current - self.started_at > self.spec.total_timeout_sec:
            active_motion = self.active_step in MOTION_SKILLS
            self._halt("composite total timeout exceeded", requires_stop=active_motion)

    def next_step(self, now: Optional[float] = None) -> Optional[str]:
        """Return the one legal next step; never exposes parallel motion."""

        self._check_deadline(now)
        if self.terminal or self.active_step is not None:
            return None
        ready = []
        for step in self.spec.steps:
            if self.step_status[step.skill_name] is not None:
                continue
            if all(self.step_status[dependency] is SkillStatus.SUCCEEDED
                   for dependency in step.depends_on):
                facts_ok = all(
                    self.step_results[dependency].facts.get(key) == expected
                    for dependency in step.depends_on
                    for key, expected in step.requires_facts.items()
                    if dependency in self.step_results
                )
                if step.requires_facts and not facts_ok:
                    self._halt(f"required condition not satisfied before {step.skill_name}",
                               requires_stop=False)
                    return None
                ready.append(step.skill_name)
        if len(ready) > 1:
            raise RuntimeError("composite plan has parallel-ready steps; explicit serialization is required")
        return ready[0] if ready else None

    def start_step(self, name: str, now: Optional[float] = None) -> None:
        self._check_deadline(now)
        if self.terminal:
            raise RuntimeError("cannot start a step after composite termination")
        if self.active_step is not None:
            raise RuntimeError(f"step already active: {self.active_step}")
        if self.next_step(now) != name:
            raise ValueError(f"step is not ready: {name}")
        if _commands_motion(name):
            self.motion_count += 1
            if self.motion_count > self.spec.max_motion_steps:
                self._halt("motion budget exhausted")
                return
        self.active_step = name

    def record_result(self, name: str, result: SkillResult, now: Optional[float] = None) -> None:
        self._check_deadline(now)
        if self.terminal:
            raise RuntimeError("cannot record a child result after composite termination")
        if self.active_step != name:
            raise ValueError(f"result does not match active step: {self.active_step!r} != {name!r}")
        if result.skill_name != name:
            raise ValueError("child result skill_name does not match composite step")
        if name == "observe" and result.status is SkillStatus.SUCCEEDED:
            self.observation_count += 1
            if self.observation_count > self.spec.max_observations:
                self._halt("observation budget exhausted")
                return
        if result.status not in TERMINAL_STATUSES:
            self.step_status[name] = result.status
            return
        self.active_step = None
        self.step_status[name] = result.status
        self.step_results[name] = result
        if result.status is not SkillStatus.SUCCEEDED or result.success is not True:
            self._halt(f"child step {name} ended {result.status.value}",
                       requires_stop=result.status in {SkillStatus.CANCELLED, SkillStatus.UNKNOWN}
                       or name in MOTION_SKILLS)
            return
        if all(status is SkillStatus.SUCCEEDED for status in self.step_status.values()):
            self.requires_reset = True

    def request_replan(self, reason: str, now: Optional[float] = None) -> None:
        self._check_deadline(now)
        if self.active_step is not None:
            raise RuntimeError("cannot replan while a child step is active")
        if self.terminal:
            raise RuntimeError("cannot replan a terminal composite execution")
        self.replan_count += 1
        if self.replan_count > self.spec.max_replans:
            self._halt("replan budget exhausted")
        elif not isinstance(reason, str) or not reason.strip():
            raise ValueError("replan reason must be recorded")

    def _halt(self, reason: str, requires_stop: bool = True) -> None:
        self.active_step = None
        self.halted_reason = reason
        self.requires_stop_confirmation = requires_stop

    def confirm_stop(self, evidence_ref: str) -> None:
        if not self.requires_stop_confirmation:
            raise ValueError("this composite execution does not require stop confirmation")
        if not isinstance(evidence_ref, str) or not evidence_ref.strip():
            raise ValueError("stop confirmation requires an evidence reference")
        self.requires_stop_confirmation = False
        self.stop_confirmation_ref = evidence_ref

    def reset(self, evidence_ref: str) -> None:
        """Allow a new trial only after an explicit externally verified reset."""

        if self.active_step is not None:
            raise RuntimeError("cannot reset while a child step is active")
        if not self.terminal:
            raise RuntimeError("reset is only allowed after a terminal composite result")
        if self.requires_stop_confirmation:
            raise RuntimeError("confirm stop before resetting the composite")
        if not isinstance(evidence_ref, str) or not evidence_ref.strip():
            raise ValueError("reset requires an evidence reference")
        self.reset_evidence_refs.append(evidence_ref)
        self.step_status = {step.skill_name: None for step in self.spec.steps}
        self.step_results = {}
        self.halted_reason = None
        self.requires_reset = False
        self.observation_count = 0
        self.replan_count = 0
        self.motion_count = 0
        self.reset_count += 1

    def snapshot(self) -> dict:
        return {
            "request_id": self.request_id,
            "skill_name": self.spec.name,
            "started_at": self.started_at,
            "step_status": {name: (status.value if status is not None else None)
                             for name, status in self.step_status.items()},
            "active_step": self.active_step,
            "halted_reason": self.halted_reason,
            "requires_stop_confirmation": self.requires_stop_confirmation,
            "requires_reset": self.requires_reset,
            "observation_count": self.observation_count,
            "replan_count": self.replan_count,
            "motion_count": self.motion_count,
            "budgets": {
                "max_observations": self.spec.max_observations,
                "max_replans": self.spec.max_replans,
                "max_motion_steps": self.spec.max_motion_steps,
                "total_timeout_sec": self.spec.total_timeout_sec,
            },
            "reset_count": self.reset_count,
            "stop_confirmation_ref": self.stop_confirmation_ref,
            "reset_evidence_refs": list(self.reset_evidence_refs),
        }


ATOMIC_SKILLS = (
    AtomicSkillSpec("observe", "1.0", preconditions=("camera stream is fresh",),
                    postconditions=("observation source and timestamp are recorded",),
                    cancellation_semantics=CancellationSemantics.READ_ONLY, exposed_to_model=True),
    AtomicSkillSpec("get_state", "1.0", preconditions=("state sources are available",),
                    postconditions=("state freshness and source are reported",),
                    cancellation_semantics=CancellationSemantics.READ_ONLY, exposed_to_model=True),
    AtomicSkillSpec("resolve_target", "1.0", required_arguments=("object_id",),
                    preconditions=("latest observation is fresh",),
                    postconditions=("one registered target or explicit ambiguity is returned",),
                    cancellation_semantics=CancellationSemantics.READ_ONLY, exposed_to_model=True),
    AtomicSkillSpec("navigate", "1.0", required_arguments=("station_id",),
                    preconditions=("station is registered", "robot is not in unknown motion state"),
                    postconditions=("measured arrival and stop state are recorded",),
                    allowed_control_modes=("NAV2",),
                    cancellation_semantics=CancellationSemantics.CANCEL_AND_CONFIRM_STOP,
                    destructive=True, exposed_to_model=True),
    AtomicSkillSpec("align_base", "1.0", required_arguments=("station_id",),
                    preconditions=("base alignment target is registered",),
                    allowed_control_modes=("ACT",),
                    cancellation_semantics=CancellationSemantics.CANCEL_AND_CONFIRM_STOP, destructive=True),
    AtomicSkillSpec("plan_grasp", "1.0", required_arguments=("object_id",),
                    preconditions=("target geometry and robot state are current",),
                    cancellation_semantics=CancellationSemantics.BOUNDED_NO_MOTION),
    AtomicSkillSpec("execute_grasp", "1.0", required_arguments=("object_id",),
                    preconditions=("grasp plan and collision checks passed", "robot is at pickup station"),
                    postconditions=("attachment is independently verified or failure is explicit",),
                    allowed_control_modes=("ACT", "MOVEIT"),
                    cancellation_semantics=CancellationSemantics.CANCEL_AND_CONFIRM_STOP,
                    destructive=True, exposed_to_model=True),
    AtomicSkillSpec("verify_attachment", "1.0", required_arguments=("object_id",),
                    preconditions=("grasp action has a terminal result",),
                    postconditions=("attachment state is measured and timestamped",),
                    cancellation_semantics=CancellationSemantics.READ_ONLY, exposed_to_model=True),
    AtomicSkillSpec("navigate_with_payload", "1.0", required_arguments=("object_id", "affordance_id"),
                    preconditions=("attachment is verified", "payload path check passed"),
                    postconditions=("measured destination arrival and stop are confirmed",),
                    allowed_control_modes=("NAV2",),
                    cancellation_semantics=CancellationSemantics.CANCEL_AND_CONFIRM_STOP,
                    destructive=True, exposed_to_model=True),
    AtomicSkillSpec("plan_place", "1.0", required_arguments=("object_id", "affordance_id"),
                    preconditions=("destination affordance is registered", "payload is verified attached"),
                    cancellation_semantics=CancellationSemantics.BOUNDED_NO_MOTION),
    AtomicSkillSpec("execute_place", "1.0", required_arguments=("object_id", "affordance_id"),
                    preconditions=("place plan and collision checks passed", "base is locked"),
                    postconditions=("release, settlement and retreat evidence are returned",),
                    allowed_control_modes=("PLACE",),
                    cancellation_semantics=CancellationSemantics.CANCEL_AND_CONFIRM_STOP,
                    destructive=True, exposed_to_model=True),
    AtomicSkillSpec("verify_release_and_settle", "1.0", required_arguments=("object_id", "affordance_id"),
                    preconditions=("place action has a terminal result",),
                    postconditions=("support contact, detached state and settling are measured",),
                    cancellation_semantics=CancellationSemantics.READ_ONLY, exposed_to_model=True),
    AtomicSkillSpec("retreat", "1.0", required_arguments=("affordance_id",),
                    preconditions=("release is verified", "retreat path is checked"),
                    postconditions=("arm clears the support surface without object contact",),
                    allowed_control_modes=("PLACE",),
                    cancellation_semantics=CancellationSemantics.CANCEL_AND_CONFIRM_STOP, destructive=True),
    AtomicSkillSpec("cancel_and_confirm_stop", "1.0", required_arguments=("request_id",),
                    preconditions=("request_id identifies an active/unknown action",),
                    postconditions=("cancel result and measured stop state are recorded",),
                    cancellation_semantics=CancellationSemantics.STOP_CONFIRMATION_OPERATION,
                    exposed_to_model=True),
)


# Compatibility names only; these mappings do not make a new capability safe
# or available until its adapter and runtime guards are implemented.
MCP_ATOMIC_TOOL_MAP = {
    "observe_scene": "observe",
    "get_robot_state": "get_state",
    "navigate_to_target": "navigate",
    "grasp_object": "execute_grasp",
    "place_to_affordance": "execute_place",
    "cancel_current_task": "cancel_and_confirm_stop",
}


COMPOSITE_SKILLS = (
    CompositeSkillSpec("pick_object", "1.0", (
        CompositeStep("observe"),
        CompositeStep("navigate", ("observe",)),
        CompositeStep("execute_grasp", ("navigate",)),
        CompositeStep("verify_attachment", ("execute_grasp",)),
    ), 6),
    CompositeSkillSpec("deliver_object", "1.0", (
        CompositeStep("navigate_with_payload"),
        CompositeStep("execute_place", ("navigate_with_payload",)),
        CompositeStep("verify_release_and_settle", ("execute_place",)),
        CompositeStep("retreat", ("verify_release_and_settle",)),
    ), 6),
    CompositeSkillSpec("fetch_and_deliver", "1.0", (
        CompositeStep("pick_object"),
        CompositeStep("deliver_object", ("pick_object",)),
        CompositeStep("get_state", ("deliver_object",)),
    ), 5),
    CompositeSkillSpec("search_and_pick", "1.0", (
        CompositeStep("observe"),
        CompositeStep("resolve_target", ("observe",)),
        CompositeStep("pick_object", ("resolve_target",)),
    ), 5),
)


def validate_catalog() -> None:
    """Validate the built-in catalog during tests/build checks."""

    names = [item.name for item in ATOMIC_SKILLS]
    if len(names) != len(set(names)):
        raise ValueError("atomic skill names must be unique")
    for composite in COMPOSITE_SKILLS:
        composite.validate()
