"""Typed skill contracts and the static skill graph."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, FrozenSet, Sequence, Tuple


class SkillName(str, Enum):
    LOOK_FOR_OBJECT = "look_for_object"
    NAVIGATE_TO_STATION = "navigate_to_station"
    FINE_ALIGN = "fine_align"
    COMPUTE_CAPABILITY = "compute_capability"
    LOCAL_ALIGN_REGION = "local_align_region"
    PLAN_AND_GRASP = "plan_and_grasp"
    VERIFY_GRASP = "verify_grasp"
    NAVIGATE_TO_DELIVERY = "navigate_to_delivery"
    PLACE_OBJECT = "place_object"
    VERIFY_PLACE = "verify_place"
    RE_OBSERVE = "reobserve"
    CHANGE_VIEW = "change_view"
    RETREAT_FROM_STATION = "retreat_from_station"
    ABORT_HOLD = "abort_hold"


class Precondition(str, Enum):
    CAMERA_FRESH = "camera_fresh"
    ODOMETRY_FRESH = "odometry_fresh"
    JOINT_STATE_FRESH = "joint_state_fresh"
    LOCALIZATION_VALID = "localization_valid"
    BASE_STOPPED = "base_stopped"
    BASE_AVAILABLE = "base_available"
    MANIPULATION_AVAILABLE = "manipulation_available"
    PERCEPTION_READY = "perception_ready"
    MOVEIT_READY = "moveit_ready"
    TIME_AVAILABLE = "time_available"
    MAP_CONSISTENT = "map_consistent"
    TARGET_VISIBLE = "target_visible"
    GRASP_AUTHORIZED = "grasp_authorized"
    CAPABILITY_VALID = "capability_valid"
    READY_REGION_BOUND = "ready_region_bound"
    OBJECT_ATTACHED = "object_attached"
    DELIVERY_TARGET_VALID = "delivery_target_valid"


class Postcondition(str, Enum):
    TARGET_FOUND = "target_found"
    STATION_REACHED = "station_reached"
    BASE_STOPPED = "base_stopped"
    CAPABILITY_COMPUTED = "capability_computed"
    READY_REGION_ENTERED = "ready_region_entered"
    GRASP_ATTACHED = "grasp_attached"
    GRASP_VERIFIED = "grasp_verified"
    DELIVERY_REACHED = "delivery_reached"
    OBJECT_RELEASED = "object_released"
    PLACE_VERIFIED = "place_verified"
    NEW_OBSERVATION = "new_observation"
    BASE_HELD = "base_held"


class ControlOwner(str, Enum):
    NONE = "none"
    BASE = "base"
    MANIPULATION = "manipulation"


@dataclass(frozen=True)
class SkillContract:
    name: SkillName
    preconditions: FrozenSet[Precondition]
    postconditions: FrozenSet[Postcondition]
    owner: ControlOwner
    timeout_sec: float
    description: str

    def __post_init__(self) -> None:
        if self.timeout_sec <= 0.0:
            raise ValueError(f"{self.name.value}: timeout_sec must be positive")
        if not self.description.strip():
            raise ValueError(f"{self.name.value}: description must not be empty")


def _contract(
    name: SkillName,
    preconditions: Sequence[Precondition],
    postconditions: Sequence[Postcondition],
    owner: ControlOwner,
    timeout_sec: float,
    description: str,
) -> SkillContract:
    return SkillContract(
        name=name,
        preconditions=frozenset(preconditions),
        postconditions=frozenset(postconditions),
        owner=owner,
        timeout_sec=timeout_sec,
        description=description,
    )


_CAMERA = (Precondition.CAMERA_FRESH,)
_NAVIGATION = (
    Precondition.ODOMETRY_FRESH,
    Precondition.LOCALIZATION_VALID,
    Precondition.BASE_AVAILABLE,
    Precondition.TIME_AVAILABLE,
)
_MANIPULATION = (
    Precondition.BASE_STOPPED,
    Precondition.MANIPULATION_AVAILABLE,
    Precondition.MOVEIT_READY,
    Precondition.JOINT_STATE_FRESH,
    Precondition.TIME_AVAILABLE,
)

CONTRACTS: Dict[SkillName, SkillContract] = {
    SkillName.LOOK_FOR_OBJECT: _contract(
        SkillName.LOOK_FOR_OBJECT,
        [*_CAMERA, Precondition.PERCEPTION_READY, Precondition.TIME_AVAILABLE],
        [Postcondition.TARGET_FOUND],
        ControlOwner.NONE,
        10.0,
        "Acquire a fresh target observation from the current viewpoint.",
    ),
    SkillName.NAVIGATE_TO_STATION: _contract(
        SkillName.NAVIGATE_TO_STATION,
        _NAVIGATION,
        [Postcondition.STATION_REACHED, Postcondition.BASE_STOPPED],
        ControlOwner.BASE,
        60.0,
        "Navigate to a registered shelf or delivery station.",
    ),
    SkillName.FINE_ALIGN: _contract(
        SkillName.FINE_ALIGN,
        [
            *_NAVIGATION,
            Precondition.CAMERA_FRESH,
            Precondition.ODOMETRY_FRESH,
        ],
        [Postcondition.BASE_STOPPED],
        ControlOwner.BASE,
        30.0,
        "Run guarded local docking or shelf alignment.",
    ),
    SkillName.COMPUTE_CAPABILITY: _contract(
        SkillName.COMPUTE_CAPABILITY,
        [
            Precondition.PERCEPTION_READY,
            Precondition.TARGET_VISIBLE,
            Precondition.MAP_CONSISTENT,
            Precondition.TIME_AVAILABLE,
        ],
        [Postcondition.CAPABILITY_COMPUTED],
        ControlOwner.NONE,
        120.0,
        "Evaluate joint grasp, base and lift-arm capability.",
    ),
    SkillName.LOCAL_ALIGN_REGION: _contract(
        SkillName.LOCAL_ALIGN_REGION,
        [
            *_NAVIGATION,
            Precondition.TARGET_VISIBLE,
            Precondition.CAPABILITY_VALID,
            Precondition.READY_REGION_BOUND,
            Precondition.MAP_CONSISTENT,
        ],
        [Postcondition.READY_REGION_ENTERED, Postcondition.BASE_STOPPED],
        ControlOwner.BASE,
        30.0,
        "Enter the selected grasp-conditioned ready region.",
    ),
    SkillName.PLAN_AND_GRASP: _contract(
        SkillName.PLAN_AND_GRASP,
        [
            *_MANIPULATION,
            Precondition.TARGET_VISIBLE,
            Precondition.GRASP_AUTHORIZED,
            Precondition.CAPABILITY_VALID,
            Precondition.MAP_CONSISTENT,
        ],
        [Postcondition.GRASP_ATTACHED],
        ControlOwner.MANIPULATION,
        45.0,
        "Plan and execute the lift-arm grasp under exclusive ownership.",
    ),
    SkillName.VERIFY_GRASP: _contract(
        SkillName.VERIFY_GRASP,
        [
            Precondition.CAMERA_FRESH,
            Precondition.JOINT_STATE_FRESH,
            Precondition.TIME_AVAILABLE,
        ],
        [Postcondition.GRASP_VERIFIED],
        ControlOwner.NONE,
        10.0,
        "Verify attachment and the transport posture.",
    ),
    SkillName.NAVIGATE_TO_DELIVERY: _contract(
        SkillName.NAVIGATE_TO_DELIVERY,
        [
            *_NAVIGATION,
            Precondition.OBJECT_ATTACHED,
            Precondition.DELIVERY_TARGET_VALID,
        ],
        [Postcondition.DELIVERY_REACHED, Postcondition.BASE_STOPPED],
        ControlOwner.BASE,
        60.0,
        "Transport the attached object to the delivery station.",
    ),
    SkillName.PLACE_OBJECT: _contract(
        SkillName.PLACE_OBJECT,
        [
            *_MANIPULATION,
            Precondition.OBJECT_ATTACHED,
            Precondition.DELIVERY_TARGET_VALID,
        ],
        [Postcondition.OBJECT_RELEASED],
        ControlOwner.MANIPULATION,
        30.0,
        "Place and release the object at the validated target.",
    ),
    SkillName.VERIFY_PLACE: _contract(
        SkillName.VERIFY_PLACE,
        [Precondition.CAMERA_FRESH, Precondition.TIME_AVAILABLE],
        [Postcondition.PLACE_VERIFIED],
        ControlOwner.NONE,
        10.0,
        "Verify the final object pose and placement stability.",
    ),
    SkillName.RE_OBSERVE: _contract(
        SkillName.RE_OBSERVE,
        [Precondition.PERCEPTION_READY, Precondition.TIME_AVAILABLE],
        [Postcondition.NEW_OBSERVATION],
        ControlOwner.NONE,
        15.0,
        "Acquire fresh perception evidence without an irreversible action.",
    ),
    SkillName.CHANGE_VIEW: _contract(
        SkillName.CHANGE_VIEW,
        [
            Precondition.CAMERA_FRESH,
            Precondition.BASE_AVAILABLE,
            Precondition.TIME_AVAILABLE,
        ],
        [Postcondition.NEW_OBSERVATION],
        ControlOwner.BASE,
        20.0,
        "Move to a bounded alternative observation pose.",
    ),
    SkillName.RETREAT_FROM_STATION: _contract(
        SkillName.RETREAT_FROM_STATION,
        [
            Precondition.ODOMETRY_FRESH,
            Precondition.LOCALIZATION_VALID,
            Precondition.BASE_AVAILABLE,
            Precondition.TIME_AVAILABLE,
        ],
        [Postcondition.BASE_STOPPED],
        ControlOwner.BASE,
        10.0,
        "Perform the guarded short retreat from a close station.",
    ),
    SkillName.ABORT_HOLD: _contract(
        SkillName.ABORT_HOLD,
        [],
        [Postcondition.BASE_HELD],
        ControlOwner.NONE,
        1.0,
        "Fail closed and keep the base and manipulator stationary.",
    ),
}


SKILL_GRAPH: Dict[SkillName, Tuple[SkillName, ...]] = {
    SkillName.LOOK_FOR_OBJECT: (
        SkillName.COMPUTE_CAPABILITY,
        SkillName.RE_OBSERVE,
        SkillName.CHANGE_VIEW,
    ),
    SkillName.NAVIGATE_TO_STATION: (
        SkillName.FINE_ALIGN,
        SkillName.LOOK_FOR_OBJECT,
        SkillName.ABORT_HOLD,
    ),
    SkillName.FINE_ALIGN: (
        SkillName.LOOK_FOR_OBJECT,
        SkillName.COMPUTE_CAPABILITY,
        SkillName.ABORT_HOLD,
    ),
    SkillName.COMPUTE_CAPABILITY: (
        SkillName.LOCAL_ALIGN_REGION,
        SkillName.RE_OBSERVE,
        SkillName.ABORT_HOLD,
    ),
    SkillName.LOCAL_ALIGN_REGION: (
        SkillName.PLAN_AND_GRASP,
        SkillName.RE_OBSERVE,
        SkillName.ABORT_HOLD,
    ),
    SkillName.PLAN_AND_GRASP: (
        SkillName.VERIFY_GRASP,
        SkillName.RETREAT_FROM_STATION,
        SkillName.COMPUTE_CAPABILITY,
        SkillName.ABORT_HOLD,
    ),
    SkillName.VERIFY_GRASP: (
        SkillName.NAVIGATE_TO_DELIVERY,
        SkillName.RETREAT_FROM_STATION,
        SkillName.ABORT_HOLD,
    ),
    SkillName.NAVIGATE_TO_DELIVERY: (
        SkillName.FINE_ALIGN,
        SkillName.PLACE_OBJECT,
        SkillName.ABORT_HOLD,
    ),
    SkillName.PLACE_OBJECT: (
        SkillName.VERIFY_PLACE,
        SkillName.FINE_ALIGN,
        SkillName.ABORT_HOLD,
    ),
    SkillName.RETREAT_FROM_STATION: (
        SkillName.RE_OBSERVE,
        SkillName.COMPUTE_CAPABILITY,
        SkillName.ABORT_HOLD,
    ),
    SkillName.RE_OBSERVE: (
        SkillName.LOOK_FOR_OBJECT,
        SkillName.COMPUTE_CAPABILITY,
        SkillName.ABORT_HOLD,
    ),
    SkillName.CHANGE_VIEW: (
        SkillName.LOOK_FOR_OBJECT,
        SkillName.ABORT_HOLD,
    ),
    SkillName.ABORT_HOLD: (),
}


class SkillRegistry:
    """Validated lookup for skill contracts and legal graph edges."""

    def __init__(self, contracts: Dict[SkillName, SkillContract] = CONTRACTS):
        missing = set(SkillName) - set(contracts)
        if missing:
            raise ValueError(
                "missing skill contracts: " + ", ".join(sorted(item.value for item in missing))
            )
        unknown = set(SKILL_GRAPH) - set(contracts)
        if unknown:
            raise ValueError(
                "skill graph contains unknown contracts: "
                + ", ".join(sorted(item.value for item in unknown))
            )
        for source, targets in SKILL_GRAPH.items():
            for target in targets:
                if target not in contracts:
                    raise ValueError(f"{source.value}: unknown target {target.value}")
        self._contracts = dict(contracts)
        self._graph = {key: tuple(value) for key, value in SKILL_GRAPH.items()}

    def contract(self, skill: SkillName) -> SkillContract:
        return self._contracts[skill]

    def successors(self, skill: SkillName) -> Tuple[SkillName, ...]:
        return self._graph.get(skill, ())

    def all_contracts(self) -> Tuple[SkillContract, ...]:
        return tuple(self._contracts[skill] for skill in SkillName)
