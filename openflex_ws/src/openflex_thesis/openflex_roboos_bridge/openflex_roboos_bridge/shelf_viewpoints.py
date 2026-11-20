"""Shelf-geometry-derived observation viewpoints (phase C).

The multi-viewpoint search must never be told where the product is.  This
module derives legal observation stands **only** from the static shelf collision
geometry: the shelf's front plane, lateral extent and a camera field of view
decide how many stands are needed and where they sit.  No registered object slot,
``ObjectMemory`` field or simulator identity is an input.

It is ROS-free and never commands motion; the caller is responsible for checking
the stands against the live planner before driving to them.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence, Tuple

from .visual_grasp_planning import Aabb, VisualGraspPlanningError


class ShelfViewpointError(VisualGraspPlanningError):
    """Raised when legal observation stands cannot be derived."""


@dataclass(frozen=True)
class Viewpoint:
    """A legal observation stand in the planning frame."""

    viewpoint_id: str
    position_xy: Tuple[float, float]
    yaw_rad: float
    standoff_m: float
    covered_y: Tuple[float, float]

    def to_dict(self) -> Mapping[str, object]:
        return {
            "viewpoint_id": self.viewpoint_id,
            "position_xy": list(self.position_xy),
            "yaw_rad": self.yaw_rad,
            "standoff_m": self.standoff_m,
            "covered_y": list(self.covered_y),
        }


def _positive(value, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ShelfViewpointError(f"{name} must be finite and positive")
    return number


def shelf_front_geometry(obstacles: Sequence[Aabb]) -> Tuple[float, float, float]:
    """Return ``(front_x, y_min, y_max)`` of the shelf collision geometry."""

    boxes = tuple(obstacles)
    if not boxes or not all(isinstance(box, Aabb) for box in boxes):
        raise ShelfViewpointError("shelf geometry requires at least one Aabb")
    front_x = min(box.min_xyz[0] for box in boxes)
    y_min = min(box.min_xyz[1] for box in boxes)
    y_max = max(box.max_xyz[1] for box in boxes)
    if not math.isfinite(front_x) or y_max <= y_min:
        raise ShelfViewpointError("degenerate shelf geometry")
    return float(front_x), float(y_min), float(y_max)


def visible_span_at(standoff_m: float, horizontal_fov_rad: float) -> float:
    """Lateral width seen at ``standoff_m`` for a horizontal field of view."""

    standoff = _positive(standoff_m, "standoff_m")
    fov = float(horizontal_fov_rad)
    if not math.isfinite(fov) or not 0 < fov < math.pi:
        raise ShelfViewpointError("horizontal_fov_rad must be within (0, pi)")
    return 2.0 * standoff * math.tan(fov / 2.0)


def generate_shelf_viewpoints(obstacles: Sequence[Aabb], *, standoff_m: float,
                              lateral_step_m: float, horizontal_fov_rad: float,
                              clearance_m: float, yaw_rad: float = 0.0,
                              center_on_shelf: bool = True) -> Tuple[Viewpoint, ...]:
    """Derive legal observation stands covering the shelf's lateral extent.

    The number of stands is the smallest that covers ``[y_min, y_max]`` with the
    requested step capped by the camera's visible span, so coverage never depends
    on where the product happens to be.
    """

    step = _positive(lateral_step_m, "lateral_step_m")
    clearance = _positive(clearance_m, "clearance_m")
    front_x, y_min, y_max = shelf_front_geometry(obstacles)
    span = visible_span_at(standoff_m, horizontal_fov_rad)
    effective_step = min(step, span)
    if effective_step <= 0:
        raise ShelfViewpointError("lateral step must be positive")
    width = y_max - y_min
    count = max(1, int(math.ceil(width / effective_step) - 1e-9))
    if count == 1:
        centers = [0.5 * (y_min + y_max) if center_on_shelf else y_min]
    else:
        spacing = min(effective_step, width / (count - 1)) if count > 1 else width
        start = 0.5 * (y_min + y_max) - 0.5 * spacing * (count - 1) if center_on_shelf else y_min
        centers = [start + index * spacing for index in range(count)]
    stand_x = front_x - _positive(standoff_m, "standoff_m")
    if stand_x > front_x - clearance:
        raise ShelfViewpointError("standoff must clear the shelf front by clearance_m")
    viewpoints = tuple(
        Viewpoint(viewpoint_id=f"shelf_view_{index:02d}", position_xy=(stand_x, float(y)),
                  yaw_rad=float(yaw_rad), standoff_m=float(standoff_m),
                  covered_y=(float(y) - span / 2.0, float(y) + span / 2.0))
        for index, y in enumerate(centers))
    _assert_covers(viewpoints, y_min, y_max)
    _assert_clear_of_shelf(viewpoints, tuple(obstacles), clearance)
    return viewpoints


def _assert_covers(viewpoints: Sequence[Viewpoint], y_min: float, y_max: float) -> None:
    intervals = sorted((box.covered_y for box in viewpoints))
    merged = []
    for low, high in intervals:
        if merged and low <= merged[-1][1] + 1e-9:
            merged[-1] = (merged[-1][0], max(merged[-1][1], high))
        else:
            merged.append((low, high))
    if not merged or merged[0][0] > y_min + 1e-9 or merged[-1][1] < y_max - 1e-9:
        raise ShelfViewpointError("generated viewpoints do not cover the shelf extent")


def _assert_clear_of_shelf(viewpoints: Sequence[Viewpoint],
                           obstacles: Sequence[Aabb], clearance_m: float) -> None:
    for viewpoint in viewpoints:
        x, y = viewpoint.position_xy
        for box in obstacles:
            if (box.min_xyz[0] - clearance_m < x < box.max_xyz[0] + clearance_m
                    and box.min_xyz[1] - clearance_m < y < box.max_xyz[1] + clearance_m
                    and box.min_xyz[2] - clearance_m < 0.0 < box.max_xyz[2] + clearance_m):
                raise ShelfViewpointError(
                    f"viewpoint {viewpoint.viewpoint_id} sits inside the shelf clearance volume")
