"""Cancellable multi-viewpoint active search state machine (phase C).

Drives the single-view RGB-D search from :mod:`target_search_contract` across the
shelf observation stands from :mod:`shelf_viewpoints`.  Every transition is
fail-closed: a stale frame, unusable depth, an ambiguous result, a cancellation or
an exhausted viewpoint list ends the search with a named outcome rather than an
authorization.

The machine is ROS-free and deterministic; the caller supplies frame timestamps
and a clock, so the whole search can be replayed offline.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping, Optional, Sequence, Tuple

from .shelf_viewpoints import Viewpoint

# Terminal outcomes.  Only ``found`` authorizes anything downstream.
FOUND = "found"
NOT_FOUND = "not_found"
AMBIGUOUS = "ambiguous"
STALE_FRAME = "stale_frame"
INVALID_DEPTH = "invalid_depth"
CANCELLED = "cancelled"
NOT_SETTLED = "not_settled"
TIMEOUT = "timeout"


class ActiveSearchError(ValueError):
    """Raised when the search machine is driven out of contract."""


@dataclass(frozen=True)
class SearchDecision:
    finished: bool
    outcome: Optional[str]
    detail: str
    viewpoint_id: Optional[str]
    selected_perception_track_id: Optional[str] = None

    def to_dict(self) -> Mapping[str, object]:
        return {"finished": self.finished, "outcome": self.outcome, "detail": self.detail,
                "viewpoint_id": self.viewpoint_id,
                "selected_perception_track_id": self.selected_perception_track_id}


@dataclass
class ActiveSearch:
    """Sequential, cancellable, fail-closed multi-viewpoint search."""

    viewpoints: Tuple[Viewpoint, ...]
    max_frame_age_sec: float = 2.0
    min_depth_valid_fraction: float = 0.2
    deadline_sec: Optional[float] = None
    _index: int = 0
    _cancelled: bool = False
    _finished: bool = False
    _outcome: Optional[str] = None
    _detail: str = "search not started"
    history: list = field(default_factory=list)

    def __post_init__(self) -> None:
        self.viewpoints = tuple(self.viewpoints)
        if not self.viewpoints:
            raise ActiveSearchError("active search requires at least one viewpoint")
        for name, value in (("max_frame_age_sec", self.max_frame_age_sec),
                            ("min_depth_valid_fraction", self.min_depth_valid_fraction)):
            if not math.isfinite(float(value)) or float(value) < 0:
                raise ActiveSearchError(f"{name} must be finite and non-negative")
        if self.min_depth_valid_fraction > 1.0:
            raise ActiveSearchError("min_depth_valid_fraction must be within [0, 1]")
        if self.deadline_sec is not None and not math.isfinite(float(self.deadline_sec)):
            raise ActiveSearchError("deadline_sec must be finite when supplied")
        self._assert_windows_cover()

    def _assert_windows_cover(self) -> None:
        """Refuse a viewpoint plan that leaves a blind spot between stands."""

        for viewpoint in self.viewpoints:
            if not (math.isfinite(float(viewpoint.position_xy[0]))
                    and math.isfinite(float(viewpoint.position_xy[1]))):
                raise ActiveSearchError(f"{viewpoint.viewpoint_id} position must be finite")
            low, high = (float(value) for value in viewpoint.covered_y)
            if not math.isfinite(low) or not math.isfinite(high) or high <= low:
                raise ActiveSearchError(f"{viewpoint.viewpoint_id} covered window is degenerate")
        windows = sorted((float(box.covered_y[0]), float(box.covered_y[1]))
                         for box in self.viewpoints)
        reach = windows[0][1]
        for low, high in windows[1:]:
            if low > reach + 1e-9:
                raise ActiveSearchError(
                    f"viewpoints leave a lateral blind spot between {reach:.3f} and {low:.3f}")
            reach = max(reach, high)

    # -- control -----------------------------------------------------------
    def cancel(self, detail: str = "search cancelled") -> SearchDecision:
        self._cancelled, self._finished, self._outcome, self._detail = True, True, CANCELLED, detail
        return SearchDecision(True, CANCELLED, detail, None)

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def outcome(self) -> Optional[str]:
        return self._outcome

    def next_viewpoint(self) -> Optional[Viewpoint]:
        """Return the stand to observe from next, or ``None`` when exhausted."""

        if self._finished or self._index >= len(self.viewpoints):
            return None
        return self.viewpoints[self._index]

    # -- driving -----------------------------------------------------------
    def submit(self, viewpoint: Viewpoint, *, frame_timestamp_sec: float, now_sec: float,
               depth_valid_fraction: float, resolution, settled: bool = True) -> SearchDecision:
        """Feed one observation for the current stand and get the next decision.

        ``settled`` states whether the platform had stopped moving before the
        frame was captured; a frame grabbed while still moving is fail-closed.
        """

        if self._finished:
            raise ActiveSearchError("active search already finished")
        expected = self.next_viewpoint()
        if expected is None:
            return self._finish(NOT_FOUND, "all viewpoints observed without a unique target")
        if not isinstance(viewpoint, Viewpoint) or viewpoint.viewpoint_id != expected.viewpoint_id:
            raise ActiveSearchError(
                f"observation is for {getattr(viewpoint, 'viewpoint_id', None)}, "
                f"expected {expected.viewpoint_id}")
        if resolution is None or not hasattr(resolution, "outcome") or not hasattr(resolution, "reason"):
            raise ActiveSearchError("resolution must be a SearchResolution")
        age = float(now_sec) - float(frame_timestamp_sec)
        if not math.isfinite(age):
            raise ActiveSearchError("frame timestamp and clock must be finite")
        self._index += 1
        self.history.append({"viewpoint_id": viewpoint.viewpoint_id, "frame_age_sec": round(age, 6),
                             "depth_valid_fraction": float(depth_valid_fraction),
                             "settled": bool(settled),
                             "resolution_outcome": resolution.outcome,
                             "resolution_reason": resolution.reason})
        if not settled:
            return self._finish(NOT_SETTLED,
                                f"platform not settled at {viewpoint.viewpoint_id}; refusing")
        if age > self.max_frame_age_sec:
            return self._finish(STALE_FRAME,
                                f"frame is {age:.3f}s old (> {self.max_frame_age_sec}s); refusing")
        if float(depth_valid_fraction) < self.min_depth_valid_fraction:
            return self._finish(INVALID_DEPTH,
                                f"depth valid fraction {float(depth_valid_fraction):.3f} below "
                                f"{self.min_depth_valid_fraction}; refusing")
        if self.deadline_sec is not None and float(now_sec) > float(self.deadline_sec):
            return self._finish(TIMEOUT, "search deadline exceeded")
        if resolution.outcome == "authorized":
            return self._finish(FOUND, resolution.selection_reason,
                                resolution.selected_perception_track_id)
        if resolution.reason == "ambiguous_candidates":
            return self._finish(AMBIGUOUS, resolution.selection_reason)
        if self._index >= len(self.viewpoints):
            return self._finish(NOT_FOUND, "all viewpoints observed without a unique target")
        return SearchDecision(False, None, f"{resolution.reason} at {viewpoint.viewpoint_id}; "
                                           f"continue to {self.next_viewpoint().viewpoint_id}",
                              viewpoint.viewpoint_id)

    def _finish(self, outcome: str, detail: str,
                track: Optional[str] = None) -> SearchDecision:
        self._finished, self._outcome, self._detail = True, outcome, detail
        return SearchDecision(True, outcome, detail, self.viewpoints[self._index - 1].viewpoint_id
                              if self._index else None, track)

    def report(self) -> Mapping[str, object]:
        return {"outcome": self._outcome, "finished": self._finished, "detail": self._detail,
                "viewpoints_total": len(self.viewpoints), "viewpoints_observed": self._index,
                "history": list(self.history), "physical_verification": False,
                "robot_motion_executed": False}
