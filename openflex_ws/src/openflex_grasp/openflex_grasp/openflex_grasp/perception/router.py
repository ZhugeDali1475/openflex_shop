from __future__ import annotations

import threading
import time

from .rgbd import FrozenRgbdFrame


class RgbdRouter:
    """Keep one validated frozen RGB-D frame per wrist; never cross route frames."""

    def __init__(self, *, clock=time.monotonic):
        self._clock = clock
        self._frames: dict[str, tuple[FrozenRgbdFrame, float]] = {}
        self._lock = threading.Lock()

    def update(self, frame: FrozenRgbdFrame) -> None:
        with self._lock:
            previous = self._frames.get(frame.arm_name)
            if previous and frame.stamp_ns < previous[0].stamp_ns:
                return
            self._frames[frame.arm_name] = (frame, self._clock())

    def freeze(self, arm_name: str, *, max_age_s: float = 0.75) -> FrozenRgbdFrame:
        arm = str(arm_name).strip().lower()
        if arm not in {"left", "right"}:
            raise ValueError("arm_name must be left or right")
        with self._lock:
            value = self._frames.get(arm)
            if value is None:
                raise RuntimeError(f"no synchronized RGB-D frame for {arm} wrist")
            frame, received = value
        age = self._clock() - received
        if age > max(0.0, float(max_age_s)):
            raise RuntimeError(f"{arm} RGB-D frame is stale ({age:.3f}s)")
        return frame
