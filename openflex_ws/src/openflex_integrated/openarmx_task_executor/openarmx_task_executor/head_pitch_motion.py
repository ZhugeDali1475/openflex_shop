"""Small deterministic helpers for head-pitch position interpolation."""

from __future__ import annotations

import math


def next_position_step(current: float, target: float, max_step: float) -> float:
    values = (float(current), float(target), float(max_step))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("head position interpolation requires finite values")
    if max_step <= 0.0:
        raise ValueError("head position interpolation max_step must be positive")
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


def position_reached(actual: float, target: float, tolerance: float) -> bool:
    values = (float(actual), float(target), float(tolerance))
    if not all(math.isfinite(value) for value in values):
        return False
    return tolerance >= 0.0 and abs(actual - target) <= tolerance
