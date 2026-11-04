"""Pure direction-policy helpers for cmd_vel_safety_gate."""

from typing import Optional


def authorization_is_fresh(
    last_heartbeat: Optional[float], now: float, timeout: float
) -> bool:
    if last_heartbeat is None:
        return False
    age = now - last_heartbeat
    return 0.0 <= age <= timeout


def reverse_policy_violation(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    *,
    allow_reverse: bool,
    reverse_authorized: bool,
    reverse_epsilon: float,
    authorized_reverse_max_speed: float,
    authorized_reverse_lateral_tolerance: float,
    authorized_reverse_angular_tolerance: float,
) -> Optional[str]:
    """Return why a reverse command is unsafe, or ``None`` when allowed."""

    if linear_x >= -reverse_epsilon or allow_reverse:
        return None
    if not reverse_authorized:
        return "reverse authorization is absent or stale"
    if -linear_x > authorized_reverse_max_speed:
        return "authorized reverse speed limit exceeded"
    if abs(linear_y) > authorized_reverse_lateral_tolerance:
        return "lateral motion is forbidden during authorized reverse"
    if abs(angular_z) > authorized_reverse_angular_tolerance:
        return "rotation is forbidden during authorized reverse"
    return None
