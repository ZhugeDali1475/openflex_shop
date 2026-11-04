"""Stable child-action error normalization for the public task API."""

from dataclasses import dataclass
from typing import Any


STABLE_TASK_ERROR_CODES = frozenset((*range(0, 31), 99))


@dataclass(frozen=True)
class TaskFailure:
    code: int
    message: str


def stable_error_code(value: Any, fallback: int) -> int:
    try:
        code = int(value)
    except (TypeError, ValueError):
        return int(fallback)
    if code == 0 or code not in STABLE_TASK_ERROR_CODES:
        return int(fallback)
    return code


def child_action_failure(
    payload: Any, fallback: int, default_message: str
) -> TaskFailure:
    code = stable_error_code(getattr(payload, "error_code", None), fallback)
    child_message = str(getattr(payload, "message", "")).strip()
    message = child_message or str(default_message).strip()
    return TaskFailure(code=code, message=message)
