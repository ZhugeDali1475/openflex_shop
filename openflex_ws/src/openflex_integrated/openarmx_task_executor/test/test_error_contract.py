from types import SimpleNamespace

import pytest

from openarmx_task_executor.error_contract import (
    child_action_failure,
    stable_error_code,
)


@pytest.mark.parametrize("code", (1, 8, 14, 15, 16, 24, 25, 27, 28, 29, 30, 99))
def test_registered_nonzero_error_code_is_preserved(code):
    assert stable_error_code(code, 10) == code


@pytest.mark.parametrize("code", (None, "bad", 0, 31, 65535))
def test_unknown_or_success_code_uses_phase_fallback(code):
    assert stable_error_code(code, 10) == 10


def test_child_message_is_preserved_with_registered_code():
    failure = child_action_failure(
        SimpleNamespace(error_code=28, message="gripper stalled"),
        10,
        "pick failed",
    )

    assert failure.code == 28
    assert failure.message == "gripper stalled"


def test_unknown_child_code_keeps_message_but_uses_stable_fallback():
    failure = child_action_failure(
        SimpleNamespace(error_code=501, message="vendor-specific failure"),
        27,
        "planning failed",
    )

    assert failure.code == 27
    assert failure.message == "vendor-specific failure"


def test_missing_child_message_uses_phase_default():
    failure = child_action_failure(SimpleNamespace(error_code=0), 8, "no object")

    assert failure.code == 8
    assert failure.message == "no object"
