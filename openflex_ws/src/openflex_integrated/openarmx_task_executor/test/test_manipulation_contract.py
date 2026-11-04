import pytest

from openarmx_task_executor.manipulation_contract import (
    detection_id_arm,
    make_mock_detection_id,
    make_nominal_detection_id,
    normalize_arm_name,
    require_detection_id,
    validate_detection_arm,
)


@pytest.mark.parametrize(("value", "expected"), (("left", "left"), (" RIGHT ", "right")))
def test_normalize_arm_name(value, expected):
    assert normalize_arm_name(value) == expected


@pytest.mark.parametrize("value", ("", "center", None))
def test_invalid_arm_name_is_rejected(value):
    with pytest.raises(ValueError):
        normalize_arm_name(value)


def test_detection_ids_preserve_arm_identity():
    nominal_id = make_nominal_detection_id("left")
    mock_id = make_mock_detection_id("sku-1", "right")

    assert detection_id_arm(nominal_id) == "left"
    assert mock_id == "mock:right:sku-1"
    validate_detection_arm(mock_id, "right")


def test_empty_or_cross_arm_detection_id_is_rejected():
    with pytest.raises(ValueError):
        require_detection_id("  ")
    with pytest.raises(ValueError):
        validate_detection_arm("mock:left:sku-1", "right")
    with pytest.raises(ValueError):
        detection_id_arm("opaque-id-without-arm")
