"""Validation helpers shared by DetectObject and PickObject callers."""

from uuid import uuid4


VALID_ARM_NAMES = ("left", "right")


def normalize_arm_name(value: str) -> str:
    arm_name = str(value).strip().lower()
    if arm_name not in VALID_ARM_NAMES:
        raise ValueError(
            f"arm_name must be one of {VALID_ARM_NAMES}, got {value!r}"
        )
    return arm_name


def require_detection_id(value: str) -> str:
    detection_id = str(value).strip()
    if not detection_id:
        raise ValueError("detection_id must not be empty")
    return detection_id


def make_nominal_detection_id(arm_name: str) -> str:
    return f"nominal:{normalize_arm_name(arm_name)}:{uuid4().hex}"


def make_mock_detection_id(object_id: str, arm_name: str) -> str:
    object_key = str(object_id).strip() or "mock_object"
    return f"mock:{normalize_arm_name(arm_name)}:{object_key}"


def detection_id_arm(detection_id: str) -> str:
    parts = require_detection_id(detection_id).split(":", 2)
    if len(parts) != 3 or parts[0] not in ("mock", "nominal", "truth"):
        raise ValueError("detection_id does not carry a supported arm namespace")
    return normalize_arm_name(parts[1])


def validate_detection_arm(detection_id: str, arm_name: str) -> None:
    normalized_arm = normalize_arm_name(arm_name)
    encoded_arm = detection_id_arm(detection_id)
    if encoded_arm != normalized_arm:
        raise ValueError(
            f"detection_id belongs to arm {encoded_arm!r}, not {normalized_arm!r}"
        )
