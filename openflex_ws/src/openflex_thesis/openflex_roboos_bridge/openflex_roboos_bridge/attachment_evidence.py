"""Validation helpers for measured MuJoCo attachment evidence."""

import math
from typing import Mapping


class AttachmentEvidenceError(ValueError):
    pass


def _finite_vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise AttachmentEvidenceError(f"{name} must contain {length} values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise AttachmentEvidenceError(f"{name} must be finite")
    return result


def validate_attachment_evidence(diagnostic: Mapping[str, object], *, object_id: str,
                                 arm: str = "right") -> dict:
    """Accept only one diagnostic sample containing measured hand/object poses."""
    if diagnostic.get("source") != "mujoco_diagnostic_only_NOT_PHYSICAL":
        raise AttachmentEvidenceError("attachment evidence source is not the diagnostic stream")
    if diagnostic.get("object_id") != object_id or diagnostic.get("attached_arm") != arm:
        raise AttachmentEvidenceError("diagnostic is not attached to the requested object/arm")
    object_pose = diagnostic.get("object_pose")
    hand_pose = diagnostic.get("attached_hand_pose")
    transform = diagnostic.get("attachment_transform")
    if not isinstance(object_pose, Mapping) or not isinstance(hand_pose, Mapping):
        raise AttachmentEvidenceError("same-sample object and hand poses are required")
    if not isinstance(transform, Mapping):
        raise AttachmentEvidenceError("same-sample attachment_transform is required")
    object_position = _finite_vector(object_pose.get("position_m"), 3, "object position")
    object_orientation = _finite_vector(object_pose.get("orientation_xyzw"), 4, "object orientation")
    hand_position = _finite_vector(hand_pose.get("position_m"), 3, "hand position")
    hand_orientation = _finite_vector(hand_pose.get("orientation_xyzw"), 4, "hand orientation")
    relative_position = _finite_vector(transform.get("translation_m"), 3, "attachment translation")
    relative_orientation = _finite_vector(transform.get("orientation_xyzw"), 4, "attachment orientation")
    for quaternion, name in ((object_orientation, "object orientation"),
                             (hand_orientation, "hand orientation"),
                             (relative_orientation, "attachment orientation")):
        if abs(sum(item * item for item in quaternion) - 1.0) > 0.01:
            raise AttachmentEvidenceError(f"{name} must be normalized")
    if transform.get("parent_frame") != "map" or not transform.get("child_frame"):
        raise AttachmentEvidenceError("attachment transform must name map parent and hand child frame")
    if object_pose.get("frame_id") != "map" or hand_pose.get("frame_id") != "map":
        raise AttachmentEvidenceError("object and hand poses must share the map frame")
    stamp = diagnostic.get("stamp")
    if (not isinstance(stamp, Mapping) or isinstance(stamp.get("sec"), bool)
            or isinstance(stamp.get("nanosec"), bool)
            or not isinstance(stamp.get("sec"), int)
            or not isinstance(stamp.get("nanosec"), int)
            or stamp["sec"] < 0 or not 0 <= stamp["nanosec"] < 1_000_000_000):
        raise AttachmentEvidenceError("same-sample diagnostic timestamp is required")
    return {
        "object_id": object_id, "arm": arm, "source": diagnostic["source"],
        "stamp": diagnostic.get("stamp"),
        "object_pose": {"position_m": object_position, "orientation_xyzw": object_orientation,
                        "frame_id": object_pose.get("frame_id")},
        "hand_pose": {"position_m": hand_position, "orientation_xyzw": hand_orientation,
                      "frame_id": hand_pose.get("frame_id")},
        "attachment_transform": {"parent_frame": transform["parent_frame"],
                                 "child_frame": transform["child_frame"],
                                 "translation_m": relative_position,
                                 "orientation_xyzw": relative_orientation},
        "independently_measured": True,
    }
