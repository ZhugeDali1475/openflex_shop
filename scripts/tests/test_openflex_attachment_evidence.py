from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.attachment_evidence import (  # noqa: E402
    AttachmentEvidenceError,
    validate_attachment_evidence,
)


def diagnostic():
    return {
        "source": "mujoco_diagnostic_only_NOT_PHYSICAL",
        "stamp": {"sec": 12, "nanosec": 34},
        "object_id": "thesis_item_01", "attached_arm": "right",
        "object_pose": {"frame_id": "map", "position_m": [1.0, 2.0, 3.0],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "attached_hand_pose": {"frame_id": "map", "position_m": [0.9, 2.0, 3.0],
                                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "attachment_transform": {"parent_frame": "map", "child_frame": "openarmx_right_hand",
                                 "translation_m": [0.1, 0.0, 0.0],
                                 "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
    }


def test_same_sample_measured_attachment_evidence_is_normalized_for_audit():
    result = validate_attachment_evidence(diagnostic(), object_id="thesis_item_01")
    assert result["independently_measured"] is True
    assert result["attachment_transform"]["translation_m"] == [0.1, 0.0, 0.0]
    assert result["stamp"] == {"sec": 12, "nanosec": 34}


@pytest.mark.parametrize("patch", [
    {"source": "oracle_grasp_candidate"}, {"attached_arm": None},
    {"object_id": "another_object"}, {"attachment_transform": None}, {"stamp": None},
])
def test_missing_or_untrusted_attachment_evidence_is_rejected(patch):
    value = diagnostic(); value.update(patch)
    with pytest.raises(AttachmentEvidenceError):
        validate_attachment_evidence(value, object_id="thesis_item_01")


def test_attachment_evidence_rejects_invalid_quaternion_or_frame():
    value = diagnostic(); value["attachment_transform"]["orientation_xyzw"] = [0.0, 0.0, 0.0, 0.0]
    with pytest.raises(AttachmentEvidenceError, match="normalized"):
        validate_attachment_evidence(value, object_id="thesis_item_01")
    value = diagnostic(); value["attached_hand_pose"]["frame_id"] = "odom"
    with pytest.raises(AttachmentEvidenceError, match="map frame"):
        validate_attachment_evidence(value, object_id="thesis_item_01")
