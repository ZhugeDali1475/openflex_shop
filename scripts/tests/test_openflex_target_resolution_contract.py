from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.target_resolution_contract import (  # noqa: E402
    RecognitionAudit,
    TargetResolutionError,
    VisualCandidate,
    authorize_grasp,
    resolve_unique_red_candidate,
)


def candidate(**kwargs):
    values = {"candidate_id": "thesis_item_01", "color": "red", "confidence": 0.95,
              "depth_m": 1.0, "pose_xyzyaw": (1.0, 0.5, 0.9, 0.0),
              "resolved_object_id": "thesis_item_01"}
    values.update(kwargs)
    return VisualCandidate(**values)


def test_candidate_requires_four_step_confirmation_before_authorization():
    selected = resolve_unique_red_candidate([candidate()])
    authorization = authorize_grasp(selected, identity_confirmed=True,
                                    depth_confirmed=True, pose_confirmed=True)
    assert authorization.object_id == "thesis_item_01"
    assert authorization.evidence["source"] == "candidate_confirmation_not_oracle_truth"


@pytest.mark.parametrize("kwargs", [
    {"identity_confirmed": False, "depth_confirmed": True, "pose_confirmed": True},
    {"identity_confirmed": True, "depth_confirmed": False, "pose_confirmed": True},
    {"identity_confirmed": True, "depth_confirmed": True, "pose_confirmed": False},
])
def test_missing_confirmation_stops_before_grasp(kwargs):
    with pytest.raises(TargetResolutionError):
        authorize_grasp(candidate(), **kwargs)


@pytest.mark.parametrize("items", [[], [candidate(visible=False)],
                                     [candidate(), candidate(candidate_id="other_red")]])
def test_no_visible_or_ambiguous_red_candidate_stops(items):
    with pytest.raises(TargetResolutionError):
        resolve_unique_red_candidate(items)


def test_oracle_truth_is_kept_separate_from_model_candidates():
    audit = RecognitionAudit(({"candidate_id": "thesis_item_01", "confidence": 0.9},),
                             {"segmentation_instance": "thesis_item_01", "visible": True})
    audit.validate()
    with pytest.raises(TargetResolutionError):
        RecognitionAudit(audit.model_candidates, {"model_output": "copied"}).validate()
