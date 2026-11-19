from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.destination_contract import (  # noqa: E402
    canonical_destination_id,
    destination_geometry_parameters,
)


@pytest.mark.parametrize("value", ["delivery_zone", "sim_delivery_top_left"])
def test_legacy_and_canonical_left_ids_share_one_explicit_identity(value):
    assert canonical_destination_id(value) == "sim_delivery_top_left"
    assert destination_geometry_parameters(value) == ("thesis_delivery_table", "thesis_delivery_top")


def test_right_identity_is_distinct_and_not_implicitly_left():
    assert canonical_destination_id("sim_delivery_top_right") == "sim_delivery_top_right"
    assert destination_geometry_parameters("sim_delivery_top_right") == (
        "counter_top_right", "counter_top_right_top")


@pytest.mark.parametrize("value", ["", "free_coordinate", "delivery_zone_b"])
def test_unknown_destination_is_rejected(value):
    with pytest.raises(ValueError):
        canonical_destination_id(value)
