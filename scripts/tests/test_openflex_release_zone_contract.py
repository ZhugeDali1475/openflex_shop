from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"))
from openflex_roboos_bridge.release_zone_contract import (  # noqa: E402
    ReleaseZoneError,
    plan_release_zone,
)


def test_left_and_right_release_zones_are_planned_independently():
    left = plan_release_zone("sim_delivery_top_left", surface_center_xyz=(-0.85, -0.8, 0.48),
                             surface_dimensions_xy=(0.7, 0.7), base_xy=(-1.47, -0.8), version="left-v1")
    right = plan_release_zone("sim_delivery_top_right", surface_center_xyz=(2.4, 3.6, 0.75),
                              surface_dimensions_xy=(1.2, 0.8), base_xy=(1.8, 3.6), version="right-v1")
    assert left.destination_id != right.destination_id
    assert left.release_center_xyz != right.release_center_xyz
    assert left.version != right.version


def test_table_center_is_not_assumed_when_unreachable():
    with pytest.raises(ReleaseZoneError, match="no reachable"):
        plan_release_zone("sim_delivery_top_left", surface_center_xyz=(0.0, 0.0, 0.48),
                          surface_dimensions_xy=(0.2, 0.2), base_xy=(0.0, 0.0), version="left-v1")


def test_unknown_destination_is_rejected():
    with pytest.raises(ReleaseZoneError):
        plan_release_zone("delivery_zone", surface_center_xyz=(0.0, 0.0, 0.48),
                          surface_dimensions_xy=(0.7, 0.7), base_xy=(-0.5, 0.0), version="legacy")
