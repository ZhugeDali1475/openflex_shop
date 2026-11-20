import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    LocalizationQuality, MappingError, OccupancyGridBuilder,
    ground_truth_localization, localization_from_odometry_covariance,
)


def diagonal_covariance(values):
    covariance = [[0.0] * 6 for _ in range(6)]
    for index, value in enumerate(values):
        covariance[index][index] = value
    return covariance


def test_ground_truth_source_leaves_estimator_signals_unset():
    quality = ground_truth_localization(stamp_sec=1.5)
    assert quality.source == "simulation_ground_truth_not_an_estimator"
    assert quality.inlier_ratio is None
    assert quality.localization_healthy is None
    assert quality.covariance_trace is None


def test_odometry_covariance_becomes_position_and_yaw_rmse():
    covariance = diagonal_covariance([0.04, 0.09, 0.25, 0.0, 0.0, 0.01])
    quality = localization_from_odometry_covariance(
        covariance, source="simulation_fastlio2_not_physical", stamp_sec=7.0)
    assert quality.source == "simulation_fastlio2_not_physical"
    assert math.isclose(quality.position_rmse_m, math.sqrt((0.04 + 0.09 + 0.25) / 3.0))
    assert math.isclose(quality.yaw_rmse_rad, 0.1)
    assert math.isclose(quality.covariance_trace, 0.39)


def test_odometry_covariance_rejects_wrong_shape_and_negative_variance():
    with pytest.raises(MappingError):
        localization_from_odometry_covariance([[0.0] * 6 for _ in range(5)], source="x")
    bad = diagonal_covariance([0.04, 0.09, 0.25, 0.0, 0.0, 0.01])
    bad[2][2] = -1.0
    with pytest.raises(MappingError):
        localization_from_odometry_covariance(bad, source="x")


def test_registration_quality_attaches_to_an_estimate():
    quality = localization_from_odometry_covariance(
        diagonal_covariance([0.01] * 6), source="icp_registration")
    enriched = quality.with_registration_quality(
        inlier_ratio=0.83, inlier_rmse_m=0.021, all_mse=0.4, p50_distance_m=0.01,
        p90_distance_m=0.05, localization_healthy=True, yaw_jump_detected=False,
        lio_wheel_divergence=0.02, low_support_seconds=0.0)
    assert enriched.inlier_ratio == pytest.approx(0.83)
    assert enriched.localization_healthy is True
    assert quality.inlier_ratio is None  # original untouched


def test_registration_quality_rejects_inconsistent_values():
    quality = LocalizationQuality(converged=True, source="icp_registration")
    with pytest.raises(MappingError):
        quality.with_registration_quality(inlier_ratio=1.2)
    with pytest.raises(MappingError):
        quality.with_registration_quality(p50_distance_m=0.2, p90_distance_m=0.1)
    with pytest.raises(MappingError):
        quality.with_registration_quality(lio_wheel_divergence=-0.01)


def test_to_dict_exposes_quality_fields_and_preserves_types():
    quality = LocalizationQuality(converged=True, source="icp_registration",
                                  inlier_ratio=0.5, localization_healthy=True,
                                  yaw_jump_detected=False)
    payload = quality.to_dict()
    assert payload["inlier_ratio"] == 0.5
    assert payload["localization_healthy"] is True
    assert payload["yaw_jump_detected"] is False
    assert payload["low_support_seconds"] is None
    assert payload["source"] == "icp_registration"


def test_snapshot_accepts_an_estimated_localization():
    builder = OccupancyGridBuilder(frame_id="map", resolution_m=0.1, origin_xy=(0.0, 0.0),
                                   width=4, height=4)
    quality = localization_from_odometry_covariance(
        diagonal_covariance([0.0025] * 6), source="simulation_fastlio2_not_physical")
    snapshot = builder.snapshot(localization=quality, stamp_sec=2.0)
    assert snapshot.localization.source == "simulation_fastlio2_not_physical"
    assert snapshot.localization.is_usable(max_position_rmse_m=0.1, max_yaw_rmse_rad=0.1)
