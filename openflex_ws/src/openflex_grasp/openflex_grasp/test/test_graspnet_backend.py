import numpy as np
import pytest

from openflex_grasp.graspnet_backend import (
    fit_graspnet_shelf_parameters,
    fit_graspnet_straight_grasp,
    select_straight_grasp_candidate,
    summarize_graspnet_candidates,
)
from openflex_grasp.grasp_geometry import robust_cloud_center


def _box_points(center, size, count=5000, seed=9):
    rng = np.random.default_rng(seed)
    return center + (rng.random((count, 3)) - 0.5) * size


def _candidate(score, center, approach, closing, width=0.05):
    approach = np.asarray(approach, dtype=np.float64)
    approach /= np.linalg.norm(approach)
    closing = np.asarray(closing, dtype=np.float64)
    closing -= np.dot(closing, approach) * approach
    closing /= np.linalg.norm(closing)
    height = np.cross(approach, closing)
    rotation = np.column_stack((approach, closing, height))
    row = np.zeros(17, dtype=np.float64)
    row[0] = score
    row[1] = width
    row[2] = 0.02
    row[3] = 0.03
    row[4:13] = rotation.reshape(-1)
    row[13:16] = center
    row[16] = -1
    return row


def test_selects_only_near_and_straight_graspnet_candidate():
    center = np.array([0.01, -0.02, 0.35])
    candidates = np.vstack((
        _candidate(0.99, center, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        _candidate(0.98, center + [0.2, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        _candidate(0.82, center + [0.01, 0.0, 0.0], [0.1, 0.0, 0.995], [0.0, 1.0, 0.0]),
    ))
    selected = select_straight_grasp_candidate(candidates, center)
    assert selected.score == pytest.approx(0.82)
    assert selected.candidate_count == 3
    assert selected.constrained_candidate_count == 1
    assert selected.center_distance == pytest.approx(0.01)
    assert selected.approach_alignment > 0.99


def test_graspnet_uses_horizontal_cloud_span_for_robot_width():
    points = _box_points(
        np.array([0.01, -0.02, 0.35]),
        np.array([0.04, 0.05, 0.08]),
    )
    _, _, target_center = robust_cloud_center(points)
    candidates = np.vstack((
        _candidate(0.75, target_center + [0.01, 0.0, 0.0], [0.1, 0.0, 0.995], [0.0, 1.0, 0.0]),
    ))
    estimate = fit_graspnet_straight_grasp(points, None, candidates)
    assert estimate.valid
    assert estimate.backend == "graspnet"
    assert estimate.network_score == pytest.approx(0.75)
    np.testing.assert_allclose(estimate.approach, [0.0, 0.0, 1.0], atol=1e-8)
    np.testing.assert_allclose(
        estimate.pregrasp,
        estimate.center - estimate.approach * 0.12,
        atol=1e-8,
    )
    np.testing.assert_allclose(estimate.closing_axis, [0.0, 1.0, 0.0], atol=1e-8)
    np.testing.assert_allclose(estimate.center, target_center, atol=1e-8)
    assert estimate.network_grasp_width == pytest.approx(candidates[0, 1])
    assert 0.04 < estimate.grasp_width < 0.055
    assert estimate.required_width == pytest.approx(estimate.grasp_width + 0.008)


def test_network_width_above_robot_limit_is_diagnostic_not_a_hard_rejection():
    points = _box_points(
        np.array([0.35, -0.02, 0.8]),
        np.array([0.08, 0.05, 0.12]),
    )
    _, _, target_center = robust_cloud_center(points)
    candidates = np.vstack((
        _candidate(
            0.9,
            target_center,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            width=0.095,
        ),
    ))
    estimate = fit_graspnet_straight_grasp(
        points,
        None,
        candidates,
        approach=[1.0, 0.0, 0.0],
        closing_axis=[0.0, 1.0, 0.0],
        gripper_max_width=0.088,
        width_margin=0.004,
    )
    assert estimate.valid
    assert estimate.network_grasp_width == pytest.approx(0.095)
    assert estimate.required_width < 0.088


def test_fixed_horizontal_closing_axis_overrides_vertical_graspnet_roll():
    points = _box_points(
        np.array([0.35, -0.02, 0.8]),
        np.array([0.08, 0.05, 0.12]),
    )
    _, _, target_center = robust_cloud_center(points)
    candidates = np.vstack((
        _candidate(
            0.9,
            target_center,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ),
    ))
    estimate = fit_graspnet_straight_grasp(
        points,
        None,
        candidates,
        approach=[1.0, 0.0, 0.0],
        closing_axis=[0.0, 1.0, 0.0],
    )
    np.testing.assert_allclose(estimate.approach, [1.0, 0.0, 0.0], atol=1e-8)
    np.testing.assert_allclose(estimate.closing_axis, [0.0, 1.0, 0.0], atol=1e-8)
    assert estimate.closing_axis[2] == pytest.approx(0.0)


def test_graspnet_roll_can_be_projected_into_table_plane():
    points = _box_points(
        np.array([0.45, -0.02, 0.5]),
        np.array([0.08, 0.05, 0.12]),
    )
    _, _, target_center = robust_cloud_center(points)
    candidates = np.vstack((
        _candidate(
            0.9,
            target_center,
            [0.0, 0.0, -1.0],
            [1.0, 1.0, 0.0],
        ),
    ))
    estimate = fit_graspnet_straight_grasp(
        points,
        None,
        candidates,
        approach=[0.0, 0.0, -1.0],
        closing_axis=None,
        closing_axis_reference=[-1.0, -1.0, 0.0],
    )
    expected = np.array([-1.0, -1.0, 0.0]) / np.sqrt(2.0)
    np.testing.assert_allclose(estimate.closing_axis, expected, atol=1e-8)
    assert np.dot(estimate.closing_axis, estimate.approach) == pytest.approx(0.0)
    assert abs(estimate.closing_axis[0]) > 0.5
    assert abs(estimate.closing_axis[1]) > 0.5


def test_basket_closing_axis_uses_rotated_cloud_box_minor_axis_not_graspnet_roll():
    theta = np.deg2rad(32.0)
    rotation = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta), np.cos(theta), 0.0],
        [0.0, 0.0, 1.0],
    ])
    center = np.array([0.45, -0.02, 0.5])
    points = _box_points(
        np.zeros(3),
        np.array([0.12, 0.04, 0.08]),
        count=20000,
    ) @ rotation.T + center
    _, _, target_center = robust_cloud_center(points)
    candidates = np.vstack((
        _candidate(
            0.95,
            target_center,
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ),
    ))
    expected_minor = rotation @ np.array([0.0, 1.0, 0.0])
    estimate = fit_graspnet_straight_grasp(
        points,
        None,
        candidates,
        approach=[0.0, 0.0, -1.0],
        closing_axis_reference=expected_minor,
        use_cloud_minor_axis=True,
    )

    assert np.dot(estimate.closing_axis, expected_minor) > 0.999
    assert abs(float(np.dot(estimate.closing_axis, [1.0, 0.0, 0.0]))) < 0.6
    assert estimate.grasp_width == pytest.approx(0.04, abs=0.004)


def test_shelf_fit_does_not_require_a_hard_aligned_candidate():
    center = np.array([0.0, 0.0, 0.3])
    candidates = np.vstack((
        _candidate(0.9, center + [0.01, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], width=0.045),
        _candidate(0.7, center + [0.0, 0.01, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], width=0.055),
    ))
    fitted = fit_graspnet_shelf_parameters(
        candidates,
        center,
        approach=[0.0, 0.0, 1.0],
        distance_scale=0.06,
        approach_angle_scale_deg=25.0,
        max_width=0.084,
    )
    assert fitted.candidate_count == 2
    assert fitted.contributor_count == 2
    assert fitted.width == pytest.approx(0.045)
    assert abs(float(np.dot(fitted.closing_axis, [0.0, 0.0, 1.0]))) < 1e-8


def test_rejects_graspnet_candidates_outside_constraints():
    center = np.array([0.0, 0.0, 0.3])
    candidates = np.vstack((
        _candidate(0.9, center, [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]),
    ))
    with pytest.raises(ValueError, match="near the product center"):
        select_straight_grasp_candidate(candidates, center)


def test_rejects_candidate_wider_than_robot_gripper():
    center = np.array([0.0, 0.0, 0.3])
    candidates = np.vstack((
        _candidate(0.9, center, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], width=0.05),
    ))
    with pytest.raises(ValueError, match="requested approach direction"):
        select_straight_grasp_candidate(candidates, center, max_width=0.044)


def test_candidate_diagnostics_preserve_rejection_reasons():
    center = np.array([0.0, 0.0, 0.3])
    candidates = np.vstack((
        _candidate(0.9, center, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        _candidate(0.8, center + [0.2, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        _candidate(0.7, center, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], width=0.09),
    ))
    summary = summarize_graspnet_candidates(
        candidates,
        center,
        max_center_distance=0.06,
        max_approach_angle_deg=25.0,
        max_width=0.084,
    )
    assert summary["candidate_count"] == 3
    assert summary["near_center_count"] == 2
    assert summary["approach_aligned_count"] == 2
    assert summary["width_feasible_count"] == 2
    assert summary["constrained_count"] == 0
    assert summary["fit_contributor_count"] >= 1
    assert summary["fit_method"] == "shelf_center_weighted_graspnet"
    reasons = {reason for candidate in summary["candidates"] for reason in candidate["reasons"]}
    assert {"center", "approach", "width"}.issubset(reasons)
