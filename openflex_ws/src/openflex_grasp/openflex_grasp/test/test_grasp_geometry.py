import numpy as np
import pytest

from openflex_grasp.grasp_geometry import (
    align_symmetric_axis,
    fit_center_straight_grasp,
    grasp_mode_profile,
)


def _box_points(center, size, count=5000, seed=7):
    rng = np.random.default_rng(seed)
    return center + (rng.random((count, 3)) - 0.5) * size


def test_center_grasp_is_straight_and_uses_minor_axis():
    points = _box_points(np.array([0.01, -0.02, 0.35]), np.array([0.05, 0.16, 0.08]))
    estimate = fit_center_straight_grasp(points, min_points=100)
    assert estimate.valid
    np.testing.assert_allclose(estimate.approach, [0.0, 0.0, 1.0], atol=1e-7)
    np.testing.assert_allclose(estimate.pregrasp, estimate.center - [0.0, 0.0, 0.12], atol=1e-7)
    assert estimate.required_width < 0.06
    assert abs(estimate.closing_axis[0]) > 0.95


def test_symmetric_horizontal_axis_uses_sign_nearest_current_tcp():
    np.testing.assert_allclose(
        align_symmetric_axis([0.0, 1.0, 0.0], [0.0, -0.99, 0.02]),
        [0.0, -1.0, 0.0],
        atol=1e-8,
    )


def test_grasp_depth_offset_moves_only_the_grasp_end_deeper():
    surface_center = np.array([0.01, -0.02, 0.35])
    points = _box_points(surface_center, np.array([0.05, 0.16, 0.08]))
    estimate = fit_center_straight_grasp(
        points,
        min_points=100,
        grasp_depth_offset=0.015,
    )
    np.testing.assert_allclose(estimate.surface_center, surface_center, atol=0.003)
    np.testing.assert_allclose(estimate.center, surface_center + [0.0, 0.0, 0.015], atol=0.003)
    np.testing.assert_allclose(estimate.pregrasp, surface_center - [0.0, 0.0, 0.12], atol=0.003)


def test_rejects_object_wider_than_gripper():
    points = _box_points(np.array([0.0, 0.0, 0.3]), np.array([0.12, 0.15, 0.06]))
    estimate = fit_center_straight_grasp(points, gripper_max_width=0.088)
    assert not estimate.valid
    assert "exceeds" in estimate.reason


def test_width_limit_can_be_disabled_for_full_open_then_close_grasp():
    points = _box_points(
        np.array([0.0, 0.0, 0.3]), np.array([0.12, 0.15, 0.06])
    )
    estimate = fit_center_straight_grasp(
        points,
        gripper_max_width=0.086,
        enforce_gripper_width=False,
    )
    assert estimate.valid
    assert estimate.required_width > 0.086


def test_rejects_obstructed_straight_path():
    target = _box_points(np.array([0.0, 0.0, 0.3]), np.array([0.04, 0.12, 0.06]))
    obstacle = _box_points(np.array([0.0, 0.0, 0.22]), np.array([0.02, 0.02, 0.02]), count=200)
    scene = np.vstack((target[::4], obstacle))
    estimate = fit_center_straight_grasp(target, scene, collision_point_limit=10)
    assert not estimate.valid
    assert estimate.obstacle_count > 10


def test_explicit_closing_axis_is_projected_onto_straight_plane():
    points = _box_points(np.array([0.0, 0.0, 0.3]), np.array([0.04, 0.06, 0.08]))
    estimate = fit_center_straight_grasp(
        points,
        closing_axis=[1.0, 1.0, 0.5],
    )
    expected = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
    np.testing.assert_allclose(estimate.approach, [0.0, 0.0, 1.0], atol=1e-7)
    np.testing.assert_allclose(estimate.closing_axis, expected, atol=1e-7)
    np.testing.assert_allclose(estimate.rotation[:, 2], estimate.approach, atol=1e-7)


def test_basket_top_down_grasp_starts_above_product():
    surface_center = np.array([0.45, -0.10, 0.50])
    points = _box_points(surface_center, np.array([0.08, 0.05, 0.10]))
    estimate = fit_center_straight_grasp(
        points,
        approach=[0.0, 0.0, -1.0],
        closing_axis=[0.0, -1.0, 0.0],
        pregrasp_distance=0.12,
        grasp_depth_offset=0.015,
    )
    np.testing.assert_allclose(estimate.approach, [0.0, 0.0, -1.0], atol=1e-8)
    np.testing.assert_allclose(
        estimate.pregrasp,
        estimate.surface_center + [0.0, 0.0, 0.12],
        atol=1e-8,
    )
    np.testing.assert_allclose(
        estimate.center,
        estimate.surface_center - [0.0, 0.0, 0.015],
        atol=1e-8,
    )
    np.testing.assert_allclose(estimate.rotation.T @ estimate.rotation, np.eye(3), atol=1e-8)
    assert np.linalg.det(estimate.rotation) == pytest.approx(1.0)


def test_grasp_mode_profiles_select_independent_axis_parameters():
    shelf = grasp_mode_profile("shelf")
    basket = grasp_mode_profile("BASKET")
    assert shelf.approach_axis_parameter == "approach_axis"
    assert basket.approach_axis_parameter == "basket_approach_axis"
    assert basket.closing_axis_parameter is None
    assert basket.fit_method.startswith("basket_")


def test_rejects_unknown_grasp_mode():
    with pytest.raises(ValueError, match="shelf.*basket"):
        grasp_mode_profile("drawer")
