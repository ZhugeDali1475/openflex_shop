import math

import pytest

from openarmx_task_executor.shelf_fine_align_core import (
    ClearanceFilter,
    ShelfCornerFilter,
    ShelfCornerMeasurement,
    ShelfPlaneFilter,
    ShelfPlaneMeasurement,
    compute_command,
    compute_shelf_corner_command,
    compute_shelf_plane_command,
    compute_shelf_traverse_command,
    effective_angular_command,
    fit_shelf_plane,
    measure_scan_clearance,
    measure_scan_shelf_corners,
    measure_scan_shelf_plane,
    rectangular_clearance,
)


def traverse_command(**changes):
    values = {
        "lateral_error": 0.40,
        "lateral_tolerance": 0.03,
        "clearance": 0.05,
        "target_clearance": 0.05,
        "clearance_tolerance": 0.01,
        "yaw_error": 0.0,
        "yaw_tolerance": math.radians(1.0),
        "clearance_hysteresis": 0.003,
        "yaw_align_gate": math.radians(3.0),
        "rotate_kp": 1.2,
        "approach_kp": 0.55,
        "lateral_kp": 0.6,
        "maximum_angular_speed": 0.10,
        "maximum_forward_speed": 0.04,
        "minimum_forward_speed": 0.03,
        "maximum_lateral_speed": 0.04,
        "minimum_lateral_speed": 0.03,
    }
    values.update(changes)
    return compute_shelf_traverse_command(**values)


def test_shelf_traverse_moves_only_laterally_inside_clearance_band():
    command = traverse_command()
    assert command.phase == "TRAVERSE"
    assert command.linear_x == 0.0
    assert command.linear_y == pytest.approx(0.04)
    assert command.angular_z == 0.0


def test_shelf_traverse_refuses_reverse_when_shelf_is_too_close():
    command = traverse_command(clearance=0.03)
    assert command.phase == "TOO_CLOSE"
    assert command.linear_x == 0.0
    assert command.linear_y == 0.0


def test_shelf_traverse_restores_forward_clearance_before_lateral_motion():
    command = traverse_command(clearance=0.08)
    assert command.phase == "RESTORE_CLEARANCE"
    assert command.linear_x > 0.0
    assert command.linear_y == 0.0


def test_lateral_only_traverse_ignores_target_band_without_longitudinal_motion():
    command = traverse_command(clearance=0.087, lateral_only=True)
    assert command.phase == "TRAVERSE"
    assert command.linear_x == 0.0
    assert command.linear_y == pytest.approx(0.04)


def test_lateral_only_traverse_still_aligns_heading_before_translation():
    command = traverse_command(
        clearance=0.087,
        yaw_error=math.radians(4.0),
        lateral_only=True,
    )
    assert command.phase == "ALIGN_HEADING"
    assert command.linear_x == 0.0
    assert command.linear_y == 0.0
    assert command.angular_z > 0.0


def test_rotation_command_clears_the_deployed_module_speed_deadband():
    angular = effective_angular_command(
        math.radians(2.0),
        math.radians(1.0),
        1.2,
        0.10,
        0.070,
    )
    assert angular == pytest.approx(0.070)
    # Chassis version 6 uses module coordinates (0.21, 0.26)m and a 0.02m/s
    # module deadband. The floor must produce a wheel speed above that limit.
    assert angular * math.hypot(0.21, 0.26) > 0.020


def test_rotation_floor_disables_inside_the_requested_yaw_tolerance():
    angular = effective_angular_command(
        math.radians(0.8),
        math.radians(1.0),
        1.2,
        0.10,
        0.070,
    )
    assert angular < 0.020


def test_shelf_traverse_requires_stable_lateral_clearance_and_yaw():
    command = traverse_command(lateral_error=0.01)
    assert command.phase == "STABILIZING"
    assert command.within_tolerance


def test_rectangular_clearance_uses_nearest_body_edge_or_corner():
    assert rectangular_clearance(0.56, 0.0, 0.46, 0.45) == pytest.approx(0.10)
    assert rectangular_clearance(0.56, 0.49, 0.46, 0.45) == pytest.approx(
        math.hypot(0.10, 0.04)
    )


def test_scan_measurement_does_not_use_only_the_centre_ray():
    ranges = [math.inf] * 5
    # Angles are -0.4, -0.2, 0, +0.2, +0.4 rad.  The closest return is near
    # the front-left body corner, while the centre ray sees a farther beam.
    ranges[2] = 0.90
    ranges[4] = 0.70
    result = measure_scan_clearance(
        ranges,
        -0.4,
        0.2,
        half_length=0.46,
        half_width=0.45,
        roi_forward_min=0.46,
        roi_forward_max=1.2,
        roi_half_width=0.55,
        maximum_clearance=0.45,
    )
    assert result is not None
    assert result.point_y > 0.0
    assert result.clearance < 0.20


def test_scan_measurement_can_project_sensor_points_into_base_link():
    # The sensor is 20 cm behind base_link. A 40 cm return in the scan frame
    # therefore lands at x=0.60 m in base_link and leaves 14 cm from the front
    # edge. Filtering before this transform would incorrectly reject it.
    result = measure_scan_clearance(
        [0.40],
        0.0,
        1.0,
        half_length=0.46,
        half_width=0.45,
        roi_forward_min=0.46,
        roi_forward_max=1.2,
        roi_half_width=0.55,
        maximum_clearance=0.45,
        point_transform=lambda x, y: (x + 0.20, y),
    )
    assert result is not None
    assert result.point_x == pytest.approx(0.60)
    assert result.clearance == pytest.approx(0.14)


def test_clearance_filter_needs_multiple_frames_and_is_conservative():
    filt = ClearanceFilter(window_size=7, minimum_samples=5, quantile=0.25)
    for value in (0.18, 0.10, 0.17, 0.11):
        assert filt.add(value) is None
    assert filt.add(0.09) == pytest.approx(0.10)


def test_controller_aligns_heading_before_approaching():
    command = compute_command(
        0.10,
        0.05,
        0.01,
        math.radians(4.0),
        math.radians(1.0),
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "ALIGN_HEADING"
    assert command.linear_x == 0.0
    assert command.angular_z > 0.0


def test_controller_stops_after_five_cm_and_one_degree_are_reached():
    command = compute_command(
        0.055,
        0.05,
        0.01,
        math.radians(0.8),
        math.radians(1.0),
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.within_tolerance
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_controller_never_reverses_when_too_close():
    command = compute_command(
        0.035,
        0.05,
        0.01,
        0.0,
        math.radians(1.0),
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "TOO_CLOSE"
    assert command.linear_x == 0.0


def test_controller_keeps_final_approach_above_swerve_wheel_deadband():
    command = compute_command(
        0.109,
        0.05,
        0.01,
        math.radians(2.17),
        math.radians(1.0),
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "APPROACH"
    assert command.linear_x == pytest.approx(0.030)
    # The deployed per-wheel deadband is 0.020 m/s, so merely exceeding the
    # 0.015 m/s chassis deadband is insufficient to move the base.
    assert 0.020 < command.linear_x <= 0.040


def test_shelf_plane_uses_both_sides_and_reports_corner_asymmetry():
    normal_yaw = math.radians(3.0)
    normal_x = math.cos(normal_yaw)
    normal_y = math.sin(normal_yaw)
    offset = 0.565
    points = [
        ((offset - normal_y * y) / normal_x, y)
        for y in (-0.45, -0.35, -0.25, 0.25, 0.35, 0.45)
    ]
    points.extend(((0.82, -0.10), (0.92, 0.15), (0.48, 0.02)))
    plane = fit_shelf_plane(
        points,
        half_length=0.46,
        half_width=0.45,
        minimum_points=5,
        minimum_lateral_span=0.40,
        minimum_points_per_side=2,
        inlier_tolerance=0.01,
        maximum_normal_yaw=math.radians(12.0),
    )
    assert plane is not None
    assert plane.shelf_yaw_error == pytest.approx(normal_yaw, abs=1e-4)
    assert plane.left_clearance < plane.right_clearance
    assert plane.corner_clearance_difference > 0.04


def test_scan_shelf_corners_tracks_sparse_front_edges_instead_of_back_panel():
    angle_min = -0.8
    angle_increment = 0.01
    ranges = [math.inf] * 161
    for index in range(len(ranges)):
        angle = angle_min + index * angle_increment
        if abs(angle) < 0.35:
            continue
        front_x = 0.52 if angle > 0.0 else 0.55
        radius = front_x / math.cos(angle)
        if abs(radius * math.sin(angle)) <= 0.55:
            ranges[index] = radius

    corners = measure_scan_shelf_corners(
        ranges,
        angle_min,
        angle_increment,
        half_length=0.46,
        half_width=0.45,
        roi_forward_min=0.46,
        roi_forward_max=1.20,
        roi_half_width=0.55,
        maximum_clearance=0.45,
        minimum_lateral_offset=0.20,
        minimum_points_per_side=2,
    )

    assert corners is not None
    assert corners.left_clearance == pytest.approx(0.06, abs=1e-4)
    assert corners.right_clearance == pytest.approx(0.09, abs=1e-4)
    assert corners.shelf_yaw_error == pytest.approx(
        math.asin((0.09 - 0.06) / 0.90), abs=1e-4
    )


def test_shelf_corner_filter_keeps_both_conservative_side_estimates():
    filt = ShelfCornerFilter(3, 3, half_width=0.45, quantile=0.25)
    for left, right in ((0.08, 0.10), (0.06, 0.09), (0.05, 0.08)):
        filt.add(ShelfCornerMeasurement(left, right, 0.0, 4, 5, 0.80))

    result = filt.value
    assert result is not None
    assert result.left_clearance == pytest.approx(0.055)
    assert result.right_clearance == pytest.approx(0.085)
    assert result.shelf_yaw_error == pytest.approx(math.asin(0.03 / 0.90))


def test_scan_shelf_plane_projects_ranges_before_fitting_both_sides():
    angle_min = -0.8
    angle_increment = 0.01
    ranges = [math.inf] * 161
    for index in range(len(ranges)):
        angle = angle_min + index * angle_increment
        cosine = math.cos(angle)
        if cosine <= 0.0:
            continue
        radius = 0.37 / cosine
        point_y = radius * math.sin(angle)
        if abs(point_y) <= 0.45:
            ranges[index] = radius

    plane = measure_scan_shelf_plane(
        ranges,
        angle_min,
        angle_increment,
        half_length=0.46,
        half_width=0.45,
        roi_forward_min=0.46,
        roi_forward_max=1.20,
        roi_half_width=0.55,
        maximum_clearance=0.45,
        minimum_points=8,
        minimum_lateral_span=0.40,
        minimum_points_per_side=2,
        inlier_tolerance=0.01,
        maximum_normal_yaw=math.radians(12.0),
        point_transform=lambda x, y: (x + 0.20, y),
    )

    assert plane is not None
    assert plane.left_clearance == pytest.approx(0.11, abs=1e-4)
    assert plane.right_clearance == pytest.approx(0.11, abs=1e-4)
    assert plane.shelf_yaw_error == pytest.approx(0.0, abs=1e-4)


def test_shelf_plane_rejects_a_one_sided_return_cluster():
    plane = fit_shelf_plane(
        [(0.57, 0.20), (0.58, 0.30), (0.59, 0.40), (0.60, 0.50)],
        half_length=0.46,
        half_width=0.45,
        minimum_points=4,
        minimum_lateral_span=0.25,
        minimum_points_per_side=1,
        inlier_tolerance=0.02,
        maximum_normal_yaw=math.radians(12.0),
    )
    assert plane is None


def test_shelf_plane_ignores_a_short_table_leg_cluster():
    front_edge = [
        (0.64, y)
        for y in (-0.50, -0.40, -0.25, 0.0, 0.25, 0.40, 0.50)
    ]
    short_leg_cluster = [(0.50, 0.31), (0.50, 0.34), (0.50, 0.37)]

    plane = fit_shelf_plane(
        front_edge + short_leg_cluster,
        half_length=0.46,
        half_width=0.45,
        minimum_points=6,
        minimum_lateral_span=0.40,
        minimum_points_per_side=2,
        inlier_tolerance=0.02,
        maximum_normal_yaw=math.radians(12.0),
    )

    assert plane is not None
    assert plane.left_clearance == pytest.approx(0.18, abs=1e-4)
    assert plane.right_clearance == pytest.approx(0.18, abs=1e-4)
    assert plane.point_count == len(front_edge)


def test_delivery_plane_accepts_recorded_sparse_front_with_temporal_tolerance():
    # One frame from nav_20260817_130458 at jiaofu2. The returns span both
    # sides of the chassis, but Livox projection noise exceeds the old 2.5cm
    # residual and previously prevented the 5-of-7 temporal filter from ever
    # receiving a candidate.
    points = [
        (0.722407, -0.541567),
        (0.769706, -0.541625),
        (0.848668, -0.533954),
        (0.872877, -0.533593),
        (0.897050, -0.491681),
        (0.730478, 0.441363),
        (0.881035, 0.537516),
        (0.716968, 0.536505),
    ]
    common = {
        "half_length": 0.46,
        "half_width": 0.45,
        "minimum_points": 5,
        "minimum_lateral_span": 0.30,
        "minimum_points_per_side": 2,
        "maximum_normal_yaw": math.radians(12.0),
    }

    assert fit_shelf_plane(points, inlier_tolerance=0.025, **common) is None
    plane = fit_shelf_plane(points, inlier_tolerance=0.040, **common)

    assert plane is not None
    assert plane.point_count >= 5
    assert plane.lateral_span >= 0.30
    assert plane.left_clearance == pytest.approx(0.267, abs=0.01)
    assert plane.right_clearance == pytest.approx(0.398, abs=0.01)


def test_shelf_plane_filter_keeps_conservative_corner_clearances():
    values = []
    for left, right, yaw in ((0.08, 0.10, 0.02), (0.06, 0.09, 0.01), (0.05, 0.08, 0.00)):
        normal_x = math.cos(yaw)
        normal_y = math.sin(yaw)
        values.append(
            (normal_x, normal_y, 0.46 * normal_x + 0.5 * (left + right), left, right)
        )
    filt = ShelfPlaneFilter(3, 3, half_length=0.46, quantile=0.25)
    for normal_x, normal_y, offset, left, right in values:
        filt.add(
            ShelfPlaneMeasurement(
                normal_x, normal_y, offset, left, right, 8, 0.80
            )
        )
    result = filt.value
    assert result is not None
    assert result.left_clearance == pytest.approx(0.055)
    assert result.right_clearance == pytest.approx(0.085)
    assert result.shelf_yaw_error == pytest.approx(0.01)


def test_delivery_plane_median_rejects_two_transient_near_surfaces():
    filt = ShelfPlaneFilter(7, 5, half_length=0.46, quantile=0.50)
    clearances = (0.18, 0.18, 0.05, 0.18, 0.18, 0.06, 0.18)
    for clearance in clearances:
        filt.add(
            ShelfPlaneMeasurement(
                1.0,
                0.0,
                0.46 + clearance,
                clearance,
                clearance,
                8,
                0.80,
            )
        )

    result = filt.value
    assert result is not None
    assert result.minimum_corner_clearance == pytest.approx(0.18)


def test_shelf_corner_controller_keeps_aligning_until_requested_yaw_tolerance():
    command = compute_shelf_corner_command(
        0.05,
        0.09,
        0.05,
        0.01,
        math.radians(2.5),
        math.radians(1.0),
        target_clearance_hysteresis=0.003,
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "ALIGN_SHELF_FACE"
    assert not command.within_tolerance
    assert command.linear_x == 0.0
    assert command.angular_z > 0.0


def test_shelf_corner_controller_stabilizes_inside_requested_yaw_tolerance():
    command = compute_shelf_corner_command(
        0.05,
        0.09,
        0.05,
        0.01,
        math.radians(0.8),
        math.radians(1.0),
        target_clearance_hysteresis=0.003,
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "STABILIZING"
    assert command.within_tolerance
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_shelf_corner_controller_advances_while_finishing_a_small_yaw_error():
    command = compute_shelf_corner_command(
        0.15,
        0.17,
        0.05,
        0.01,
        math.radians(1.27),
        math.radians(1.0),
        target_clearance_hysteresis=0.003,
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "APPROACH"
    assert command.linear_x >= 0.030
    assert command.angular_z > 0.0


def test_shelf_plane_controller_requires_both_corners_at_target_to_finish():
    command = compute_shelf_plane_command(
        0.052,
        0.055,
        0.05,
        0.01,
        math.radians(0.4),
        math.radians(1.0),
        target_clearance_hysteresis=0.003,
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "STABILIZING"
    assert command.within_tolerance


def test_shelf_plane_controller_fails_before_moving_an_overclose_corner():
    command = compute_shelf_plane_command(
        0.035,
        0.07,
        0.05,
        0.01,
        0.0,
        math.radians(1.0),
        target_clearance_hysteresis=0.003,
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "TOO_CLOSE"
    assert command.linear_x == 0.0


def test_shelf_corner_controller_accepts_close_range_lidar_hysteresis():
    command = compute_shelf_corner_command(
        0.080,
        0.039,
        0.05,
        0.01,
        math.radians(-0.6),
        math.radians(1.0),
        target_clearance_hysteresis=0.003,
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "STABILIZING"
    assert command.within_tolerance
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_old_near_range_band_settles_the_recorded_item_alignment_trace():
    # Test-09 bag nav_20260816_181711 oscillated between 5.7 and 6.29 cm
    # before entering its final approach. The old 1 cm tolerance keeps that
    # entire filtered sequence stopped long enough to satisfy 10 settle frames.
    recorded_clearances = (
        0.057003,
        0.058651,
        0.058525,
        0.062861,
        0.058044,
        0.053324,
        0.052598,
        0.050779,
        0.049133,
        0.049133,
    )
    stable_samples = 0
    for clearance in recorded_clearances:
        command = compute_shelf_corner_command(
            clearance,
            clearance + 0.03,
            0.05,
            0.01,
            math.radians(0.45),
            math.radians(1.0),
            target_clearance_hysteresis=0.003,
            yaw_align_gate=math.radians(3.0),
            rotate_kp=1.2,
            approach_kp=0.55,
            maximum_angular_speed=0.10,
            maximum_linear_speed=0.040,
            minimum_linear_speed=0.030,
        )
        assert command.phase == "STABILIZING"
        assert command.linear_x == 0.0
        stable_samples = stable_samples + 1 if command.within_tolerance else 0

    assert stable_samples == 10


def test_shelf_corner_controller_keeps_hard_undershoot_outside_hysteresis():
    command = compute_shelf_corner_command(
        0.080,
        0.036,
        0.05,
        0.01,
        0.0,
        math.radians(1.0),
        target_clearance_hysteresis=0.003,
        yaw_align_gate=math.radians(3.0),
        rotate_kp=1.2,
        approach_kp=0.55,
        maximum_angular_speed=0.10,
        maximum_linear_speed=0.040,
        minimum_linear_speed=0.030,
    )
    assert command.phase == "TOO_CLOSE"
    assert not command.within_tolerance
