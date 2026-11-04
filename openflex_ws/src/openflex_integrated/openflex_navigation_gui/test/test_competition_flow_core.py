import math
from pathlib import Path

import pytest
import yaml

from openflex_navigation_gui.benchmark import BenchmarkWaypoint
from openflex_navigation_gui.competition_flow_core import (
    WristCameraCalibration,
    camera_aware_waypoint_from_anchor,
    compatible_approach_waypoints,
    index_shelf_face_anchors,
    match_anchor_shelf_face,
    measure_start_clearance,
    parse_competition_order,
)


def waypoint(name, x, y, yaw, role="shelf_approach"):
    return BenchmarkWaypoint(name, name, x, y, yaw, 0.05, role)


def shelf(x=0.0, y=0.0, yaw=0.0):
    return {
        "id": 1,
        "world_x": x,
        "world_y": y,
        "yaw": yaw,
        "shelf_width": 1.0,
        "shelf_length": 2.0,
    }


def slot(face, local_y, world_x, world_y):
    return {
        "face": face,
        "local_y": local_y,
        "world_x": world_x,
        "world_y": world_y,
    }


def test_start_clearance_uses_initial_heading_projection():
    progress = measure_start_clearance(
        (1.2, -0.8, math.pi / 2.0),
        (1.18, -0.49, math.pi / 2.0 + math.radians(1.0)),
    )

    assert progress.forward_distance_m == pytest.approx(0.31)
    assert progress.lateral_drift_m == pytest.approx(0.02)
    assert progress.yaw_error_rad == pytest.approx(math.radians(1.0))


def test_lateral_motion_does_not_complete_start_clearance():
    progress = measure_start_clearance((0.0, 0.0, 0.0), (0.0, 0.30, 0.0))

    assert progress.forward_distance_m == pytest.approx(0.0)
    assert progress.lateral_drift_m == pytest.approx(0.30)


def test_start_clearance_rejects_non_finite_pose():
    with pytest.raises(ValueError, match="finite"):
        measure_start_clearance((0.0, 0.0, 0.0), (math.nan, 0.0, 0.0))


def test_face_zero_selects_nearest_station_outside_low_x_face():
    stations = [
        waypoint("SHELF_D", -0.6, 0.4, 0.0),
        waypoint("SHELF_C", -0.6, 1.4, 0.0),
        waypoint("OTHER_SHELF", -2.0, 0.4, 0.0),
        waypoint("WRONG_SIDE", 1.4, 0.4, math.pi),
        waypoint("DELIVERY", -0.2, 0.4, 0.0, "delivery_table"),
    ]

    candidates = compatible_approach_waypoints(
        slot(0, 0.4, 0.15, 0.4), shelf(), stations
    )

    assert [value.waypoint.waypoint_id for value in candidates] == [
        "SHELF_D",
        "SHELF_C",
        "OTHER_SHELF",
    ]
    assert candidates[0].distance_m == pytest.approx(0.75)
    assert candidates[0].outward_distance_m == pytest.approx(0.6)
    assert candidates[0].yaw_error_rad == pytest.approx(0.0)


def test_face_one_rejects_nearer_station_with_opposite_heading():
    stations = [
        waypoint("CORRECT", 1.4, 1.4, math.pi),
        waypoint("WRONG_HEADING", 1.2, 1.4, 0.0),
        waypoint("WRONG_SIDE", 0.8, 1.4, math.pi),
    ]

    candidates = compatible_approach_waypoints(
        slot(1, 1.4, 0.85, 1.4), shelf(), stations
    )

    assert [value.waypoint.waypoint_id for value in candidates] == ["CORRECT"]
    assert candidates[0].desired_yaw == pytest.approx(math.pi)


def test_rotated_shelf_transforms_face_normal_and_desired_heading():
    rotated = shelf(x=2.0, y=3.0, yaw=math.pi / 2.0)
    target = slot(0, 0.5, 1.5, 3.15)
    stations = [
        waypoint("BELOW_FACE", 1.5, 2.4, math.pi / 2.0),
        waypoint("ABOVE_FACE", 1.5, 3.4, -math.pi / 2.0),
        waypoint("OLD_HEADING", 1.5, 2.3, 0.0),
    ]

    candidates = compatible_approach_waypoints(target, rotated, stations)

    assert [value.waypoint.waypoint_id for value in candidates] == ["BELOW_FACE"]
    assert candidates[0].desired_yaw == pytest.approx(math.pi / 2.0)


def test_returns_no_candidate_when_fixed_station_headings_do_not_match():
    candidates = compatible_approach_waypoints(
        slot(0, 0.5, -0.15, 0.5),
        shelf(yaw=math.pi / 2.0),
        [waypoint("SHELF_A", -0.6, 0.5, 0.0)],
    )
    assert candidates == ()


def test_rejects_invalid_face():
    with pytest.raises(ValueError, match="unsupported shelf face"):
        compatible_approach_waypoints(
            slot(2, 0.5, 0.0, 0.0), shelf(), []
        )


def test_test9_a_anchor_matches_registered_shelf_two_face_one():
    anchor = waypoint("SHELF-A", 1.945, 1.323, math.pi)
    registered = {
        "id": 2,
        "world_x": 0.5706867418984594,
        "world_y": 0.8047947063020753,
        "yaw": 0.06356114806348771,
        "shelf_width": 0.77,
        "shelf_length": 1.83,
    }

    match = match_anchor_shelf_face(anchor, [registered])

    assert match.shelf_id == 2
    assert match.face == 1
    assert match.anchor_local_y == pytest.approx(0.4298647)
    assert match.outward_distance_m == pytest.approx(0.6344536)


def test_camera_waypoint_shifts_a_laterally_and_keeps_its_coarse_normal():
    anchor = waypoint("SHELF-A", 1.945, 1.323, math.pi)
    registered = {
        "id": 2,
        "world_x": 0.5706867418984594,
        "world_y": 0.8047947063020753,
        "yaw": 0.06356114806348771,
        "shelf_width": 0.77,
        "shelf_length": 1.83,
    }
    target_slot = {
        "slot_id": "2-1-2-70",
        "shelf_id": 2,
        "face": 1,
        "level": 2,
        "world_x": 1.1524,
        "world_y": 1.5432,
        "world_z": 0.84,
    }
    camera = WristCameraCalibration(
        2,
        (0.203, -0.174, 0.805),
        (0.661, -0.591, 0.313, -0.341),
        480,
        270,
        244.53268432617188,
        243.911376953125,
        242.3506622314453,
        137.34237670898438,
    )

    result = camera_aware_waypoint_from_anchor(
        target_slot,
        registered,
        anchor,
        (1.845, 1.323, math.pi),
        camera,
    )

    assert result.waypoint.x == pytest.approx(anchor.x, abs=1e-6)
    assert result.waypoint.y == pytest.approx(1.388, abs=0.01)
    assert result.aligned_x == pytest.approx(1.845, abs=1e-6)
    assert result.lateral_shift_m == pytest.approx(-0.065, abs=0.01)
    assert 0.15 * camera.image_width <= result.pixel_u <= 0.85 * camera.image_width
    assert 0.15 * camera.image_height <= result.pixel_v <= 0.85 * camera.image_height


def test_camera_waypoint_rejects_an_unvalidated_shelf_face():
    anchor = waypoint("SHELF-A", 1.945, 1.323, math.pi)
    registered = {
        "id": 2,
        "world_x": 0.5706867418984594,
        "world_y": 0.8047947063020753,
        "yaw": 0.06356114806348771,
        "shelf_width": 0.77,
        "shelf_length": 1.83,
    }
    camera = WristCameraCalibration(
        2, (0.203, -0.174, 0.805), (0.661, -0.591, 0.313, -0.341),
        480, 270, 244.5, 243.9, 242.3, 137.3,
    )
    target_slot = {
        "slot_id": "2-0-2-70", "shelf_id": 2, "face": 0, "level": 2,
        "world_x": 0.70, "world_y": 1.50, "world_z": 0.84,
    }

    with pytest.raises(ValueError, match="not on the A anchor shelf face"):
        camera_aware_waypoint_from_anchor(
            target_slot, registered, anchor, (1.845, 1.323, math.pi), camera
        )


def test_level4_observation_pose_centres_test9_db_level3_cup():
    anchor = waypoint("SHELF-A", 1.945, 1.323, math.pi)
    registered = {
        "id": 2,
        "world_x": 0.5706867418984594,
        "world_y": 0.8047947063020753,
        "yaw": 0.06356114806348771,
        "shelf_width": 0.77,
        "shelf_length": 1.83,
    }
    target_slot = {
        "slot_id": "2-1-3-100",
        "shelf_id": 2,
        "face": 1,
        "level": 3,
        "world_x": 1.1334012537941174,
        "world_y": 1.8426331465720582,
        "world_z": 1.1045,
    }
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "right_wrist_shelf_camera.yaml"
    )
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    calibration = document["levels"][3]
    image = calibration["image"]
    camera = WristCameraCalibration(
        level=3,
        translation_xyz_m=tuple(calibration["translation_xyz_m"]),
        quaternion_xyzw=tuple(calibration["quaternion_xyzw"]),
        image_width=image["width"],
        image_height=image["height"],
        fx=image["fx"],
        fy=image["fy"],
        cx=image["cx"],
        cy=image["cy"],
        base_height_m=document["base_height_m"],
        safe_roi_fraction=calibration["safe_roi_fraction"],
        arm_level=calibration["arm_level"],
    )

    result = camera_aware_waypoint_from_anchor(
        target_slot,
        registered,
        anchor,
        (1.7980846187490276, 1.2364545168923673, -3.1298734942977875),
        camera,
    )

    assert camera.arm_level == 4
    assert result.lateral_shift_m == pytest.approx(-0.223, abs=0.01)
    assert result.pixel_u == pytest.approx(239.0, abs=2.0)
    assert 0.10 * camera.image_height <= result.pixel_v <= 0.90 * camera.image_height
    assert result.camera_depth_m == pytest.approx(0.274, abs=0.01)


def test_indexes_one_anchor_for_each_registered_shelf_face():
    registered = shelf()
    anchors = [
        waypoint("FACE-0", -0.65, 0.8, 0.0),
        waypoint("FACE-1", 1.65, 0.8, math.pi),
    ]

    indexed = index_shelf_face_anchors(anchors, [registered])

    assert set(indexed) == {(1, 0), (1, 1)}
    assert indexed[(1, 0)][0].waypoint_id == "FACE-0"
    assert indexed[(1, 1)][0].waypoint_id == "FACE-1"


def test_rejects_multiple_anchors_for_one_shelf_face():
    with pytest.raises(ValueError, match="multiple anchors"):
        index_shelf_face_anchors(
            [
                waypoint("FACE-0-A", -0.65, 0.6, 0.0),
                waypoint("FACE-0-B", -0.65, 1.0, 0.0),
            ],
            [shelf()],
        )


def test_parses_two_line_ticket_and_validates_quantities():
    order = parse_competition_order(
        '{"items":[{"sku":"杯子","quantity":1},'
        '{"sku":"舒克牙膏海盐薄荷","quantity":2}]}',
        known_skus=("杯子", "舒克牙膏海盐薄荷"),
    )

    assert [(item.sku, item.quantity) for item in order] == [
        ("杯子", 1),
        ("舒克牙膏海盐薄荷", 2),
    ]


def test_rejects_unknown_or_duplicate_ticket_sku():
    with pytest.raises(ValueError, match="not in the current scene"):
        parse_competition_order(
            {"items": [{"sku": "不存在", "quantity": 1}]},
            known_skus=("杯子",),
        )
    with pytest.raises(ValueError, match="duplicated"):
        parse_competition_order(
            {"items": [
                {"sku": "杯子", "quantity": 1},
                {"sku": "杯子", "quantity": 1},
            ]},
            known_skus=("杯子",),
        )
