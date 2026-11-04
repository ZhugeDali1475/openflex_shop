"""Geometry helpers for selecting measured supermarket approach waypoints."""

from __future__ import annotations

import math
import json
from dataclasses import dataclass, replace
from typing import Iterable, Mapping, Sequence

from .benchmark import BenchmarkWaypoint
from .model import normalize_angle


DEFAULT_MAX_APPROACH_YAW_ERROR_RAD = math.radians(20.0)


@dataclass(frozen=True)
class ApproachWaypointCandidate:
    waypoint: BenchmarkWaypoint
    distance_m: float
    outward_distance_m: float
    yaw_error_rad: float
    desired_yaw: float


@dataclass(frozen=True)
class AnchorFaceMatch:
    shelf_id: int
    face: int
    anchor_local_y: float
    outward_distance_m: float
    yaw_error_rad: float


@dataclass(frozen=True)
class WristCameraCalibration:
    level: int
    translation_xyz_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    image_width: int
    image_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    base_height_m: float = 0.17
    safe_roi_fraction: float = 0.15
    arm_level: int | None = None


@dataclass(frozen=True)
class CameraAwareWaypoint:
    waypoint: BenchmarkWaypoint
    aligned_x: float
    aligned_y: float
    aligned_yaw: float
    lateral_shift_m: float
    pixel_u: float
    pixel_v: float
    camera_depth_m: float
    base_local_y_on_shelf_m: float


@dataclass(frozen=True)
class CompetitionOrderItem:
    sku: str
    quantity: int


@dataclass(frozen=True)
class StartClearanceProgress:
    forward_distance_m: float
    lateral_drift_m: float
    yaw_error_rad: float


def measure_start_clearance(
    start_pose: tuple[float, float, float],
    current_pose: tuple[float, float, float],
) -> StartClearanceProgress:
    """Measure motion in the chassis frame captured at physical START."""
    start_x, start_y, start_yaw = (float(value) for value in start_pose)
    current_x, current_y, current_yaw = (float(value) for value in current_pose)
    values = (start_x, start_y, start_yaw, current_x, current_y, current_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("start clearance poses must contain only finite values")

    delta_x = current_x - start_x
    delta_y = current_y - start_y
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    return StartClearanceProgress(
        forward_distance_m=delta_x * cosine + delta_y * sine,
        lateral_drift_m=-delta_x * sine + delta_y * cosine,
        yaw_error_rad=normalize_angle(current_yaw - start_yaw),
    )


def parse_competition_order(
    payload: str | Mapping[str, object] | Sequence[Mapping[str, object]],
    *,
    known_skus: Iterable[str] = (),
    max_lines: int = 2,
    max_quantity: int = 9,
) -> tuple[CompetitionOrderItem, ...]:
    """Parse the shopping-ticket JSON contract into validated order lines."""
    if isinstance(payload, str):
        try:
            document = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError(f"shopping ticket is not valid JSON: {exc.msg}") from exc
    else:
        document = payload
    if isinstance(document, Mapping):
        raw_items = document.get("items")
    else:
        raw_items = document
    if not isinstance(raw_items, Sequence) or isinstance(raw_items, (str, bytes)):
        raise ValueError("shopping ticket must contain an items array")
    if not 1 <= len(raw_items) <= int(max_lines):
        raise ValueError(f"shopping ticket must contain 1 to {max_lines} order lines")
    known = {str(value).strip() for value in known_skus if str(value).strip()}
    result = []
    seen = set()
    for raw in raw_items:
        if not isinstance(raw, Mapping):
            raise ValueError("each shopping-ticket item must be an object")
        sku = str(raw.get("sku", "")).strip()
        if not sku:
            raise ValueError("shopping-ticket SKU cannot be empty")
        if known and sku not in known:
            raise ValueError(f"shopping-ticket SKU is not in the current scene: {sku}")
        if sku in seen:
            raise ValueError(f"shopping-ticket SKU is duplicated: {sku}")
        try:
            quantity = int(raw.get("quantity", 1))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"shopping-ticket quantity is invalid for {sku}") from exc
        if not 1 <= quantity <= int(max_quantity):
            raise ValueError(
                f"shopping-ticket quantity for {sku} must be within 1..{max_quantity}"
            )
        seen.add(sku)
        result.append(CompetitionOrderItem(sku=sku, quantity=quantity))
    return tuple(result)


def index_shelf_face_anchors(
    anchors: Iterable[BenchmarkWaypoint],
    shelves: Iterable[Mapping[str, object]],
) -> dict[tuple[int, int], tuple[BenchmarkWaypoint, AnchorFaceMatch]]:
    """Match exactly one shelf-approach anchor to each registered shelf face."""
    shelf_values = tuple(shelves)
    result = {}
    for anchor in anchors:
        if anchor.role != "shelf_approach":
            continue
        match = match_anchor_shelf_face(anchor, shelf_values)
        key = (match.shelf_id, match.face)
        if key in result:
            other = result[key][0]
            raise ValueError(
                f"shelf {key[0]} face={key[1]} has multiple anchors: "
                f"{other.waypoint_id}, {anchor.waypoint_id}"
            )
        result[key] = (anchor, match)
    return result


def compatible_approach_waypoints(
    slot: Mapping[str, object],
    shelf: Mapping[str, object],
    waypoints: Iterable[BenchmarkWaypoint],
    *,
    max_yaw_error_rad: float = DEFAULT_MAX_APPROACH_YAW_ERROR_RAD,
) -> tuple[ApproachWaypointCandidate, ...]:
    """Return shelf-approach waypoints on the requested face, nearest first."""
    face = int(slot["face"])
    if face not in (0, 1):
        raise ValueError(f"unsupported shelf face {face}; expected 0 or 1")
    if not 0.0 <= float(max_yaw_error_rad) <= math.pi:
        raise ValueError("max_yaw_error_rad must be within [0, pi]")

    shelf_x = float(shelf["world_x"])
    shelf_y = float(shelf["world_y"])
    shelf_yaw = float(shelf["yaw"])
    shelf_width = float(shelf["shelf_width"])
    slot_local_y = float(slot["local_y"])
    slot_world_x = float(slot["world_x"])
    slot_world_y = float(slot["world_y"])
    values = (
        shelf_x,
        shelf_y,
        shelf_yaw,
        shelf_width,
        slot_local_y,
        slot_world_x,
        slot_world_y,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("shelf and slot geometry must contain only finite values")
    if shelf_width <= 0.0:
        raise ValueError("shelf_width must be greater than zero")

    cosine = math.cos(shelf_yaw)
    sine = math.sin(shelf_yaw)
    face_local_x = 0.0 if face == 0 else shelf_width
    face_world_x = shelf_x + face_local_x * cosine - slot_local_y * sine
    face_world_y = shelf_y + face_local_x * sine + slot_local_y * cosine
    outward_sign = -1.0 if face == 0 else 1.0
    outward_x = outward_sign * cosine
    outward_y = outward_sign * sine
    desired_yaw = normalize_angle(shelf_yaw + (math.pi if face == 1 else 0.0))

    candidates = []
    for waypoint in waypoints:
        if waypoint.role != "shelf_approach":
            continue
        waypoint_values = (
            float(waypoint.x),
            float(waypoint.y),
            float(waypoint.yaw),
        )
        if not all(math.isfinite(value) for value in waypoint_values):
            continue
        delta_x = float(waypoint.x) - face_world_x
        delta_y = float(waypoint.y) - face_world_y
        outward_distance = delta_x * outward_x + delta_y * outward_y
        if outward_distance <= 0.0:
            continue
        yaw_error = abs(normalize_angle(float(waypoint.yaw) - desired_yaw))
        if yaw_error > max_yaw_error_rad:
            continue
        candidates.append(
            ApproachWaypointCandidate(
                waypoint=waypoint,
                distance_m=math.hypot(
                    float(waypoint.x) - slot_world_x,
                    float(waypoint.y) - slot_world_y,
                ),
                outward_distance_m=outward_distance,
                yaw_error_rad=yaw_error,
                desired_yaw=desired_yaw,
            )
        )

    candidates.sort(
        key=lambda candidate: (
            candidate.distance_m,
            candidate.yaw_error_rad,
            candidate.waypoint.waypoint_id,
        )
    )
    return tuple(candidates)


def match_anchor_shelf_face(
    anchor: BenchmarkWaypoint,
    shelves: Iterable[Mapping[str, object]],
    *,
    max_yaw_error_rad: float = DEFAULT_MAX_APPROACH_YAW_ERROR_RAD,
    lateral_margin_m: float = 0.20,
) -> AnchorFaceMatch:
    """Identify the one registered shelf face approached by a taught anchor."""
    matches: list[tuple[float, float, int, AnchorFaceMatch]] = []
    for shelf in shelves:
        shelf_id = int(shelf["id"])
        shelf_x = float(shelf["world_x"])
        shelf_y = float(shelf["world_y"])
        shelf_yaw = float(shelf["yaw"])
        shelf_width = float(shelf["shelf_width"])
        shelf_length = float(shelf["shelf_length"])
        cosine = math.cos(shelf_yaw)
        sine = math.sin(shelf_yaw)
        delta_x = float(anchor.x) - shelf_x
        delta_y = float(anchor.y) - shelf_y
        local_x = delta_x * cosine + delta_y * sine
        local_y = -delta_x * sine + delta_y * cosine
        if not -lateral_margin_m <= local_y <= shelf_length + lateral_margin_m:
            continue
        for face in (0, 1):
            face_x = 0.0 if face == 0 else shelf_width
            outward_sign = -1.0 if face == 0 else 1.0
            outward_distance = outward_sign * (local_x - face_x)
            if outward_distance <= 0.0:
                continue
            desired_yaw = normalize_angle(
                shelf_yaw + (math.pi if face == 1 else 0.0)
            )
            yaw_error = abs(normalize_angle(float(anchor.yaw) - desired_yaw))
            if yaw_error > max_yaw_error_rad:
                continue
            value = AnchorFaceMatch(
                shelf_id=shelf_id,
                face=face,
                anchor_local_y=local_y,
                outward_distance_m=outward_distance,
                yaw_error_rad=yaw_error,
            )
            matches.append((outward_distance, yaw_error, shelf_id, value))
    if not matches:
        raise ValueError("A anchor does not match a registered shelf face")
    matches.sort(key=lambda item: item[:3])
    return matches[0][3]


def _rotation_matrix_xyzw(
    quaternion: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        raise ValueError("camera quaternion must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def camera_aware_waypoint_from_anchor(
    slot: Mapping[str, object],
    shelf: Mapping[str, object],
    anchor: BenchmarkWaypoint,
    aligned_anchor_pose: tuple[float, float, float],
    camera: WristCameraCalibration,
    *,
    shelf_edge_margin_m: float = 0.45,
    max_lateral_shift_m: float = 1.20,
) -> CameraAwareWaypoint:
    """Shift a proven A anchor so the selected item is centred in the wrist camera.

    The aligned A pose owns shelf-normal distance. Only the base lateral axis is
    translated, so the generated coarse goal reaches the same front-surface
    fine-alignment geometry as A.
    """
    if int(slot["shelf_id"]) != int(shelf["id"]):
        raise ValueError("slot and shelf IDs do not match")
    if int(slot["level"]) != int(camera.level):
        raise ValueError("camera calibration does not match the slot level")
    match = match_anchor_shelf_face(anchor, (shelf,))
    if int(slot["face"]) != match.face:
        raise ValueError("slot is not on the A anchor shelf face")

    aligned_x, aligned_y, aligned_yaw = (
        float(value) for value in aligned_anchor_pose
    )
    if abs(normalize_angle(aligned_yaw - float(anchor.yaw))) > math.radians(5.0):
        raise ValueError("aligned A heading is inconsistent with the taught anchor")

    rotation = _rotation_matrix_xyzw(camera.quaternion_xyzw)
    camera_x, camera_y, camera_z = camera.translation_xyz_m
    optical_forward = (rotation[0][2], rotation[1][2], rotation[2][2])
    if optical_forward[2] >= -1e-4:
        raise ValueError("wrist camera calibration must look downward toward the shelf")
    item_z_in_base = float(slot["world_z"]) - float(camera.base_height_m)
    ray_distance = (item_z_in_base - camera_z) / optical_forward[2]
    if ray_distance <= 0.0:
        raise ValueError("item height is behind the calibrated camera view ray")
    desired_item_base_y = camera_y + ray_distance * optical_forward[1]

    yaw = float(anchor.yaw)
    base_y_axis = (-math.sin(yaw), math.cos(yaw))
    item_x = float(slot["world_x"])
    item_y = float(slot["world_y"])
    current_item_lateral = (
        (item_x - aligned_x) * base_y_axis[0]
        + (item_y - aligned_y) * base_y_axis[1]
    )
    lateral_shift = current_item_lateral - desired_item_base_y
    if abs(lateral_shift) > max_lateral_shift_m:
        raise ValueError("item needs a lateral shelf traverse beyond the validated limit")

    final_x = aligned_x + lateral_shift * base_y_axis[0]
    final_y = aligned_y + lateral_shift * base_y_axis[1]
    coarse_x = float(anchor.x) + lateral_shift * base_y_axis[0]
    coarse_y = float(anchor.y) + lateral_shift * base_y_axis[1]

    shelf_yaw = float(shelf["yaw"])
    shelf_dx = final_x - float(shelf["world_x"])
    shelf_dy = final_y - float(shelf["world_y"])
    base_local_y = -math.sin(shelf_yaw) * shelf_dx + math.cos(shelf_yaw) * shelf_dy
    shelf_length = float(shelf["shelf_length"])
    if not shelf_edge_margin_m <= base_local_y <= shelf_length - shelf_edge_margin_m:
        raise ValueError("camera-centred base pose is too close to a shelf end")

    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    relative_world_x = item_x - final_x
    relative_world_y = item_y - final_y
    point_base = (
        cosine * relative_world_x + sine * relative_world_y,
        -sine * relative_world_x + cosine * relative_world_y,
        item_z_in_base,
    )
    camera_delta = (
        point_base[0] - camera_x,
        point_base[1] - camera_y,
        point_base[2] - camera_z,
    )
    point_camera = tuple(
        sum(rotation[row][column] * camera_delta[row] for row in range(3))
        for column in range(3)
    )
    depth = point_camera[2]
    if depth <= 0.10:
        raise ValueError("item is outside the calibrated camera depth range")
    pixel_u = camera.fx * point_camera[0] / depth + camera.cx
    pixel_v = camera.fy * point_camera[1] / depth + camera.cy
    margin = float(camera.safe_roi_fraction)
    if not (
        camera.image_width * margin <= pixel_u <= camera.image_width * (1.0 - margin)
        and camera.image_height * margin <= pixel_v <= camera.image_height * (1.0 - margin)
    ):
        raise ValueError("item projects outside the calibrated wrist-camera safe ROI")

    waypoint = replace(
        anchor,
        label=f"{anchor.label} / {slot['slot_id']}",
        x=coarse_x,
        y=coarse_y,
    )
    return CameraAwareWaypoint(
        waypoint=waypoint,
        aligned_x=final_x,
        aligned_y=final_y,
        aligned_yaw=yaw,
        lateral_shift_m=lateral_shift,
        pixel_u=pixel_u,
        pixel_v=pixel_v,
        camera_depth_m=depth,
        base_local_y_on_shelf_m=base_local_y,
    )
