from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GraspEstimate:
    valid: bool
    reason: str
    center: np.ndarray
    pregrasp: np.ndarray
    rotation: np.ndarray
    quaternion_xyzw: np.ndarray
    approach: np.ndarray
    closing_axis: np.ndarray
    grasp_width: float
    required_width: float
    object_spans: np.ndarray
    obstacle_count: int
    collision_checked: bool
    score: float
    backend: str = "center"
    network_score: float | None = None
    candidate_count: int = 0
    constrained_candidate_count: int = 0
    candidate_center_distance: float | None = None
    candidate_approach_alignment: float | None = None
    surface_center: np.ndarray | None = None
    network_grasp_width: float | None = None


@dataclass(frozen=True)
class GraspModeProfile:
    name: str
    approach_axis_parameter: str
    approach_frame_parameter: str
    closing_axis_parameter: str | None
    closing_axis_frame_parameter: str | None
    orientation_policy: str
    fit_method: str


def grasp_mode_profile(value: str) -> GraspModeProfile:
    mode = str(value).strip().lower()
    profiles = {
        "shelf": GraspModeProfile(
            name="shelf",
            approach_axis_parameter="approach_axis",
            approach_frame_parameter="approach_frame",
            closing_axis_parameter="closing_axis",
            closing_axis_frame_parameter="closing_axis_frame",
            orientation_policy="shelf_front_approach_horizontal_closing_axis",
            fit_method="shelf_center_horizontal_span_graspnet",
        ),
        "basket": GraspModeProfile(
            name="basket",
            approach_axis_parameter="basket_approach_axis",
            approach_frame_parameter="basket_approach_frame",
            closing_axis_parameter=None,
            closing_axis_frame_parameter=None,
            orientation_policy="basket_top_down_cloud_box_minor_axis",
            fit_method="basket_center_cloud_box_minor_axis",
        ),
    }
    try:
        return profiles[mode]
    except KeyError as error:
        raise ValueError("grasp_mode must be 'shelf' or 'basket'") from error


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("zero-length direction vector")
    return np.asarray(vector, dtype=np.float64) / norm


def align_symmetric_axis(reference: np.ndarray, current: np.ndarray) -> np.ndarray:
    reference = _normalize(np.asarray(reference, dtype=np.float64))
    current = _normalize(np.asarray(current, dtype=np.float64))
    return reference if float(np.dot(reference, current)) >= 0.0 else -reference


def robust_cloud_center(points: np.ndarray, min_points: int = 100):
    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if points.shape[0] < min_points:
        raise ValueError(f"product cloud has only {points.shape[0]} finite points; need {min_points}")

    low = np.quantile(points, 0.01, axis=0)
    high = np.quantile(points, 0.99, axis=0)
    trimmed = points[np.all((points >= low) & (points <= high), axis=1)]
    if trimmed.shape[0] < min_points:
        trimmed = points
    return points, trimmed, np.median(trimmed, axis=0)


def _basis_from_closing_axis(approach: np.ndarray, closing_axis: np.ndarray):
    closing = np.asarray(closing_axis, dtype=np.float64)
    closing = closing - float(np.dot(closing, approach)) * approach
    closing = _normalize(closing)
    height_axis = _normalize(np.cross(closing, approach))
    rotation = np.column_stack((height_axis, closing, approach))
    return rotation, closing


def _straight_grasp_basis(points: np.ndarray, approach: np.ndarray):
    reference = np.array([0.0, 1.0, 0.0])
    if abs(float(np.dot(reference, approach))) > 0.9:
        reference = np.array([1.0, 0.0, 0.0])
    plane_u = _normalize(np.cross(approach, reference))
    plane_v = _normalize(np.cross(approach, plane_u))
    planar = np.column_stack((points @ plane_u, points @ plane_v))
    covariance = np.cov(planar, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    minor = eigenvectors[:, int(np.argmin(eigenvalues))]
    closing = _normalize(minor[0] * plane_u + minor[1] * plane_v)
    sign_index = int(np.argmax(np.abs(closing)))
    if closing[sign_index] < 0:
        closing = -closing
    rotation, closing = _basis_from_closing_axis(approach, closing)
    return rotation, closing, plane_u, plane_v


def _path_obstacles(
    scene_points: np.ndarray,
    target_points: np.ndarray,
    pregrasp: np.ndarray,
    approach: np.ndarray,
    path_length: float,
    corridor_radius: float,
    object_clearance: float,
) -> int:
    lower = np.quantile(target_points, 0.01, axis=0) - 0.015
    upper = np.quantile(target_points, 0.99, axis=0) + 0.015
    outside_target = np.any((scene_points < lower) | (scene_points > upper), axis=1)
    candidates = scene_points[outside_target]
    offsets = candidates - pregrasp
    progress = offsets @ approach
    radial = np.linalg.norm(offsets - progress[:, None] * approach, axis=1)
    in_path = (progress >= 0.0) & (progress <= max(0.0, path_length - object_clearance))
    return int(np.count_nonzero(in_path & (radial <= corridor_radius)))


def fit_center_straight_grasp(
    points: np.ndarray,
    scene_points: np.ndarray | None = None,
    *,
    approach=(0.0, 0.0, 1.0),
    pregrasp_distance: float = 0.12,
    grasp_depth_offset: float = 0.0,
    gripper_max_width: float = 0.088,
    enforce_gripper_width: bool = True,
    width_margin: float = 0.008,
    corridor_radius: float = 0.055,
    object_clearance: float = 0.04,
    collision_point_limit: int = 20,
    min_points: int = 100,
    closing_axis=None,
    closing_axis_reference=None,
    grasp_center=None,
    grasp_width=None,
    backend: str = "center",
    network_score: float | None = None,
    network_grasp_width: float | None = None,
    candidate_count: int = 0,
    constrained_candidate_count: int = 0,
    candidate_center_distance: float | None = None,
    candidate_approach_alignment: float | None = None,
) -> GraspEstimate:
    points, trimmed, cloud_center = robust_cloud_center(points, min_points=min_points)
    surface_center = cloud_center if grasp_center is None else np.asarray(grasp_center, dtype=np.float64)
    if surface_center.shape != (3,) or not np.isfinite(surface_center).all():
        raise ValueError("grasp_center must contain three finite values")
    approach = _normalize(np.asarray(approach, dtype=np.float64))
    depth_offset = float(grasp_depth_offset)
    if not np.isfinite(depth_offset) or depth_offset < 0.0:
        raise ValueError("grasp_depth_offset must be finite and non-negative")
    center = surface_center + approach * depth_offset
    if closing_axis is None:
        rotation, closing, _, _ = _straight_grasp_basis(trimmed - center, approach)
        if closing_axis_reference is not None:
            closing = align_symmetric_axis(closing, closing_axis_reference)
            rotation, closing = _basis_from_closing_axis(approach, closing)
    else:
        rotation, closing = _basis_from_closing_axis(approach, closing_axis)

    closing_projection = trimmed @ closing
    orthogonal_projection = trimmed @ rotation[:, 0]
    approach_projection = trimmed @ approach
    closing_span = float(np.quantile(closing_projection, 0.98) - np.quantile(closing_projection, 0.02))
    selected_width = closing_span if grasp_width is None else float(grasp_width)
    if not np.isfinite(selected_width) or selected_width <= 0.0:
        raise ValueError("grasp_width must be finite and positive")
    required_width = selected_width + width_margin
    spans = np.array([
        float(np.quantile(orthogonal_projection, 0.98) - np.quantile(orthogonal_projection, 0.02)),
        closing_span,
        float(np.quantile(approach_projection, 0.98) - np.quantile(approach_projection, 0.02)),
    ])
    pregrasp = surface_center - approach * pregrasp_distance

    collision_checked = scene_points is not None and len(scene_points) > 0
    obstacle_count = 0
    if collision_checked:
        scene = np.asarray(scene_points, dtype=np.float64)
        scene = scene[np.isfinite(scene).all(axis=1)]
        obstacle_count = _path_obstacles(
            scene,
            points,
            pregrasp,
            approach,
            float(pregrasp_distance) + depth_offset,
            corridor_radius,
            object_clearance,
        )

    reasons = []
    if enforce_gripper_width and required_width > gripper_max_width:
        reasons.append(f"required gripper width {required_width:.3f} m exceeds {gripper_max_width:.3f} m")
    if collision_checked and obstacle_count > collision_point_limit:
        reasons.append(f"straight path contains {obstacle_count} obstacle points")
    if pregrasp[2] <= 0.03 and approach[2] > 0.5:
        reasons.append("pregrasp pose is behind or too close to the camera")
    valid = not reasons
    score = max(0.0, 1.0 - required_width / max(gripper_max_width, 1e-6) * 0.25)
    if collision_checked:
        score *= max(0.0, 1.0 - obstacle_count / max(collision_point_limit + 1, 1))
    if network_score is not None:
        score *= float(np.clip(network_score, 0.0, 1.0))
    return GraspEstimate(
        valid=valid,
        reason="ok" if valid else "; ".join(reasons),
        center=center,
        pregrasp=pregrasp,
        rotation=rotation,
        quaternion_xyzw=rotation_matrix_to_quaternion(rotation),
        approach=approach,
        closing_axis=closing,
        grasp_width=selected_width,
        required_width=required_width,
        object_spans=spans,
        obstacle_count=obstacle_count,
        collision_checked=collision_checked,
        score=score,
        backend=backend,
        network_score=network_score,
        candidate_count=candidate_count,
        constrained_candidate_count=constrained_candidate_count,
        candidate_center_distance=candidate_center_distance,
        candidate_approach_alignment=candidate_approach_alignment,
        surface_center=surface_center,
        network_grasp_width=network_grasp_width,
    )
