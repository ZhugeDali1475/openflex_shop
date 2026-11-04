import cv2
import numpy as np

from .rgbd import CameraIntrinsics


def product_points(
    rgb: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    depth_scale: float = 0.001,
    min_depth_m: float = 0.07,
    max_depth_m: float = 1.5,
    max_depth_spread_m: float = 0.25,
    stride: int = 1,
) -> np.ndarray:
    height, width = rgb.shape[:2]
    if depth.shape[:2] != (height, width):
        depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
    if mask.shape[:2] != (height, width):
        mask = cv2.resize(
            mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST
        ) > 0
    intrinsics = intrinsics.scaled(width, height)

    z = (
        depth.astype(np.float32) * float(depth_scale)
        if np.issubdtype(depth.dtype, np.integer)
        else depth.astype(np.float32)
    )
    valid = np.asarray(mask, dtype=bool) & np.isfinite(z)
    valid &= (z >= min_depth_m) & (z <= max_depth_m)
    if stride > 1:
        sampling = np.zeros_like(valid)
        sampling[::stride, ::stride] = True
        valid &= sampling
    if not np.any(valid):
        return np.empty((0, 6), dtype=np.float32)
    if max_depth_spread_m > 0:
        foreground_depth = float(np.percentile(z[valid], 10.0))
        valid &= z <= foreground_depth + max_depth_spread_m
    rows, cols = np.nonzero(valid)
    depth_values = z[rows, cols]
    x = (cols.astype(np.float32) - intrinsics.cx) * depth_values / intrinsics.fx
    y = (rows.astype(np.float32) - intrinsics.cy) * depth_values / intrinsics.fy
    colors = rgb[rows, cols].astype(np.float32)
    return np.column_stack((x, y, depth_values, colors)).astype(
        np.float32, copy=False
    )


def product_instance_graspability(
    mask: np.ndarray,
    points: np.ndarray,
    box,
    image_shape,
    *,
    detection_confidence: float,
    segmentation_score: float,
    min_points: int = 100,
) -> dict:
    binary = np.asarray(mask, dtype=bool)
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("instance points must have shape (N, >=3)")
    xyz = xyz[np.isfinite(xyz[:, :3]).all(axis=1), :3]
    mask_pixels = int(np.count_nonzero(binary))
    if mask_pixels <= 0 or len(xyz) == 0:
        return {
            "score": 0.0,
            "valid": False,
            "reason": "no valid depth points",
            "point_count": int(len(xyz)),
            "mask_pixels": mask_pixels,
        }

    height, width = image_shape[:2]
    x1, y1, x2, y2 = (int(value) for value in box)
    center = np.median(xyz, axis=0)
    depth = max(float(center[2]), 1e-6)
    lateral_ratio = float(np.linalg.norm(center[:2]) / depth)
    optical_axis_score = float(np.clip(1.0 - lateral_ratio / 0.45, 0.0, 1.0))
    valid_depth_ratio = float(np.clip(len(xyz) / mask_pixels, 0.0, 1.0))
    point_support_score = float(
        np.clip(len(xyz) / max(4 * min_points, 1), 0.0, 1.0)
    )
    edge_clearance = min(x1, y1, width - x2, height - y2)
    desired_clearance = max(1.0, 0.08 * min(width, height))
    edge_score = float(np.clip(edge_clearance / desired_clearance, 0.0, 1.0))
    depth_values = xyz[:, 2]
    depth_spread = float(
        np.quantile(depth_values, 0.90) - np.quantile(depth_values, 0.10)
    )
    compact_depth_score = float(np.clip(1.0 - depth_spread / 0.20, 0.0, 1.0))
    detection_score = float(np.clip(detection_confidence, 0.0, 1.0))
    sam_score = float(np.clip(segmentation_score, 0.0, 1.0))
    score = (
        0.20 * sam_score
        + 0.15 * detection_score
        + 0.20 * valid_depth_ratio
        + 0.20 * optical_axis_score
        + 0.10 * edge_score
        + 0.10 * point_support_score
        + 0.05 * compact_depth_score
    )
    valid = len(xyz) >= int(min_points)
    return {
        "score": float(score if valid else score * 0.25),
        "valid": bool(valid),
        "reason": "ok" if valid else f"only {len(xyz)} depth points",
        "point_count": int(len(xyz)),
        "mask_pixels": mask_pixels,
        "center_camera_m": center.tolist(),
        "valid_depth_ratio": valid_depth_ratio,
        "optical_axis_score": optical_axis_score,
        "edge_score": edge_score,
        "point_support_score": point_support_score,
        "compact_depth_score": compact_depth_score,
        "depth_spread_m": depth_spread,
        "detection_score": detection_score,
        "segmentation_score": sam_score,
    }


def semantic_instance_selection_key(
    detection_confidence: float,
    segmentation_score: float,
    graspability_score: float,
) -> tuple[float, float, float]:
    values = (
        float(detection_confidence),
        float(segmentation_score),
        float(graspability_score),
    )
    if not all(np.isfinite(value) for value in values):
        raise ValueError("instance selection scores must be finite")
    return tuple(float(np.clip(value, 0.0, 1.0)) for value in values)
