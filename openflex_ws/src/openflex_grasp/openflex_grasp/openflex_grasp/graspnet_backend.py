from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np

from .grasp_geometry import (
    align_symmetric_axis,
    fit_center_straight_grasp,
    robust_cloud_center,
)


@dataclass(frozen=True)
class GraspNetCandidate:
    score: float
    width: float
    center: np.ndarray
    approach: np.ndarray
    closing_axis: np.ndarray
    center_distance: float
    approach_alignment: float
    candidate_count: int
    constrained_candidate_count: int


@dataclass(frozen=True)
class GraspNetShelfFit:
    width: float
    closing_axis: np.ndarray
    network_score: float
    candidate_count: int
    contributor_count: int
    mean_center_distance: float
    mean_approach_alignment: float
    weights: np.ndarray


def _normalize_rows(vectors: np.ndarray):
    norms = np.linalg.norm(vectors, axis=1)
    normalized = np.zeros_like(vectors, dtype=np.float64)
    valid = norms > 1e-9
    normalized[valid] = vectors[valid] / norms[valid, None]
    return normalized, valid


def _candidate_metrics(
    candidates: np.ndarray,
    target_center: np.ndarray,
    *,
    approach=(0.0, 0.0, 1.0),
    max_center_distance: float = 0.06,
    max_approach_angle_deg: float = 25.0,
    max_width: float | None = None,
):
    candidates = np.asarray(candidates, dtype=np.float64)
    if candidates.ndim != 2 or candidates.shape[1] < 16:
        raise ValueError(f"GraspNet candidates must have shape (N, >=16), got {candidates.shape}")
    if candidates.shape[0] == 0:
        raise ValueError("GraspNet produced no candidates")

    target_center = np.asarray(target_center, dtype=np.float64)
    approach = np.asarray(approach, dtype=np.float64).copy()
    approach_norm = float(np.linalg.norm(approach))
    if approach_norm < 1e-9:
        raise ValueError("approach direction must be nonzero")
    approach /= approach_norm
    rotations = candidates[:, 4:13].reshape(-1, 3, 3)
    centers = candidates[:, 13:16]
    candidate_approaches, valid_approaches = _normalize_rows(rotations[:, :, 0])
    closing_axes = rotations[:, :, 1]
    closing_axes = closing_axes - (closing_axes @ approach)[:, None] * approach
    closing_axes, valid_closing = _normalize_rows(closing_axes)

    finite = np.isfinite(candidates[:, :16]).all(axis=1)
    center_distances = np.linalg.norm(centers - target_center, axis=1)
    alignments = candidate_approaches @ approach
    minimum_alignment = float(np.cos(np.deg2rad(max_approach_angle_deg)))
    center_ok = finite & (center_distances <= max_center_distance)
    width_ok = finite & (candidates[:, 1] > 0.0)
    if max_width is not None:
        width_ok &= candidates[:, 1] <= float(max_width)
    approach_ok = finite & valid_approaches & valid_closing & (alignments >= minimum_alignment)
    constrained = center_ok & width_ok & approach_ok
    return {
        "candidates": candidates,
        "approach": approach,
        "centers": centers,
        "candidate_approaches": candidate_approaches,
        "closing_axes": closing_axes,
        "valid_closing": valid_closing,
        "center_distances": center_distances,
        "alignments": alignments,
        "finite": finite,
        "center_ok": center_ok,
        "width_ok": width_ok,
        "approach_ok": approach_ok,
        "constrained": constrained,
    }


def _weighted_median(values: np.ndarray, weights: np.ndarray) -> float:
    order = np.argsort(values)
    sorted_values = values[order]
    sorted_weights = weights[order]
    threshold = float(np.sum(sorted_weights)) * 0.5
    index = int(np.searchsorted(np.cumsum(sorted_weights), threshold, side="left"))
    return float(sorted_values[min(index, sorted_values.size - 1)])


def _fit_shelf_parameters_from_metrics(
    metrics,
    *,
    distance_scale: float,
    approach_angle_scale_deg: float,
    max_width: float | None,
) -> GraspNetShelfFit:
    candidates = metrics["candidates"]
    finite = metrics["finite"]
    valid = finite & metrics["valid_closing"] & (candidates[:, 1] > 0.0)
    if max_width is not None:
        valid &= candidates[:, 1] <= float(max_width)
    if not np.any(valid):
        width_limit = "unbounded" if max_width is None else f"{float(max_width):.3f} m"
        raise ValueError(
            "GraspNet produced no finite candidate width compatible with the robot gripper "
            f"(network width limit={width_limit})"
        )

    distance_sigma = max(0.01, float(distance_scale))
    angle_sigma = max(np.deg2rad(5.0), np.deg2rad(float(approach_angle_scale_deg)))
    distances = metrics["center_distances"]
    absolute_alignments = np.clip(np.abs(metrics["alignments"]), 0.0, 1.0)
    approach_angles = np.arccos(absolute_alignments)
    scores = np.clip(candidates[:, 0], 0.0, 1.0)
    weights = (
        (0.05 + 0.95 * scores)
        * np.exp(-0.5 * np.square(distances / distance_sigma))
        * np.exp(-0.5 * np.square(approach_angles / angle_sigma))
    )
    weights[~valid] = 0.0
    weight_sum = float(np.sum(weights))
    if not math.isfinite(weight_sum) or weight_sum <= 1e-12:
        weights = valid.astype(np.float64)
        weight_sum = float(np.sum(weights))

    approach = metrics["approach"]
    reference = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(reference, approach))) > 0.9:
        reference = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    plane_u = reference - float(np.dot(reference, approach)) * approach
    plane_u /= np.linalg.norm(plane_u)
    plane_v = np.cross(approach, plane_u)
    plane_v /= np.linalg.norm(plane_v)

    closing_axes = metrics["closing_axes"]
    angles = np.arctan2(closing_axes @ plane_v, closing_axes @ plane_u)
    cosine = float(np.sum(weights * np.cos(2.0 * angles)))
    sine = float(np.sum(weights * np.sin(2.0 * angles)))
    if math.hypot(cosine, sine) <= 1e-9 * weight_sum:
        fitted_closing = closing_axes[int(np.argmax(weights))].copy()
    else:
        fitted_angle = 0.5 * math.atan2(sine, cosine)
        fitted_closing = math.cos(fitted_angle) * plane_u + math.sin(fitted_angle) * plane_v
    fitted_closing /= np.linalg.norm(fitted_closing)
    sign_index = int(np.argmax(np.abs(fitted_closing)))
    if fitted_closing[sign_index] < 0.0:
        fitted_closing = -fitted_closing

    valid_indices = np.flatnonzero(valid)
    valid_weights = weights[valid_indices]
    fitted_width = _weighted_median(candidates[valid_indices, 1], valid_weights)
    normalized_weights = weights / max(float(np.max(weights)), 1e-12)
    contributor_count = int(np.count_nonzero(normalized_weights >= 0.05))
    normalized = weights / weight_sum
    return GraspNetShelfFit(
        width=fitted_width,
        closing_axis=fitted_closing,
        network_score=float(np.sum(normalized * scores)),
        candidate_count=int(candidates.shape[0]),
        contributor_count=max(1, contributor_count),
        mean_center_distance=float(np.sum(normalized * distances)),
        mean_approach_alignment=float(np.sum(normalized * absolute_alignments)),
        weights=weights,
    )


def fit_graspnet_shelf_parameters(
    candidates: np.ndarray,
    target_center: np.ndarray,
    *,
    approach=(0.0, 0.0, 1.0),
    distance_scale: float = 0.06,
    approach_angle_scale_deg: float = 25.0,
    max_width: float | None = None,
) -> GraspNetShelfFit:
    """Fit width and roll while fixing the grasp center and requested approach."""
    metrics = _candidate_metrics(
        candidates,
        target_center,
        approach=approach,
        max_center_distance=distance_scale,
        max_approach_angle_deg=approach_angle_scale_deg,
        max_width=max_width,
    )
    return _fit_shelf_parameters_from_metrics(
        metrics,
        distance_scale=distance_scale,
        approach_angle_scale_deg=approach_angle_scale_deg,
        max_width=max_width,
    )


def _json_vector(vector):
    return [float(value) if math.isfinite(float(value)) else None for value in vector]


def summarize_graspnet_candidates(
    candidates: np.ndarray,
    target_center: np.ndarray,
    *,
    approach=(0.0, 0.0, 1.0),
    max_center_distance: float = 0.06,
    max_approach_angle_deg: float = 25.0,
    max_width: float | None = None,
    limit: int = 80,
):
    """Return web/RViz-friendly diagnostics without discarding rejected candidates."""
    metrics = _candidate_metrics(
        candidates,
        target_center,
        approach=approach,
        max_center_distance=max_center_distance,
        max_approach_angle_deg=max_approach_angle_deg,
        max_width=max_width,
    )
    values = metrics["candidates"]
    finite = metrics["finite"]
    center_ok = metrics["center_ok"]
    width_ok = metrics["width_ok"]
    approach_ok = metrics["approach_ok"]
    constrained = metrics["constrained"]
    distances = metrics["center_distances"]
    alignments = metrics["alignments"]
    try:
        shelf_fit = _fit_shelf_parameters_from_metrics(
            metrics,
            distance_scale=max_center_distance,
            approach_angle_scale_deg=max_approach_angle_deg,
            max_width=max_width,
        )
        maximum_weight = max(float(np.max(shelf_fit.weights)), 1e-12)
        fit_weights = shelf_fit.weights / maximum_weight
    except ValueError:
        shelf_fit = None
        fit_weights = np.zeros(values.shape[0], dtype=np.float64)

    order = sorted(
        range(values.shape[0]),
        key=lambda index: (
            -float(fit_weights[index]),
            -float(values[index, 0]) if finite[index] else float("inf"),
            float(distances[index]) if finite[index] else float("inf"),
        ),
    )[: max(1, int(limit))]
    records = []
    for index in order:
        alignment = float(alignments[index])
        angle = (
            float(np.rad2deg(np.arccos(np.clip(alignment, -1.0, 1.0))))
            if finite[index] and math.isfinite(alignment)
            else None
        )
        reasons = []
        if not finite[index]:
            reasons.append("nonfinite")
        if not center_ok[index]:
            reasons.append("center")
        if not approach_ok[index]:
            reasons.append("approach")
        if not width_ok[index]:
            reasons.append("width")
        records.append({
            "index": int(index),
            "score": float(values[index, 0]) if finite[index] else None,
            "width_m": float(values[index, 1]) if finite[index] else None,
            "center": _json_vector(metrics["centers"][index]),
            "approach": _json_vector(metrics["candidate_approaches"][index]),
            "center_distance_m": float(distances[index]) if finite[index] else None,
            "approach_alignment": alignment if math.isfinite(alignment) else None,
            "approach_angle_deg": angle,
            "accepted": bool(constrained[index]),
            "fit_weight": float(fit_weights[index]),
            "fit_contributor": bool(fit_weights[index] >= 0.05),
            "reasons": reasons,
        })
    return {
        "candidate_count": int(values.shape[0]),
        "shown_count": len(records),
        "near_center_count": int(np.count_nonzero(center_ok)),
        "width_feasible_count": int(np.count_nonzero(width_ok)),
        "approach_aligned_count": int(np.count_nonzero(approach_ok)),
        "constrained_count": int(np.count_nonzero(constrained)),
        "fit_contributor_count": 0 if shelf_fit is None else shelf_fit.contributor_count,
        "fitted_width_m": None if shelf_fit is None else shelf_fit.width,
        "fitted_closing_axis": (
            None if shelf_fit is None else _json_vector(shelf_fit.closing_axis)
        ),
        "fit_method": "shelf_center_weighted_graspnet",
        "target_center": _json_vector(target_center),
        "requested_approach": _json_vector(metrics["approach"]),
        "max_center_distance_m": float(max_center_distance),
        "max_approach_angle_deg": float(max_approach_angle_deg),
        "max_width_m": None if max_width is None else float(max_width),
        "candidates": records,
    }


def select_straight_grasp_candidate(
    candidates: np.ndarray,
    target_center: np.ndarray,
    *,
    approach=(0.0, 0.0, 1.0),
    max_center_distance: float = 0.06,
    max_approach_angle_deg: float = 25.0,
    max_width: float | None = None,
) -> GraspNetCandidate:
    metrics = _candidate_metrics(
        candidates,
        target_center,
        approach=approach,
        max_center_distance=max_center_distance,
        max_approach_angle_deg=max_approach_angle_deg,
        max_width=max_width,
    )
    candidates = metrics["candidates"]
    centers = metrics["centers"]
    candidate_approaches = metrics["candidate_approaches"]
    closing_axes = metrics["closing_axes"]
    center_distances = metrics["center_distances"]
    alignments = metrics["alignments"]
    constrained = metrics["constrained"]
    indices = np.flatnonzero(constrained)
    if indices.size == 0:
        raise ValueError(
            "no GraspNet candidate is both near the product center and aligned with the "
            f"requested approach direction (total={candidates.shape[0]}, "
            f"near_center={np.count_nonzero(metrics['center_ok'])}, "
            f"aligned={np.count_nonzero(metrics['approach_ok'])}, "
            f"width_feasible={np.count_nonzero(metrics['width_ok'])})"
        )

    best = int(indices[np.lexsort((center_distances[indices], -candidates[indices, 0]))[0]])
    return GraspNetCandidate(
        score=float(candidates[best, 0]),
        width=float(candidates[best, 1]),
        center=centers[best].copy(),
        approach=candidate_approaches[best].copy(),
        closing_axis=closing_axes[best].copy(),
        center_distance=float(center_distances[best]),
        approach_alignment=float(alignments[best]),
        candidate_count=int(candidates.shape[0]),
        constrained_candidate_count=int(indices.size),
    )


def fit_graspnet_straight_grasp(
    target_points: np.ndarray,
    scene_points: np.ndarray | None,
    candidates: np.ndarray,
    *,
    approach=(0.0, 0.0, 1.0),
    closing_axis=None,
    closing_axis_reference=None,
    use_cloud_minor_axis: bool = False,
    candidate_center_tolerance: float = 0.06,
    candidate_approach_max_angle_deg: float = 25.0,
    **fit_kwargs,
):
    _, _, center = robust_cloud_center(
        target_points,
        min_points=int(fit_kwargs.get("min_points", 100)),
    )
    fitted = fit_graspnet_shelf_parameters(
        candidates,
        center,
        approach=approach,
        distance_scale=candidate_center_tolerance,
        approach_angle_scale_deg=candidate_approach_max_angle_deg,
        max_width=None,
    )
    selected_closing_axis = None
    if not use_cloud_minor_axis:
        selected_closing_axis = (
            fitted.closing_axis
            if closing_axis is None
            else np.asarray(closing_axis, dtype=np.float64)
        )
    if selected_closing_axis is not None and closing_axis_reference is not None:
        selected_closing_axis = align_symmetric_axis(
            selected_closing_axis,
            np.asarray(closing_axis_reference, dtype=np.float64),
        )
    return fit_center_straight_grasp(
        target_points,
        scene_points,
        # The segmented cloud fixes center and robot opening width. Basket mode
        # also derives orientation from its planar minor axis; GraspNet then
        # supplies confidence and width diagnostics only.
        approach=approach,
        closing_axis=selected_closing_axis,
        closing_axis_reference=(
            closing_axis_reference if use_cloud_minor_axis else None
        ),
        grasp_center=center,
        grasp_width=None,
        backend="graspnet",
        network_score=fitted.network_score,
        network_grasp_width=fitted.width,
        candidate_count=fitted.candidate_count,
        constrained_candidate_count=fitted.contributor_count,
        candidate_center_distance=fitted.mean_center_distance,
        candidate_approach_alignment=fitted.mean_approach_alignment,
        **fit_kwargs,
    )


class GraspNetBackend:
    def __init__(
        self,
        repo_path: str,
        checkpoint_path: str,
        *,
        num_point: int = 20000,
        num_view: int = 300,
        device: str = "cuda:0",
        seed: int = 17,
    ):
        self.repo_path = Path(repo_path).expanduser().resolve()
        self.checkpoint_path = Path(checkpoint_path).expanduser().resolve()
        if not self.repo_path.is_dir():
            raise FileNotFoundError(f"GraspNet repository not found: {self.repo_path}")
        if not self.checkpoint_path.is_file():
            raise FileNotFoundError(f"GraspNet checkpoint not found: {self.checkpoint_path}")
        if self.checkpoint_path.stat().st_size < 1_000_000:
            raise ValueError(f"GraspNet checkpoint is unexpectedly small: {self.checkpoint_path}")

        for path in (
            self.repo_path,
            self.repo_path / "models",
            self.repo_path / "pointnet2",
            self.repo_path / "utils",
        ):
            path_string = str(path)
            if path_string not in sys.path:
                sys.path.insert(0, path_string)

        import torch
        from graspnet import GraspNet, pred_decode

        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                "GraspNet CUDA backend requested, but torch.cuda.is_available() is false"
            )
        self._torch = torch
        self._pred_decode = pred_decode
        self.device = torch.device(device)
        self.num_point = int(num_point)
        self.seed = int(seed)
        self.net = GraspNet(
            input_feature_dim=0,
            num_view=int(num_view),
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        ).to(self.device)
        checkpoint = torch.load(
            str(self.checkpoint_path),
            map_location=self.device,
            weights_only=False,
        )
        state_dict = checkpoint.get("model_state_dict", checkpoint)
        self.net.load_state_dict(state_dict)
        self.net.eval()
        self.epoch = checkpoint.get("epoch") if isinstance(checkpoint, dict) else None

    def predict(self, scene_points: np.ndarray) -> np.ndarray:
        points = np.asarray(scene_points, dtype=np.float32)
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] < 2:
            raise ValueError("scene cloud does not contain enough finite points for GraspNet")
        rng = np.random.default_rng(self.seed)
        indices = rng.choice(
            points.shape[0],
            self.num_point,
            replace=points.shape[0] < self.num_point,
        )
        sampled = np.ascontiguousarray(points[indices][None, ...])
        end_points = {
            "point_clouds": self._torch.from_numpy(sampled).to(self.device),
        }
        with self._torch.inference_mode():
            predictions = self._pred_decode(self.net(end_points))[0]
        return predictions.detach().cpu().numpy()
