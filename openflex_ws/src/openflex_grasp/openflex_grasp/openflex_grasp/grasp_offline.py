import argparse
import json
from pathlib import Path

import numpy as np

from .graspnet_backend import GraspNetBackend, fit_graspnet_straight_grasp
from .paths import GRASPNET_CHECKPOINT, GRASPNET_REPO


def _load_cloud(path: str) -> np.ndarray:
    import open3d as o3d

    cloud_path = Path(path).expanduser().resolve()
    if not cloud_path.is_file():
        raise FileNotFoundError(f"point cloud not found: {cloud_path}")
    cloud = o3d.io.read_point_cloud(str(cloud_path))
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.shape[0] == 0:
        raise ValueError(f"point cloud is empty: {cloud_path}")
    return points


def _payload(estimate, frame_id: str):
    return {
        "valid": estimate.valid,
        "reason": estimate.reason,
        "backend": estimate.backend,
        "frame_id": frame_id,
        "center_m": estimate.center.tolist(),
        "pregrasp_m": estimate.pregrasp.tolist(),
        "quaternion_xyzw": estimate.quaternion_xyzw.tolist(),
        "approach_axis": estimate.approach.tolist(),
        "closing_axis": estimate.closing_axis.tolist(),
        "required_width_m": estimate.required_width,
        "object_spans_m": estimate.object_spans.tolist(),
        "collision_checked": estimate.collision_checked,
        "obstacle_count": estimate.obstacle_count,
        "score": estimate.score,
        "network_score": estimate.network_score,
        "candidate_count": estimate.candidate_count,
        "constrained_candidate_count": estimate.constrained_candidate_count,
        "fit_contributor_count": estimate.constrained_candidate_count,
        "fit_method": "shelf_center_weighted_graspnet",
        "candidate_center_distance_m": estimate.candidate_center_distance,
        "candidate_approach_alignment": estimate.candidate_approach_alignment,
        "execution_enabled": False,
    }


def build_parser():
    parser = argparse.ArgumentParser(
        description="Fit a camera-frame straight grasp from saved PLY clouds"
    )
    parser.add_argument("--product", required=True, help="segmented product PLY")
    parser.add_argument(
        "--scene",
        help="optional full scene PLY for inference and path collision checks",
    )
    parser.add_argument(
        "--backend",
        choices=("graspnet",),
        default="graspnet",
    )
    parser.add_argument(
        "--graspnet-repo",
        default=str(GRASPNET_REPO),
    )
    parser.add_argument(
        "--graspnet-checkpoint",
        default=str(GRASPNET_CHECKPOINT),
    )
    parser.add_argument("--frame-id", default="camera_color_optical_frame")
    parser.add_argument("--pregrasp-distance", type=float, default=0.12)
    parser.add_argument("--gripper-max-width", type=float, default=0.088)
    parser.add_argument("--width-margin", type=float, default=0.008)
    parser.add_argument("--corridor-radius", type=float, default=0.055)
    parser.add_argument("--object-clearance", type=float, default=0.04)
    parser.add_argument("--collision-point-limit", type=int, default=20)
    parser.add_argument("--candidate-center-tolerance", type=float, default=0.06)
    parser.add_argument("--candidate-approach-max-angle-deg", type=float, default=25.0)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    product = _load_cloud(args.product)
    scene = _load_cloud(args.scene) if args.scene else None
    fit_kwargs = {
        "pregrasp_distance": args.pregrasp_distance,
        "gripper_max_width": args.gripper_max_width,
        "width_margin": args.width_margin,
        "corridor_radius": args.corridor_radius,
        "object_clearance": args.object_clearance,
        "collision_point_limit": args.collision_point_limit,
    }

    backend = GraspNetBackend(args.graspnet_repo, args.graspnet_checkpoint)
    candidates = backend.predict(product)
    estimate = fit_graspnet_straight_grasp(
        product,
        scene,
        candidates,
        candidate_center_tolerance=args.candidate_center_tolerance,
        candidate_approach_max_angle_deg=args.candidate_approach_max_angle_deg,
        **fit_kwargs,
    )
    print(json.dumps(_payload(estimate, args.frame_id), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
