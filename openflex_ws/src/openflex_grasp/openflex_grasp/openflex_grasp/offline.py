import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .paths import QWEN_MODEL, SAM_MODEL
from .perception.io import write_ply
from .perception.masks import clean_mask
from .perception.pipeline import ProductPerceptionPipeline
from .perception.pointcloud import product_points
from .perception.rgbd import CameraIntrinsics
from .perception.visualization import render_overlay


def main():
    parser = argparse.ArgumentParser(description="Offline Qwen3.5 + SAM2 RGB-D product segmentation")
    parser.add_argument("--rgb", required=True)
    parser.add_argument("--depth", required=True, help="16-bit PNG in millimeters or NumPy .npy")
    parser.add_argument("--intrinsics", required=True, help="JSON with fx, fy, cx, cy, width, height")
    parser.add_argument("--target", required=True)
    parser.add_argument("--output-dir", default="product_perception_output")
    parser.add_argument("--depth-scale", type=float, default=0.0001)
    parser.add_argument("--qwen-model", default=str(QWEN_MODEL))
    parser.add_argument("--sam-model", default=str(SAM_MODEL))
    args = parser.parse_args()

    bgr = cv2.imread(args.rgb, cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(args.rgb)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    depth = np.load(args.depth) if args.depth.endswith(".npy") else cv2.imread(args.depth, cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise FileNotFoundError(args.depth)
    intrinsics_data = json.loads(Path(args.intrinsics).read_text(encoding="utf-8"))
    intrinsics = CameraIntrinsics(**intrinsics_data)

    segmenter = ProductPerceptionPipeline(args.qwen_model, args.sam_model)
    result = segmenter.infer(rgb, args.target)
    mask = clean_mask(result.mask)
    points = product_points(rgb, depth, mask, intrinsics, depth_scale=args.depth_scale)
    scene_points = product_points(
        rgb,
        depth,
        np.ones(depth.shape[:2], dtype=bool),
        intrinsics,
        depth_scale=args.depth_scale,
        max_depth_spread_m=0.0,
        stride=2,
    )
    overlay = render_overlay(rgb, mask, result.detection.box_xyxy, result.detection.label)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "mask.png"), mask.astype(np.uint8) * 255)
    cv2.imwrite(str(output_dir / "overlay.png"), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
    write_ply(output_dir / "product.ply", points)
    write_ply(output_dir / "scene.ply", scene_points)
    metadata = {
        "target": args.target,
        "label": result.detection.label,
        "bbox_xyxy": list(result.detection.box_xyxy),
        "grounding_confidence": result.detection.confidence,
        "sam_score": result.sam_score,
        "mask_pixels": int(mask.sum()),
        "point_count": int(points.shape[0]),
        "scene_point_count": int(scene_points.shape[0]),
        "qwen_response": result.detection.raw_response,
    }
    (output_dir / "result.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
