from dataclasses import dataclass

import numpy as np

from .masks import clean_mask
from .pipeline import SegmentationResult
from .pointcloud import product_instance_graspability, product_points, semantic_instance_selection_key
from .rgbd import FrozenRgbdFrame


@dataclass(frozen=True)
class ProductInstance:
    result: SegmentationResult
    mask: np.ndarray
    points: np.ndarray
    quality: dict


def build_instances(
    frame: FrozenRgbdFrame,
    results: list[SegmentationResult],
    *,
    min_depth_m: float = 0.07,
    max_depth_m: float = 1.5,
    max_depth_spread_m: float = 0.25,
    stride: int = 1,
    min_points: int = 100,
) -> list[ProductInstance]:
    instances = []
    for result in results:
        mask = clean_mask(result.mask)
        points = product_points(
            frame.rgb, frame.depth, mask, frame.intrinsics,
            depth_scale=frame.depth_scale, min_depth_m=min_depth_m,
            max_depth_m=max_depth_m, max_depth_spread_m=max_depth_spread_m, stride=stride,
        )
        quality = product_instance_graspability(
            mask, points, result.detection.box_xyxy, frame.rgb.shape,
            detection_confidence=result.detection.confidence,
            segmentation_score=result.sam_score, min_points=min_points,
        )
        mask.setflags(write=False)
        points.setflags(write=False)
        instances.append(ProductInstance(result, mask, points, quality))
    return instances


def select_instance(instances: list[ProductInstance]) -> ProductInstance:
    valid = [item for item in instances if item.quality.get("valid")]
    if not valid:
        reasons = "; ".join(str(item.quality.get("reason", "unknown")) for item in instances)
        raise RuntimeError(f"no product instance has usable same-frame depth: {reasons}")
    return max(
        valid,
        key=lambda item: semantic_instance_selection_key(
            item.result.detection.confidence, item.result.sam_score, item.quality["score"]
        ),
    )


def scene_points(frame: FrozenRgbdFrame, *, min_depth_m=0.07, max_depth_m=1.5, stride=2):
    return product_points(
        frame.rgb, frame.depth, np.ones(frame.depth.shape, dtype=bool), frame.intrinsics,
        depth_scale=frame.depth_scale, min_depth_m=min_depth_m, max_depth_m=max_depth_m,
        max_depth_spread_m=0.0, stride=stride,
    )
