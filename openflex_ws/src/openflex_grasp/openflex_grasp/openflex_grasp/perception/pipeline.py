from dataclasses import dataclass

import numpy as np

from ..grounding import Detection
from .providers.base import BoxRequest
from .providers.qwen_grounder import QwenGrounder
from .sam2_segmenter import Sam2BoxSegmenter


@dataclass(frozen=True)
class SegmentationResult:
    detection: Detection
    mask: np.ndarray
    sam_score: float


class ProductPerceptionPipeline:
    """Compatibility pipeline while providers are formalized in P5-02/P5-03."""

    def __init__(self, qwen_path: str, sam_path: str, device: str = "cuda"):
        self.box_provider = QwenGrounder(qwen_path, device)
        self.segmenter = Sam2BoxSegmenter(sam_path, device)

    def locate_all(
        self,
        rgb: np.ndarray,
        target: str,
        qwen_grounding_prompt: str = "",
        max_boxes: int = 8,
    ) -> list[Detection]:
        response = self.box_provider.detect(
            BoxRequest(
                rgb=rgb,
                sku_name=target,
                qwen_grounding_prompt=qwen_grounding_prompt,
                max_boxes=max_boxes,
            )
        )
        return [
            Detection(
                box.label,
                box.box_xyxy,
                1.0 if box.confidence is None else box.confidence,
                "",
            )
            for box in response.boxes
        ]

    def locate(self, rgb: np.ndarray, target: str) -> Detection:
        return self.locate_all(rgb, target)[0]

    def segment_box(self, rgb: np.ndarray, box) -> tuple[np.ndarray, float]:
        return self.segmenter.segment_box(rgb, box)

    def infer(self, rgb: np.ndarray, target: str) -> SegmentationResult:
        return self.infer_all(rgb, target)[0]

    def infer_all(self, rgb: np.ndarray, target: str) -> list[SegmentationResult]:
        results = []
        for detection in self.locate_all(rgb, target):
            mask, score = self.segment_box(rgb, detection.box_xyxy)
            results.append(SegmentationResult(detection, mask, score))
        if not results:
            raise RuntimeError(f"no visible instances found for {target!r}")
        return results


ProductSegmenter = ProductPerceptionPipeline
