"""Compatibility imports for the pre-migration perception API."""

from .perception.pipeline import (
    ProductPerceptionPipeline,
    ProductSegmenter,
    SegmentationResult,
)

__all__ = [
    "ProductPerceptionPipeline",
    "ProductSegmenter",
    "SegmentationResult",
]
