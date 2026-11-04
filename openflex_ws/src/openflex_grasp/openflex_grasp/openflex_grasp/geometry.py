"""Compatibility imports for perception geometry helpers."""

from .perception.masks import clean_mask, largest_component
from .perception.pointcloud import (
    product_instance_graspability,
    product_points,
    semantic_instance_selection_key,
)
from .perception.rgbd import CameraIntrinsics
from .perception.visualization import render_overlay

__all__ = [
    "CameraIntrinsics",
    "clean_mask",
    "largest_component",
    "product_instance_graspability",
    "product_points",
    "render_overlay",
    "semantic_instance_selection_key",
]
