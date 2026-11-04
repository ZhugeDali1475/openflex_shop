import numpy as np

from openflex_grasp.geometry import (
    CameraIntrinsics,
    product_instance_graspability,
    product_points,
    semantic_instance_selection_key,
)


def test_projects_masked_depth_to_xyz():
    rgb = np.zeros((3, 3, 3), dtype=np.uint8)
    rgb[1, 2] = [10, 20, 30]
    depth = np.zeros((3, 3), dtype=np.uint16)
    depth[1, 2] = 1000
    mask = depth > 0
    intrinsics = CameraIntrinsics(fx=2.0, fy=2.0, cx=1.0, cy=1.0, width=3, height=3)
    points = product_points(rgb, depth, mask, intrinsics)
    np.testing.assert_allclose(points[0, :3], [0.5, 0.0, 1.0])
    np.testing.assert_array_equal(points[0, 3:], [10, 20, 30])


def test_instance_graspability_prefers_centered_complete_product():
    mask = np.ones((100, 100), dtype=bool)
    centered_points = np.tile([0.0, 0.0, 0.5], (10000, 1))
    edge_points = np.tile([0.2, 0.0, 0.5], (5000, 1))
    centered = product_instance_graspability(
        mask,
        centered_points,
        (10, 10, 90, 90),
        mask.shape,
        detection_confidence=0.8,
        segmentation_score=0.9,
    )
    edge = product_instance_graspability(
        mask,
        edge_points,
        (0, 10, 80, 90),
        mask.shape,
        detection_confidence=0.8,
        segmentation_score=0.9,
    )
    assert centered["valid"]
    assert centered["score"] > edge["score"]


def test_instance_selection_prioritizes_semantics_over_graspability():
    semantic_match = semantic_instance_selection_key(0.95, 0.75, 0.20)
    easy_but_weaker_match = semantic_instance_selection_key(0.80, 0.99, 0.99)
    assert semantic_match > easy_but_weaker_match
