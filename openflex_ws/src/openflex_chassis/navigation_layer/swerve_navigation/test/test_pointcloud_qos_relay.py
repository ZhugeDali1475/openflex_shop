import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "pointcloud_qos_relay.py"
SPEC = importlib.util.spec_from_file_location("pointcloud_qos_relay", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_radius_outlier_removes_isolated_point():
    points = np.array([
        [0.00, 0.00, 0.00],
        [0.03, 0.00, 0.00],
        [0.00, 0.04, 0.00],
        [1.00, 1.00, 0.00],
    ])

    mask = MODULE.radius_outlier_mask(points, radius=0.10, min_neighbors=2)

    assert mask.tolist() == [True, True, True, False]


def test_voxel_filter_keeps_stable_source_indices():
    points = np.array([
        [0.01, 0.01, 0.01],
        [0.02, 0.02, 0.02],
        [0.11, 0.01, 0.01],
    ])

    indices = MODULE.voxel_representative_indices(points, leaf_size=0.05)

    assert indices.tolist() == [0, 2]


def test_spatial_filter_applies_outlier_removal_before_voxel_filter():
    points = np.array([
        [0.00, 0.00, 0.00],
        [0.01, 0.01, 0.00],
        [0.03, 0.00, 0.00],
        [2.00, 2.00, 0.00],
    ])

    indices = MODULE.spatial_filter_indices(
        points,
        outlier_radius=0.10,
        outlier_min_neighbors=2,
        voxel_leaf_size=0.05,
    )

    assert indices.tolist() == [0]
