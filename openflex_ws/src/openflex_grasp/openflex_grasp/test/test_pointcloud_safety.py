import math
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from openflex_grasp.pointcloud_safety import pointcloud_xyz, transformed_minimum_z


def test_pointcloud_xyz_reads_field_offsets():
    cloud = PointCloud2()
    cloud.height = 1
    cloud.width = 2
    cloud.point_step = 16
    cloud.row_step = 32
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.data = b"".join(
        struct.pack("<ffff", *point, 99.0)
        for point in ((0.1, 0.2, 0.3), (0.4, 0.5, -0.2))
    )

    points = pointcloud_xyz(cloud)

    np.testing.assert_allclose(
        points,
        [[0.1, 0.2, 0.3], [0.4, 0.5, -0.2]],
        atol=1e-7,
    )


def test_transformed_minimum_z_applies_translation():
    points = np.array([[0.0, 0.0, -0.2], [0.0, 0.0, 0.3]])

    result = transformed_minimum_z(
        points,
        (1.0, 2.0, 0.8),
        (0.0, 0.0, 0.0, 1.0),
    )

    assert result == pytest.approx(0.6)


def test_transformed_minimum_z_applies_rotation():
    half = math.sqrt(0.5)
    points = np.array([[0.0, 0.2, 0.0], [0.0, -0.4, 0.0]])

    result = transformed_minimum_z(
        points,
        (0.0, 0.0, 1.0),
        (half, 0.0, 0.0, half),
    )

    assert result == pytest.approx(0.6)
