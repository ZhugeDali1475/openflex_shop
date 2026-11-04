#!/usr/bin/env python3

from itertools import product

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField


PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_NEIGHBOR_OFFSETS = tuple(product((-1, 0, 1), repeat=3))


def _find_field_offset(fields, name):
    for f in fields:
        if f.name == name:
            return f.offset
    return None


def radius_outlier_mask(points, radius, min_neighbors):
    """Keep points with enough neighbors inside a 3D radius.

    ``min_neighbors`` excludes the query point itself. A voxel index limits
    distance checks to the 27 adjacent cells, avoiding an O(N^2) full-cloud
    allocation while retaining exact Euclidean radius tests.
    """
    count = len(points)
    if count == 0 or radius <= 0.0 or min_neighbors <= 0:
        return np.ones(count, dtype=bool)

    cells = np.floor(points / radius).astype(np.int64)
    buckets = {}
    for index, cell in enumerate(cells):
        buckets.setdefault(tuple(cell), []).append(index)

    keep = np.zeros(count, dtype=bool)
    radius_squared = radius * radius
    for cell, member_list in buckets.items():
        candidate_lists = [
            buckets.get((cell[0] + dx, cell[1] + dy, cell[2] + dz))
            for dx, dy, dz in _NEIGHBOR_OFFSETS
        ]
        candidate_lists = [indices for indices in candidate_lists if indices]
        if not candidate_lists:
            continue
        members = np.asarray(member_list, dtype=np.int64)
        candidates = np.fromiter(
            (index for indices in candidate_lists for index in indices),
            dtype=np.int64,
        )
        delta = points[members, None, :] - points[candidates][None, :, :]
        neighbor_counts = np.count_nonzero(
            np.einsum('ijk,ijk->ij', delta, delta) <= radius_squared,
            axis=1,
        ) - 1
        keep[members] = neighbor_counts >= min_neighbors
    return keep


def voxel_representative_indices(points, leaf_size):
    """Return one stable representative index for every occupied voxel."""
    if len(points) == 0 or leaf_size <= 0.0:
        return np.arange(len(points), dtype=np.int64)
    cells = np.floor(points / leaf_size).astype(np.int64)
    _unique, first_indices = np.unique(cells, axis=0, return_index=True)
    return np.sort(first_indices)


def spatial_filter_indices(points, outlier_radius, outlier_min_neighbors, voxel_leaf_size):
    """Return source indices retained by outlier removal and downsampling."""
    outlier_keep = radius_outlier_mask(points, outlier_radius, outlier_min_neighbors)
    retained = np.flatnonzero(outlier_keep)
    if len(retained) == 0:
        return retained
    voxel_indices = voxel_representative_indices(points[retained], voxel_leaf_size)
    return retained[voxel_indices]


class PointCloudQoSRelay(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_qos_relay')
        self.declare_parameter('input_topic', '/camera/d435/depth/color/points')
        self.declare_parameter('output_topic', '/camera/d435/depth/color/points_nav2')
        self.declare_parameter('restamp_on_receive', False)
        # Z-axis filtering in the point cloud's own frame (e.g. base_link)
        self.declare_parameter('filter_min_z', float('-inf'))
        self.declare_parameter('filter_max_z', float('inf'))
        # Self-box filtering: remove points inside a 3D box (robot body)
        self.declare_parameter('self_filter_enabled', False)
        self.declare_parameter('self_filter_min_x', -0.42)
        self.declare_parameter('self_filter_max_x', 0.42)
        self.declare_parameter('self_filter_min_y', -0.34)
        self.declare_parameter('self_filter_max_y', 0.34)
        self.declare_parameter('self_filter_min_z', -0.30)
        # The self-box is inside the chassis footprint, so filter the full
        # robot-height column to remove arm/body returns as well as wheels.
        self.declare_parameter('self_filter_max_z', 1.80)
        # Planning-cloud filters. Keep disabled for conservative safety clouds.
        self.declare_parameter('outlier_radius', 0.0)
        self.declare_parameter('outlier_min_neighbors', 0)
        self.declare_parameter('voxel_leaf_size', 0.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.restamp_on_receive = bool(self.get_parameter('restamp_on_receive').value)
        self.filter_min_z = float(self.get_parameter('filter_min_z').value)
        self.filter_max_z = float(self.get_parameter('filter_max_z').value)
        self.self_filter_enabled = bool(self.get_parameter('self_filter_enabled').value)
        self.self_filter_min_x = float(self.get_parameter('self_filter_min_x').value)
        self.self_filter_max_x = float(self.get_parameter('self_filter_max_x').value)
        self.self_filter_min_y = float(self.get_parameter('self_filter_min_y').value)
        self.self_filter_max_y = float(self.get_parameter('self_filter_max_y').value)
        self.self_filter_min_z = float(self.get_parameter('self_filter_min_z').value)
        self.self_filter_max_z = float(self.get_parameter('self_filter_max_z').value)
        self.outlier_radius = float(self.get_parameter('outlier_radius').value)
        self.outlier_min_neighbors = int(self.get_parameter('outlier_min_neighbors').value)
        self.voxel_leaf_size = float(self.get_parameter('voxel_leaf_size').value)

        if self.outlier_radius < 0.0:
            raise ValueError('outlier_radius must be >= 0')
        if self.outlier_min_neighbors < 0:
            raise ValueError('outlier_min_neighbors must be >= 0')
        if self.voxel_leaf_size < 0.0:
            raise ValueError('voxel_leaf_size must be >= 0')

        self.filtering_enabled = (
            self.filter_min_z != float('-inf')
            or self.filter_max_z != float('inf')
            or self.self_filter_enabled
            or (self.outlier_radius > 0.0 and self.outlier_min_neighbors > 0)
            or self.voxel_leaf_size > 0.0
        )

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, PUB_QOS)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._callback,
            qos_profile_sensor_data,
        )
        self.forwarded = 0

        filter_info = ''
        if self.filter_min_z != float('-inf') or self.filter_max_z != float('inf'):
            filter_info += f', z_filter=[{self.filter_min_z:.2f}, {self.filter_max_z:.2f}]'
        if self.self_filter_enabled:
            filter_info += (
                f', self_filter=x[{self.self_filter_min_x:.2f},{self.self_filter_max_x:.2f}]'
                f' y[{self.self_filter_min_y:.2f},{self.self_filter_max_y:.2f}]'
                f' z[{self.self_filter_min_z:.2f},{self.self_filter_max_z:.2f}]'
            )
        if self.outlier_radius > 0.0 and self.outlier_min_neighbors > 0:
            filter_info += (
                f', radius_outlier={self.outlier_radius:.2f}m/'
                f'{self.outlier_min_neighbors}neighbors'
            )
        if self.voxel_leaf_size > 0.0:
            filter_info += f', voxel={self.voxel_leaf_size:.2f}m'
        self.get_logger().info(
            f'Relaying {self.input_topic} -> {self.output_topic} with RELIABLE QoS '
            f'(restamp_on_receive={self.restamp_on_receive}{filter_info})'
        )

    def _filter_cloud(self, msg: PointCloud2) -> PointCloud2:
        """Apply configured geometric, self, outlier, and voxel filters."""
        x_off = _find_field_offset(msg.fields, 'x')
        y_off = _find_field_offset(msg.fields, 'y')
        z_off = _find_field_offset(msg.fields, 'z')
        if x_off is None or y_off is None or z_off is None:
            self.get_logger().warning(
                'Point cloud has no float XYZ fields; forwarding without filtering',
                throttle_duration_sec=5.0,
            )
            return msg

        point_step = msg.point_step
        data = bytes(msg.data)
        n_points = msg.width * msg.height

        endian_dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
        xs = np.ndarray(n_points, dtype=endian_dtype,
                        buffer=data, offset=x_off, strides=(point_step,))
        ys = np.ndarray(n_points, dtype=endian_dtype,
                        buffer=data, offset=y_off, strides=(point_step,))
        zs = np.ndarray(n_points, dtype=endian_dtype,
                        buffer=data, offset=z_off, strides=(point_step,))
        points = np.column_stack((xs, ys, zs)).astype(np.float64, copy=False)
        mask = np.isfinite(points).all(axis=1)
        mask &= (zs >= self.filter_min_z) & (zs <= self.filter_max_z)

        if self.self_filter_enabled:
            in_box = (
                (xs >= self.self_filter_min_x) & (xs <= self.self_filter_max_x)
                & (ys >= self.self_filter_min_y) & (ys <= self.self_filter_max_y)
                & (zs >= self.self_filter_min_z) & (zs <= self.self_filter_max_z)
            )
            mask = mask & ~in_box

        spatial_filtering_enabled = (
            (self.outlier_radius > 0.0 and self.outlier_min_neighbors > 0)
            or self.voxel_leaf_size > 0.0
        )
        if mask.all() and not spatial_filtering_enabled:
            return msg

        # Build filtered message
        indices = np.flatnonzero(mask)
        spatial_indices = spatial_filter_indices(
            points[indices],
            self.outlier_radius,
            self.outlier_min_neighbors,
            self.voxel_leaf_size,
        )
        indices = indices[spatial_indices]
        raw = np.frombuffer(data, dtype=np.uint8).reshape(n_points, point_step)
        filtered_data = raw[indices].tobytes()

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(indices)
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.point_step * out.width
        out.data = filtered_data
        out.is_dense = True
        return out

    def _callback(self, msg: PointCloud2) -> None:
        if self.restamp_on_receive:
            msg.header.stamp = self.get_clock().now().to_msg()
        if self.filtering_enabled:
            msg = self._filter_cloud(msg)
        self.publisher.publish(msg)
        self.forwarded += 1
        if self.forwarded == 1:
            self.get_logger().info(
                f'First point cloud relayed on {self.output_topic} '
                f'(stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})'
            )
        elif self.forwarded % 100 == 0:
            self.get_logger().info(
                f'Point cloud relay health: forwarded={self.forwarded}, '
                f'latest_points={msg.width * msg.height}'
            )


def main() -> None:
    rclpy.init()
    node = PointCloudQoSRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
