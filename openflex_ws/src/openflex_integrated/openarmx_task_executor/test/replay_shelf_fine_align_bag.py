#!/usr/bin/env python3
"""Offline validation of two-corner shelf filtering against a recorded bag."""

import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

from openarmx_task_executor.shelf_fine_align_core import (
    ShelfCornerFilter,
    ShelfCornerMeasurement,
    ShelfPlaneFilter,
    ShelfPlaneMeasurement,
    measure_scan_shelf_corners,
    measure_scan_shelf_plane,
)


def replay(
    path: str,
    start_ns: int = 0,
    end_ns: int = 0,
    observation: str = "front_corners",
    plane_minimum_points: int = 5,
    plane_inlier_tolerance: float = 0.040,
) -> tuple[int, ShelfCornerMeasurement | ShelfPlaneMeasurement]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    if observation == "front_plane":
        filt = ShelfPlaneFilter(7, 5, half_length=0.46, quantile=0.50)
    else:
        filt = ShelfCornerFilter(7, 5, half_width=0.45, quantile=0.25)
    count = 0
    estimate = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if end_ns > 0 and timestamp > end_ns:
            break
        if topic != "/scan" or timestamp < start_ns:
            continue
        scan = deserialize_message(data, LaserScan)
        if scan.header.frame_id not in ("", "base_link"):
            raise RuntimeError(
                "offline replay requires /scan to be expressed in base_link; "
                f"recording uses {scan.header.frame_id!r}"
            )
        common = {
            "half_length": 0.46,
            "half_width": 0.45,
            "roi_forward_min": 0.46,
            "roi_forward_max": 1.20,
            "roi_half_width": 0.55,
            "maximum_clearance": 0.45,
        }
        if observation == "front_plane":
            measurement = measure_scan_shelf_plane(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                minimum_points=plane_minimum_points,
                minimum_lateral_span=0.30,
                minimum_points_per_side=2,
                inlier_tolerance=plane_inlier_tolerance,
                maximum_normal_yaw=0.2094395102,
                **common,
            )
        else:
            measurement = measure_scan_shelf_corners(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                minimum_lateral_offset=0.20,
                minimum_points_per_side=2,
                **common,
            )
        if measurement is not None:
            estimate = filt.add(measurement)
            count += 1
    if estimate is None:
        raise RuntimeError("recording did not produce a stable shelf estimate")
    return count, estimate


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--start-ns", type=int, default=0)
    parser.add_argument("--end-ns", type=int, default=0)
    parser.add_argument(
        "--observation",
        choices=("front_corners", "front_plane"),
        default="front_corners",
    )
    parser.add_argument("--plane-minimum-points", type=int, default=5)
    parser.add_argument("--plane-inlier-tolerance", type=float, default=0.040)
    args = parser.parse_args()
    samples, corners = replay(
        args.bag,
        args.start_ns,
        args.end_ns,
        args.observation,
        args.plane_minimum_points,
        args.plane_inlier_tolerance,
    )
    details = (
        f"plane_points={corners.point_count}"
        if isinstance(corners, ShelfPlaneMeasurement)
        else (
            f"minimum_left_points={corners.left_point_count} "
            f"minimum_right_points={corners.right_point_count}"
        )
    )
    print(
        f"observation={args.observation} samples={samples} "
        f"left_clearance={corners.left_clearance:.4f}m "
        f"right_clearance={corners.right_clearance:.4f}m "
        f"minimum_clearance={corners.minimum_corner_clearance:.4f}m "
        f"shelf_yaw_error={corners.shelf_yaw_error:.4f}rad "
        f"{details} minimum_span={corners.lateral_span:.4f}m"
    )
