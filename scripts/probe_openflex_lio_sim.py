#!/usr/bin/env python3
"""Read-only smoke probe for the simulated MID-360 + FAST-LIO2 bridge.

Subscribes to ``/lio_odom`` and ``/world_cloud`` published by ``fastlio2/lio_node``
while the MuJoCo bridge feeds it the simulated MID-360 point cloud and IMU, and
reports whether the odometry is present and stationary-consistent.  It never
commands motion and never writes to the TF tree.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    localization_from_odometry_covariance,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--max-stationary-span-m", type=float, default=0.2)
    arguments = parser.parse_args(argv)

    rclpy.init()
    node = Node("openflex_lio_sim_probe")
    positions = []
    clouds = []
    covariances = []

    def on_odometry(message):
        pose = message.pose.pose
        positions.append((pose.position.x, pose.position.y, pose.position.z))
        covariances.append([list(row) for row in np.asarray(message.pose.covariance).reshape(6, 6)])

    node.create_subscription(
        Odometry, "/lio_odom", on_odometry, 20,
    )
    node.create_subscription(
        PointCloud2, "/world_cloud",
        lambda msg: clouds.append((int(msg.width), msg.header.frame_id)),
        5,
    )

    deadline = time.monotonic() + arguments.seconds
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = bool(positions) and bool(clouds)
    print(f"lio_odom samples: {len(positions)}"
          + (f"  rate~{len(positions) / arguments.seconds:.1f} Hz" if positions else ""))
    if positions:
        array = np.asarray(positions)
        span = array.max(axis=0) - array.min(axis=0)
        print(f"  position x/y/z span: {span[0]:.4f} / {span[1]:.4f} / {span[2]:.4f} m")
        print(f"  last position: {np.round(array[-1], 4).tolist()}")
        if float(span.max()) > arguments.max_stationary_span_m:
            print(f"FAIL: stationary odometry drifted more than {arguments.max_stationary_span_m} m")
            ok = False
    else:
        print("FAIL: no /lio_odom samples")
    print(f"world_cloud messages: {len(clouds)}"
          + (f"  widths={sorted({width for width, _ in clouds})}"
             f"  frames={sorted({frame for _, frame in clouds})}" if clouds else ""))
    if not clouds or min(width for width, _ in clouds) <= 0:
        print("FAIL: /world_cloud is empty")
        ok = False

    if covariances:
        quality = localization_from_odometry_covariance(
            covariances[-1], source="simulation_fastlio2_not_physical")
        print("estimated localization quality:"
              f" position_rmse={quality.position_rmse_m:.4f} m"
              f" yaw_rmse={quality.yaw_rmse_rad:.4f} rad"
              f" covariance_trace={quality.covariance_trace:.5f}")

    node.destroy_node()
    rclpy.shutdown()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
