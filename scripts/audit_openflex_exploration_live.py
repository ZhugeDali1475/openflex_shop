#!/usr/bin/env python3
"""Read-only live audit of phase C2 mapping + frontier selection.

Subscribes to the isolated MuJoCo head RGB-D + CameraInfo and TF, builds a
2-D occupancy grid from one live depth frame with ``mapping_contract``, then asks
``exploration_contract.ExplorationSearch`` for the next observation stand.  The
plan comes only from the map snapshot that was just built; no preset route or
station list is consulted.

The node **publishes nothing** and never commands motion; it only observes and
writes evidence.  Localization is the simulator's ground-truth TF, so the report
labels it explicitly as "not an estimator".
"""

import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image as RosImage
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.exploration_contract import (  # noqa: E402
    ExplorationSearch, find_frontier_clusters, information_gain,
)
from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    OCCUPIED, UNKNOWN, OccupancyGridBuilder, ground_truth_localization,
)
from openflex_roboos_bridge.target_search_contract import CameraIntrinsics, CameraPose  # noqa: E402

CORRIDOR_HALF_M = 2.4  # the simulated floor plane is 2.4 m half-extent


class CaptureNode(Node):
    def __init__(self):
        super().__init__("openflex_exploration_audit_readonly")
        self.color = self.depth = self.info = None
        self.create_subscription(RosImage, "/thesis_sim/head/color/image_raw",
                                 self.on_color, qos_profile_sensor_data)
        self.create_subscription(RosImage, "/thesis_sim/head/aligned_depth_to_color/image_raw",
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/thesis_sim/head/color/camera_info",
                                 self.on_info, qos_profile_sensor_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def on_color(self, message):
        self.color = message

    def on_depth(self, message):
        self.depth = message

    def on_info(self, message):
        self.info = message

    def wait(self, timeout_sec: float = 20.0) -> bool:
        started = time.monotonic()
        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.color is not None and self.depth is not None and self.info is not None:
                return True
        return False


def decode_rgb(message) -> np.ndarray:
    data = np.frombuffer(message.data, dtype=np.uint8)
    return data.reshape((message.height, message.width, -1))[:, :, :3].copy()


def decode_depth(message) -> np.ndarray:
    data = np.frombuffer(message.data, dtype="<u2")
    return data.reshape((message.height, message.width)).copy()


def wait_for_tf(node: Node, target_frame: str, source_frame: str, timeout_sec: float = 10.0):
    started = time.monotonic()
    last_error = None
    while time.monotonic() - started < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return lookup_pose(node, target_frame, source_frame)
        except Exception as error:  # tf2 raises Lookup/Extrapolation exceptions
            last_error = error
    raise RuntimeError(f"timed out waiting for TF {target_frame}->{source_frame}: {last_error}")


def lookup_pose(node: Node, target_frame: str, source_frame: str):
    transform = node.tf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (translation.x, translation.y, translation.z), (rotation.x, rotation.y, rotation.z,
                                                           rotation.w)


def run(arguments) -> dict:
    rclpy.init()
    node = None
    output = Path(arguments.output) if arguments.output else (
        ROOT / "local_dependencies/openflex_roboos/runs"
        / time.strftime("exploration_%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=True)
    try:
        node = CaptureNode()
        if not node.wait(float(arguments.timeout)):
            raise RuntimeError("timed out waiting for head RGB-D + CameraInfo")
        rgb = decode_rgb(node.color)
        depth = decode_depth(node.depth)
        width, height = node.info.width, node.info.height
        intrinsics = CameraIntrinsics.from_k(width, height, node.info.k)
        translation, quaternion = wait_for_tf(node, "map", "thesis_head_color_optical_frame")
        base_translation, _ = wait_for_tf(node, "map", "base_link")
        stamp_sec = node.depth.header.stamp.sec + node.depth.header.stamp.nanosec * 1e-9

        resolution = float(arguments.resolution)
        builder = OccupancyGridBuilder(
            frame_id="map", resolution_m=resolution,
            origin_xy=(-CORRIDOR_HALF_M, -CORRIDOR_HALF_M),
            width=int(round(2 * CORRIDOR_HALF_M / resolution)),
            height=int(round(2 * CORRIDOR_HALF_M / resolution)))
        integrated = builder.integrate_depth_frame(
            depth, intrinsics, CameraPose(translation, quaternion),
            stride=int(arguments.stride), min_range_m=0.15, max_range_m=float(arguments.max_range),
            min_height_m=float(arguments.min_height))
        snapshot = builder.snapshot(localization=ground_truth_localization(stamp_sec=stamp_sec),
                                    stamp_sec=stamp_sec)
        search = ExplorationSearch(frame_id="map", min_known_fraction=float(arguments.min_known),
                                   sensor_radius_m=float(arguments.sensor_radius),
                                   max_steps=4)
        decision = search.plan(snapshot, robot_xy=(base_translation[0], base_translation[1]),
                               now_sec=stamp_sec, target_observed=False)
        clusters = find_frontier_clusters(snapshot, sensor_radius_m=float(arguments.sensor_radius))
        occupied = sum(1 for cell in snapshot.occupancy if cell == OCCUPIED)
        unknown = sum(1 for cell in snapshot.occupancy if cell == UNKNOWN)
        occupied_cells = [(i, j) for j in range(snapshot.height) for i in range(snapshot.width)
                          if snapshot.cell(i, j) == OCCUPIED]
        occupied_bbox = None
        if occupied_cells:
            xs = [snapshot.grid_to_world(i, j)[0] for i, j in occupied_cells]
            ys = [snapshot.grid_to_world(i, j)[1] for i, j in occupied_cells]
            occupied_bbox = {"x": [min(xs), max(xs)], "y": [min(ys), max(ys)]}

        Image.fromarray(rgb).save(output / "head_color.png")
        Image.fromarray(depth).save(output / "head_depth.png")
        report = {
            "scope": "phase_C2_live_mapping_and_frontier_selection_readonly_NOT_PHYSICAL",
            "physical_verification": False,
            "robot_motion_executed": False,
            "node_publishes": [],
            "node_subscribes": ["/thesis_sim/head/color/image_raw",
                                "/thesis_sim/head/aligned_depth_to_color/image_raw",
                                "/thesis_sim/head/color/camera_info", "/tf"],
            "localization_source": "simulation_ground_truth_not_an_estimator",
            "preset_route_used": False,
            "next_position_source": "current_map_frontier_information_gain_only",
            "depth_sha256_available": False,
            "camera": {"frame": "thesis_head_color_optical_frame", "width": width, "height": height,
                       "k": list(node.info.k), "pose_map_translation": list(translation),
                       "pose_map_quaternion_xyzw": list(quaternion)},
            "robot_base_map_xy": [base_translation[0], base_translation[1]],
            "grid": {"resolution_m": resolution,
                     "origin_xy": [-CORRIDOR_HALF_M, -CORRIDOR_HALF_M],
                     "width": snapshot.width, "height": snapshot.height,
                     "rays_integrated": integrated, "map_version": snapshot.map_version,
                     "occupied_cells": occupied, "unknown_cells": unknown,
                     "occupied_bbox_xy": occupied_bbox,
                     "coverage_fraction": snapshot.coverage_fraction()},
            "frontier_clusters": [cluster.to_dict() for cluster in clusters],
            "decision": decision.to_dict(),
            "detail": search.report()["detail"],
        }
        (output / "exploration_live.json").write_text(json.dumps(report, ensure_ascii=False,
                                                                 indent=2) + "\n")
        return {"output": str(output), "report": report}
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument("--timeout", default=20.0)
    parser.add_argument("--resolution", default=0.05)
    parser.add_argument("--stride", default=3)
    parser.add_argument("--max-range", default=3.0)
    parser.add_argument("--min-height", default=0.30,
                        help="drop map-frame points below this z (floor filtering)")
    parser.add_argument("--sensor-radius", default=1.0)
    parser.add_argument("--min-known", default=0.02)
    arguments = parser.parse_args(argv)
    result = run(arguments)
    report = result["report"]
    print(json.dumps({"output": result["output"], "outcome": report["decision"]["outcome"],
                      "coverage_fraction": report["grid"]["coverage_fraction"],
                      "occupied_cells": report["grid"]["occupied_cells"],
                      "frontier_clusters": len(report["frontier_clusters"]),
                      "goal_xy": report["decision"]["goal_xy"]},
                     ensure_ascii=False, indent=2))
    return 0 if report["decision"]["outcome"] == "next_position" else 1


if __name__ == "__main__":
    sys.exit(main())
