#!/usr/bin/env python3
"""Read-only live audit of phase C2 mapping + frontier selection.

Subscribes to the isolated MuJoCo head RGB-D + CameraInfo and TF, builds a
2-D occupancy grid from one live depth frame with ``mapping_contract``, then asks
``exploration_contract.ExplorationSearch`` for the next observation stand.  The
plan comes only from the map snapshot that was just built; no preset route or
station list is consulted.

The node **publishes nothing** and never commands motion; it only observes and
writes evidence. Localization defaults to simulator ground-truth TF for
baseline checks. ``--estimator`` requires fresh FAST-LIO and ICP-quality
messages and looks up the TF chain at the depth frame timestamp.
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
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]

# Estimator-assumed localization usability budget (m / rad).  The occupancy
# planner's own default (0.05 m) is a *known-map* gate; simulated FAST-LIO2
# reports a fixed 0.1 m covariance RMSE, so the audit must use the same
# budget for its estimator gate and its exploration gate or it always
# fail-closes as 'localization_lost'.
MAX_POSITION_RMSE_M = 0.25
MAX_YAW_RMSE_RAD = 0.25
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.exploration_contract import (  # noqa: E402
    ExplorationSearch, find_frontier_clusters, information_gain,
)
from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    OCCUPIED, UNKNOWN, OccupancyGridBuilder, ground_truth_localization,
    localization_from_odometry_covariance, localization_from_registration_diagnostic,
)
from openflex_roboos_bridge.target_search_contract import CameraIntrinsics, CameraPose  # noqa: E402

CORRIDOR_HALF_M = 2.4  # the simulated floor plane is 2.4 m half-extent


class CaptureNode(Node):
    def __init__(self):
        super().__init__("openflex_exploration_audit_readonly")
        self.color = self.depth = self.info = None
        self.lio_odom = None
        self.localization_quality = None
        self.localization_quality_stamp_sec = None
        self.create_subscription(RosImage, "/thesis_sim/head/color/image_raw",
                                 self.on_color, qos_profile_sensor_data)
        self.create_subscription(RosImage, "/thesis_sim/head/aligned_depth_to_color/image_raw",
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/thesis_sim/head/color/camera_info",
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/fastlio2/lio_odom", self.on_lio,
                                 qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/localization_quality", self.on_quality, 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def on_color(self, message):
        self.color = message

    def on_depth(self, message):
        self.depth = message

    def on_info(self, message):
        self.info = message

    def on_lio(self, message):
        self.lio_odom = message

    def on_quality(self, message):
        for status in message.status:
            if status.name != "icp_registration":
                continue
            self.localization_quality = {item.key: item.value for item in status.values}
            self.localization_quality_stamp_sec = (
                message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

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


def wait_for_tf(node: Node, target_frame: str, source_frame: str,
                timeout_sec: float = 10.0, stamp_sec: float = 0.0):
    started = time.monotonic()
    last_error = None
    while time.monotonic() - started < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return lookup_pose(node, target_frame, source_frame, stamp_sec)
        except Exception as error:  # tf2 raises Lookup/Extrapolation exceptions
            last_error = error
    raise RuntimeError(f"timed out waiting for TF {target_frame}->{source_frame}: {last_error}")


def lookup_pose(node: Node, target_frame: str, source_frame: str, stamp_sec: float = 0.0):
    stamp = (rclpy.time.Time(nanoseconds=round(stamp_sec * 1e9))
             if stamp_sec > 0.0 else rclpy.time.Time())
    transform = node.tf.lookup_transform(target_frame, source_frame, stamp)
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (translation.x, translation.y, translation.z), (rotation.x, rotation.y, rotation.z,
                                                           rotation.w)


def estimated_localization(node: CaptureNode, timeout_sec: float):
    """Wait for FAST-LIO odometry + ICP quality that are fresh against the
    *latest* head depth frame, and return ``(localization, depth_stamp_sec)``.

    The reference must be re-read every iteration: an earlier implementation
    captured one depth stamp up front and then compared it against the
    continuously-advancing latest odometry, so once the (service-call) delay
    exceeded the 0.15 s window the fresh sample could never be observed again
    and the audit always timed out.
    """
    started = time.monotonic()
    while time.monotonic() - started < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (node.depth is None or node.lio_odom is None
                or node.localization_quality is None
                or node.localization_quality_stamp_sec is None):
            continue
        stamp_sec = node.depth.header.stamp.sec + node.depth.header.stamp.nanosec * 1e-9
        odom = node.lio_odom
        odom_stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        if (abs(stamp_sec - odom_stamp) > 0.15
                or abs(stamp_sec - node.localization_quality_stamp_sec) > 0.75):
            continue
        covariance = [odom.pose.covariance[row * 6:(row + 1) * 6] for row in range(6)]
        base = localization_from_odometry_covariance(
            covariance, source="simulation_fastlio2_not_physical", stamp_sec=odom_stamp)
        return localization_from_registration_diagnostic(
            base, node.localization_quality), stamp_sec
    raise RuntimeError("timed out waiting for fresh FAST-LIO odometry + ICP quality")


def require_ground_truth_tf_disabled(node: CaptureNode, timeout_sec: float) -> None:
    """Refuse estimator scoring if the simulator may still publish oracle TF."""
    client = node.create_client(GetParameters, "/openflex_thesis_mujoco_bridge/get_parameters")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError("cannot verify MuJoCo publish_ground_truth_tf parameter")
    request = GetParameters.Request(names=["publish_ground_truth_tf"])
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        raise RuntimeError("timed out reading MuJoCo ground-truth TF parameter")
    values = future.result().values
    if (len(values) != 1 or values[0].type != ParameterType.PARAMETER_BOOL
            or values[0].bool_value):
        raise RuntimeError("estimator audit requires publish_ground_truth_tf:=false")


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
        # Resolve localization before grabbing the depth frame so the estimator
        # gate is observed live; the RGB-D frame is then read while the
        # localization sample is still fresh.
        localization = None
        if arguments.estimator:
            require_ground_truth_tf_disabled(node, float(arguments.timeout))
            localization, _ = estimated_localization(node, float(arguments.timeout))
            if not localization.is_usable(max_position_rmse_m=MAX_POSITION_RMSE_M,
                                          max_yaw_rmse_rad=MAX_YAW_RMSE_RAD):
                raise RuntimeError(f"estimated localization quality is unusable: {localization.to_dict()}")
        rgb = decode_rgb(node.color)
        depth = decode_depth(node.depth)
        width, height = node.info.width, node.info.height
        intrinsics = CameraIntrinsics.from_k(width, height, node.info.k)
        stamp_sec = node.depth.header.stamp.sec + node.depth.header.stamp.nanosec * 1e-9
        if localization is None:
            localization = ground_truth_localization(stamp_sec=stamp_sec)
        translation, quaternion = wait_for_tf(
            node, "map", "thesis_head_color_optical_frame", stamp_sec=stamp_sec)
        base_translation, _ = wait_for_tf(node, "map", "base_link", stamp_sec=stamp_sec)

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
        snapshot = builder.snapshot(localization=localization,
                                    stamp_sec=stamp_sec)
        search = ExplorationSearch(frame_id="map", min_known_fraction=float(arguments.min_known),
                                   sensor_radius_m=float(arguments.sensor_radius),
                                   max_position_rmse_m=MAX_POSITION_RMSE_M,
                                   max_yaw_rmse_rad=MAX_YAW_RMSE_RAD,
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
                                "/thesis_sim/head/color/camera_info", "/tf",
                                "/fastlio2/lio_odom", "/localization_quality"],
            "localization_source": localization.source,
            "estimator_mode": bool(arguments.estimator),
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
    parser.add_argument("--estimator", action="store_true",
                        help="require FAST-LIO + ICP quality and use estimator TF; fail closed otherwise")
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
