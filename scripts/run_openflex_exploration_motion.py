#!/usr/bin/env python3
"""Bounded, base-only moving exploration probe (phase C2).

Closes the C2 loop with real motion: it builds an occupancy map from live head
depth across frames, asks ``exploration_contract.ExplorationSearch`` for the next
observation stand, and drives the *base only* toward it through the audited
velocity arbiter (``/thesis/nav_cmd_vel`` in NAV2 mode) - never the arm, lift or
gripper, and never ``/thesis/cmd_vel_selected`` directly.

Safety envelope: low speed, arrival radius, a map-derived obstacle stop (zero
command when an observed obstacle is closer than ``--stop-distance``), a
wall-clock deadline and a bounded number of arrivals/blocks.  A zero Twist is
published on every exit path and the arbiter is returned to HOLD.  The next
stand is chosen only from the current map snapshot; no preset route or station
list is used.
"""

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image as RosImage
from tf2_ros import Buffer, TransformListener

from openflex_thesis_msgs.srv import SetControlMode

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.exploration_contract import (  # noqa: E402
    NEXT_POSITION, ExplorationSearch,
)
from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    OCCUPIED, OccupancyGridBuilder, ground_truth_localization,
)
from openflex_roboos_bridge.target_search_contract import CameraIntrinsics, CameraPose  # noqa: E402

FLOOR_HALF_M = 2.4
HOLD, NAV2 = 0, 1


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ExplorationProbe(Node):
    def __init__(self):
        super().__init__("openflex_exploration_probe")
        self.color = self.depth = self.info = None
        self.create_subscription(RosImage, "/thesis_sim/head/color/image_raw",
                                 self.on_color, qos_profile_sensor_data)
        self.create_subscription(RosImage, "/thesis_sim/head/aligned_depth_to_color/image_raw",
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/thesis_sim/head/color/camera_info",
                                 self.on_info, qos_profile_sensor_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.command_publisher = self.create_publisher(Twist, "/thesis/nav_cmd_vel", 10)
        self.mode_client = self.create_client(SetControlMode, "/thesis/set_control_mode")
        self.command = Twist()
        self.create_timer(0.05, self.publish_command)  # 20 Hz keep-alive for the arbiter

    def on_color(self, message):
        self.color = message

    def on_depth(self, message):
        self.depth = message

    def on_info(self, message):
        self.info = message

    def publish_command(self):
        self.command_publisher.publish(self.command)

    def wait_for_inputs(self, timeout_sec):
        started = time.monotonic()
        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.color is not None and self.depth is not None and self.info is not None:
                return True
        return False

    def wait_for_tf(self, target, source, timeout_sec=10.0):
        started = time.monotonic()
        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self.tf.lookup_transform(target, source, rclpy.time.Time())
                return True
            except Exception:
                continue
        return False

    def base_xy_yaw(self):
        transform = self.tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
        rotation = transform.rotation
        return (transform.translation.x, transform.translation.y,
                yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w))

    def camera_pose(self):
        transform = self.tf.lookup_transform(
            "map", "thesis_head_color_optical_frame", rclpy.time.Time()).transform
        translation = transform.translation
        rotation = transform.rotation
        return CameraPose((translation.x, translation.y, translation.z),
                          (rotation.x, rotation.y, rotation.z, rotation.w))

    def set_mode(self, mode, reason, timeout_sec=5.0):
        started = time.monotonic()
        while not self.mode_client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() - started > timeout_sec:
                raise RuntimeError("set_control_mode service unavailable")
        request = SetControlMode.Request()
        request.requested_mode = int(mode)
        request.requester = "openflex_exploration_probe"
        request.reason = reason
        future = self.mode_client.call_async(request)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        response = future.result()
        if response is None or not response.accepted:
            raise RuntimeError(f"control mode {mode} rejected: {getattr(response, 'message', None)}")
        return response


def decode_depth(message):
    return np.frombuffer(message.data, dtype="<u2").reshape((message.height, message.width)).copy()


def min_obstacle_distance(snapshot, robot_xy):
    best = float("inf")
    for j in range(snapshot.height):
        for i in range(snapshot.width):
            if snapshot.cell(i, j) != OCCUPIED:
                continue
            wx, wy = snapshot.grid_to_world(i, j)
            best = min(best, math.hypot(wx - robot_xy[0], wy - robot_xy[1]))
    return best


def run(arguments) -> dict:
    rclpy.init()
    node = None
    output = Path(arguments.output) if arguments.output else (
        ROOT / "local_dependencies/openflex_roboos/runs"
        / time.strftime("exploration_motion_%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=True)
    samples = []
    zero_issued = False
    try:
        node = ExplorationProbe()
        if not node.wait_for_inputs(20.0):
            raise RuntimeError("timed out waiting for head RGB-D")
        if not node.wait_for_tf("map", "base_link"):
            raise RuntimeError("timed out waiting for map->base_link TF")
        node.set_mode(NAV2, "phase C2 exploration probe")

        resolution = float(arguments.resolution)
        builder = OccupancyGridBuilder(
            frame_id="map", resolution_m=resolution, origin_xy=(-FLOOR_HALF_M, -FLOOR_HALF_M),
            width=int(round(2 * FLOOR_HALF_M / resolution)),
            height=int(round(2 * FLOOR_HALF_M / resolution)))
        search = ExplorationSearch(frame_id="map", min_known_fraction=float(arguments.min_known),
                                   sensor_radius_m=float(arguments.sensor_radius),
                                   max_steps=int(arguments.max_steps))
        last_frame_stamp = None
        arrivals = blocks = 0
        outcome = "deadline"
        deadline = time.monotonic() + float(arguments.duration)
        goal = None
        while time.monotonic() < deadline:
            # gather frames for ~0.5 s of wall time
            gather_until = time.monotonic() + float(arguments.replan_period)
            while time.monotonic() < gather_until:
                rclpy.spin_once(node, timeout_sec=0.05)
                depth_message = node.depth
                if depth_message is not None:
                    stamp = (depth_message.header.stamp.sec, depth_message.header.stamp.nanosec)
                    if stamp != last_frame_stamp:
                        last_frame_stamp = stamp
                        intrinsics = CameraIntrinsics.from_k(node.info.width, node.info.height,
                                                             node.info.k)
                        builder.integrate_depth_frame(
                            decode_depth(depth_message), intrinsics, node.camera_pose(),
                            stride=int(arguments.stride),
                            min_range_m=0.15, max_range_m=float(arguments.max_range),
                            min_height_m=float(arguments.min_height))
            robot_xy = node.base_xy_yaw()
            snapshot = builder.snapshot(localization=ground_truth_localization(stamp_sec=0.0),
                                        stamp_sec=0.0)
            decision = search.plan(snapshot, robot_xy=(robot_xy[0], robot_xy[1]),
                                   now_sec=time.monotonic(), target_observed=False)
            obstacle_distance = min_obstacle_distance(snapshot, (robot_xy[0], robot_xy[1]))
            sample = {"t": round(time.monotonic() - (deadline - float(arguments.duration)), 3),
                      "robot_xy_yaw": [round(robot_xy[0], 4), round(robot_xy[1], 4),
                                       round(robot_xy[2], 4)],
                      "coverage_fraction": round(snapshot.coverage_fraction(), 6),
                      "map_version": snapshot.map_version,
                      "obstacle_distance_m": (None if obstacle_distance == float("inf")
                                              else round(obstacle_distance, 4)),
                      "decision_outcome": decision.outcome,
                      "goal_xy": list(decision.goal_xy) if decision.goal_xy else None}
            samples.append(sample)
            if decision.finished:
                outcome = decision.outcome
                break
            if decision.outcome != NEXT_POSITION:
                outcome = decision.outcome
                break
            if obstacle_distance < float(arguments.stop_distance):
                blocks += 1
                node.command = Twist()
                sample["blocked"] = True
                if blocks >= int(arguments.max_blocks):
                    outcome = "obstacle_stop"
                    break
                continue
            goal = decision.goal_xy
            distance = math.hypot(goal[0] - robot_xy[0], goal[1] - robot_xy[1])
            if distance <= float(arguments.arrival_radius):
                arrivals += 1
                node.command = Twist()
                sample["arrived"] = True
                if arrivals >= int(arguments.max_arrivals):
                    outcome = "arrival_limit"
                    break
                continue
            heading_error = normalize_angle(math.atan2(goal[1] - robot_xy[1],
                                                       goal[0] - robot_xy[0]) - robot_xy[2])
            command = Twist()
            if abs(heading_error) < 0.6:
                command.linear.x = min(float(arguments.max_linear),
                                       float(arguments.linear_gain) * distance)
            command.angular.z = max(-float(arguments.max_angular),
                                    min(float(arguments.max_angular),
                                        float(arguments.angular_gain) * heading_error))
            node.command = command
        else:
            outcome = "deadline"
    finally:
        if node is not None:
            node.command = Twist()
            for _ in range(10):
                node.publish_command()
                rclpy.spin_once(node, timeout_sec=0.02)
            zero_issued = True
            try:
                node.set_mode(HOLD, "exploration probe finished")
            except Exception:
                pass
        final_pose = None
        report = {
            "scope": "phase_C2_moving_exploration_base_only_NOT_PHYSICAL",
            "physical_verification": False,
            "robot_motion_executed": True,
            "motion_scope": "base_only_via_velocity_arbiter_NAV2",
            "arm_lift_gripper_commanded": False,
            "preset_route_used": False,
            "next_position_source": "current_map_frontier_information_gain_only",
            "localization_source": "simulation_ground_truth_not_an_estimator",
            "zero_command_issued": zero_issued,
            "outcome": locals().get("outcome", "error"),
            "arrivals": locals().get("arrivals", 0),
            "blocks": locals().get("blocks", 0),
            "samples": samples,
        }
        (output / "exploration_motion.json").write_text(json.dumps(report, ensure_ascii=False,
                                                                  indent=2) + "\n")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--replan-period", type=float, default=1.0)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--max-range", type=float, default=3.0)
    parser.add_argument("--min-height", type=float, default=0.30)
    parser.add_argument("--sensor-radius", type=float, default=1.0)
    parser.add_argument("--min-known", type=float, default=0.02)
    parser.add_argument("--max-steps", type=int, default=40)
    parser.add_argument("--max-linear", type=float, default=0.15)
    parser.add_argument("--max-angular", type=float, default=0.25)
    parser.add_argument("--linear-gain", type=float, default=0.8)
    parser.add_argument("--angular-gain", type=float, default=1.5)
    parser.add_argument("--arrival-radius", type=float, default=0.35)
    parser.add_argument("--stop-distance", type=float, default=0.30)
    parser.add_argument("--max-arrivals", type=int, default=3)
    parser.add_argument("--max-blocks", type=int, default=2)
    arguments = parser.parse_args(argv)
    report = run(arguments)
    print(json.dumps({"outcome": report["outcome"], "arrivals": report["arrivals"],
                      "blocks": report["blocks"], "samples": len(report["samples"]),
                      "final_coverage": report["samples"][-1]["coverage_fraction"]
                      if report["samples"] else None,
                      "zero_command_issued": report["zero_command_issued"]},
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
