#!/usr/bin/env python3
"""Bounded base-motion audit: simulated FAST-LIO2 odometry vs ground truth.

Drives the base *only* through the audited velocity arbiter (NAV2 mode,
``/thesis/nav_cmd_vel``) with a short, low-speed, scripted pattern (forward,
turn in place, forward), then compares the FAST-LIO2 estimate on ``/lio_odom``
against the MuJoCo bridge ground truth on ``/swerve_drive_controller/odom``
after aligning each trajectory to its own pose at the moment motion starts.

It never commands the arm, lift or gripper, publishes a zero ``Twist`` on every
exit path, and returns the arbiter to HOLD.  Nothing here is physical evidence:
the ground truth is a digital-experiment pose and the lidar is a ray cast.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from openflex_thesis_msgs.srv import SetControlMode

ROOT = Path(__file__).resolve().parents[1]
HOLD, NAV2 = 0, 1


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LioMotionProbe(Node):
    def __init__(self) -> None:
        super().__init__("openflex_lio_motion_probe")
        self.estimate = None          # (x, y, yaw) from FAST-LIO2
        self.truth = None             # (x, y, yaw) from the MuJoCo bridge
        self.create_subscription(Odometry, "/lio_odom", self.on_estimate, 20)
        self.create_subscription(Odometry, "/swerve_drive_controller/odom", self.on_truth, 20)
        self.command = Twist()
        self.command_publisher = self.create_publisher(Twist, "/thesis/nav_cmd_vel", 10)
        self.mode_client = self.create_client(SetControlMode, "/thesis/set_control_mode")
        self.create_timer(0.05, lambda: self.command_publisher.publish(self.command))

    def on_estimate(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.estimate = (pose.position.x, pose.position.y,
                         yaw_from_quaternion(pose.orientation.x, pose.orientation.y,
                                             pose.orientation.z, pose.orientation.w))

    def on_truth(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.truth = (pose.position.x, pose.position.y,
                      yaw_from_quaternion(pose.orientation.x, pose.orientation.y,
                                          pose.orientation.z, pose.orientation.w))

    def wait_for_inputs(self, timeout_sec: float) -> bool:
        started = time.monotonic()
        while time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.estimate is not None and self.truth is not None:
                return True
        return False

    def set_mode(self, mode: int, reason: str, timeout_sec: float = 5.0) -> None:
        started = time.monotonic()
        while not self.mode_client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() - started > timeout_sec:
                raise RuntimeError("set_control_mode service unavailable")
        request = SetControlMode.Request()
        request.requested_mode = int(mode)
        request.requester = "openflex_lio_motion_probe"
        request.reason = reason
        future = self.mode_client.call_async(request)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        response = future.result()
        if response is None or not response.accepted:
            raise RuntimeError(f"control mode {mode} rejected: {getattr(response, 'message', None)}")

    def sample(self) -> dict:
        return {
            "t": round(time.monotonic(), 4),
            "estimate": None if self.estimate is None else [round(v, 5) for v in self.estimate],
            "truth": None if self.truth is None else [round(v, 5) for v in self.truth],
        }


def align_and_compare(samples, reference_index):
    reference = samples[reference_index]
    gt0 = np.array(reference["truth"][:2])
    lio0 = np.array(reference["estimate"][:2])
    gt_yaw0 = reference["truth"][2]
    lio_yaw0 = reference["estimate"][2]
    position_errors = []
    yaw_errors = []
    for entry in samples[reference_index:]:
        gt = np.array(entry["truth"])
        lio = np.array(entry["estimate"])
        gt_rot = np.array([[math.cos(-gt_yaw0), -math.sin(-gt_yaw0)],
                           [math.sin(-gt_yaw0), math.cos(-gt_yaw0)]]) @ (gt[:2] - gt0)
        lio_rot = np.array([[math.cos(-lio_yaw0), -math.sin(-lio_yaw0)],
                            [math.sin(-lio_yaw0), math.cos(-lio_yaw0)]]) @ (lio[:2] - lio0)
        position_errors.append(float(np.linalg.norm(gt_rot - lio_rot)))
        yaw_errors.append(float(normalize_angle((lio[2] - lio_yaw0) - (gt[2] - gt_yaw0))))
    position_errors = np.asarray(position_errors)
    yaw_errors = np.asarray(yaw_errors)
    return {
        "samples_compared": int(position_errors.size),
        "position_rmse_m": float(np.sqrt(np.mean(position_errors ** 2))) if position_errors.size else None,
        "position_max_m": float(position_errors.max()) if position_errors.size else None,
        "position_final_m": float(position_errors[-1]) if position_errors.size else None,
        "yaw_max_rad": float(np.abs(yaw_errors).max()) if yaw_errors.size else None,
        "truth_travel_m": float(np.linalg.norm(np.array(samples[-1]["truth"][:2]) - gt0)),
        "estimate_travel_m": float(np.linalg.norm(np.array(samples[-1]["estimate"][:2]) - lio0)),
    }


def drive_phase(node, *, linear, angular, seconds, samples):
    node.command = Twist()
    node.command.linear.x = float(linear)
    node.command.angular.z = float(angular)
    deadline = time.monotonic() + seconds
    last_sample = 0.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now - last_sample >= 0.1:
            last_sample = now
            samples.append(node.sample())
    node.command = Twist()


def run(arguments) -> dict:
    rclpy.init()
    node = None
    output = Path(arguments.output) if arguments.output else (
        ROOT / "local_dependencies/openflex_roboos/runs"
        / time.strftime("lio_motion_%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=True)
    samples = []
    zero_issued = False
    metrics = {}
    try:
        node = LioMotionProbe()
        if not node.wait_for_inputs(arguments.timeout):
            raise RuntimeError("timed out waiting for /lio_odom and ground-truth odometry")
        node.set_mode(NAV2, "phase C2 LIO motion audit")

        settle_deadline = time.monotonic() + arguments.settle_seconds
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        samples.append(node.sample())
        reference_index = len(samples) - 1

        # Reverse away from the shelf first, then turn in place and drive forward
        # along the new heading.  This keeps base_link well inside the verified
        # free zone (the arm touches the shelf face at base_link x ~= 0.69 m,
        # measured with MUJOCO_SAFETY_STOP), so the batch never triggers the
        # MuJoCo safety freeze.
        drive_phase(node, linear=-arguments.speed, angular=0.0,
                    seconds=arguments.forward_seconds, samples=samples)
        drive_phase(node, linear=0.0, angular=arguments.turn_rate,
                    seconds=arguments.turn_seconds, samples=samples)
        drive_phase(node, linear=arguments.speed, angular=0.0,
                    seconds=arguments.forward_seconds, samples=samples)

        time.sleep(0.5)
        samples.append(node.sample())
        metrics = align_and_compare(samples, reference_index)
    finally:
        if node is not None:
            node.command = Twist()
            for _ in range(10):
                node.command_publisher.publish(node.command)
                rclpy.spin_once(node, timeout_sec=0.02)
            zero_issued = True
            try:
                node.set_mode(HOLD, "LIO motion audit finished")
            except Exception:
                pass
        report = {
            "scope": "phase_C2_lio_motion_base_only_NOT_PHYSICAL",
            "physical_verification": False,
            "lidar": "simulation_raycast_not_physical",
            "ground_truth": "mujoco_ground_truth_not_an_estimator",
            "motion_scope": "base_only_via_velocity_arbiter_NAV2",
            "arm_lift_gripper_commanded": False,
            "zero_command_issued": zero_issued,
            "thresholds": {
                "position_rmse_m": arguments.max_position_rmse,
                "position_max_m": arguments.max_position_error,
                "yaw_max_rad": arguments.max_yaw_error,
            },
            "metrics": metrics,
            "samples": samples,
        }
        passed = (
            bool(metrics)
            and metrics["position_rmse_m"] is not None
            and metrics["position_rmse_m"] <= arguments.max_position_rmse
            and metrics["position_max_m"] <= arguments.max_position_error
            and metrics["yaw_max_rad"] <= arguments.max_yaw_error
            and metrics["truth_travel_m"] > arguments.min_travel_m
        )
        report["result"] = "PASS" if passed else "FAIL"
        (output / "lio_motion.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps({k: report[k] for k in ("metrics", "thresholds", "result")},
                         ensure_ascii=False, indent=2))
        print(f"report: {output / 'lio_motion.json'}")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not passed:
            return 1
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Bounded base-motion audit of simulated FAST-LIO2 vs ground truth.")
    parser.add_argument("--output")
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--turn-rate", type=float, default=0.30)
    parser.add_argument("--forward-seconds", type=float, default=3.0)
    parser.add_argument("--turn-seconds", type=float, default=3.0)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-position-rmse", type=float, default=0.10)
    parser.add_argument("--max-position-error", type=float, default=0.20)
    parser.add_argument("--max-yaw-error", type=float, default=0.15)
    parser.add_argument("--min-travel-m", type=float, default=0.2)
    return run(parser.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
