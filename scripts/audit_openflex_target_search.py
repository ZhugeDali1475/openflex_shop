#!/usr/bin/env python3
"""Capture one live head-camera frame and audit the search fail-closed cases.

Runs the *same* image-derived candidate + resolution contract that the online
``search_and_pick`` skill uses, over one real frame captured from the isolated
MuJoCo stack, plus deterministic frame variants that reproduce the four
required conditions (visible / occluded / wrong class / empty slot).  It never
moves the robot and never reads the oracle object id for selection.

The three non-visible conditions cannot be produced by the single-red scene
itself (there is one object, no occluder, no other class), so they are produced
by editing the captured frame only; every variant and its edit is recorded.
"""

import argparse
import base64
import hashlib
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
from urllib.request import ProxyHandler, build_opener, Request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.target_search_contract import (  # noqa: E402
    CameraIntrinsics,
    CameraPose,
    analyze_frame,
    project_point,
    resolve_search_target,
)


RED = (220, 40, 40)
BLUE = (40, 60, 220)
GREY = (150, 150, 150)


class CaptureNode(Node):
    def __init__(self):
        super().__init__("openflex_target_search_audit")
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


def snapshot():
    opener = build_opener(ProxyHandler({}))
    with opener.open(Request("http://127.0.0.1:18552/state"), timeout=5) as response:
        return json.load(response)


def capture(node, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.color and node.depth and node.info:
            color, depth = node.color.header.stamp, node.depth.header.stamp
            if abs((color.sec - depth.sec) + (color.nanosec - depth.nanosec) * 1e-9) <= 0.05:
                break
    else:
        raise RuntimeError("no fresh head camera frame")
    rgb = np.frombuffer(node.color.data, dtype=np.uint8).reshape(
        node.color.height, node.color.width, 3).copy()
    depth_mm = np.frombuffer(node.depth.data, dtype=np.uint16).reshape(
        node.depth.height, node.depth.width).copy()
    intrinsics = CameraIntrinsics.from_k(node.info.width, node.info.height, list(node.info.k))
    transform = None
    while time.monotonic() < deadline and transform is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = node.tf.lookup_transform("map", node.color.header.frame_id, rclpy.time.Time())
        except Exception:
            transform = None
    if transform is None:
        raise RuntimeError("camera transform unavailable")
    translation, rotation = transform.transform.translation, transform.transform.rotation
    pose = CameraPose((translation.x, translation.y, translation.z),
                      (rotation.x, rotation.y, rotation.z, rotation.w))
    return rgb, depth_mm, intrinsics, pose


def paint(rgb, depth_mm, center, radius, color, depth_m):
    u, v = center
    y0, y1 = max(0, v - radius), min(rgb.shape[0], v + radius)
    x0, x1 = max(0, u - radius), min(rgb.shape[1], u + radius)
    rgb[y0:y1, x0:x1] = color
    depth_mm[y0:y1, x0:x1] = int(round(depth_m * 1000))


def run_case(name, rgb, depth_mm, intrinsics, pose, slot, query, color, edit=None):
    frame_rgb, frame_depth = rgb.copy(), depth_mm.copy()
    detail = "captured frame unmodified"
    if edit is not None:
        detail = edit(frame_rgb, frame_depth)
    analysis = analyze_frame(frame_rgb, frame_depth, intrinsics, pose, expected_point_map=slot,
                             color=color)
    resolution = resolve_search_target(analysis, query, color=color)
    return {
        "case": name,
        "edit": detail,
        "outcome": resolution.outcome,
        "reason": resolution.reason,
        "selection_reason": resolution.selection_reason,
        "selected_object_id": resolution.selected_object_id,
        "slot_verdict": analysis.region.state,
        "candidate_count": len(analysis.candidates),
        "frame_sha256": analysis.frame_sha256,
        "rgb": frame_rgb,
    }, resolution


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--object-id", default="thesis_item_01")
    parser.add_argument("--query", default="红色的快递盒")
    parser.add_argument("--color", default="red")
    parser.add_argument("--wrong-color", default="blue")
    parser.add_argument("--capture-timeout", type=float, default=20.0)
    arguments = parser.parse_args()
    output = arguments.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output}")
    output.mkdir(parents=True)

    rclpy.init()
    node = CaptureNode()
    try:
        rgb, depth_mm, intrinsics, pose = capture(node, arguments.capture_timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    state = snapshot()
    registered = state["scene_registry"]["objects"][arguments.object_id]
    slot = tuple(registered["nominal_pose_xyzyaw"][:3])
    projection = project_point(pose, intrinsics, slot)
    if projection is None:
        raise RuntimeError("registered slot is not in the camera frustum")
    center = (int(round(projection[0])), int(round(projection[1])))
    expected_depth = float(projection[2])
    oracle = state["sources"]["object_observation"]["data"]

    Image.fromarray(rgb).save(output / "frame.png")
    Image.fromarray(depth_mm).save(output / "depth.png")
    capture_record = {
        "scope": "isolated_MuJoCo_online_head_camera_frame_NOT_PHYSICAL",
        "object_id": arguments.object_id,
        "frame_sha256": hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
        "width": intrinsics.width, "height": intrinsics.height,
        "k": [intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0],
        "camera_pose_map": {"translation": pose.translation_xyz, "quaternion_xyzw": pose.quaternion_xyzw},
        "slot_point_map": slot, "slot_pixel": list(center), "slot_depth_m": round(expected_depth, 5),
        "oracle_object_observation": {"object_id": oracle.get("object_id"),
                                      "visible": oracle.get("visible"),
                                      "confidence": oracle.get("confidence")},
        "oracle_used_for_selection": False,
        "scene_sha256": state["scene_registry"]["sources"]["model_sha256"],
    }
    (output / "capture.json").write_text(json.dumps(capture_record, ensure_ascii=False, indent=2) + "\n")

    def occluded(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, 14, GREY, expected_depth - 0.25)
        return "grey occluder painted 0.25 m in front of the registered slot"

    def wrong_class(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, 12, BLUE, expected_depth)
        return f"a {arguments.wrong_color} object painted at the registered slot depth"

    def empty_slot(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, 14, GREY, expected_depth + 0.35)
        return "slot cleared to plain background 0.35 m behind the object"

    def ambiguous(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, 7, RED, expected_depth)
        paint(frame_rgb, frame_depth, (center[0] + 20, center[1]), 7, RED, expected_depth)
        return "a second red instance painted 20 px from the slot (guard case; out of P4.3 scope)"

    cases = [
        ("visible", None),
        ("occluded", occluded),
        ("wrong_class", wrong_class),
        ("empty_slot", empty_slot),
        ("ambiguous_guard", ambiguous),
    ]
    results = []
    for name, edit in cases:
        record, resolution = run_case(name, rgb, depth_mm, intrinsics, pose, slot,
                                      arguments.query, arguments.color, edit)
        Image.fromarray(record.pop("rgb")).save(output / f"case_{name}.png")
        record["resolution"] = resolution.to_dict()
        results.append(record)

    report = {
        "scope": "P4.3_target_search_fail_closed_audit_isolated_MuJoCo_NOT_PHYSICAL",
        "physical_verification": False,
        "robot_motion_executed": False,
        "selection_basis": "image_derived_candidates_not_oracle_object_id",
        "query": arguments.query,
        "cases": results,
        "expected": {"visible": "target_visible", "occluded": "target_occluded",
                     "wrong_class": "wrong_class", "empty_slot": "empty_slot",
                     "ambiguous_guard": "ambiguous_candidates"},
    }
    report["passed"] = all(item["outcome"] == ("authorized" if item["case"] == "visible" else "failed")
                           and item["reason"] == report["expected"][item["case"]] for item in results)
    (output / "cases.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "passed": report["passed"],
                      "cases": {item["case"]: item["reason"] for item in results}},
                     ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
