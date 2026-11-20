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
    TargetSearchError,
    analyze_frame,
    depth_valid_fraction,
    observation_from_resolution,
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


def run_case(name, rgb, depth_mm, intrinsics, pose, query, color, *, expected_point_map=None,
             edit=None):
    frame_rgb, frame_depth = rgb.copy(), depth_mm.copy()
    detail = "captured frame unmodified"
    if edit is not None:
        detail = edit(frame_rgb, frame_depth)
    analysis = analyze_frame(frame_rgb, frame_depth, intrinsics, pose,
                             expected_point_map=expected_point_map, color=color)
    resolution = resolve_search_target(analysis, query, color=color)
    record = {
        "case": name,
        "mode": "region_anchored_diagnostics" if expected_point_map is not None else "full_frame_search",
        "edit": detail,
        "outcome": resolution.outcome,
        "reason": resolution.reason,
        "selection_reason": resolution.selection_reason,
        "selected_object_id": resolution.selected_object_id,
        "slot_verdict": analysis.region.state,
        "candidate_count": len(analysis.candidates),
        "frame_sha256": analysis.frame_sha256,
        "depth_valid_fraction": analysis.model_output["depth_valid_fraction"],
        "rgb": frame_rgb,
    }
    if resolution.outcome == "authorized":
        observation = observation_from_resolution(
            resolution, intrinsics=intrinsics, camera_pose=pose, timestamp_sec=time.time(),
            camera_frame="thesis_head_color_optical_frame", map_frame="map",
            task_target=query, depth_frame_mm=frame_depth)
        record["observation"] = observation.to_dict()
    return record, resolution


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

    # The look-at anchor for the occluded/empty diagnostics is taken from the
    # image itself (the visible target-colour candidate), never from the
    # registered object slot.  Authorization never uses it at all.
    base_analysis = analyze_frame(rgb, depth_mm, intrinsics, pose, color=arguments.color)
    visible = [candidate for candidate in base_analysis.candidates
               if candidate.color == arguments.color and candidate.visible
               and candidate.depth_m is not None and candidate.pose_xyzyaw is not None]
    if len(visible) != 1:
        raise RuntimeError(f"expected exactly one visible {arguments.color} candidate, got {len(visible)}")
    anchor_candidate = visible[0]
    slot = tuple(float(value) for value in anchor_candidate.pose_xyzyaw[:3])
    x0, y0, x1, y1 = anchor_candidate.bbox_px
    center = (int((x0 + x1) // 2), int((y0 + y1) // 2))
    expected_depth = float(anchor_candidate.depth_m)
    oracle = {"unavailable": "read-only snapshot endpoint not queried"}
    try:
        state = snapshot()
        oracle = state["sources"]["object_observation"]["data"]
    except Exception as error:  # cross-check only; never used for selection
        oracle = {"unavailable": str(error)}

    Image.fromarray(rgb).save(output / "frame.png")
    Image.fromarray(depth_mm).save(output / "depth.png")
    capture_record = {
        "scope": "isolated_MuJoCo_online_head_camera_frame_NOT_PHYSICAL",
        "object_id": arguments.object_id,
        "frame_sha256": hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest(),
        "depth_sha256": hashlib.sha256(np.ascontiguousarray(depth_mm).tobytes()).hexdigest(),
        "depth_valid_fraction": round(depth_valid_fraction(depth_mm), 6),
        "width": intrinsics.width, "height": intrinsics.height,
        "k": [intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0],
        "camera_pose_map": {"translation": pose.translation_xyz, "quaternion_xyzw": pose.quaternion_xyzw},
        "diagnostic_anchor_source": "image_derived_visible_candidate_not_registered_slot",
        "anchor_candidate_id": anchor_candidate.candidate_id,
        "anchor_point_map": list(slot), "anchor_pixel": list(center),
        "anchor_depth_m": round(expected_depth, 5),
        "oracle_object_observation_cross_check": {"object_id": oracle.get("object_id"),
                                                  "visible": oracle.get("visible"),
                                                  "confidence": oracle.get("confidence")},
        "oracle_used_for_selection": False,
    }
    (output / "capture.json").write_text(json.dumps(capture_record, ensure_ascii=False, indent=2) + "\n")

    # Cover the whole detected blob so an edit cannot leave target-colour residue.
    cover = max(x1 - x0, y1 - y0) // 2 + 4

    def occluded(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, cover, GREY, expected_depth - 0.25)
        return "grey occluder painted 0.25 m in front of the look-at anchor"

    def wrong_class(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, cover, BLUE, expected_depth)
        return f"a {arguments.wrong_color} object painted over the observed object"

    def empty_slot(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, cover, GREY, expected_depth + 0.35)
        return "look-at anchor cleared to plain background 0.35 m behind the object"

    def secondary_pixel():
        height, width = rgb.shape[:2]
        gap = 2 * cover + 8
        for du, dv in ((gap, 0), (-gap, 0), (0, -gap), (gap, -gap), (-gap, -gap)):
            u2, v2 = center[0] + du, center[1] + dv
            if cover <= u2 < width - cover and cover <= v2 < height - cover:
                return (u2, v2)
        raise RuntimeError("no in-frame space for the second red instance")

    def ambiguous(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, secondary_pixel(), cover, RED, expected_depth)
        return "a second red instance painted clear of the first (guard case)"

    def absent(frame_rgb, frame_depth):
        paint(frame_rgb, frame_depth, center, cover, GREY, expected_depth + 0.35)
        return "no target-colour surface anywhere in the frame"

    # Full-frame authorization: no slot coordinate is ever supplied.
    full_frame_cases = [("full_visible", None), ("full_wrong_class", wrong_class),
                        ("full_ambiguous", ambiguous), ("full_absent", absent)]
    # Region-anchored diagnostics: a caller-supplied look-at anchor is required to
    # separate an occluder from an empty region; these never authorize.
    anchored_cases = [("anchored_visible", None), ("anchored_occluded", occluded),
                      ("anchored_empty", empty_slot)]

    results = []
    for name, edit in full_frame_cases:
        record, resolution = run_case(name, rgb, depth_mm, intrinsics, pose, arguments.query,
                                      arguments.color, edit=edit)
        Image.fromarray(record.pop("rgb")).save(output / f"case_{name}.png")
        record["resolution"] = resolution.to_dict()
        results.append(record)
    for name, edit in anchored_cases:
        record, resolution = run_case(name, rgb, depth_mm, intrinsics, pose, arguments.query,
                                      arguments.color, expected_point_map=slot, edit=edit)
        Image.fromarray(record.pop("rgb")).save(output / f"case_{name}.png")
        record["resolution"] = resolution.to_dict()
        results.append(record)

    by_case = {item["case"]: item for item in results}
    expected = {"full_visible": "target_visible", "full_wrong_class": "wrong_class",
                "full_ambiguous": "ambiguous_candidates", "full_absent": "target_absent",
                "anchored_visible": "target_visible", "anchored_occluded": "target_occluded",
                "anchored_empty": "empty_slot"}
    authorized = {"full_visible", "anchored_visible"}
    report = {
        "scope": "P4.3_target_search_fail_closed_audit_isolated_MuJoCo_NOT_PHYSICAL",
        "physical_verification": False,
        "robot_motion_executed": False,
        "selection_basis": "image_derived_candidates_not_oracle_object_id",
        "authorization_reads_slot_coordinates": False,
        "region_anchor_source": "registered_scene_slot_for_occluded_empty_diagnostics_only",
        "query": arguments.query,
        "cases": results,
        "expected": expected,
        "full_frame_mode": "authorizes only from image-derived candidates; no look-at prior",
        "anchored_mode": "occluded/empty diagnostics at a caller-supplied anchor; never authorizes",
    }
    report["passed"] = all(
        by_case[name]["outcome"] == ("authorized" if name in authorized else "failed")
        and by_case[name]["reason"] == expected[name] for name in expected)
    (output / "cases.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "passed": report["passed"],
                      "cases": {item["case"]: item["reason"] for item in results}},
                     ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
