"""Online image-derived ``search_and_pick`` skill (P4.3).

Wired into ``NavigationNode`` next to ``ManipulationSkill``/``PlaceSkill``.  It
subscribes to the head camera (RGB + aligned depth + intrinsics), resolves the
natural-language query from the *live* frame with ``target_search_contract``
(oracle object observation is never used for selection), records per-attempt
evidence (frame PNG + depth PNG + candidate list + model output + selection
reason), and only when the target is uniquely observed delegates to the
verified grasp skill.

Failure is always fail-closed: an absent / occluded / wrong-class / ambiguous /
uncertain observation re-observes a bounded number of times and then ends the
operation safely with the recorded reason and no motion.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
from PIL import Image
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image as RosImage
from tf2_ros import Buffer, TransformListener

from .target_search_contract import (
    CameraIntrinsics,
    CameraPose,
    analyze_frame,
    resolve_search_target,
)
from .target_observation import observation_from_candidate


COLORS = ("red", "blue", "green", "yellow", "magenta")
MAX_FRAME_AGE_SEC = 0.75
STAMP_TOLERANCE_SEC = 0.05


def _stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class SearchSkill:
    def __init__(self, node):
        if os.environ.get("OPENFLEX_ENABLE_SIM_MANIPULATION") != "1":
            raise RuntimeError("Search requires the isolated simulation manipulation opt-in")
        self.node = node
        self.root = Path(__file__).resolve().parents[5]
        self.runs = self.root / "local_dependencies/openflex_roboos/runs"
        self.color = None
        self.depth = None
        self.info = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)
        node.create_subscription(RosImage, "/thesis_sim/head/color/image_raw",
                                 self.on_color, qos_profile_sensor_data, callback_group=node.group)
        node.create_subscription(RosImage, "/thesis_sim/head/aligned_depth_to_color/image_raw",
                                 self.on_depth, qos_profile_sensor_data, callback_group=node.group)
        node.create_subscription(CameraInfo, "/thesis_sim/head/color/camera_info",
                                 self.on_info, qos_profile_sensor_data, callback_group=node.group)
        self.color_frame = "thesis_head_color_optical_frame"

    def on_color(self, message):
        self.color = message

    def on_depth(self, message):
        self.depth = message

    def on_info(self, message):
        self.info = message

    # ---- observation -------------------------------------------------
    def _fresh_frame(self, timeout=6.0):
        deadline = time.monotonic() + timeout
        while True:
            color, depth, info = self.color, self.depth, self.info
            if color is not None and depth is not None and info is not None:
                color_stamp, depth_stamp = _stamp_seconds(color.header.stamp), _stamp_seconds(depth.header.stamp)
                now = _stamp_seconds(self.node.get_clock().now().to_msg())
                fresh = (abs(color_stamp - depth_stamp) <= STAMP_TOLERANCE_SEC
                         and 0.0 <= now - color_stamp <= MAX_FRAME_AGE_SEC
                         and color.width == depth.width == info.width
                         and color.height == depth.height == info.height)
                if fresh:
                    return color, depth, info
            if time.monotonic() > deadline:
                raise RuntimeError("No fresh head camera RGB/depth/info triplet available")
            time.sleep(0.02)

    def _camera_pose(self):
        transform = self.tf.lookup_transform("map", self.color_frame, rclpy_time())
        translation, rotation = transform.transform.translation, transform.transform.rotation
        return CameraPose((translation.x, translation.y, translation.z),
                          (rotation.x, rotation.y, rotation.z, rotation.w))

    def analyze_once(self, query, color):
        """Capture one frame and resolve it; returns (resolution, artifacts)."""
        color_message, depth_message, info_message = self._fresh_frame()
        rgb = np.frombuffer(color_message.data, dtype=np.uint8).reshape(
            color_message.height, color_message.width, 3).copy()
        depth = np.frombuffer(depth_message.data, dtype=np.uint16).reshape(
            depth_message.height, depth_message.width).copy()
        intrinsics = CameraIntrinsics.from_k(info_message.width, info_message.height, list(info_message.k))
        pose = self._camera_pose()
        analysis = analyze_frame(rgb, depth, intrinsics, pose, color=color)
        resolution = resolve_search_target(analysis, query, color=color, object_id=None)
        observation = None
        if resolution.outcome == "authorized":
            candidate = next(item for item in analysis.candidates
                             if item.candidate_id == resolution.selected_candidate_id)
            x0, y0, x1, y1 = candidate.bbox_px
            u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            depth_m = float(candidate.depth_m)
            position_camera = ((u - intrinsics.cx) * depth_m / intrinsics.fx,
                               (v - intrinsics.cy) * depth_m / intrinsics.fy, depth_m)
            depth_roi = depth[max(0, y0):min(depth.shape[0], y1 + 1),
                              max(0, x0):min(depth.shape[1], x1 + 1)]
            depth_valid_fraction = float(np.count_nonzero(depth_roi > 0) / depth_roi.size) if depth_roi.size else 0.0
            timestamp = _stamp_seconds(color_message.header.stamp)
            observation = observation_from_candidate(
                candidate, task_target=query, timestamp_sec=timestamp,
                camera_frame=self.color_frame, map_frame="map",
                position_camera_xyz=position_camera,
                depth_valid_fraction=depth_valid_fraction,
                track_id=resolution.selected_perception_track_id)
        artifacts = {"rgb": rgb, "depth": depth, "analysis": analysis,
                     "observation": observation}
        return resolution, artifacts

    def save_evidence(self, directory: Path, index: int, resolution, artifacts):
        directory.mkdir(parents=True, exist_ok=True)
        image_path = directory / f"attempt{index:02d}_frame.png"
        depth_path = directory / f"attempt{index:02d}_depth.png"
        Image.fromarray(artifacts["rgb"]).save(image_path)
        Image.fromarray(artifacts["depth"]).save(depth_path)
        record = resolution.to_dict()
        if artifacts.get("observation") is not None:
            record["target_observation"] = artifacts["observation"].to_dict()
        record["frame_ref"] = str(image_path.relative_to(self.root))
        record["depth_ref"] = str(depth_path.relative_to(self.root))
        record["attempt"] = index
        (directory / f"attempt{index:02d}_resolution.json").write_text(
            json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        return record

    # ---- operation ---------------------------------------------------
    def submit(self, arguments):
        if set(arguments) != {"query", "request_id", "timeout_sec", "color"}:
            raise ValueError("Expected query, request_id, timeout_sec and color")
        if not isinstance(arguments["query"], str) or not arguments["query"].strip():
            raise ValueError("query must be a non-empty string")
        if arguments["color"] not in COLORS:
            raise ValueError(f"color must be one of {COLORS}")
        if isinstance(arguments["timeout_sec"], bool) or not isinstance(arguments["timeout_sec"], (int, float)):
            raise ValueError("timeout_sec must be a JSON number")
        timeout = float(arguments["timeout_sec"])
        if not math.isfinite(timeout) or not 1.0 <= timeout <= 600.0:
            raise ValueError("timeout_sec must be 1-600")
        arguments = dict(arguments, timeout_sec=timeout)
        record, accepted = self.node.store.begin(arguments["request_id"], "search_and_pick", arguments)
        if accepted:
            threading.Thread(target=self.execute, args=(arguments,), daemon=True).start()
        return record

    def execute(self, arguments):
        request_id = arguments["request_id"]
        deadline = time.monotonic() + float(arguments["timeout_sec"])
        directory = self.runs / f"search_{request_id}"
        status, success, detail, stopped = "failed", False, "", False
        attempts, resolution, record = 0, None, None
        try:
            while attempts < 3:
                attempts += 1
                try:
                    resolution, artifacts = self.analyze_once(arguments["query"], arguments["color"])
                except Exception as error:
                    detail = f"observation {attempts} failed: {error}"
                    time.sleep(0.4)
                    continue
                record = self.save_evidence(directory, attempts, resolution, artifacts)
                self.node.store.event(request_id, "search_observation", record)
                if resolution.outcome == "authorized":
                    break
                detail = f"{resolution.reason}: {resolution.selection_reason}"
                time.sleep(0.4)
            if resolution is None or resolution.outcome != "authorized":
                status = "failed"
                stopped = self.node.stopped()
                detail = detail or "target was never uniquely observed"
            elif time.monotonic() >= deadline:
                status, detail = "timed_out", "deadline reached before grasp"
                stopped = self.node.stopped()
            else:
                # The visual observation is not yet wired through the grasp
                # planner.  Never fall back to the registered MuJoCo object id.
                status = "failed"
                detail = "visual candidate found, but visual-observation grasp input is not integrated; no motion issued"
                stopped = self.node.stopped()
        except Exception as error:
            detail = str(error)
            try:
                stopped = self.node.stopped()
            except Exception:
                stopped = False
        if not stopped:
            try:
                stopped = self.node.stopped()
            except Exception:
                stopped = False
        self.node.store.finish(
            request_id, status, success=success and stopped, detail=detail,
            stopped_confirmed=stopped, physical_verification=False,
            attempts=attempts, search=record, evidence_dir=str(directory.relative_to(self.root)),
            backend="image_derived_colour_candidates_NOT_oracle_id")


def rclpy_time():
    import rclpy
    return rclpy.time.Time()
