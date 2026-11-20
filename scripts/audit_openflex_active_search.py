#!/usr/bin/env python3
"""Offline, ROS-free audit of the phase C multi-viewpoint active search.

Observation stands are derived *only* from the static shelf collision geometry
(``shelf_viewpoints``), then a set of deterministic searches is replayed where
the product is rendered into a synthetic head-camera frame at each stand and
passed through the *real* target-search contract
(``target_search_contract.analyze_frame`` / ``resolve_search_target``).

The renderer is given the product's hidden lateral position, but that value never
reaches the viewpoint plan or the state machine: the plan depends solely on the
shelf geometry and the search is driven only by image-derived resolutions.  The
audit never starts ROS, never reads a registered slot and never commands motion.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.active_search import (  # noqa: E402
    AMBIGUOUS, CANCELLED, FOUND, INVALID_DEPTH, NOT_FOUND, NOT_SETTLED, STALE_FRAME,
    TIMEOUT, ActiveSearch,
)
from openflex_roboos_bridge.shelf_viewpoints import (  # noqa: E402
    generate_shelf_viewpoints, shelf_front_geometry,
)
from openflex_roboos_bridge.target_search_contract import (  # noqa: E402
    CameraIntrinsics, CameraPose, analyze_frame, depth_valid_fraction, project_point,
    resolve_search_target,
)
from openflex_roboos_bridge.visual_grasp_scene import load_shelf_scene  # noqa: E402

SCENE_PATH = (ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/"
              "sim_shelf_scene.yaml")

# Head camera reported by the sim CameraInfo snapshot (see phase B audit section 20).
IMAGE_WIDTH, IMAGE_HEIGHT = 320, 240
FOCAL_LENGTH_PX = 171.37776080905377
HEAD_CAMERA_HEIGHT_M = 1.312
PRODUCT_PLAN_X_M = 1.12
PRODUCT_PLAN_Z_M = 0.84

STANDOFF_M = 0.75
LATERAL_STEP_M = 0.30
CLEARANCE_M = 0.05
HORIZONTAL_FOV_RAD = 2.0 * math.atan(0.5 * IMAGE_WIDTH / FOCAL_LENGTH_PX)

QUERY = "red box"
RED = (220, 40, 40)
BLUE = (40, 60, 220)
GREY = (150, 150, 150)
BACKGROUND_MM = 4000


def head_intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics.from_k(IMAGE_WIDTH, IMAGE_HEIGHT, [
        FOCAL_LENGTH_PX, 0.0, IMAGE_WIDTH / 2.0,
        0.0, FOCAL_LENGTH_PX, IMAGE_HEIGHT / 2.0,
        0.0, 0.0, 1.0])


def _quaternion_xyzw_from_matrix(matrix) -> tuple:
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, \
            (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, \
            (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, \
            (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, \
            (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return (float(x), float(y), float(z), float(w))


# Head camera axes expressed in the map frame: camera +z looks along map +x.
HEAD_CAMERA_QUATERNION_XYZW = _quaternion_xyzw_from_matrix(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def plan_viewpoints(scene_path=SCENE_PATH):
    obstacles = load_shelf_scene(scene_path).obstacles
    viewpoints = generate_shelf_viewpoints(
        obstacles, standoff_m=STANDOFF_M, lateral_step_m=LATERAL_STEP_M,
        horizontal_fov_rad=HORIZONTAL_FOV_RAD, clearance_m=CLEARANCE_M)
    return obstacles, viewpoints


def camera_pose_at(viewpoint, *, camera_height_m=HEAD_CAMERA_HEIGHT_M) -> CameraPose:
    return CameraPose((viewpoint.position_xy[0], viewpoint.position_xy[1], camera_height_m),
                      HEAD_CAMERA_QUATERNION_XYZW)


def render_view(viewpoint, lateral_positions, *, intrinsics, color=RED, blob_half_px=10,
                camera_height_m=HEAD_CAMERA_HEIGHT_M, background_mm=BACKGROUND_MM):
    """Render one synthetic head frame with product blobs at hidden ``lateral_positions``."""

    rgb = np.full((IMAGE_HEIGHT, IMAGE_WIDTH, 3), GREY, dtype=np.uint8)
    depth = np.full((IMAGE_HEIGHT, IMAGE_WIDTH), int(background_mm), dtype=np.uint16)
    pose = camera_pose_at(viewpoint, camera_height_m=camera_height_m)
    projected = []
    for lateral in lateral_positions:
        point = (PRODUCT_PLAN_X_M, float(lateral), PRODUCT_PLAN_Z_M)
        u, v, forward = _project(pose, intrinsics, point)
        if u is None:
            continue
        projected.append({"lateral_y": float(lateral), "pixel": [round(u, 2), round(v, 2)],
                          "frame_mm": round(forward * 1000.0, 1)})
        ui, vi = int(round(u)), int(round(v))
        rgb[vi - blob_half_px:vi + blob_half_px, ui - blob_half_px:ui + blob_half_px] = color
        depth[vi - blob_half_px:vi + blob_half_px, ui - blob_half_px:ui + blob_half_px] = \
            int(round(forward * 1000.0))
    return rgb, depth, pose, projected


def _project(pose, intrinsics, point):
    try:
        u, v, forward = project_point(pose, intrinsics, point)
    except ValueError:
        return None, None, None
    if not (0.0 <= u < IMAGE_WIDTH and 0.0 <= v < IMAGE_HEIGHT):
        return None, None, None
    return u, v, forward


def observe(viewpoint, lateral_positions, *, intrinsics, color=RED, blob_half_px=10,
            background_mm=BACKGROUND_MM):
    rgb, depth, pose, projected = render_view(
        viewpoint, lateral_positions, intrinsics=intrinsics, color=color,
        blob_half_px=blob_half_px, background_mm=background_mm)
    analysis = analyze_frame(rgb, depth, intrinsics, pose, color="red")
    resolution = resolve_search_target(analysis, QUERY, color="red")
    return {"resolution": resolution, "depth_valid_fraction": depth_valid_fraction(depth),
            "projected": projected, "candidate_count": len(analysis.candidates)}


def drive(search, per_view, *, settled=True, frame_timestamp_sec=100.0, now_sec=100.1,
          depth_override=None):
    steps = []
    while not search.finished:
        viewpoint = search.next_viewpoint()
        if viewpoint is None:
            break
        observation = per_view[viewpoint.viewpoint_id]
        fraction = observation["depth_valid_fraction"] if depth_override is None else depth_override
        decision = search.submit(
            viewpoint, frame_timestamp_sec=frame_timestamp_sec, now_sec=now_sec,
            depth_valid_fraction=fraction, resolution=observation["resolution"], settled=settled)
        steps.append({"viewpoint_id": viewpoint.viewpoint_id,
                      "resolution_outcome": observation["resolution"].outcome,
                      "resolution_reason": observation["resolution"].reason,
                      "decision_outcome": decision.outcome})
    return search, steps


def _scenario(name, expected, search, steps, *, extra=None):
    record = {"name": name, "expected_outcome": expected, "outcome": search.outcome,
              "passed": search.outcome == expected, "steps": steps,
              "detail": search.report()["detail"]}
    if extra:
        record.update(extra)
    return record


def run_audit(scene_path=SCENE_PATH) -> dict:
    intrinsics = head_intrinsics()
    obstacles, viewpoints = plan_viewpoints(scene_path)
    front_x, y_min, y_max = shelf_front_geometry(obstacles)
    results = []

    def per_view(lateral_positions, *, color=RED, background_mm=BACKGROUND_MM):
        return {viewpoint.viewpoint_id: observe(
            viewpoint, lateral_positions, intrinsics=intrinsics, color=color,
            background_mm=background_mm) for viewpoint in viewpoints}

    # 1. Hidden position sweep: viewpoint plan never changes with the product slot.
    for lateral in (-0.6, -0.2, 0.0, 0.3, 0.6):
        search = ActiveSearch(viewpoints=viewpoints)
        search, steps = drive(search, per_view([lateral]))
        results.append(_scenario(f"hidden_position_{lateral:+.2f}", FOUND, search, steps,
                                 extra={"hidden_lateral_y": lateral}))

    # 2. Product outside every observed window -> fail closed, no grasp.
    search = ActiveSearch(viewpoints=viewpoints)
    search, steps = drive(search, per_view([1.5]))
    results.append(_scenario("outside_shelf", NOT_FOUND, search, steps,
                             extra={"hidden_lateral_y": 1.5}))

    # 3. Two visible reds -> ambiguous, never grasped.
    search = ActiveSearch(viewpoints=viewpoints)
    search, steps = drive(search, per_view([0.0, 0.15]))
    results.append(_scenario("two_reds_ambiguous", AMBIGUOUS, search, steps))

    # 4. Wrong class only -> never authorized.
    search = ActiveSearch(viewpoints=viewpoints)
    search, steps = drive(search, per_view([0.0], color=BLUE))
    results.append(_scenario("wrong_class_only", NOT_FOUND, search, steps))

    # 5. Stale frame -> refuted before any authorization.
    search = ActiveSearch(viewpoints=viewpoints)
    search, steps = drive(search, per_view([0.0]), frame_timestamp_sec=0.0, now_sec=200.0)
    results.append(_scenario("stale_frame", STALE_FRAME, search, steps))

    # 6. No usable depth -> refuted.
    search = ActiveSearch(viewpoints=viewpoints)
    search, steps = drive(search, per_view([0.0]), depth_override=0.0)
    results.append(_scenario("invalid_depth", INVALID_DEPTH, search, steps))

    # 7. Platform still moving -> refuted.
    search = ActiveSearch(viewpoints=viewpoints)
    search, steps = drive(search, per_view([0.0]), settled=False)
    results.append(_scenario("not_settled", NOT_SETTLED, search, steps))

    # 8. Deadline exceeded with a fresh frame -> timeout.
    search = ActiveSearch(viewpoints=viewpoints, deadline_sec=100.0)
    search, steps = drive(search, per_view([0.0]), frame_timestamp_sec=200.0, now_sec=200.0)
    results.append(_scenario("deadline_timeout", TIMEOUT, search, steps))

    # 9. Operator cancellation is terminal.
    search = ActiveSearch(viewpoints=viewpoints)
    search.cancel()
    results.append(_scenario("cancelled", CANCELLED, search, [], extra={"steps": []}))

    report = {
        "scope": "phase_C_multi_viewpoint_active_search_offline_NOT_PHYSICAL",
        "physical_verification": False,
        "robot_motion_executed": False,
        "ros_started": False,
        "viewpoint_source": "static_shelf_geometry_only",
        "authorization_reads_slot_coordinates": False,
        "perception_backend":
            "target_search_contract.analyze_frame+resolve_search_target on synthetic head RGB-D",
        "scene_path": str(scene_path),
        "shelf_front_geometry": {"front_x": front_x, "y_min": y_min, "y_max": y_max},
        "viewpoint_config": {"standoff_m": STANDOFF_M, "lateral_step_m": LATERAL_STEP_M,
                             "clearance_m": CLEARANCE_M,
                             "horizontal_fov_rad": HORIZONTAL_FOV_RAD},
        "viewpoints": [viewpoint.to_dict() for viewpoint in viewpoints],
        "scenarios": results,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    report["passed"] = all(item["passed"] for item in results)
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default=str(SCENE_PATH))
    parser.add_argument("--output")
    arguments = parser.parse_args(argv)
    report = run_audit(Path(arguments.scene))
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if arguments.output:
        Path(arguments.output).write_text(text + "\n", encoding="utf-8")
        print(arguments.output)
    else:
        print(text)
    summary = {"passed": report["passed"],
               "viewpoints": len(report["viewpoints"]),
               "scenarios": {item["name"]: item["outcome"] for item in report["scenarios"]}}
    print(json.dumps(summary, ensure_ascii=False, indent=2), file=sys.stderr)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
