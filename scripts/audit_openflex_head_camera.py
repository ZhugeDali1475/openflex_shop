#!/usr/bin/env python3
import argparse
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import sys
from urllib.request import ProxyHandler, build_opener
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"))
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_thesis_mujoco.camera_spec import HEAD_CAMERA
from openflex_thesis_mujoco.model_builder import build_thesis_model
from openflex_roboos_bridge.scene_registry import load_scene_registry


def replay_snapshot(model, data, snapshot):
    for key in ("joint_states", "object_observation"):
        if snapshot["sources"][key]["status"] != "fresh":
            raise ValueError(f"Cannot replay stale or missing source: {key}")
    joints = snapshot["sources"]["joint_states"]["data"]
    if len(joints["name"]) != len(joints["position"]):
        raise ValueError("Joint position count mismatch")
    for name, position in zip(joints["name"], joints["position"]):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint >= 0 and model.jnt_type[joint] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            if position is None or not np.isfinite(position):
                raise ValueError("Invalid joint position")
            data.qpos[model.jnt_qposadr[joint]] = position
    observation = snapshot["sources"]["object_observation"]["data"]
    if observation["header"]["frame_id"] != "map":
        raise ValueError("Object replay requires map frame")
    pose = observation["pose"]["pose"]
    position, quaternion = pose["position"], pose["orientation"]
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "thesis_object_free")
    offset = model.jnt_qposadr[joint]
    data.qpos[offset:offset + 7] = [position["x"], position["y"], position["z"],
                                   quaternion["w"], quaternion["x"], quaternion["y"], quaternion["z"]]
    if not np.isfinite(data.qpos).all():
        raise ValueError("Invalid replay pose")
    mujoco.mj_forward(model, data)


def camera_metrics(model, data, camera_name, output):
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "thesis_object_geom")
    if min(camera, target) < 0:
        raise ValueError("Missing camera or target geometry")
    position = data.cam_xpos[camera].copy()
    rotation = data.cam_xmat[camera].reshape(3, 3)
    direction = -rotation[:, 2]
    ray_geom = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(model, data, position, direction, None, 1, -1, ray_geom)
    geom = int(ray_geom[0])
    renderer = mujoco.Renderer(model, height=240, width=320)
    try:
        renderer.update_scene(data, camera=camera_name)
        Image.fromarray(renderer.render()).save(output)
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera=camera_name)
        segmentation = renderer.render()
        mask = (segmentation[:, :, 0] == target) & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
        rows, columns = np.where(mask)
        bounding_box = [int(columns.min()), int(rows.min()), int(columns.max()), int(rows.max())] if rows.size else None
    finally:
        renderer.close()
    half_size = model.geom_size[target]
    corners = np.array([[right, up, depth] for right in (-1, 1) for up in (-1, 1)
                        for depth in (-1, 1)]) * half_size
    world_corners = corners @ data.geom_xmat[target].reshape(3, 3).T + data.geom_xpos[target]
    camera_corners = (world_corners - position) @ rotation
    tangent = np.tan(np.deg2rad(model.cam_fovy[camera]) / 2)
    depths = -camera_corners[:, 2]
    inside = bool(np.all(depths > 0) and np.all(np.abs(camera_corners[:, 1]) < depths * tangent)
                  and np.all(np.abs(camera_corners[:, 0]) < depths * tangent * 320 / 240))
    return {
        "image": str(output.relative_to(ROOT)), "camera_position_map": position.tolist(),
        "forward_map": direction.tolist(), "fovy_degrees": float(model.cam_fovy[camera]),
        "center_ray_distance_m": float(distance),
        "center_ray_body": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom]) if geom >= 0 else None,
        "target_pixels": int(mask.sum()), "target_bbox_xyxy": bounding_box,
        "all_target_corners_inside_frustum": inside,
        "provenance": "renderer_segmentation_ORACLE_not_model_detection",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--live", action="store_true")
    args = parser.parse_args()
    output = ROOT / "local_dependencies/openflex_roboos/runs" / datetime.now().strftime("camera_%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    config_path = ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml"
    corrected_path = build_thesis_model(str(ROOT / "openflex_mujoco-1.0_basic/openflex_mujoco.xml"), str(output / "corrected.xml"))
    tree = ET.parse(corrected_path)
    camera = tree.find(f".//body[@name='{HEAD_CAMERA.optical_frame}']/camera")
    tree.find(f".//body[@name='{HEAD_CAMERA.optical_frame}']").remove(camera)
    camera.set("fovy", "58")
    tree.find(".//body[@name='d435_color_optical_frame']").append(camera)
    legacy_path = output / "legacy.xml"
    tree.write(legacy_path)
    model = mujoco.MjModel.from_xml_path(str(corrected_path))
    legacy_model = mujoco.MjModel.from_xml_path(str(legacy_path))
    data, legacy_data = mujoco.MjData(model), mujoco.MjData(legacy_model)
    if args.live:
        with build_opener(ProxyHandler({})).open("http://127.0.0.1:18552/state", timeout=3) as response:
            snapshot = json.load(response)
        (output / "replayed_state.json").write_text(json.dumps(snapshot, ensure_ascii=False, indent=2))
        replay_snapshot(model, data, snapshot)
    else:
        config = yaml.safe_load(config_path.read_text())["openflex_thesis_mujoco_bridge"]["ros__parameters"]
        for index, position in enumerate(config["inactive_arm_park_positions"], start=1):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"openarmx_left_joint{index}")
            data.qpos[model.jnt_qposadr[joint]] = position
        mujoco.mj_forward(model, data)
    legacy_data.qpos[:] = data.qpos
    mujoco.mj_forward(legacy_model, legacy_data)
    before_qpos = data.qpos.copy()
    corrected = camera_metrics(model, data, "thesis_head_camera", output / "corrected.png")
    legacy = camera_metrics(legacy_model, legacy_data, "thesis_head_camera", output / "legacy.png")
    geometry_unchanged = all(np.array_equal(getattr(model, field), getattr(legacy_model, field))
                             for field in ("geom_size", "geom_pos", "geom_quat", "geom_rgba", "geom_group", "geom_contype", "geom_conaffinity"))
    report = {"camera_spec": asdict(HEAD_CAMERA), "legacy": legacy, "corrected": corrected,
              "live_telemetry_replay": args.live, "robot_motion_executed": False,
              "qpos_unchanged": bool(np.array_equal(before_qpos, data.qpos)),
              "geometry_and_visibility_settings_unchanged": geometry_unchanged,
              "scene_registry": load_scene_registry(config_path, corrected_path)}
    report["passed"] = bool(corrected["target_pixels"] >= 40 and corrected["all_target_corners_inside_frustum"]
                            and legacy["target_pixels"] == 0 and geometry_unchanged and report["qpos_unchanged"])
    (output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(output / "result.json")
    if not report["passed"]:
        raise RuntimeError("Camera visibility audit failed; inspect evidence")


if __name__ == "__main__":
    main()
