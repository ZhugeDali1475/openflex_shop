"""Statically audit offline retail search viewpoints; never move ROS/live simulation."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"))
sys.path.insert(0, str(ROOT / "scripts"))
from build_openflex_retail_scene import render_segmentation, sha256


def _set_joint(model, data, name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"Missing search-planner joint: {name}")
    if model.jnt_type[joint_id] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
        raise ValueError(f"Search-planner joint is not scalar: {name}")
    if model.jnt_limited[joint_id] and not (model.jnt_range[joint_id, 0] <= value <= model.jnt_range[joint_id, 1]):
        raise ValueError(f"Search plan violates {name} limits")
    data.qpos[model.jnt_qposadr[joint_id]] = value


def audit(scene_path, manifest_path, plan_path, output_directory):
    scene_path = Path(scene_path).expanduser().resolve(strict=True)
    manifest_path = Path(manifest_path).expanduser().resolve(strict=True)
    plan_path = Path(plan_path).expanduser().resolve(strict=True)
    output = Path(output_directory).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output}")
    output.mkdir(parents=True)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan.get("execution_enabled") is not False or plan.get("requires_visual_confirmation") is not True:
        raise ValueError("Only explicit offline search plans can be audited")
    if plan.get("manifest_scene_sha256") != manifest.get("scene_sha256"):
        raise ValueError("Search plan and manifest scene hashes differ")
    if sha256(scene_path) != manifest.get("scene_sha256"):
        raise ValueError("Scene and manifest hashes differ")
    products = {item["instance_id"]: item for item in manifest["products"]}
    observations = [item for item in plan["observations"] if item.get("phase") == "target_observation"]
    if not observations:
        raise ValueError("Search plan contains no target observation viewpoints")
    requested = plan.get("candidate_instance_ids", [])
    if {item["target_instance_ids"][0] for item in observations} != set(requested):
        raise ValueError("Search plan does not have one target viewpoint per candidate")
    os.environ.setdefault("MUJOCO_GL", "egl")
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    camera_name = "thesis_head_camera"
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name) < 0:
        raise ValueError("Scene is missing the calibrated-name head camera")
    candidates = []
    with mujoco.Renderer(model, height=384, width=512) as renderer:
        for observation in observations:
            target_id = observation["target_instance_ids"][0]
            product = products[target_id]
            pose = observation["base_pose"]
            for joint_name, value in (
                ("base_x", float(pose["x_m"])), ("base_y", float(pose["y_m"])),
                ("base_yaw", float(pose["yaw_rad"])),
                ("lift_joint", float(observation.get("lift_m", 0.0))),
                ("openarmx_head_yaw_joint", float(observation["head_yaw_rad"])),
                ("openarmx_head_pitch_joint", float(observation["head_pitch_rad"])),
            ):
                _set_joint(model, data, joint_name, value)
            mujoco.mj_forward(model, data)
            segmentation = render_segmentation(
                renderer, model, data, camera_name, option, np, mujoco)
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, product["visual_geom"])
            pixels = int(np.count_nonzero(
                (segmentation[:, :, 0] == geom_id) &
                (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))))
            contacts = [{
                "geom1": model.geom(contact.geom1).name,
                "geom2": model.geom(contact.geom2).name,
                "body1": model.body(model.geom_bodyid[contact.geom1]).name,
                "body2": model.body(model.geom_bodyid[contact.geom2]).name,
                "distance_m": float(contact.dist),
            } for contact in data.contact[:data.ncon] if contact.dist < -1e-6]
            candidates.append({
                "instance_id": target_id, "simulation_sku": product["simulation_sku"],
                "base_pose": pose, "head_yaw_rad": observation["head_yaw_rad"],
                "head_pitch_rad": observation["head_pitch_rad"],
                "attempt": observation.get("attempt", 0),
                "lift_m": observation.get("lift_m", 0.0),
                "target_visible_pixels": pixels,
                "target_visible": pixels > 0,
                "static_penetrations_over_1um": contacts,
                "viewpoint_transition_validated": False,
            })
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "validation": "isolated_static_viewpoint_and_segmentation_oracle_audit",
        "scene_path": str(scene_path), "scene_sha256": sha256(scene_path),
        "manifest_path": str(manifest_path), "plan_path": str(plan_path),
        "ros_accessed": False, "live_runtime_modified": False, "simulation_steps": 0,
        "segmentation_is_evaluation_ground_truth": True,
        "visual_recognition_performed": False,
        "static_viewpoints_with_target_visible": sum(item["target_visible"] for item in candidates),
        "target_viewpoints": len(candidates),
        "viewpoints_with_robot_or_scene_penetration": sum(
            bool(item["static_penetrations_over_1um"]) for item in candidates),
        "candidates": candidates,
        "execution_ready": False,
        "limitations": [
            "Static joint assignment is a camera-visibility snapshot, not a reachable trajectory",
            "Segmentation pixels are MuJoCo ground truth, not model recognition output",
            "No depth localization, grasp planning, control lease or motion safety was evaluated",
        ],
    }
    (output / "search_viewpoint_audit.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.scene, args.manifest, args.plan, args.output)
    print(json.dumps({
        "output": str(args.output.resolve()),
        "target_viewpoints": report["target_viewpoints"],
        "visible": report["static_viewpoints_with_target_visible"],
        "penetrating_viewpoints": report["viewpoints_with_robot_or_scene_penetration"],
        "execution_ready": report["execution_ready"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
