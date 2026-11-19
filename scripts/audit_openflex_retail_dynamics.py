"""Audit isolated retail-product settling; never connects to ROS or live files."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

# Make both `python scripts/...` and import-by-path tests resolve the sibling CLI.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_openflex_retail_scene import sha256


DEFAULT_SCENE = (Path(__file__).resolve().parents[1] /
                 "local_dependencies/openflex_thesis/retail_fixture_20261112_v4/scene.xml")


def _product_contact(model, contact, body_id):
    return model.geom_bodyid[contact.geom1] == body_id or model.geom_bodyid[contact.geom2] == body_id


def _hold_position_actuators(model, data):
    """Hold every position actuator at the source model's qpos0 for isolation."""
    for actuator_id in range(model.nu):
        if model.actuator_trntype[actuator_id] != mujoco.mjtTrn.mjTRN_JOINT:
            raise ValueError("Retail isolation requires joint position actuators")
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        data.ctrl[actuator_id] = model.qpos0[model.jnt_qposadr[joint_id]]


def _disable_robot_collisions(model):
    """Remove robot geometry from the product-only contact subproblem."""
    root_id = model.body("openflex_robot_yaw_root").id
    robot_bodies = {root_id}
    for body_id in range(model.nbody):
        parent = int(model.body_parentid[body_id])
        while parent and parent not in robot_bodies:
            parent = int(model.body_parentid[parent])
        if parent in robot_bodies:
            robot_bodies.add(body_id)
    robot_geoms = [geom_id for geom_id in range(model.ngeom)
                   if int(model.geom_bodyid[geom_id]) in robot_bodies]
    for geom_id in robot_geoms:
        model.geom_contype[geom_id] = 0
        model.geom_conaffinity[geom_id] = 0
    return len(robot_geoms)


def audit(scene_path, duration_s=10.0, output_directory=None):
    scene_path = Path(scene_path).expanduser().resolve(strict=True)
    if not np.isfinite(duration_s) or duration_s <= 0.0 or duration_s > 120.0:
        raise ValueError("duration must be finite and in (0, 120] seconds")
    if output_directory is not None:
        output = Path(output_directory).expanduser().resolve()
        if output.exists():
            raise FileExistsError(f"Refusing to overwrite an existing directory: {output}")
        output.mkdir(parents=True)
    else:
        output = None

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    disabled_robot_geoms = _disable_robot_collisions(model)
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    _hold_position_actuators(model, data)
    mujoco.mj_forward(model, data)
    names = sorted(name for name in [model.body(i).name for i in range(model.nbody)]
                   if (name.startswith("retail_") or name.startswith("product_")) and
                   not name.endswith("_collision"))
    body_ids = {name: int(model.body(name).id) for name in names}
    initial_positions = {name: data.xpos[body_ids[name]].copy() for name in names}
    joint_names = {model.joint(index).name for index in range(model.njnt)}
    joint_dofs = {}
    for name in names:
        joint_name = name + "_free" if name + "_free" in joint_names else name + "_joint"
        joint_dofs[name] = int(model.joint(joint_name).dofadr[0])
    tracked_bodies = {body_id: name for name, body_id in body_ids.items()}
    robot_root = model.body("openflex_robot_yaw_root").id
    robot_qpos_indices = []
    for joint_id in range(model.njnt):
        body_id = int(model.jnt_bodyid[joint_id])
        while body_id != 0 and body_id != robot_root:
            body_id = int(model.body_parentid[body_id])
        if body_id != robot_root:
            continue
        joint_type = model.jnt_type[joint_id]
        width = 7 if joint_type == mujoco.mjtJoint.mjJNT_FREE else (
            4 if joint_type == mujoco.mjtJoint.mjJNT_BALL else 1)
        start = int(model.jnt_qposadr[joint_id])
        robot_qpos_indices.extend(range(start, start + width))
    if not robot_qpos_indices:
        raise ValueError("Could not resolve robot generalized coordinates")
    robot_qpos_indices = np.asarray(robot_qpos_indices, dtype=int)
    initial_robot_qpos = data.qpos[robot_qpos_indices].copy()
    steps = int(np.ceil(duration_s / model.opt.timestep))
    peak_linear = {name: 0.0 for name in names}
    peak_angular = {name: 0.0 for name in names}
    peak_penetration = {name: 0.0 for name in names}
    recent_samples = {name: [] for name in names}
    sample_stride = max(1, int(round(0.1 / model.opt.timestep)))

    for step in range(steps):
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError(f"non-finite isolated MuJoCo state at step {step}")
        penetrations = {name: 0.0 for name in names}
        for contact in data.contact[:data.ncon]:
            for geom_id in (contact.geom1, contact.geom2):
                body_id = int(model.geom_bodyid[geom_id])
                name = tracked_bodies.get(body_id)
                if name is not None:
                    penetrations[name] = min(penetrations[name], float(contact.dist))
        for name in names:
            body_id = body_ids[name]
            dof = joint_dofs[name]
            linear = float(np.linalg.norm(data.qvel[dof:dof + 3]))
            angular = float(np.linalg.norm(data.qvel[dof + 3:dof + 6]))
            peak_linear[name] = max(peak_linear[name], linear)
            peak_angular[name] = max(peak_angular[name], angular)
            penetration = penetrations[name]
            peak_penetration[name] = min(peak_penetration[name], penetration)
            if step >= steps - max(sample_stride * 10, 1) and step % sample_stride == 0:
                recent_samples[name].append({
                    "time_s": float(data.time), "linear_speed_m_s": linear,
                    "angular_speed_rad_s": angular, "z_m": float(data.xpos[body_id, 2]),
                    "minimum_contact_distance_m": penetration,
                })

    products = []
    for name in names:
        body_id = body_ids[name]
        displacement = float(np.linalg.norm(data.xpos[body_id] - initial_positions[name]))
        samples = recent_samples[name]
        max_recent_linear = max((sample["linear_speed_m_s"] for sample in samples), default=float("inf"))
        max_recent_angular = max((sample["angular_speed_rad_s"] for sample in samples), default=float("inf"))
        products.append({
            "instance_id": name,
            "displacement_m": displacement,
            "final_position_m": data.xpos[body_id].tolist(),
            "final_quaternion_wxyz": data.xquat[body_id].tolist(),
            "peak_linear_speed_m_s": peak_linear[name],
            "peak_angular_speed_rad_s": peak_angular[name],
            "peak_penetration_m": peak_penetration[name],
            "recent_max_linear_speed_m_s": max_recent_linear,
            "recent_max_angular_speed_rad_s": max_recent_angular,
            "settled_under_current_heuristic": bool(
                max_recent_linear <= 0.01 and max_recent_angular <= 0.05 and
                peak_penetration[name] >= -0.001),
            "samples": samples,
        })

    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scene_path": str(scene_path), "scene_sha256": sha256(scene_path),
        "validation": "isolated_mj_step_product_settling_audit",
        "simulation_steps": steps, "simulated_duration_s": float(data.time),
        "ros_accessed": False, "live_runtime_modified": False,
        "initial_state": "scene_qpos0_and_zero_velocity; position_actuators_hold_qpos0; not_a_safe_parking_certificate",
        "robot_control_for_isolation": "position_actuators_commanded_to_source_qpos0",
        "robot_collision_for_isolation": {
            "disabled_geoms": disabled_robot_geoms,
            "meaning": "product-only contact subproblem; not whole-robot collision validation",
        },
        "robot_qpos_drift_max": float(np.max(np.abs(
            data.qpos[robot_qpos_indices] - initial_robot_qpos))),
        "criteria": {
            "recent_linear_speed_max_m_s": 0.01,
            "recent_angular_speed_max_rad_s": 0.05,
            "maximum_allowed_contact_penetration_m": -0.001,
            "meaning": "screening thresholds for this isolated audit, not safety limits",
        },
        "products": products,
        "all_products_settled_under_current_heuristic": all(
            product["settled_under_current_heuristic"] for product in products),
        "execution_ready": False,
        "conclusion": "Dynamics/contact calibration remains open; this does not validate grasp, navigation, placement or hardware.",
    }
    if output is not None:
        (output / "dynamic_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True,
                        help="new output directory; must not already exist")
    args = parser.parse_args()
    report = audit(args.scene, args.duration, args.output)
    print(json.dumps({
        "output": str(args.output.resolve()),
        "products": len(report["products"]),
        "all_products_settled_under_current_heuristic": report["all_products_settled_under_current_heuristic"],
        "robot_qpos_drift_max": report["robot_qpos_drift_max"],
        "execution_ready": report["execution_ready"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
