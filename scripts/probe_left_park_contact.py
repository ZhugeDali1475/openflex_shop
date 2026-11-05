#!/usr/bin/env python3
"""Reproduce the G5 left-park / floor-contact defect without ROS.

The MuJoCo bridge holds the inactive left arm by writing the park joint angles
into `qpos` after every `mj_step` (`hold_left_arm_park`). When the lift is low
enough, the hanging left gripper is below the floor plane, so that kinematic
hold presses the fingers into the floor. The reaction travels through the
carriage into `lift_joint`, which is *not* pinned, and drags the lift tens of
millimetres off its commanded height. The right TCP then arrives above the
object and the MuJoCo jaw gate rejects the grasp.

This script settles the same model with the same loop and reports:

  * the settled `lift_joint` error for the current hold, and
  * the same error with the left finger collision geoms disabled (control), and
  * the deepest left-arm mesh vertex as a function of the lift command.

Usage:
    OPENFLEX_MUJOCO_MODEL=/path/to/openflex_mujoco.xml \
        python3 scripts/probe_left_park_contact.py
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws" / "src" / "openflex_thesis" / "openflex_thesis_mujoco"))

from openflex_thesis_mujoco.model_builder import build_thesis_model  # noqa: E402

LEFT_ARM_JOINTS = [f"openarmx_left_joint{index}" for index in range(1, 8)]
# The G5 candidate the mission actually executed (moveit_grasp_executor log for
# the run that produced `MUJOCO_MISSION_FAILED code=53`).
RIGHT_TARGET = {
    "lift_joint": -0.444954,
    "openarmx_right_joint1": 0.020869,
    "openarmx_right_joint2": 3.064634,
    "openarmx_right_joint3": 0.343968,
    "openarmx_right_joint4": 1.792248,
    "openarmx_right_joint5": -0.11444,
    "openarmx_right_joint6": -0.059744,
    "openarmx_right_joint7": -0.174868,
}
# mujoco.yaml: inactive_arm_park_positions / lift_joint range.
DEFAULT_PARK = [-0.7, -1.2, 0.4, 1.2, 0.0, 0.0, 0.0]
LIFT_RANGE = (-0.65, 0.30)


def _id_map(model, obj_type):
    return {
        mujoco.mj_id2name(model, obj_type, index): index
        for index in range(
            model.njnt if obj_type == mujoco.mjtObj.mjOBJ_JOINT else model.nu
        )
    }


def _initial_state(model, joints, actuators, park):
    data = mujoco.MjData(model)
    for name, joint_id in joints.items():
        if model.jnt_type[joint_id] in (2, 3):
            data.qpos[int(model.jnt_qposadr[joint_id])] = 0.0
    for name, value in zip(LEFT_ARM_JOINTS, park):
        data.qpos[int(model.jnt_qposadr[joints[name]])] = value
    for actuator_name, actuator_id in actuators.items():
        joint_name = actuator_name[: -len("_pos")]
        data.ctrl[actuator_id] = (
            data.qpos[int(model.jnt_qposadr[joints[joint_name]])]
            if joint_name in joints
            else 0.0
        )
    mujoco.mj_forward(model, data)
    return data


def _deepest_left_point(model, data) -> float:
    lowest = float("inf")
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        mesh_id = int(model.geom_dataid[geom_id])
        if not name.startswith("openarmx_left") or model.geom_type[geom_id] != 7 or mesh_id < 0:
            continue
        start = model.mesh_vertadr[mesh_id]
        stop = start + model.mesh_vertnum[mesh_id]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        world = model.mesh_vert[start:stop] @ rotation.T + data.geom_xpos[geom_id]
        lowest = min(lowest, float(world[:, 2].min()))
    return lowest


def settle(model, park, hold_park, disable_left_finger_collision=False):
    joints = _id_map(model, mujoco.mjtObj.mjOBJ_JOINT)
    actuators = _id_map(model, mujoco.mjtObj.mjOBJ_ACTUATOR)
    actuated_dofs = [
        int(model.jnt_dofadr[joints[name[: -len("_pos")]]]) for name in actuators
    ]
    data = _initial_state(model, joints, actuators, park)
    for name, value in RIGHT_TARGET.items():
        data.ctrl[actuators[name + "_pos"]] = value
    if disable_left_finger_collision:
        for geom_id in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            if name.startswith("openarmx_left") and "finger" in name:
                model.geom_contype[geom_id] = 0
                model.geom_conaffinity[geom_id] = 0
    for _ in range(6000):
        data.qfrc_applied[:] = 0.0
        for dof in actuated_dofs:
            data.qfrc_applied[dof] = data.qfrc_bias[dof]
        mujoco.mj_step(model, data)
        if hold_park:
            for name, value in zip(LEFT_ARM_JOINTS, park):
                data.qpos[int(model.jnt_qposadr[joints[name]])] = value
                data.qvel[int(model.jnt_dofadr[joints[name]])] = 0.0
    return data, joints, actuators


def _floor_contacts(model, data):
    out = []
    force = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        if 0 not in (int(contact.geom1), int(contact.geom2)):
            continue
        mujoco.mj_contactForce(model, data, index, force)
        for geom_id in (int(contact.geom1), int(contact.geom2)):
            if geom_id == 0:
                continue
            body = int(model.geom_bodyid[geom_id])
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
            out.append((body_name, float(contact.dist), float(np.linalg.norm(force[:3]))))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", default="", help="path to openflex_mujoco.xml")
    parser.add_argument(
        "--derived",
        default="/tmp/openflex_probe_left_park_scene.xml",
        help="where to write the derived MJCF",
    )
    parser.add_argument("--park", type=float, nargs=7, default=DEFAULT_PARK)
    args = parser.parse_args()

    model_path = build_thesis_model(args.upstream, args.derived)
    print(f"derived model: {model_path}")
    print(f"park pose    : {args.park}")
    print(f"lift command : {RIGHT_TARGET['lift_joint']:.6f}")
    print()

    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = 0.002
    data, joints, _ = settle(model, args.park, hold_park=True)
    lift = float(data.qpos[int(model.jnt_qposadr[joints["lift_joint"]])])
    print(
        "[A] hold_left_arm_park = true (what the bridge does)\n"
        f"    settled lift = {lift:+.6f}  error = {(lift - RIGHT_TARGET['lift_joint']) * 1000:+.2f} mm"
    )
    for name, dist, magnitude in _floor_contacts(model, data):
        print(f"    floor contact {name:32s} dist={dist:+.5f} m  |F|={magnitude:7.2f} N")
    print()

    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = 0.002
    data, joints, _ = settle(model, args.park, hold_park=True, disable_left_finger_collision=True)
    lift = float(data.qpos[int(model.jnt_qposadr[joints["lift_joint"]])])
    print(
        "[B] control: left finger collision geoms disabled\n"
        f"    settled lift = {lift:+.6f}  error = {(lift - RIGHT_TARGET['lift_joint']) * 1000:+.2f} mm"
    )
    print()

    print("[C] deepest left-arm mesh vertex vs lift command (floor plane is z = 0)")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    print(f"    current park {args.park}")
    for lift_command in (-0.65, -0.445, -0.222, 0.0, 0.30):
        data = _initial_state(model, _id_map(model, mujoco.mjtObj.mjOBJ_JOINT),
                              _id_map(model, mujoco.mjtObj.mjOBJ_ACTUATOR), args.park)
        joints = _id_map(model, mujoco.mjtObj.mjOBJ_JOINT)
        data.qpos[int(model.jnt_qposadr[joints["lift_joint"]])] = lift_command
        data.qpos[int(model.jnt_qposadr[joints["openarmx_left_finger_joint1"]])] = 0.04
        mujoco.mj_forward(model, data)
        print(f"      lift={lift_command:+.3f}  deepest z={_deepest_left_point(model, data):+.5f} m")

    print()
    print("[D] candidate tucked park poses (same measurement)")
    for park in (
        [0.0, -0.9, 0.6, -1.2, 0.0, 0.4, 0.0],
        [0.2, -1.2, 0.9, -1.4, 0.0, 0.5, 0.0],
    ):
        data = _initial_state(model, _id_map(model, mujoco.mjtObj.mjOBJ_JOINT),
                              _id_map(model, mujoco.mjtObj.mjOBJ_ACTUATOR), park)
        joints = _id_map(model, mujoco.mjtObj.mjOBJ_JOINT)
        data.qpos[int(model.jnt_qposadr[joints["lift_joint"]])] = LIFT_RANGE[0]
        data.qpos[int(model.jnt_qposadr[joints["openarmx_left_finger_joint1"]])] = 0.04
        mujoco.mj_forward(model, data)
        print(
            f"      park={park}  deepest z at lift={LIFT_RANGE[0]:+.2f} = "
            f"{_deepest_left_point(model, data):+.5f} m"
        )


if __name__ == "__main__":
    main()
