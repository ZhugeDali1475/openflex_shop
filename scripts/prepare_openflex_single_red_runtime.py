"""Prepare a non-running RoboOS/MuJoCo profile for the isolated red-block scene."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "local_dependencies/openflex_thesis/retail_fixture_single_red_20261113_v3"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def shelf_scene_boxes(scene_root, body_name):
    """World-aligned boxes for one MJCF body subtree, for the TC-WAM evaluator."""

    def local_offset(element):
        values = element.get("pos")
        return [float(value) for value in values.split()] if values else [0.0, 0.0, 0.0]

    def collect(body, origin, boxes):
        if body.get("quat") or body.get("euler"):
            raise ValueError("Rotated shelf geometry is not supported by the evaluator scene")
        position = [origin[axis] + value for axis, value in enumerate(local_offset(body))]
        for geometry in body.findall("geom"):
            if geometry.get("type") != "box" or geometry.get("quat") or geometry.get("euler"):
                raise ValueError("Expected world-aligned shelf boxes")
            offset = local_offset(geometry)
            boxes.append({
                "id": geometry.get("name"),
                "shape": "box",
                "dimensions": [2 * float(value) for value in geometry.get("size").split()],
                "position": [position[axis] + offset[axis] for axis in range(3)],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            })
        for child in body.findall("body"):
            collect(child, position, boxes)

    body = scene_root.find(f"worldbody/body[@name='{body_name}']")
    if body is None:
        raise ValueError(f"Scene is missing shelf body: {body_name}")
    boxes = []
    collect(body, [0.0, 0.0, 0.0], boxes)
    return boxes


def prepare_runtime_profile(source_directory, output_directory):
    source = Path(source_directory).expanduser().resolve(strict=True)
    output = Path(output_directory).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite runtime profile: {output}")
    source_scene = source / "scene.xml"
    source_manifest_path = source / "manifest.json"
    manifest = json.loads(source_manifest_path.read_text())
    root = ET.parse(source_scene).getroot()
    world = root.find("worldbody")
    if world is None:
        raise ValueError("Scene is missing worldbody")
    robot = world.find("body[@name='openflex_robot_yaw_root']")
    shelf = world.find("body[@name='shelf_group_2']")
    counter = world.find("body[@name='counter_top_left']")
    target = world.find("body[@name='thesis_object']")
    if any(item is None for item in (robot, shelf, counter, target)):
        raise ValueError("Scene does not match the reviewed single-red fixture structure")
    table_top = counter.find("geom[@name='counter_top_left_top']")
    if table_top is None:
        raise ValueError("Delivery counter has no reviewed top geometry")

    robot.set("pos", "0 0 0.17")
    robot.attrib.pop("euler", None)
    robot.attrib.pop("quat", None)
    shelf.set("name", "thesis_shelf")
    counter.set("name", "thesis_delivery_table")
    table_top.set("name", "thesis_delivery_top")
    environment_names = []
    excluded_names = {"openflex_robot_yaw_root", "floor", "thesis_object", "start_zone_top"}
    for body in world.findall("body"):
        name = body.get("name", "")
        if name and name not in excluded_names:
            environment_names.append(name)

    output.mkdir(parents=True, exist_ok=False)
    scene_output = output / "scene.xml"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(scene_output, encoding="utf-8", xml_declaration=True)

    import mujoco
    model = mujoco.MjModel.from_xml_path(str(scene_output))
    data = mujoco.MjData(model)
    start = manifest["start_pose_world"]
    for joint_name, value in (("base_x", start["x_m"]), ("base_y", start["y_m"]),
                              ("base_yaw", start["yaw_rad"])):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"Missing mobile base joint: {joint_name}")
        data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)
    root_position = data.xpos[model.body("openflex_robot_yaw_root").id].tolist()
    if any(abs(root_position[index] - value) > 1e-5
           for index, value in enumerate((start["x_m"], start["y_m"], 0.17))):
        raise ValueError(f"Start pose frame mapping failed: {root_position}")
    delivery_geom = model.geom("thesis_delivery_top")
    delivery_center = [float(value) for value in data.geom_xpos[delivery_geom.id]]
    delivery_half_size = [float(value) for value in model.geom_size[delivery_geom.id]]
    delivery_top_center = [delivery_center[0], delivery_center[1],
                           delivery_center[2] + delivery_half_size[2]]
    object_item = manifest["products"][0]
    # Keep the fixture's outward yaw so the oracle grasps from the free-space
    # side of the shelf instead of the sealed back-panel side.
    object_pose = [*object_item["position_world_m"], float(object_item.get("yaw_rad", 0.0))]
    pickup_observation = [start["x_m"], start["y_m"], start["yaw_rad"]]
    pickup_candidate = [0.20, object_pose[1], 0.0]
    delivery_candidate = [delivery_center[0] + delivery_half_size[0] + 0.50,
                          delivery_center[1], 3.141592653589793]
    table_top_z = delivery_top_center[2]
    release_center_z = table_top_z + object_item["half_size_m"][2]

    config = {
        "openflex_thesis_mujoco_bridge": {"ros__parameters": {
            "scene_model_path": str(scene_output),
            "derived_model_path": str(output / "unused_legacy_scene.xml"),
            "object_id": object_item["object_id"],
            "initial_base_pose": [start["x_m"], start["y_m"], start["yaw_rad"]],
            "object_nominal_pose": object_pose,
            "object_randomization_xy_m": [0.0, 0.0],
            "object_randomization_yaw_rad": 0.0,
            "occlusion_probability": 0.0,
            "delivery_surface_z_m": release_center_z,
            "delivery_table_body": "thesis_delivery_table",
            "delivery_surface_geom": "thesis_delivery_top",
            "environment_body_names": environment_names,
            "occluder_free_joint": "",
        }},
        "thesis_mujoco_mission_client": {"ros__parameters": {
            "task_id": "PHASE1_SINGLE_RED",
            "object_id": object_item["object_id"],
            "active_arm": 2,
            "pickup_observation_pose": pickup_observation,
            "pickup_navigation_pose": pickup_candidate,
            "delivery_pose": delivery_candidate,
            # Flat [x, y, yaw, ...] because ROS parameter files cannot nest sequences.
            # The last leg is a pure in-place rotation followed by a pure translation:
            # turning the final ~pi while strafing crosses the angle wrap, and the
            # sim_navigation_node baseline then rotates the long way round while the
            # base drifts sideways into the delivery counter foot.
            "delivery_route_waypoints": [0.20, 2.50, 0.0,
                                         -1.30, 2.80, 0.0,
                                         -1.30, 2.80, 3.141592653589793],
            "coarse_top_k": 5,
            "refined_top_k": 5,
            "max_recovery_attempts": 2,
        }},
    }
    import yaml
    config_output = output / "runtime.yaml"
    config_output.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    shelf_scene = {
        "calibrated": False,
        "frame_id": "map",
        "collision_objects": shelf_scene_boxes(root, "thesis_shelf"),
    }
    shelf_scene_output = output / "shelf_scene.yaml"
    shelf_scene_output.write_text(yaml.safe_dump(shelf_scene, allow_unicode=True, sort_keys=False))
    report = {
        "status": "prepared_offline_not_execution_authorized",
        "source_scene": str(source_scene),
        "source_sha256": sha256(source_scene),
        "runtime_scene": str(scene_output),
        "runtime_scene_sha256": sha256(scene_output),
        "runtime_config": str(config_output),
        "runtime_shelf_scene": str(shelf_scene_output),
        "runtime_shelf_scene_sha256": sha256(shelf_scene_output),
        "runtime_shelf_boxes": len(shelf_scene["collision_objects"]),
        "robot_start_world_xyz_m": root_position,
        "robot_start_yaw_rad": float(start["yaw_rad"]),
        "object_world_xyzyaw": object_pose,
        "pickup_observation_pose_xyyaw": pickup_observation,
        "pickup_navigation_candidate_xyyaw_unvalidated": pickup_candidate,
        "delivery_surface_world_center_xyz_m": delivery_top_center,
        "delivery_surface_half_size_m": delivery_half_size,
        "delivery_navigation_candidate_xyyaw_unvalidated": delivery_candidate,
        "release_object_center_z_m": release_center_z,
        "environment_roots": environment_names,
        "mujoco_compile": "passed",
        "mujoco_forward_only": True,
        "simulation_steps": 0,
        "ros_accessed": False,
        "execution_enabled": False,
        "limitations": [
            "Navigation candidates are geometric guesses, not full-robot collision-checked paths",
            "No MoveIt planning-scene synchronization or grasp reachability was tested",
            "The object identity is fixture metadata, not live visual recognition",
            "The S0 UNKNOWN motion request still blocks starting or resetting the live simulator",
        ],
    }
    (output / "profile_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()
    print(json.dumps(prepare_runtime_profile(options.source, options.output),
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
