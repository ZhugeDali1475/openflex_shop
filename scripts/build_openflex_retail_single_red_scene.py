"""Build an isolated one-red-object phase-one scene from the accepted half fixture."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "local_dependencies/openflex_thesis/retail_fixture_half_left_20261113_v5"
TARGET_SHELF = "shelf_group_2"
TARGET_FACE = "neg"
TARGET_LEVEL = 2
OBJECT_HALF_SIZE_M = (0.035, 0.03, 0.08)
SUPPORT_GAP_M = 0.003
# The oracle contract is that the object's +x axis points from the shelf out to
# free space. A "neg" face sits on the -x side of the gondola, so its outward
# axis is -x and the object must be yawed by pi.
FACE_OUTWARD_YAW_RAD = {"neg": math.pi, "pos": 0.0}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def vector(values):
    return " ".join(format(value, ".12g") for value in values)


def build_scene(source_directory, output_directory):
    source = Path(source_directory).expanduser().resolve(strict=True)
    output = Path(output_directory).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing scene directory: {output}")
    source_xml = source / "scene.xml"
    source_manifest = json.loads((source / "manifest.json").read_text())
    product_candidates = [product for product in source_manifest["products"]
                          if product["shelf_id"] == TARGET_SHELF
                          and product["face"] == TARGET_FACE
                          and product["level"] == TARGET_LEVEL]
    if not product_candidates:
        raise ValueError("No matching aisle-facing product slot exists in the selected fixture")
    reference = product_candidates[0]
    support = reference["support_bounds"]
    product_bounds = reference["world_mesh_bounds"]
    support_center_x = (support["min"][0] + support["max"][0]) / 2
    slot_center_y = (product_bounds["min"][1] + product_bounds["max"][1]) / 2
    object_center = (support_center_x, slot_center_y,
                     support["max"][2] + OBJECT_HALF_SIZE_M[2] + SUPPORT_GAP_M)
    outward_yaw = FACE_OUTWARD_YAW_RAD[TARGET_FACE]
    object_quat = (math.cos(outward_yaw / 2.0), 0.0, 0.0, math.sin(outward_yaw / 2.0))

    root = ET.parse(source_xml).getroot()
    world = root.find("worldbody")
    if world is None:
        raise ValueError("Source scene has no worldbody")
    robot_root = world.find("body[@name='openflex_robot_yaw_root']")
    if robot_root is None:
        raise ValueError("Source scene has no OpenFlex robot root")
    robot_root.set("pos", "0 0 0.17")
    robot_root.attrib.pop("euler", None)
    robot_root.attrib.pop("quat", None)
    for marker in world.findall("body[@name='start_zone_top']/geom"):
        marker.set("contype", "0")
        marker.set("conaffinity", "0")
    for element in list(world):
        if element.tag == "body" and element.get("name", "").startswith("product_"):
            world.remove(element)
    if world.find("body[@name='thesis_object']") is not None:
        raise ValueError("Source already contains thesis_object")
    object_body = ET.SubElement(world, "body", {
        "name": "thesis_object",
        "pos": vector(object_center),
        "quat": vector(object_quat),
    })
    ET.SubElement(object_body, "freejoint", {"name": "thesis_object_free"})
    ET.SubElement(object_body, "geom", {
        "name": "thesis_object_geom",
        "type": "box",
        "size": vector(OBJECT_HALF_SIZE_M),
        "rgba": "0.88 0.12 0.10 1",
        "mass": "0.22",
        "margin": "0.001",
        "condim": "3",
        "friction": "0.5 0.1 0.1",
        "contype": "1",
        "conaffinity": "1",
    })
    ET.SubElement(object_body, "site", {
        "name": "thesis_object_center",
        "size": "0.008",
        "rgba": "1 1 1 1",
    })

    output.mkdir(parents=True, exist_ok=False)
    scene_output = output / "scene.xml"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(scene_output, encoding="utf-8", xml_declaration=True)

    item = {
        "object_id": "thesis_item_01",
        "body": "thesis_object",
        "free_joint": "thesis_object_free",
        "geom": "thesis_object_geom",
        "color": "red",
        "half_size_m": list(OBJECT_HALF_SIZE_M),
        "mass_kg": 0.22,
        "shelf_id": TARGET_SHELF,
        "face": TARGET_FACE,
        "level": TARGET_LEVEL,
        "slot_id": reference["slot_id"],
        "position_world_m": list(object_center),
        "yaw_rad": outward_yaw,
        "identity_source": "known_single_object_test_fixture_not_live_model_recognition",
    }
    manifest = {
        "schema_version": 1,
        "execution_enabled": False,
        "validation_scope": "offline_single_red_fixture_compile_and_mj_forward_only",
        "source_scene": str(source_xml),
        "source_scene_sha256": sha256(source_xml),
        "scene_sha256": sha256(scene_output),
        "products": [item],
        "start_pose_world": {"x_m": 0.0, "y_m": 3.6, "yaw_rad": 4.71238898038469},
        "pickup_target_source": "preassigned_simulation_fixture_not_vision_oracle",
        "delivery_region": "sim_delivery_top_left",
        "delivery_table_geom": "counter_top_left_top",
        "simulation_steps": 0,
        "ros_accessed": False,
        "live_runtime_modified": False,
        "handoff": ["fresh visual identity and depth localization",
                    "pickup-zone and delivery-zone mapping into this scene's frames",
                    "runtime planning-scene synchronization",
                    "independent grasp attachment and placement settlement evidence"],
        "limitations": ["Static compilation is not reachability, trajectory safety or task success",
                        "The red block is a fixture object, not a recognized supermarket SKU",
                        "No ROS action, controller, MoveIt execution or MuJoCo physics stepping was run"],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")

    import mujoco
    model = mujoco.MjModel.from_xml_path(str(scene_output))
    data = mujoco.MjData(model)
    start_pose = manifest["start_pose_world"]
    for name, value in (("base_x", start_pose["x_m"]),
                        ("base_y", start_pose["y_m"]),
                        ("base_yaw", start_pose["yaw_rad"])):
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)
    target_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "thesis_object_geom")
    wheel_names = {"fl_wheel_link", "fr_wheel_link", "bl_wheel_link", "br_wheel_link"}
    contacts = []
    for contact in data.contact:
        body1 = model.body(model.geom_bodyid[contact.geom1]).name
        body2 = model.body(model.geom_bodyid[contact.geom2]).name
        names = {model.geom(contact.geom1).name, model.geom(contact.geom2).name}
        bodies = {body1, body2}
        contacts.append({"geom1": model.geom(contact.geom1).name,
                         "geom2": model.geom(contact.geom2).name,
                         "body1": body1, "body2": body2,
                         "distance_m": float(contact.dist)})
        if target_geom_id in (contact.geom1, contact.geom2) and contact.dist < -1e-6:
            raise RuntimeError(f"Red target has initial contact/penetration: {contacts[-1]}")
        if any(name.startswith("openarmx_") for name in bodies):
            raise RuntimeError(f"Robot arm starts in contact with scene: {contacts[-1]}")
        if any(name.startswith("openflex_") or name.endswith("_wheel_link") for name in bodies):
            if not any(name.endswith("_wheel_link") for name in bodies):
                raise RuntimeError(f"Robot base starts in scene contact: {contacts[-1]}")
            if any(name.endswith("_wheel_link") for name in bodies) and not any(
                    name.startswith("floor") for name in names | bodies):
                raise RuntimeError(f"Unexpected robot contact: {contacts[-1]}")

    report = {
        "scene": str(scene_output),
        "scene_sha256": sha256(scene_output),
        "compiled": True,
        "simulation_steps": 0,
        "ros_accessed": False,
        "live_runtime_modified": False,
        "product_count": 1,
        "target_position_world_m": list(data.xpos[model.body("thesis_object").id]),
        "robot_root_world_position_m": list(data.xpos[model.body("openflex_robot_yaw_root").id]),
        "robot_base_world_yaw_rad": float(start_pose["yaw_rad"]),
        "target_initial_contacts": [],
        "all_initial_contacts": contacts,
        "nbody": model.nbody,
        "ngeom": model.ngeom,
        "execution_enabled": False,
    }
    if len([name for name in (model.body(i).name for i in range(model.nbody))
            if name == "thesis_object"]) != 1:
        raise RuntimeError("Expected exactly one named red test object")
    (output / "static_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()
    print(json.dumps(build_scene(options.source, options.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
