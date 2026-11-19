"""Create an offline +Y half-scene from the audited full retail fixture."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "local_dependencies/openflex_thesis/retail_fixture_full_20261112_v3"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def set_vector(element, attribute, values):
    element.set(attribute, " ".join(format(value, ".12g") for value in values))


def build_half_scene(source_directory, output_directory):
    source = Path(source_directory).expanduser().resolve(strict=True)
    output = Path(output_directory).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing scene directory: {output}")
    scene_source = source / "scene.xml"
    manifest_source = source / "manifest.json"
    root = ET.parse(scene_source).getroot()
    manifest = json.loads(manifest_source.read_text())
    world = root.find("worldbody")
    if world is None:
        raise ValueError("Scene XML has no worldbody")

    keep_exact = {
        "floor", "divider_line", "start_zone_top", "boundary_top",
        "boundary_left", "boundary_right", "wall_top", "wall_left", "wall_right",
        "shelf_group_1", "shelf_group_2", "counter_top_left", "counter_top_right",
    }
    removed = []
    for element in list(world):
        name = element.get("name", "")
        keep = element.tag == "light" or element.tag == "camera"
        keep = keep or name in keep_exact or name == "openflex_robot_yaw_root"
        keep = keep or name.startswith("divider_black_")
        keep = keep or name.startswith("product_") and any(
            item["instance_id"] == name and item["shelf_id"] in {"shelf_group_1", "shelf_group_2"}
            for item in manifest["products"]
        )
        if not keep:
            world.remove(element)
            removed.append(name or element.tag)

    floor = world.find("body[@name='floor']")
    floor_geom = floor.find("geom[@name='floor_geom']")
    set_vector(floor, "pos", (0, 2, -0.01))
    set_vector(floor_geom, "size", (3, 2, 0.01))

    for name in ("boundary_left", "boundary_right"):
        body = world.find(f"body[@name='{name}']")
        set_vector(body, "pos", (float(body.get("pos").split()[0]), 2, 0.001))
        geom = body.find("geom")
        set_vector(geom, "size", (0.01, 2, 0.001))

    for name in ("wall_left", "wall_right"):
        body = world.find(f"body[@name='{name}']")
        set_vector(body, "pos", (float(body.get("pos").split()[0]), 2, 0.125))
        geom = body.find("geom")
        set_vector(geom, "size", (0.025, 2.025, 0.125))

    robot_root = world.find("body[@name='openflex_robot_yaw_root']")
    if robot_root is None:
        raise ValueError("OpenFlex robot root body is missing")
    set_vector(robot_root, "pos", (0, 3.6, 0.17))
    set_vector(robot_root, "euler", (0, 0, 4.71238898038469))
    base_x = root.find("worldbody/body[@name='openflex_robot_yaw_root']//joint[@name='base_x']")
    base_y = root.find("worldbody/body[@name='openflex_robot_yaw_root']//joint[@name='base_y']")
    if base_x is None or base_y is None:
        raise ValueError("Expected planar robot base_x/base_y joints")
    base_x.set("ref", "0")
    base_y.set("ref", "0")
    for joint in (base_x, base_y):
        joint.set("limited", "false")
        joint.attrib.pop("range", None)

    output.mkdir(parents=True, exist_ok=False)
    scene_output = output / "scene.xml"
    ET.indent(root, space="  ")
    root_tree = ET.ElementTree(root)
    root_tree.write(scene_output, encoding="utf-8", xml_declaration=True)

    kept_products = [item for item in manifest["products"]
                     if item["shelf_id"] in {"shelf_group_1", "shelf_group_2"}]
    half_manifest = dict(manifest)
    half_manifest.update({
        "scene_sha256": sha256(scene_output),
        "parent_scene": str(scene_source),
        "parent_scene_sha256": sha256(scene_source),
        "fixture_region": "+Y half of warning divider; world Y from 0 to 4 m",
        "products": kept_products,
        "delivery_regions": [item for item in manifest["delivery_regions"]
                             if item["region_id"] in {"sim_delivery_top_left", "sim_delivery_top_right"}],
        "initial_robot_state": "base_centered_on_start_zone_top_static_initial_configuration_not_motion_or_safety_validation",
        "initial_robot_pose_world": {"x_m": 0.0, "y_m": 3.6, "yaw_rad": 4.71238898038469},
        "half_scene_filter": {"kept_shelf_groups": ["shelf_group_1", "shelf_group_2"],
                              "removed_top_level_elements": removed},
        "limitations": list(manifest["limitations"]) + [
            "Robot base is initialized at the ground start marker; this is a static qpos configuration, not a navigation result",
            "Half-scene extraction does not establish continuous-path safety or manipulation reachability",
        ],
    })
    (output / "manifest.json").write_text(json.dumps(half_manifest, ensure_ascii=False, indent=2) + "\n")

    import mujoco
    model = mujoco.MjModel.from_xml_path(str(scene_output))
    data = mujoco.MjData(model)
    data.qpos[model.joint("base_x").qposadr[0]] = 0.0
    data.qpos[model.joint("base_y").qposadr[0]] = 0.0
    mujoco.mj_forward(model, data)
    robot_world_position = data.xpos[model.body("openflex_robot_yaw_root").id].tolist()
    report = {
        "scene": str(scene_output),
        "scene_sha256": sha256(scene_output),
        "products": len(kept_products),
        "categories": len({item["simulation_sku"] for item in kept_products}),
        "shelf_groups": ["shelf_group_1", "shelf_group_2"],
        "delivery_tables": ["counter_top_left", "counter_top_right"],
        "robot_base_xy_m": [float(data.qpos[model.joint("base_x").qposadr[0]]),
                             float(data.qpos[model.joint("base_y").qposadr[0]])],
        "robot_root_world_position_m": robot_world_position,
        "robot_root_yaw_rad": 4.71238898038469,
        "simulation_steps": 0,
        "ros_accessed": False,
        "live_runtime_modified": False,
        "compiled": True,
        "nbody": model.nbody,
        "ngeom": model.ngeom,
    }
    (output / "static_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    if len(kept_products) != 52 or len(report["delivery_tables"]) != 2:
        raise RuntimeError(f"Unexpected half-scene contents: {report}")
    if abs(robot_world_position[0]) > 1e-6 or abs(robot_world_position[1] - 3.6) > 1e-6:
        raise RuntimeError(f"Robot is not centered on the start marker: {robot_world_position}")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build_half_scene(args.source, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
