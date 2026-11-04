"""Build a combined OpenFlex robot and supermarket MuJoCo model."""

import argparse
import copy
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET


ROBOT_MODEL_ENV = "OPENFLEX_MUJOCO_MODEL"
SUPERMARKET_MODEL_ENV = "OPENFLEX_SUPERMARKET_MODEL"
DEFAULT_OUTPUT = "/tmp/openarmx_supermarket_competition.xml"
ASSET_TAGS = {"hfield", "material", "mesh", "skin", "texture"}
ASSET_REFERENCE_ATTRIBUTES = {"hfield", "material", "mesh", "texture"}


def _workspace_candidates(relative_path: Path) -> list:
    here = Path(__file__).resolve()
    candidates = []
    for parent in (Path.cwd(), *here.parents):
        candidates.append(parent / relative_path)
    return candidates


def _resolve_model(explicit_path: str, env_name: str, candidates: list, label: str) -> Path:
    paths = []
    if explicit_path:
        paths.append(Path(explicit_path))
    configured = os.environ.get(env_name, "")
    if configured:
        paths.append(Path(configured))
    paths.extend(candidates)
    for path in paths:
        if path.is_file():
            return path.expanduser().resolve()
    checked = "\n  ".join(str(path) for path in paths[:16])
    raise FileNotFoundError(f"cannot find {label}; checked:\n  {checked}")


def resolve_robot_model(explicit_path: str = "") -> Path:
    return _resolve_model(
        explicit_path,
        ROBOT_MODEL_ENV,
        _workspace_candidates(Path("src/openflex_mujoco/openflex_mujoco.xml")),
        "OpenFlex robot MJCF",
    )


def resolve_supermarket_model(explicit_path: str = "") -> Path:
    candidates = [
        Path("/home/openflex/openflex_shop/supermarket_scene/supermarket_scene_from_db.xml"),
        *_workspace_candidates(Path("supermarket_scene/supermarket_scene_from_db.xml")),
    ]
    return _resolve_model(
        explicit_path,
        SUPERMARKET_MODEL_ENV,
        candidates,
        "supermarket MJCF",
    )


def _absolutize_files(root: ET.Element, source: Path) -> None:
    for element in root.iter():
        file_name = element.get("file")
        if file_name and not os.path.isabs(file_name):
            element.set("file", str((source.parent / file_name).resolve()))


def _namespace_assets(root: ET.Element, prefix: str) -> None:
    names = {}
    asset = root.find("asset")
    if asset is None:
        return
    for element in asset:
        if element.tag in ASSET_TAGS and element.get("name"):
            old_name = element.get("name")
            new_name = prefix + old_name
            names[old_name] = new_name
            element.set("name", new_name)
    for element in root.iter():
        for attribute in ASSET_REFERENCE_ATTRIBUTES:
            value = element.get(attribute)
            if value in names:
                element.set(attribute, names[value])


def _convert_angles_to_radians(root: ET.Element) -> None:
    compiler = root.find("compiler")
    if compiler is None or compiler.get("angle", "degree") != "degree":
        return
    for element in root.iter():
        for attribute in ("euler", "axisangle"):
            value = element.get(attribute)
            if not value:
                continue
            parts = value.split()
            angle_indexes = range(len(parts)) if attribute == "euler" else (3,)
            for index in angle_indexes:
                parts[index] = str(math.radians(float(parts[index])))
            element.set(attribute, " ".join(parts))
    compiler.set("angle", "radian")


def _add_planar_base(robot_root: ET.Element) -> None:
    existing = {joint.get("name") for joint in robot_root.findall("joint")}
    definitions = (
        ("base_x", "slide", "1 0 0"),
        ("base_y", "slide", "0 1 0"),
        ("base_yaw", "hinge", "0 0 1"),
    )
    insertion_index = 0
    for name, joint_type, axis in definitions:
        if name in existing:
            continue
        robot_root.insert(
            insertion_index,
            ET.Element(
                "joint",
                {
                    "name": name,
                    "type": joint_type,
                    "axis": axis,
                    "limited": "false",
                    "damping": "2.0",
                    "armature": "0.01",
                },
            ),
        )
        insertion_index += 1


def _add_robot_cameras(worldbody: ET.Element) -> None:
    cameras = (
        ("d435_color_optical_frame", "competition_head_camera"),
        ("openarmx_left_link7_pico", "competition_left_wrist_camera"),
        ("openarmx_right_link7_pico", "competition_right_wrist_camera"),
    )
    existing = {camera.get("name") for camera in worldbody.iter("camera")}
    for body_name, camera_name in cameras:
        if camera_name in existing:
            continue
        body = worldbody.find(f".//body[@name='{body_name}']")
        if body is not None:
            ET.SubElement(
                body,
                "camera",
                {
                    "name": camera_name,
                    "mode": "fixed",
                    "fovy": "58",
                    # MuJoCo cameras look along local -Z with +Y up, while a
                    # ROS optical frame looks along +Z with +Y down.  The
                    # half-turn keeps rendered RGB-D rays in the named ROS
                    # optical/body frame instead of silently mirroring them.
                    "quat": "0 1 0 0",
                },
            )


def _remove_robot_viewer_scene(worldbody: ET.Element) -> None:
    for child in list(worldbody):
        if child.tag == "light" or (child.tag == "geom" and child.get("name") == "floor"):
            worldbody.remove(child)


def _apply_supermarket_geom_defaults(scene_root: ET.Element) -> None:
    default_geom = scene_root.find("default/geom")
    if default_geom is None:
        return
    inherited = dict(default_geom.attrib)
    worldbody = scene_root.find("worldbody")
    if worldbody is None:
        return
    for geom in worldbody.iter("geom"):
        for name, value in inherited.items():
            if geom.get(name) is None:
                geom.set(name, value)


def _configure_product_collisions(scene_root: ET.Element) -> None:
    """Keep product/robot/static contact while disabling product/product contact."""
    worldbody = scene_root.find("worldbody")
    if worldbody is None:
        return
    for body in worldbody.findall("body"):
        if not body.get("name", "").startswith("product_"):
            continue
        for geom in body.iter("geom"):
            if geom.get("contype", "1") == "0":
                continue
            geom.set("contype", "4")
            geom.set("conaffinity", "3")


def _merge_visual(robot_root: ET.Element, scene_root: ET.Element) -> None:
    scene_visual = scene_root.find("visual")
    if scene_visual is None:
        return
    robot_visual = robot_root.find("visual")
    if robot_visual is None:
        robot_visual = ET.SubElement(robot_root, "visual")
    existing_tags = {child.tag for child in robot_visual}
    for child in scene_visual:
        if child.tag not in existing_tags:
            robot_visual.append(copy.deepcopy(child))


def _missing_files(root: ET.Element) -> list:
    return sorted(
        {
            element.get("file")
            for element in root.iter()
            if element.get("file") and not Path(element.get("file")).is_file()
        }
    )


def build_competition_model(
    robot_model_path: str = "",
    supermarket_model_path: str = "",
    output_path: str = DEFAULT_OUTPUT,
    validate_assets: bool = True,
) -> Path:
    robot_source = resolve_robot_model(robot_model_path)
    scene_source = resolve_supermarket_model(supermarket_model_path)
    robot_tree = ET.parse(robot_source)
    robot_root = robot_tree.getroot()
    scene_root = ET.parse(scene_source).getroot()

    _absolutize_files(robot_root, robot_source)
    _absolutize_files(scene_root, scene_source)
    _convert_angles_to_radians(scene_root)
    _namespace_assets(scene_root, "supermarket_")
    _apply_supermarket_geom_defaults(scene_root)
    _configure_product_collisions(scene_root)

    robot_root.set("model", "openarmx_supermarket_competition")
    compiler = robot_root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        robot_root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")

    robot_world = robot_root.find("worldbody")
    scene_world = scene_root.find("worldbody")
    robot_asset = robot_root.find("asset")
    scene_asset = scene_root.find("asset")
    if robot_world is None or scene_world is None:
        raise ValueError("both MJCF models must contain a worldbody")
    if robot_asset is None or scene_asset is None:
        raise ValueError("both MJCF models must contain an asset section")

    robot_body = robot_world.find("body[@name='openflex_robot_yaw_root']")
    if robot_body is None:
        raise ValueError("robot MJCF has no openflex_robot_yaw_root body")
    robot_body.attrib.pop("euler", None)
    robot_body.attrib.pop("quat", None)
    _add_planar_base(robot_body)
    _add_robot_cameras(robot_world)
    _remove_robot_viewer_scene(robot_world)

    for asset in scene_asset:
        robot_asset.append(copy.deepcopy(asset))
    for child in scene_world:
        robot_world.append(copy.deepcopy(child))

    scene_option = scene_root.find("option")
    robot_option = robot_root.find("option")
    if scene_option is not None and robot_option is None:
        compiler_index = list(robot_root).index(compiler)
        robot_root.insert(compiler_index + 1, copy.deepcopy(scene_option))
    _merge_visual(robot_root, scene_root)

    if validate_assets:
        missing = _missing_files(robot_root)
        if missing:
            sample = "\n  ".join(missing[:10])
            raise FileNotFoundError(f"combined MJCF references missing assets:\n  {sample}")

    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot_tree, space="  ")
    robot_tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-model", default="")
    parser.add_argument("--supermarket-model", default="")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--skip-asset-validation", action="store_true")
    options = parser.parse_args(args)
    output = build_competition_model(
        options.robot_model,
        options.supermarket_model,
        options.output,
        not options.skip_asset_validation,
    )
    print(output)


if __name__ == "__main__":
    main()
