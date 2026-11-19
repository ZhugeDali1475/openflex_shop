"""Build a thesis scene by extending the existing converted OpenFleX MJCF."""

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from .camera_spec import HEAD_CAMERA


@dataclass(frozen=True)
class SceneSpec:
    object_position: tuple = (1.12, 0.0, 0.84)
    object_yaw_rad: float = 3.141592653589793
    object_half_size: tuple = (0.035, 0.030, 0.080)
    object_contact_margin_m: float = 0.001
    shelf_center_x: float = 1.24
    shelf_half_width_y: float = 0.65
    shelf_surface_z: float = 0.74
    delivery_position: tuple = (-0.85, -0.80, 0.48)


def _candidate_model_paths() -> list:
    candidates = []
    configured = os.environ.get("OPENFLEX_MUJOCO_MODEL", "")
    if configured:
        candidates.append(Path(configured))
    here = Path(__file__).resolve()
    for parent in [Path.cwd(), *here.parents]:
        candidates.extend(
            [
                parent / "src" / "openflex_mujoco" / "openflex_mujoco.xml",
                parent / "openflex_mujoco" / "openflex_mujoco.xml",
            ]
        )
    return candidates


def resolve_upstream_model(explicit_path: str = "") -> Path:
    candidates = [Path(explicit_path)] if explicit_path else _candidate_model_paths()
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    rendered = "\n  ".join(str(item) for item in candidates[:12])
    raise FileNotFoundError(
        "cannot find openflex_mujoco.xml; set upstream_model_path or "
        f"OPENFLEX_MUJOCO_MODEL. Checked:\n  {rendered}"
    )


def _add_geom(parent, name, geom_type, pos, size, rgba, **attributes):
    values = {
        "name": name,
        "type": geom_type,
        "pos": " ".join(str(value) for value in pos),
        "size": " ".join(str(value) for value in size),
        "rgba": rgba,
        "contype": "1",
        "conaffinity": "3",
        "friction": "0.9 0.01 0.001",
    }
    values.update({key: str(value) for key, value in attributes.items()})
    return ET.SubElement(parent, "geom", values)


def _add_scene(worldbody, spec: SceneSpec) -> None:
    if not math.isfinite(spec.object_contact_margin_m) or not 0.0 <= spec.object_contact_margin_m <= 0.002:
        raise ValueError('Object contact margin must be a finite conservative 0-2 mm envelope')
    shelf = ET.SubElement(worldbody, "body", {"name": "thesis_shelf"})
    _add_geom(
        shelf,
        "thesis_shelf_back",
        "box",
        (spec.shelf_center_x + 0.22, 0.0, 1.05),
        (0.025, spec.shelf_half_width_y, 1.05),
        "0.30 0.34 0.38 1",
    )
    for index, z in enumerate((spec.shelf_surface_z, 1.18, 1.62)):
        _add_geom(
            shelf,
            f"thesis_shelf_board_{index}",
            "box",
            (spec.shelf_center_x, 0.0, z),
            (0.25, spec.shelf_half_width_y, 0.018),
            "0.55 0.58 0.60 1",
        )
    for side, y in (("left", spec.shelf_half_width_y), ("right", -spec.shelf_half_width_y)):
        _add_geom(
            shelf,
            f"thesis_shelf_side_{side}",
            "box",
            (spec.shelf_center_x, y, 1.05),
            (0.25, 0.018, 1.05),
            "0.40 0.44 0.47 1",
        )

    delivery = ET.SubElement(worldbody, "body", {"name": "thesis_delivery_table"})
    dx, dy, dz = spec.delivery_position
    _add_geom(
        delivery,
        "thesis_delivery_top",
        "box",
        (dx, dy, dz - 0.025),
        (0.35, 0.35, 0.025),
        "0.18 0.48 0.38 1",
    )
    _add_geom(
        delivery,
        "thesis_delivery_leg",
        "box",
        (dx, dy, 0.5 * (dz - 0.05)),
        (0.08, 0.08, 0.5 * (dz - 0.05)),
        "0.22 0.26 0.28 1",
    )

    ox, oy, oz = spec.object_position
    object_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "thesis_object",
            "pos": f"{ox} {oy} {oz}",
            "euler": f"0 0 {spec.object_yaw_rad}",
        },
    )
    ET.SubElement(object_body, "freejoint", {"name": "thesis_object_free"})
    _add_geom(
        object_body,
        "thesis_object_geom",
        "box",
        (0, 0, 0),
        spec.object_half_size,
        "0.88 0.18 0.12 1",
        mass="0.22",
        margin=spec.object_contact_margin_m,
    )
    ET.SubElement(
        object_body,
        "site",
        {"name": "thesis_object_center", "size": "0.008", "rgba": "1 1 1 1"},
    )

    # 2026-11-09: Moved occluder far from work area to avoid interference
    # Original position: (1.02, 0.32, 0.86) - blocked grasp candidates
    # New position: (-5.0, -5.0, 0.5) - safely out of robot workspace
    occluder = ET.SubElement(
        worldbody,
        "body",
        {"name": "thesis_occluder", "pos": "-5.0 -5.0 0.5"},
    )
    ET.SubElement(occluder, "freejoint", {"name": "thesis_occluder_free"})
    _add_geom(
        occluder,
        "thesis_occluder_geom",
        "box",
        (0, 0, 0),
        (0.025, 0.10, 0.12),
        "0.12 0.34 0.72 0.85",
        mass="0.08",
    )


def prepare_robot_tree(upstream_model_path: str = ""):
    """Prepare an independent robot XML tree without adding a task scene."""
    source = resolve_upstream_model(upstream_model_path)
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "openflex_thesis_mujoco")
    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("autolimits", "true")
    for element in root.iter():
        file_name = element.get("file")
        if file_name and not os.path.isabs(file_name):
            element.set("file", str((source.parent / file_name).resolve()))

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("upstream MJCF has no worldbody")
    robot_root = worldbody.find("body[@name='openflex_robot_yaw_root']")
    if robot_root is None:
        raise ValueError("upstream MJCF has no openflex_robot_yaw_root")
    # The upstream yaw is a viewer presentation offset, not part of the ROS URDF.
    robot_root.attrib.pop("euler", None)
    robot_root.attrib.pop("quat", None)
    for index, attributes in enumerate(
        (
            {"name": "base_x", "type": "slide", "axis": "1 0 0"},
            {"name": "base_y", "type": "slide", "axis": "0 1 0"},
            {"name": "base_yaw", "type": "hinge", "axis": "0 0 1"},
        )
    ):
        attributes.update({"limited": "false", "damping": "2.0", "armature": "0.01"})
        robot_root.insert(index, ET.Element("joint", attributes))

    head = worldbody.find(f".//body[@name='{HEAD_CAMERA.parent_body}']")
    if head is None:
        raise ValueError(f"upstream MJCF is missing {HEAD_CAMERA.parent_body}")
    optical = ET.SubElement(head, "body", {
        "name": HEAD_CAMERA.optical_frame,
        "pos": " ".join(map(str, HEAD_CAMERA.position)),
        "quat": " ".join(map(str, HEAD_CAMERA.optical_quaternion_wxyz)),
    })
    ET.SubElement(optical, "camera", {
        "name": "thesis_head_camera", "mode": "fixed",
        "fovy": str(HEAD_CAMERA.fovy_degrees), "quat": "0 1 0 0",
    })

    for body_name, camera_name in (
        ("openarmx_left_link7_pico", "thesis_left_wrist_camera"),
        ("openarmx_right_link7_pico", "thesis_right_wrist_camera"),
    ):
        body = worldbody.find(f".//body[@name='{body_name}']")
        if body is not None:
            ET.SubElement(
                body,
                "camera",
                {
                    "name": camera_name,
                    "mode": "fixed",
                    "fovy": "58",
                    # MuJoCo cameras look along local -Z with +Y up. ROS optical
                    # frames look along +Z with +Y down.
                    "quat": "0 1 0 0",
                },
            )

    return tree, worldbody


def build_thesis_model(
    upstream_model_path: str = "",
    output_path: str = "/tmp/openflex_thesis_mujoco_scene.xml",
    scene: SceneSpec = SceneSpec(),
) -> Path:
    tree, worldbody = prepare_robot_tree(upstream_model_path)
    root = tree.getroot()
    _add_scene(worldbody, scene)
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    for arm in ("left", "right"):
        ET.SubElement(
            equality,
            "weld",
            {
                "name": f"thesis_object_{arm}_weld",
                "body1": "thesis_object",
                "body2": f"openarmx_{arm}_hand_tcp",
                "active": "false",
                "solref": "0.01 1",
                "solimp": "0.95 0.99 0.001",
            },
        )

    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream-model", default="")
    parser.add_argument("--output", default="/tmp/openflex_thesis_mujoco_scene.xml")
    options = parser.parse_args(args)
    print(build_thesis_model(options.upstream_model, options.output))


if __name__ == "__main__":
    main()
