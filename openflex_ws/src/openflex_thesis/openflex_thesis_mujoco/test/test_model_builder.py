from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from openflex_thesis_mujoco.model_builder import build_thesis_model


def upstream_model() -> Path:
    return Path(__file__).resolve().parents[3] / "openflex_mujoco" / "openflex_mujoco.xml"


def test_derived_scene_contains_thesis_contract(tmp_path):
    output = build_thesis_model(str(upstream_model()), str(tmp_path / "scene.xml"))
    root = ET.parse(output).getroot()
    world = root.find("worldbody")
    assert world is not None
    robot = world.find("body[@name='openflex_robot_yaw_root']")
    assert robot is not None
    assert "euler" not in robot.attrib
    assert [robot[index].get("name") for index in range(3)] == [
        "base_x",
        "base_y",
        "base_yaw",
    ]
    assert world.find("body[@name='thesis_shelf']") is not None
    assert world.find("body[@name='thesis_object']/freejoint[@name='thesis_object_free']") is not None
    assert world.find(".//camera[@name='thesis_head_camera']") is not None
    weld = root.find("equality/weld[@name='thesis_object_right_weld']")
    assert weld is not None and weld.get("active") == "false"
    mesh_files = [Path(mesh.get("file")) for mesh in root.findall("asset/mesh")]
    assert mesh_files and all(path.is_absolute() and path.is_file() for path in mesh_files)


def test_moveit_scene_covers_every_generated_shelf_box(tmp_path):
    scene = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config" / "sim_shelf_scene.yaml").read_text()
    )
    configured = {
        (tuple(item["position"]), tuple(item["dimensions"]))
        for item in scene["collision_objects"]
    }
    output = build_thesis_model(str(upstream_model()), str(tmp_path / "scene.xml"))
    world = ET.parse(output).getroot().find("worldbody")
    generated = []
    for geom in world.findall("body[@name='thesis_shelf']/geom"):
        generated.append(
            (
                tuple(float(value) for value in geom.get("pos").split()),
                tuple(2.0 * float(value) for value in geom.get("size").split()),
            )
        )
    assert generated
    assert set(generated) <= configured


def test_object_contact_envelope_is_conservative_without_shrinking_geometry(tmp_path):
    output = build_thesis_model(str(upstream_model()), str(tmp_path / 'scene.xml'))
    geometry = ET.parse(output).find("worldbody/body[@name='thesis_object']/geom")
    assert [float(value) for value in geometry.get('size').split()] == [0.035, 0.03, 0.08]
    assert float(geometry.get('margin')) == 0.001
    assert geometry.get('contype') == '1' and geometry.get('conaffinity') == '3'
