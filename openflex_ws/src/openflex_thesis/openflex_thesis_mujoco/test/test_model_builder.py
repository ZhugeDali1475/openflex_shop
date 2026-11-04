from pathlib import Path
import xml.etree.ElementTree as ET

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
