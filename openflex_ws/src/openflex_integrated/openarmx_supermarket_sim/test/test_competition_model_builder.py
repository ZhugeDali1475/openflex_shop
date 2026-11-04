from pathlib import Path
import xml.etree.ElementTree as ET

from openarmx_supermarket_sim.competition_model_builder import build_competition_model


WORKSPACE = Path(__file__).resolve().parents[4]
ROBOT_MODEL = WORKSPACE / "src" / "openflex_mujoco" / "openflex_mujoco.xml"
SUPERMARKET_MODEL = Path(
    "/home/openflex/openflex_shop/supermarket_scene/supermarket_scene_from_db.xml"
)


def test_combined_model_contains_robot_scene_and_planar_base(tmp_path):
    output = build_competition_model(
        str(ROBOT_MODEL), str(SUPERMARKET_MODEL), str(tmp_path / "competition.xml")
    )
    root = ET.parse(output).getroot()
    worldbody = root.find("worldbody")
    robot = worldbody.find("body[@name='openflex_robot_yaw_root']")

    assert root.get("model") == "openarmx_supermarket_competition"
    assert robot.get("euler") is None
    assert [joint.get("name") for joint in robot.findall("joint")[:3]] == [
        "base_x",
        "base_y",
        "base_yaw",
    ]
    assert worldbody.find("body[@name='floor']") is not None
    assert worldbody.find("body[@name='shelf_group_1']") is not None
    assert worldbody.find(".//body[@name='product_cracker_box_uid5']") is not None
    camera = worldbody.find(".//camera[@name='competition_head_camera']")
    assert camera is not None
    assert camera.get("quat") == "0 1 0 0"


def test_combined_model_has_namespaced_assets_and_valid_files(tmp_path):
    output = build_competition_model(
        str(ROBOT_MODEL), str(SUPERMARKET_MODEL), str(tmp_path / "competition.xml")
    )
    root = ET.parse(output).getroot()
    scene_mesh = root.find("asset/mesh[@name='supermarket_mesh_cracker_box']")
    product_geom = root.find(
        ".//geom[@name='product_cracker_box_uid5_visual']"
    )

    assert scene_mesh is not None
    assert Path(scene_mesh.get("file")).is_absolute()
    assert Path(scene_mesh.get("file")).is_file()
    assert product_geom.get("mesh") == "supermarket_mesh_cracker_box"
    assert product_geom.get("material") == "supermarket_mat_cracker_box"
    assert root.find("worldbody/geom[@name='floor']") is None
    collision = root.find(".//geom[@name='product_cracker_box_uid5_collision']")
    assert collision.get("contype") == "4"
    assert collision.get("conaffinity") == "3"
