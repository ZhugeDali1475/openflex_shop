import importlib.util
from pathlib import Path

import mujoco
import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "build_openflex_retail_single_red_scene",
    ROOT / "scripts/build_openflex_retail_single_red_scene.py",
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
SOURCE = ROOT / "local_dependencies/openflex_thesis/retail_fixture_half_left_20261113_v5"


def test_single_red_scene_has_one_visible_target_and_safe_static_contacts(tmp_path):
    output = tmp_path / "single_red"
    report = BUILDER.build_scene(SOURCE, output)
    manifest = __import__("json").loads((output / "manifest.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(output / "scene.xml"))
    data = mujoco.MjData(model)
    for name, value in (("base_x", 0.0), ("base_y", 3.6), ("base_yaw", 4.71238898038469)):
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)

    assert report["execution_enabled"] is False
    assert report["simulation_steps"] == 0
    assert report["ros_accessed"] is False
    assert manifest["products"][0]["object_id"] == "thesis_item_01"
    assert manifest["products"][0]["color"] == "red"
    assert model.nbody > 0
    assert model.geom("thesis_object_geom").rgba.tolist() == pytest.approx([0.88, 0.12, 0.10, 1.0])
    assert data.xpos[model.body("openflex_robot_yaw_root").id].tolist() == pytest.approx(
        [0.0, 3.6, 0.17], abs=1e-6
    )
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "thesis_object_geom")
    wheel_bodies = {"fl_wheel_link", "fr_wheel_link", "bl_wheel_link", "br_wheel_link"}
    for contact in data.contact:
        body_names = {
            model.body(model.geom_bodyid[contact.geom1]).name,
            model.body(model.geom_bodyid[contact.geom2]).name,
        }
        assert target_id not in {contact.geom1, contact.geom2}
        assert body_names & wheel_bodies
        assert "floor" in body_names


def test_builder_refuses_to_overwrite_scene(tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    marker = output / "keep.txt"
    marker.write_text("keep")
    with pytest.raises(FileExistsError):
        BUILDER.build_scene(SOURCE, output)
    assert marker.read_text() == "keep"
