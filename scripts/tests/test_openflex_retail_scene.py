import importlib.util
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"))
from openflex_thesis_mujoco import retail_scene_builder as retail


ROBOT = ROOT / "openflex_mujoco-1.0_basic/openflex_mujoco.xml"
SUPERMARKET = ROOT / "supermarket_scene/mujoco/supermarket_scene_6x8_competition.xml"


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    return retail.build_retail_scene(ROBOT, SUPERMARKET, tmp_path_factory.mktemp("retail") / "scene")


def test_sources_unchanged_and_inventory_untouched(tmp_path):
    database = ROOT / "supermarket_scene/shelf_inventory.db"
    tracked = [ROBOT, SUPERMARKET, database]
    before = [retail.sha256(path) for path in tracked]
    _, manifest = retail.build_retail_scene(ROBOT, SUPERMARKET, tmp_path / "scene")
    assert before == [retail.sha256(path) for path in tracked]
    assert manifest["execution_enabled"] is False
    assert manifest["database_accessed"] is False
    assert manifest["inventory_mapping_complete"] is False


def test_explicit_identity_and_support_geometry(built):
    scene, manifest = built
    products = manifest["products"]
    assert len(products) == 6
    assert len({product["instance_id"] for product in products}) == 6
    assert len({product["slot_id"] for product in products}) == 6
    assert {product["simulation_sku"] for product in products} == {row[0] for row in retail.PRODUCTS}
    root = ET.parse(scene).getroot()
    assert root.find("worldbody/body[@name='thesis_object']") is None
    assert root.find("equality/weld") is None
    for product in products:
        assert product["inventory_sku"] is None
        assert product["world_mesh_bounds"]["min"][2] == pytest.approx(
            product["support_bounds"]["max"][2] + 0.001)
        for axis in (0, 1):
            assert product["world_mesh_bounds"]["min"][axis] > product["support_bounds"]["min"][axis]
            assert product["world_mesh_bounds"]["max"][axis] < product["support_bounds"]["max"][axis]
        visual = root.find(f".//geom[@name='{product['visual_geom']}']")
        collision = root.find(f".//geom[@name='{product['collision_geom']}']")
        assert visual.get("density") == "0"
        assert visual.get("contype") == visual.get("conaffinity") == "0"
        assert collision.get("contype") == "1"
        assert collision.get("conaffinity") == "3"
        assert collision.get("type") == "box"
        assert product["collision_representation"].startswith("conservative_axis_aligned_box")


def test_static_geometry_and_assets_preserved(built):
    scene, manifest = built
    root = ET.parse(scene).getroot()
    source = ET.parse(SUPERMARKET).getroot()
    static = retail._static_elements(source.find("worldbody"))
    for element in static:
        for geom in element.iter("geom"):
            imported = root.find(f".//geom[@name='{geom.get('name')}']")
            assert imported is not None
            for attribute in ("pos", "size", "type"):
                assert imported.get(attribute) == geom.get(attribute)
            if geom.get("material"):
                assert imported.get("material") == "retail_asset_" + geom.get("material")
            assert imported.get("contype") == geom.get("contype", "1")
    assert {mesh.get("name") for mesh in root.findall("asset/mesh") if
            mesh.get("name").startswith("retail_asset_mesh_")} == {
                f"retail_asset_mesh_{row[0]}" for row in retail.PRODUCTS}
    for asset in manifest["assets"]:
        assert Path(asset["path"]).is_absolute()
        assert retail.sha256(asset["path"]) == asset["sha256"]
    assert len(manifest["delivery_regions"]) == 4
    assert all(region["surface_z"] == pytest.approx(0.765) for region in manifest["delivery_regions"])


def test_orientation_conversion_keeps_fovy_degrees():
    element = ET.fromstring('<body euler="0 90 180"><geom axisangle="1 0 0 90"/>'
                            '<camera fovy="60"/></body>')
    retail.convert_angles(element, "degree")
    assert retail._numbers(element.get("euler")) == pytest.approx([0, math.pi / 2, math.pi])
    assert retail._numbers(element.find("geom").get("axisangle")) == pytest.approx([1, 0, 0, math.pi / 2])
    assert element.find("camera").get("fovy") == "60"
    with pytest.raises(ValueError):
        retail.convert_angles(element, "unknown")


def test_never_overwrites_or_clears_existing_directory(tmp_path):
    sentinel = tmp_path / "user_work.txt"
    sentinel.write_text("keep")
    with pytest.raises(FileExistsError):
        retail.build_retail_scene(ROBOT, SUPERMARKET, tmp_path)
    assert sentinel.read_text() == "keep"
    assert not (tmp_path / "scene.xml").exists()


def test_mesh_collision_baseline_remains_explicitly_reproducible(tmp_path):
    scene, manifest = retail.build_retail_scene(
        ROBOT, SUPERMARKET, tmp_path / "mesh_baseline", collision_mode="mesh")
    root = ET.parse(scene).getroot()
    assert manifest["collision_mode"] == "mesh"
    for product in manifest["products"]:
        collision = root.find(f".//geom[@name='{product['collision_geom']}']")
        visual = root.find(f".//geom[@name='{product['visual_geom']}']")
        assert collision.get("type") == visual.get("type") == "mesh"
        assert collision.get("mesh") == visual.get("mesh")
    with pytest.raises(ValueError, match="collision_mode"):
        retail.build_retail_scene(ROBOT, SUPERMARKET, tmp_path / "invalid", collision_mode="disabled")


def test_moveit_product_export_preserves_unique_geometry_and_blocks_runtime_use(built, tmp_path):
    import mujoco

    spec = importlib.util.spec_from_file_location(
        "retail_moveit_export", ROOT / "scripts/export_openflex_retail_moveit_objects.py")
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    scene, manifest = built
    manifest_path = scene.parent / "manifest.json"
    output = tmp_path / "moveit_objects"
    destination, result = exporter.export_objects(scene, manifest_path, output)
    assert destination.is_file()
    assert result["frame_id"] == "mujoco_sim_world"
    assert result["frame_registered_to_live_map"] is False
    assert result["runtime_consumable"] is False
    assert result["execution_enabled"] is False
    assert len(result["objects"]) == 6
    assert len({item["id"] for item in result["objects"]}) == 6
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for obj, product in zip(result["objects"], manifest["products"]):
        assert obj["id"] == product["instance_id"]
        assert obj["slot_id"] == product["slot_id"]
        assert obj["role"] == "movable_product_world_object"
        assert obj["shape"] == "box"
        assert len(obj["dimensions_m"]) == 3 and all(value > 0 for value in obj["dimensions_m"])
        assert len(obj["pose"]["orientation_xyzw"]) == 4
        assert sum(value * value for value in obj["pose"]["orientation_xyzw"]) == pytest.approx(1.0)
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, product["collision_geom"])
        assert obj["dimensions_m"] == pytest.approx((2.0 * model.geom_size[geom_id]).tolist())
        assert obj["pose"]["position_m"] == pytest.approx(data.geom_xpos[geom_id].tolist())
    with pytest.raises(FileExistsError):
        exporter.export_objects(scene, manifest_path, output)
    altered_scene = tmp_path / "altered_scene.xml"
    altered_scene.write_text(scene.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="hash"):
        exporter.export_objects(altered_scene, manifest_path, tmp_path / "hash_mismatch")
    assert not (tmp_path / "hash_mismatch").exists()


def test_empty_mesh_and_rotated_support_fail_closed(tmp_path):
    mesh = tmp_path / "bad.obj"
    mesh.write_text("v nan 0 0\n")
    with pytest.raises(ValueError):
        retail.mesh_bounds(mesh)
    world = ET.fromstring('<worldbody><body euler="0 0 90"><geom name="support" '
                          'type="box" size="1 1 1"/></body></worldbody>')
    with pytest.raises(ValueError, match="Rotated"):
        retail.box_world_bounds(world, "support")


def test_mujoco_compiles_mass_and_positions_without_steps(built):
    pytest.importorskip("mujoco")
    spec = importlib.util.spec_from_file_location("retail_cli", ROOT / "scripts/build_openflex_retail_scene.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    report = cli.inspect_scene(*built)
    assert report["simulation_steps"] == 0
    assert not report["ros_accessed"]
    assert not report["execution_ready"]
    assert all(product["mass_matches_assumption"] and product["collision_enabled"] and
               product["body_origin_matches_manifest"] for product in report["products"])
    assert all(not product["initial_product_contacts"] for product in report["products"])


def test_full_source_product_set_rebuilds_source_slots_and_uses_compiled_mesh_boxes(tmp_path):
    import mujoco

    scene, manifest = retail.build_retail_scene(
        ROBOT, SUPERMARKET, tmp_path / "full_scene", product_set="full",
        assumed_product_mass_kg=0.3)
    assert len(manifest["products"]) == 102
    assert len({product["instance_id"] for product in manifest["products"]}) == 102
    assert len({product["simulation_sku"] for product in manifest["products"]}) == 19
    assert all(product["inventory_sku"] is None for product in manifest["products"])
    assert all(product["mass_provenance"] == "uniform_simulation_assumption_not_measured"
               for product in manifest["products"])
    assert all(product["source_slot_assignment_preserved"] for product in manifest["products"])
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for product in manifest["products"]:
        body = model.body(product["body"])
        collision = model.geom(product["collision_geom"])
        assert body.mass[0] == pytest.approx(0.3)
        assert collision.type[0] == mujoco.mjtGeom.mjGEOM_BOX
        assert data.xpos[body.id] == pytest.approx(product["initial_position"])
        assert product["support_geom"]
        assert product["support_bounds"] is not None
        assert product["support_gap_m"] == pytest.approx(0.001)
        assert product["world_mesh_bounds"]["min"][2] == pytest.approx(
            product["support_bounds"]["max"][2] + product["support_gap_m"])
    cli_spec = importlib.util.spec_from_file_location(
        "retail_full_audit_cli", ROOT / "scripts/build_openflex_retail_scene.py")
    cli = importlib.util.module_from_spec(cli_spec)
    cli_spec.loader.exec_module(cli)
    report = cli.inspect_scene(scene, manifest)
    product_contacts = [contact for contact in report["contacts"]
                        if contact["body1"].startswith("product_") or
                        contact["body2"].startswith("product_")]
    assert not [contact for contact in product_contacts if contact["distance_m"] < -1e-6]


def test_rendered_observation_is_explicitly_static(built, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    spec = importlib.util.spec_from_file_location("retail_render_cli", ROOT / "scripts/build_openflex_retail_scene.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    report = cli.inspect_scene(*built, render=True)
    assert report["static_observation_all_instances_visible"]
    assert not report["observation_configuration"]["penetrations_over_1um"]
    assert report["observation_configuration"]["transition_validated"] is False
    assert report["recognition_performed"] is False
    assert report["simulation_steps"] == 0
    assert all(count == 0 for count in report["visibility"]["head"].values())


def test_isolated_dynamics_audit_reports_contact_calibration_without_execution(built, tmp_path):
    spec = importlib.util.spec_from_file_location(
        "retail_dynamics_cli", ROOT / "scripts/audit_openflex_retail_dynamics.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    output = tmp_path / "dynamics"
    report = cli.audit(built[0], duration_s=1.0, output_directory=output)
    assert report["simulation_steps"] > 0
    assert report["ros_accessed"] is False
    assert report["live_runtime_modified"] is False
    assert report["execution_ready"] is False
    assert report["robot_collision_for_isolation"]["disabled_geoms"] > 0
    assert len(report["products"]) == 6
    assert (output / "dynamic_audit.json").is_file()
