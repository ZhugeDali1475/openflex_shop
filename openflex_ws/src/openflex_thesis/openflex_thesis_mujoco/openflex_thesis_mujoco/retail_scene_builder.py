"""Build an opt-in, offline retail fixture; never replace the live task scene."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from .model_builder import prepare_robot_tree, resolve_upstream_model


PRODUCTS = (
    ("cracker_box", "盒装饼干仿真资产", 0.25),
    ("mustard_bottle", "芥末瓶仿真资产", 0.35),
    ("tomato_soup_can", "番茄汤罐仿真资产", 0.20),
)
SUPPORT_GAP_M = 0.001
DEFAULT_FULL_PRODUCT_MASS_KG = 0.25


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _numbers(text):
    values = [float(value) for value in text.split()]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Non-finite scene geometry")
    return values


def _vector(values):
    return " ".join(format(value, ".12g") for value in values)


def mesh_bounds(path):
    """Measure raw OBJ coordinates, including the non-centered YCB origin."""
    lower = [math.inf] * 3
    upper = [-math.inf] * 3
    count = 0
    with Path(path).open() as stream:
        for line in stream:
            fields = line.split()
            if not fields or fields[0] != "v":
                continue
            vertex = _numbers(" ".join(fields[1:4]))
            if len(vertex) != 3:
                raise ValueError("Expected three-dimensional OBJ vertices")
            lower = [min(previous, value) for previous, value in zip(lower, vertex)]
            upper = [max(previous, value) for previous, value in zip(upper, vertex)]
            count += 1
    if not count or any(high <= low for low, high in zip(lower, upper)):
        raise ValueError("Empty or degenerate product mesh")
    return {"min": lower, "max": upper, "vertices": count}


def convert_angles(element, source_unit):
    """Convert orientation angles only; MJCF camera fovy is always degrees."""
    if source_unit not in {"degree", "radian"}:
        raise ValueError("Unsupported compiler angle unit")
    if source_unit == "radian":
        return
    for child in element.iter():
        for attribute in ("euler", "axisangle"):
            if attribute in child.attrib:
                values = _numbers(child.get(attribute))
                expected = 3 if attribute == "euler" else 4
                if len(values) != expected:
                    raise ValueError("Malformed orientation")
                converted = [math.radians(value) for value in values] if attribute == "euler" else (
                    values[:3] + [math.radians(values[3])]
                )
                child.set(attribute, _vector(converted))


def box_world_bounds(worldbody, geom_name):
    """Resolve the translated, axis-aligned supports in the source fixture."""
    matches = []

    def visit(element, offset):
        if element.tag not in {"worldbody", "body", "geom"}:
            return
        if any(key in element.attrib for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("Rotated fixture requires an explicit support transform")
        position = _numbers(element.get("pos", "0 0 0"))
        if len(position) != 3:
            raise ValueError("Invalid fixture position")
        current = [value + previous for value, previous in zip(position, offset)]
        if element.tag == "geom" and element.get("name") == geom_name:
            if element.get("type") != "box":
                raise ValueError("Fixture support must be a box")
            size = _numbers(element.get("size", ""))
            if len(size) != 3 or any(value <= 0 for value in size):
                raise ValueError("Invalid support half-size")
            matches.append({"min": [value - half for value, half in zip(current, size)],
                            "max": [value + half for value, half in zip(current, size)]})
        for child in element:
            visit(child, current)

    visit(worldbody, [0.0, 0.0, 0.0])
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one support geom: {geom_name}")
    return matches[0]


def _static_elements(source_world):
    result = []
    for element in source_world:
        if element.tag == "body" and element.get("name", "").startswith("product_"):
            continue
        if any(child.tag in {"joint", "freejoint"} for child in element.iter()):
            raise ValueError("Unexpected dynamic body in static retail fixture")
        result.append(deepcopy(element))
    return result


def _rotation_matrix(quaternion):
    w, x, y, z = quaternion
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _rotate_points(points, quaternion):
    rotation = _rotation_matrix(quaternion)
    return [[sum(rotation[row][column] * point[column] for column in range(3))
             for row in range(3)] for point in points]


def _compiled_mesh_points(model, geom_id):
    mesh_id = int(model.geom_dataid[geom_id])
    first = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    mesh_vertices = model.mesh_vert[first:first + count].tolist()
    rotation = _rotation_matrix(model.geom_quat[geom_id])
    geom_position = model.geom_pos[geom_id].tolist()
    return [[sum(rotation[row][column] * point[column] for column in range(3)) + geom_position[row]
             for row in range(3)] for point in mesh_vertices]


def _bounds(points):
    return {key: [function(point[axis] for point in points) for axis in range(3)]
            for key, function in (("min", min), ("max", max))}


def _full_source_products(source_world, source_root, source_path, assumed_mass_kg, collision_mode):
    import mujoco

    source_model = mujoco.MjModel.from_xml_path(str(source_path))
    compiler = source_root.find("compiler")
    mesh_directory = compiler.get("meshdir", ".") if compiler is not None else "."
    mesh_assets = {element.get("name"): element for element in source_root.findall("asset/mesh")}
    shelf_centers = ((-1.1, 1.18), (1.1, 1.18), (-1.1, -1.18), (1.1, -1.18))
    bodies = []
    products = []
    source_products = [body for body in source_world.findall("body")
                       if body.get("name", "").startswith("product_")]
    if not source_products:
        raise ValueError("Full product mode requires product_* bodies in the source scene")
    if not math.isfinite(assumed_mass_kg) or assumed_mass_kg <= 0:
        raise ValueError("assumed product mass must be a finite positive value")
    for source_body in source_products:
        body_name = source_body.get("name", "")
        match = re.fullmatch(r"product_(.+)_uid(\d+)", body_name)
        if match is None or set(source_body.attrib) != {"name", "pos"}:
            raise ValueError(f"Unsupported full product body: {body_name}")
        source_position = _numbers(source_body.get("pos", ""))
        if len(source_position) != 3:
            raise ValueError(f"Invalid product position: {body_name}")
        joints = [element for element in source_body if element.tag in {"joint", "freejoint"}]
        geoms = source_body.findall("geom")
        if len(joints) != 1 or joints[0].tag != "freejoint" or len(geoms) != 2:
            raise ValueError(f"Unsupported product joint/geometry structure: {body_name}")
        visual = next((geom for geom in geoms if geom.get("contype") == "0"), None)
        collision = next((geom for geom in geoms if geom is not visual), None)
        if visual is None or collision is None or visual.get("type") != "mesh" or collision.get("type") != "mesh":
            raise ValueError(f"Product must have separate visual and mesh collision geoms: {body_name}")
        mesh_name = visual.get("mesh")
        if not mesh_name or collision.get("mesh") != mesh_name:
            raise ValueError(f"Visual/collision mesh mismatch: {body_name}")
        if any(key in geom.attrib for geom in geoms for key in ("pos", "quat", "euler", "axisangle")):
            raise ValueError(f"Transformed product mesh requires explicit conversion: {body_name}")
        mesh_asset = mesh_assets.get(mesh_name)
        if mesh_asset is None or set(mesh_asset.attrib) != {"name", "file"}:
            raise ValueError(f"Unsupported product mesh asset: {mesh_name}")
        mesh_path = (source_path.parent / mesh_directory / mesh_asset.get("file")).resolve(strict=True)
        source_geom_id = mujoco.mj_name2id(
            source_model, mujoco.mjtObj.mjOBJ_GEOM, collision.get("name"))
        if source_geom_id < 0:
            raise ValueError(f"Could not compile product collision geometry: {body_name}")
        local_mesh_points = _compiled_mesh_points(source_model, source_geom_id)
        local_bounds = _bounds(local_mesh_points)
        source_bounds = mesh_bounds(mesh_path)

        shelf_group = min(range(4), key=lambda index: math.hypot(
            source_position[0] - shelf_centers[index][0],
            source_position[1] - shelf_centers[index][1])) + 1
        shelf_x, shelf_y = shelf_centers[shelf_group - 1]
        face = "pos" if source_position[0] >= shelf_x else "neg"
        level = max(0, min(4, int(round((source_position[2] - 0.21) / 0.4))))
        slot_centers = (shelf_y - 0.73, shelf_y, shelf_y + 0.73)
        source_slot = min(range(3), key=lambda index: abs(source_position[1] - slot_centers[index]))
        support_name = f"shelf_group_{shelf_group}_shelf_L{level}_{face}_body"
        support = box_world_bounds(source_world, support_name)
        support_center = [(support["min"][axis] + support["max"][axis]) / 2 for axis in range(3)]
        normalized_slot_y = shelf_y + (-0.65, 0.0, 0.65)[source_slot]

        body = deepcopy(source_body)
        visual_copy = body.find(f"geom[@name='{visual.get('name')}']")
        visual_copy.set("density", "0")
        collision_copy = body.find(f"geom[@name='{collision.get('name')}']")
        original_yaw = [1.0, 0.0, 0.0, 0.0]
        selected_pose = None
        for yaw in (0.0, math.pi / 2):
            yaw_quaternion = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
            rotated_points = _rotate_points(local_mesh_points, yaw_quaternion)
            rotated_bounds = _bounds(rotated_points)
            position = [support_center[0] - (rotated_bounds["min"][0] + rotated_bounds["max"][0]) / 2,
                        normalized_slot_y - (rotated_bounds["min"][1] + rotated_bounds["max"][1]) / 2,
                        support["max"][2] + SUPPORT_GAP_M - rotated_bounds["min"][2]]
            world_bounds = {key: [value + offset for value, offset in zip(rotated_bounds[key], position)]
                            for key in ("min", "max")}
            fits = all(world_bounds["min"][axis] >= support["min"][axis] + 0.005 and
                       world_bounds["max"][axis] <= support["max"][axis] - 0.005
                       for axis in (0, 1))
            if fits:
                selected_pose = yaw_quaternion, position, world_bounds, rotated_bounds
                break
        if selected_pose is None:
            raise ValueError(f"Product footprint does not fit its source shelf slot: {body_name}")
        yaw_quaternion, position, world_bounds, rotated_bounds = selected_pose
        if position[2] + rotated_bounds["max"][2] > support["max"][2] + 0.4 - 0.015 and level < 4:
            raise ValueError(f"Product does not clear the next shelf: {body_name}")
        body.set("pos", _vector(position))
        if yaw_quaternion != original_yaw:
            body.set("quat", _vector(yaw_quaternion))
        if collision_mode == "box":
            box_position = [(local_bounds["min"][axis] + local_bounds["max"][axis]) / 2
                            for axis in range(3)]
            box_half = [(local_bounds["max"][axis] - local_bounds["min"][axis]) / 2
                        for axis in range(3)]
            collision_copy.attrib.pop("mesh", None)
            collision_copy.set("type", "box")
            collision_copy.set("pos", _vector(box_position))
            collision_copy.set("size", _vector(box_half))
            collision_representation = "conservative_body_aligned_box_from_compiled_mesh_bounds_not_calibrated"
        else:
            collision_copy.set("type", "mesh")
            collision_copy.set("mesh", mesh_name)
            collision_representation = "source_high_detail_convex_mesh_not_calibrated"
        collision_copy.set("mass", format(float(assumed_mass_kg), ".12g"))
        collision_copy.set("condim", "3")
        collision_copy.set("friction", "0.5 0.1 0.1")
        collision_copy.set("rgba", "0 0 0 0")
        bodies.append(body)
        sku = match.group(1)
        products.append({
            "instance_id": body_name,
            "simulation_sku": sku,
            "display_name": sku,
            "inventory_sku": None,
            "identity_source": "source_scene_asset_name_not_live_visual_recognition",
            "slot_id": f"sim_shelf{ shelf_group }_{face}_L{level}_slot{source_slot + 1}",
            "shelf_id": f"shelf_group_{shelf_group}",
            "level": level,
            "face": face,
            "body": body_name,
            "free_joint": joints[0].get("name"),
            "visual_geom": visual.get("name"),
            "collision_geom": collision.get("name"),
            "mesh_path": str(mesh_path),
            "mesh_sha256": sha256(mesh_path),
            "mesh_bounds": source_bounds,
            "compiled_mesh_bounds_in_body_frame": local_bounds,
            "initial_position": position,
            "source_position": source_position,
            "initial_quaternion_wxyz": yaw_quaternion,
            "world_mesh_bounds": world_bounds,
            "support_geom": support_name,
            "support_bounds": support,
            "support_gap_m": SUPPORT_GAP_M,
            "mass_kg": float(assumed_mass_kg),
            "mass_provenance": "uniform_simulation_assumption_not_measured",
            "collision_representation": collision_representation,
            "source_slot_assignment_preserved": True,
        })
    return bodies, products


def _import_assets(robot, source, source_path, elements):
    asset = robot.find("asset")
    if asset is None:
        asset = ET.SubElement(robot, "asset")
    available = {(element.tag, element.get("name")): element for element in source.findall("asset/*")}
    imported = set()
    compiler = source.find("compiler")

    def require(kind, name):
        key = (kind, name)
        imported_name = "retail_asset_" + name
        if key in imported:
            return imported_name
        if key not in available:
            raise ValueError(f"Missing retail asset: {key}")
        if any(child.tag == kind and child.get("name") == imported_name for child in asset):
            raise ValueError(f"Robot/retail asset name collision: {key}")
        element = deepcopy(available[key])
        element.set("name", imported_name)
        if element.get("texture"):
            element.set("texture", require("texture", element.get("texture")))
        if element.get("file"):
            directory = compiler.get(kind + "dir", ".") if compiler is not None else "."
            path = (source_path.parent / directory / element.get("file")).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            element.set("file", str(path))
        asset.append(element)
        imported.add(key)
        return imported_name

    for element in elements:
        for child in element.iter():
            for kind in ("material", "mesh"):
                if child.get(kind):
                    child.set(kind, require(kind, child.get(kind)))


def build_retail_scene(upstream_model_path, supermarket_path, output_directory, collision_mode="box",
                       product_set="sample", assumed_product_mass_kg=DEFAULT_FULL_PRODUCT_MASS_KG):
    """Write only to a new directory. No ROS imports, actions, steps or live paths."""
    output = Path(output_directory).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite an existing directory: {output}")
    if collision_mode not in {"box", "mesh"}:
        raise ValueError("collision_mode must be 'box' or 'mesh'")
    if product_set not in {"sample", "full"}:
        raise ValueError("product_set must be 'sample' or 'full'")
    upstream = resolve_upstream_model(str(upstream_model_path))
    supermarket = Path(supermarket_path).expanduser().resolve(strict=True)
    tree, robot_world = prepare_robot_tree(str(upstream))
    robot = tree.getroot()
    robot.set("model", "openflex_retail_offline_fixture_v1")
    if robot.find("compiler").get("angle", "degree") != "radian":
        raise ValueError("The prepared OpenFlex robot must use radians")
    source = ET.parse(supermarket).getroot()
    if source.find("include") is not None or source.find("contact") is not None:
        raise ValueError("Retail includes and contact overrides require explicit review")
    source_world = source.find("worldbody")
    if source_world is None:
        raise ValueError("Retail source has no worldbody")
    defaults = source.find("default")
    if defaults is not None and (len(defaults) != 1 or defaults[0].tag != "geom"
                                 or set(defaults[0].attrib) - {"condim", "friction"}):
        raise ValueError("Retail defaults require explicit inheritance handling")
    elements = _static_elements(source_world)
    compiler = source.find("compiler")
    source_unit = compiler.get("angle", "degree") if compiler is not None else "degree"
    if compiler is not None and compiler.get("eulerseq", "xyz") != "xyz":
        raise ValueError("Retail Euler sequence must match the robot compiler")
    for element in elements:
        convert_angles(element, source_unit)
        if any(child.get("class") or child.get("childclass") for child in element.iter()):
            raise ValueError("Retail default classes must be flattened explicitly")
        for geom in element.iter("geom"):
            for name, value in {"condim": "3", "friction": "0.5 0.1 0.1",
                                "contype": "1", "conaffinity": "1"}.items():
                geom.set(name, geom.get(name, defaults[0].get(name, value) if defaults is not None else value))

    instances = []
    if product_set == "full":
        product_bodies, instances = _full_source_products(
            source_world, source, supermarket, float(assumed_product_mass_kg), collision_mode)
        elements.extend(product_bodies)
    else:
        for sku_index, (sku, display_name, mass) in enumerate(PRODUCTS):
            mesh = source.find(f"asset/mesh[@name='mesh_{sku}']")
            if mesh is None or set(mesh.attrib) - {"name", "file"}:
                raise ValueError("Product mesh must have unscaled, untransformed OBJ coordinates")
            mesh_directory = compiler.get("meshdir", ".") if compiler is not None else "."
            mesh_path = (supermarket.parent / mesh_directory / mesh.get("file")).resolve(strict=True)
            bounds = mesh_bounds(mesh_path)
            for level in (1, 2):
                support_name = f"shelf_group_1_shelf_L{level}_pos_body"
                support = box_world_bounds(source_world, support_name)
                center_xy = [(support["min"][0] + support["max"][0]) / 2,
                             (support["min"][1] + support["max"][1]) / 2 + (sku_index - 1) * 0.50]
                position = [center_xy[axis] - (bounds["min"][axis] + bounds["max"][axis]) / 2
                            for axis in range(2)]
                position.append(support["max"][2] - bounds["min"][2] + SUPPORT_GAP_M)
                world_bounds = {key: [value + offset for value, offset in zip(bounds[key], position)]
                                for key in ("min", "max")}
                if any(world_bounds["min"][axis] < support["min"][axis] + 0.005 or
                       world_bounds["max"][axis] > support["max"][axis] - 0.005 for axis in (0, 1)):
                    raise ValueError("Product footprint does not fit the shelf")
                instance_id = f"retail_{sku}_L{level}"
                body = ET.Element("body", {"name": instance_id, "pos": _vector(position)})
                ET.SubElement(body, "freejoint", {"name": instance_id + "_free"})
                visual_name, collision_name = instance_id + "_visual", instance_id + "_collision"
                ET.SubElement(body, "geom", {"name": visual_name, "type": "mesh", "mesh": f"mesh_{sku}",
                                            "material": f"mat_{sku}", "contype": "0", "conaffinity": "0",
                                            "density": "0", "group": "2"})
                local_center = [(bounds["min"][axis] + bounds["max"][axis]) / 2 for axis in range(3)]
                half_extents = [(bounds["max"][axis] - bounds["min"][axis]) / 2 for axis in range(3)]
                collision_attributes = {"name": collision_name, "contype": "1", "conaffinity": "3",
                                        "mass": str(mass), "condim": "3", "friction": "0.5 0.1 0.1",
                                        "rgba": "0 0 0 0", "group": "3"}
                if collision_mode == "box":
                    collision_attributes.update({"type": "box", "pos": _vector(local_center),
                                                 "size": _vector(half_extents)})
                    collision_description = "conservative_axis_aligned_box_from_obj_bounds_not_calibrated"
                else:
                    collision_attributes.update({"type": "mesh", "mesh": f"mesh_{sku}"})
                    collision_description = "same_high_detail_convex_mesh_as_visual_not_calibrated"
                ET.SubElement(body, "geom", collision_attributes)
                elements.append(body)
                instances.append({
                    "instance_id": instance_id, "simulation_sku": sku, "display_name": display_name,
                    "inventory_sku": None, "identity_source": "explicit_simulation_fixture_not_inventory_database",
                    "slot_id": f"sim_shelf1_pos_L{level}_slot{sku_index + 1}",
                    "shelf_id": "shelf_group_1", "level": level, "face": "pos_x",
                    "body": instance_id, "free_joint": instance_id + "_free",
                    "visual_geom": visual_name, "collision_geom": collision_name,
                    "mesh_path": str(mesh_path), "mesh_sha256": sha256(mesh_path), "mesh_bounds": bounds,
                    "initial_position": position, "initial_quaternion_wxyz": [1, 0, 0, 0],
                    "world_mesh_bounds": world_bounds, "support_geom": support_name, "support_bounds": support,
                    "support_gap_m": SUPPORT_GAP_M, "mass_kg": mass,
                    "mass_provenance": "simulation_assumption_not_measured",
                    "collision_representation": collision_description,
                })

    original_floor = robot_world.find("geom[@name='floor']")
    if original_floor is not None:
        robot_world.remove(original_floor)
    for element in elements:
        for child in element.iter():
            name = child.get("name")
            if name and any(existing.get("name") == name for existing in robot.iter(child.tag)):
                raise ValueError(f"Robot/retail element name collision: {child.tag} {name}")
    _import_assets(robot, source, supermarket, elements)
    robot_world.extend(elements)
    ET.SubElement(robot_world, "camera", {
        "name": "retail_fixture_close", "pos": "0.65 1.18 1.6",
        "xyaxes": "0 1 0 -0.342 0 0.940", "fovy": "55",
    })
    visual = robot.find("visual")
    if visual is None:
        visual = ET.SubElement(robot, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", "1280")
    global_visual.set("offheight", "960")
    delivery = []
    for location in ("top_left", "top_right", "bottom_left", "bottom_right"):
        geom_name = f"counter_{location}_top"
        bounds = box_world_bounds(source_world, geom_name)
        delivery.append({"region_id": f"sim_delivery_{location}", "support_geom": geom_name,
                         "support_bounds": bounds, "surface_z": bounds["max"][2],
                         "status": "support_mapped_approach_and_clearance_not_validated"})
    assets = [{"kind": element.tag, "name": element.get("name"), "path": element.get("file"),
               "sha256": sha256(element.get("file"))} for element in robot.findall("asset/*")
              if element.get("file")]
    manifest = {
        "schema_version": 1, "execution_enabled": False, "runtime_scene_replaced": False,
        "validation_scope": "offline_fixture_only_no_ROS_no_motion_no_mj_step",
        "source_robot": {"path": str(upstream), "sha256": sha256(upstream)},
        "source_supermarket": {"path": str(supermarket), "sha256": sha256(supermarket)},
        "database_accessed": False, "inventory_mapping_complete": False,
        "frame": "simulation_world_meters_radians_not_registered_to_live_map",
        "initial_robot_state": "upstream_qpos0_not_a_validated_safe_parking_posture",
        "product_set": product_set,
        "assumed_product_mass_kg": float(assumed_product_mass_kg) if product_set == "full" else None,
        "products": instances, "delivery_regions": delivery, "assets": assets,
        "collision_mode": collision_mode,
        "limitations": ["No multi-SKU bridge or planning-scene adapter yet",
                        "No recognition accuracy, grasp success or continuous-path safety claim",
                        "No measured mass, friction, camera calibration or hardware validation"],
    }
    output.mkdir(parents=True, exist_ok=False)
    destination = output / "scene.xml"
    ET.indent(tree, space="  ")
    with destination.open("xb") as stream:
        tree.write(stream, encoding="utf-8", xml_declaration=True)
    manifest["scene_sha256"] = sha256(destination)
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    return destination, manifest
