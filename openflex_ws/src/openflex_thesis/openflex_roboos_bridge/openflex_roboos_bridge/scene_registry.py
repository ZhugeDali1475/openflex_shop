import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def finite_vector(values, count):
    result = list(map(float, values))
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f"Expected {count} finite values")
    return result


def load_scene_registry(config_path, model_path):
    try:
        return _load_scene_registry(config_path, model_path)
    except (ET.ParseError, yaml.YAMLError) as error:
        raise ValueError(f"Invalid scene configuration: {error}") from error


def _load_scene_registry(config_path, model_path):
    config_path, model_path = Path(config_path), Path(model_path)
    config_bytes, model_bytes = config_path.read_bytes(), model_path.read_bytes()
    config = yaml.safe_load(config_bytes)
    simulation = config["openflex_thesis_mujoco_bridge"]["ros__parameters"]
    mission = config["thesis_mujoco_mission_client"]["ros__parameters"]
    object_id = simulation["object_id"]
    if not isinstance(object_id, str) or not object_id or object_id != mission["object_id"]:
        raise ValueError("Simulation and mission object IDs do not match")
    world = ET.fromstring(model_bytes).find("worldbody")
    table = world.find("body[@name='thesis_delivery_table']/geom[@name='thesis_delivery_top']")
    table_position = finite_vector(table.get("pos").split(), 3)
    half_size = finite_vector(table.get("size").split(), 3)
    if table.get("type") != "box" or any(value <= 0 for value in half_size):
        raise ValueError("Delivery mapping requires a positive box")
    parent = world.find("body[@name='thesis_delivery_table']")
    if set(parent.attrib) != {"name"}:
        raise ValueError("Delivery table has an unsupported parent transform")
    object_body = world.find("body[@name='thesis_object']")
    object_geom = object_body.find("geom[@name='thesis_object_geom']")
    nominal = finite_vector(simulation["object_nominal_pose"], 4)
    model_position = finite_vector(object_body.get("pos").split(), 3)
    if any(abs(actual - expected) > 1e-8 for actual, expected in zip(model_position, nominal[:3])):
        raise ValueError("Configured nominal object position differs from generated scene")
    pickup = finite_vector(mission["pickup_observation_pose"], 3)
    legacy_delivery = finite_vector(mission["delivery_pose"], 3)
    inside_table = all(abs(legacy_delivery[axis] - table_position[axis]) <= half_size[axis]
                       for axis in (0, 1))
    top = table_position[2] + half_size[2]
    object_half_size = finite_vector(object_geom.get("size").split(), 3)
    release_surface = float(simulation["delivery_surface_z_m"])
    if not math.isfinite(release_surface):
        raise ValueError("Release surface must be finite")
    return {
        "schema_version": 1, "status": "loaded", "frame_id": "map",
        "provenance": "static_simulation_config_and_generated_MJCF_not_visual_detection",
        "runtime_geometry_verified": False,
        "physical_calibration": False, "execution_enabled": False,
        "sources": {
            "config": str(config_path), "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
            "model": str(model_path), "model_sha256": hashlib.sha256(model_bytes).hexdigest(),
        },
        "objects": {object_id: {
            "body": "thesis_object", "geom": "thesis_object_geom",
            "nominal_pose_xyzyaw": nominal, "live_pose_source": "/thesis/object_observation",
            "dimensions_m": [2 * value for value in object_half_size],
            "visual_identity_verified": False,
        }},
        "stations": {
            "pickup_zone": {"role": "pickup_observation", "configured_pose_xyyaw": pickup,
                            "navigation_pose": None, "approach_candidate_xyyaw": pickup,
                            "validation": "configured_only_path_not_validated"},
            "delivery_zone": {"role": "delivery_approach", "configured_pose_xyyaw": legacy_delivery,
                              "navigation_pose": None,
                              "approach_candidate_xyyaw": [table_position[0] + half_size[0] + 0.5,
                                                           table_position[1], math.pi],
                              "candidate_basis": "table_east_edge_plus_0.5m_requires_runtime_full_robot_path_check",
                              "validation": "rejected_inside_table_footprint" if inside_table else "configured_only_path_not_validated"},
        },
        "affordances": {"delivery_zone": {
            "role": "table_support_surface", "geom": "thesis_delivery_top",
            "surface_center_xyz": [table_position[0], table_position[1], top],
            "surface_dimensions_xy": [2 * half_size[0], 2 * half_size[1]],
            "configured_release_center_z_m": release_surface,
            "expected_upright_object_center_z_m": top + object_half_size[2],
            "upright_center_height_consistent": abs(release_surface - top - object_half_size[2]) <= 0.005,
            "legacy_place_mechanism": "teleport_then_immediate_status_not_verified_trajectory",
            "release_validation": "not_validated_no_place_execution",
        }},
    }
