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


def _world_box_position(world, body_name, geom_name):
    body = world.find(f"body[@name='{body_name}']")
    geom = world.find(f".//body[@name='{body_name}']/geom[@name='{geom_name}']")
    if body is None or geom is None or geom.get("type") != "box":
        raise ValueError(f"Missing box geometry mapping: {body_name}/{geom_name}")
    position = [0.0, 0.0, 0.0]
    chain = []
    current = geom
    while current is not None and current is not world:
        chain.append(current)
        current = next((parent for parent in world.iter()
                        if current in list(parent)), None)
    for element in reversed(chain):
        if any(key in element.attrib for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("Rotated delivery geometry requires explicit frame calibration")
        offset = finite_vector(element.get("pos", "0 0 0").split(), 3)
        position = [value + delta for value, delta in zip(position, offset)]
    return geom, position


def _delivery_mapping(simulation):
    """Return explicit logical destination -> MJCF body/geom mappings.

    The legacy runtime parameters remain the source of the left-table mapping.
    The right mapping is the stable competition-scene identity; it is only
    registered as geometry here and receives no executable route by default.
    A profile may override either mapping explicitly in delivery_destinations.
    """
    configured = simulation.get("delivery_destinations", {})
    if not isinstance(configured, dict):
        raise ValueError("delivery_destinations must be a mapping")
    result = {
        "sim_delivery_top_left": (
            str(simulation.get("delivery_table_body", "thesis_delivery_table")),
            str(simulation.get("delivery_surface_geom", "thesis_delivery_top")),
        ),
        "sim_delivery_top_right": ("counter_top_right", "counter_top_right_top"),
    }
    for destination_id, mapping in configured.items():
        if destination_id not in result or not isinstance(mapping, dict):
            raise ValueError(f"Unknown delivery destination mapping: {destination_id}")
        body, geom = mapping.get("body"), mapping.get("surface_geom")
        if not isinstance(body, str) or not body or not isinstance(geom, str) or not geom:
            raise ValueError(f"Incomplete delivery mapping for {destination_id}")
        result[destination_id] = (body, geom)
    if result.get("sim_delivery_top_right") and result["sim_delivery_top_left"] == result["sim_delivery_top_right"]:
        raise ValueError("Left and right delivery destinations must map to distinct body/geom pairs")
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
    destination_geometry = {}
    mappings = _delivery_mapping(simulation)
    # Keep old single-table fixtures loadable; reviewed competition scenes
    # expose counter_top_right and receive both canonical destinations.
    if (world.find("body[@name='counter_top_right']") is None
            and "sim_delivery_top_right" not in simulation.get("delivery_destinations", {})):
        mappings.pop("sim_delivery_top_right", None)
    for destination_id, (body_name, geom_name) in mappings.items():
        table, table_position = _world_box_position(world, body_name, geom_name)
        half_size = finite_vector(table.get("size").split(), 3)
        if table.get("type") != "box" or any(value <= 0 for value in half_size):
            raise ValueError(f"Delivery mapping for {destination_id} requires a positive box")
        destination_geometry[destination_id] = (body_name, geom_name, table, table_position, half_size)
    left_body, left_geom, table, table_position, half_size = destination_geometry["sim_delivery_top_left"]
    object_body = world.find("body[@name='thesis_object']")
    if object_body is None:
        raise ValueError("Scene has no thesis_object body")
    object_geom = object_body.find("geom[@name='thesis_object_geom']")
    if object_geom is None:
        raise ValueError("Scene has no thesis_object_geom")
    nominal = finite_vector(simulation["object_nominal_pose"], 4)
    model_position = finite_vector(object_body.get("pos").split(), 3)
    if any(abs(actual - expected) > 1e-8 for actual, expected in zip(model_position, nominal[:3])):
        raise ValueError("Configured nominal object position differs from generated scene")
    pickup_observation = finite_vector(mission["pickup_observation_pose"], 3)
    pickup = finite_vector(mission.get("pickup_navigation_pose", pickup_observation), 3)
    legacy_delivery = finite_vector(mission["delivery_pose"], 3)
    route = mission.get("delivery_route_waypoints", [])
    if not isinstance(route, list):
        raise ValueError("delivery_route_waypoints must be a list")
    if route and isinstance(route[0], (list, tuple)):
        # Legacy nested form, kept for existing offline profiles.
        route = [finite_vector(waypoint, 3) for waypoint in route]
    else:
        # Flat [x, y, yaw, x, y, yaw, ...] form is required for ROS param files,
        # which reject sequences nested inside sequences.
        if len(route) % 3:
            raise ValueError("delivery_route_waypoints must be a flat multiple-of-3 list")
        route = [finite_vector(route[index:index + 3], 3) for index in range(0, len(route), 3)]
    inside_table = all(abs(legacy_delivery[axis] - table_position[axis]) <= half_size[axis]
                       for axis in (0, 1))
    top = table_position[2] + half_size[2]
    object_half_size = finite_vector(object_geom.get("size").split(), 3)
    release_surface = float(simulation["delivery_surface_z_m"])
    if not math.isfinite(release_surface):
        raise ValueError("Release surface must be finite")
    destinations = {}
    affordances = {}
    stations = {
        "pickup_zone": {"role": "pickup_observation", "configured_pose_xyyaw": pickup_observation,
                        "navigation_pose": None, "approach_candidate_xyyaw": pickup,
                        "validation": "configured_only_path_not_validated"},
    }
    delivery_routes = mission.get("delivery_routes", {})
    if not isinstance(delivery_routes, dict):
        raise ValueError("delivery_routes must be a mapping")
    for destination_id, (body_name, geom_name, geometry, position, half) in destination_geometry.items():
        top_z = position[2] + half[2]
        dimensions_xy = [2 * half[0], 2 * half[1]]
        route_values = delivery_routes.get(destination_id, route if destination_id == "sim_delivery_top_left" else [])
        if not isinstance(route_values, list):
            raise ValueError(f"delivery route for {destination_id} must be a list")
        if route_values and isinstance(route_values[0], (list, tuple)):
            destination_route = [finite_vector(waypoint, 3) for waypoint in route_values]
        else:
            if len(route_values) % 3:
                raise ValueError(f"delivery route for {destination_id} must be a flat multiple-of-3 list")
            destination_route = [finite_vector(route_values[index:index + 3], 3)
                                 for index in range(0, len(route_values), 3)]
        approach = [position[0] + half[0] + 0.5, position[1], math.pi]
        station_validation = ("rejected_inside_table_footprint" if inside_table
                              else "configured_only_path_not_validated") if destination_id == "sim_delivery_top_left" else (
                                  "configured_only_path_not_validated" if destination_route else "geometry_registered_no_route")
        station = {"role": "delivery_approach", "configured_pose_xyyaw": legacy_delivery if destination_id == "sim_delivery_top_left" else None,
                   "navigation_pose": None, "approach_candidate_xyyaw": approach,
                   "route_waypoints": destination_route,
                   "candidate_basis": "table_east_edge_plus_0.5m_requires_runtime_full_robot_path_check",
                   "validation": station_validation}
        stations[destination_id] = station
        affordance = {
            "role": "table_support_surface", "body": body_name, "geom": geom_name,
            "surface_center_xyz": [position[0], position[1], top_z],
            "surface_dimensions_xy": dimensions_xy,
            "configured_release_center_z_m": release_surface if destination_id == "sim_delivery_top_left" else None,
            "expected_upright_object_center_z_m": top_z + object_half_size[2],
            "upright_center_height_consistent": (
                destination_id == "sim_delivery_top_left"
                and abs(release_surface - top_z - object_half_size[2]) <= 0.005
            ),
            "legacy_place_mechanism": "teleport_then_immediate_status_not_verified_trajectory",
            "release_validation": "not_validated_no_place_execution",
        }
        affordances[destination_id] = affordance
        destinations[destination_id] = {"body": body_name, "surface_geom": geom_name,
                                         "approach_candidate_xyyaw": approach,
                                         "route_waypoints": destination_route,
                                         "validated": False}
    # Preserve the old single-table key for existing read-only consumers.
    affordances["delivery_zone"] = dict(affordances["sim_delivery_top_left"])
    stations["delivery_zone"] = dict(stations["sim_delivery_top_left"])
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
        "stations": stations,
        "affordances": affordances,
        "delivery_destinations": destinations,
    }
