"""Canonical model-facing tool surface for the isolated OpenFlex delivery loop.

The language model is allowed to select **only** the tools listed in
``MODEL_FACING_TOOLS``.  Everything else the MCP server happens to serve --
request-id/status/cancel plumbing, internal diagnostics, controller or
command-line handles, arbitrary paths or coordinates -- is host-managed and must
never be advertised to the model.

The model-facing projection additionally narrows every enum-like argument to the
runtime-validated registered values, so the model cannot invent an object, an
unauthorized delivery table, an unbounded timeout, a free coordinate or a
parallel motion request:

* ``request_id`` / ``timeout_sec`` are created by the host and are removed from
  the model-facing schema entirely (the runner records them for audit);
* ``target_id`` / ``affordance_id`` are narrowed to the validated left delivery
  table and pickup station, so the unvalidated right table is not even offered;
* free text is length-bounded.

This module is pure Python on purpose: the runner, the bridge tests and the
offline contract tests all import it without ROS, MuJoCo or MCP.
"""

from copy import deepcopy

# Tools the model may select. Observation/state first, then the registered
# restriction-compliant atomic skills, then the registered composite skill.
MODEL_FACING_TOOLS = (
    "get_robot_state",
    "observe_scene",
    "navigate_to_target",
    "grasp_object",
    "place_to_affordance",
    "search_and_pick",
)

# Served by the MCP server for the host/runner, deliberately NOT model-facing.
HOST_MANAGED_TOOLS = (
    "get_navigation_result",
    "get_skill_result",
    "cancel_current_task",
)

OBSERVATION_TOOLS = ("get_robot_state", "observe_scene")

# Atomic skills that command motion. ``search_and_pick`` is a composite skill,
# but it commands motion internally, so it is grouped here for the runtime.
MOTION_TOOLS = ("navigate_to_target", "grasp_object", "place_to_affordance")

COMPOSITE_TOOLS = ("search_and_pick",)

# Exact model-facing argument set per tool. Host-managed request_id/timeout_sec
# are never part of this set.
MODEL_FACING_ARGUMENTS = {
    "get_robot_state": (),
    "observe_scene": (),
    "navigate_to_target": ("target_id",),
    "grasp_object": ("perception_track_id",),
    "place_to_affordance": ("object_id", "affordance_id"),
    "search_and_pick": ("query", "color"),
}

# The only values the model may choose for enum-like arguments. These mirror the
# runtime-validated registry: the right delivery table is intentionally absent.
REGISTERED_VALUES = {
    "target_id": ("pickup_zone", "delivery_zone"),
    "object_id": ("thesis_item_01",),  # placement remains simulator-lifecycle scoped
    "affordance_id": ("delivery_zone",),
    "color": ("red", "blue", "green", "yellow", "magenta"),
}

# Free-text bounds for the one natural-language argument (the search query).
TEXT_BOUNDS = {"query": (1, 200)}

# Argument names that would leak free coordinates, arbitrary paths, controller
# handles or debug/protocol plumbing into the tool surface.
FORBIDDEN_ARGUMENT_NAMES = frozenset({
    "x", "y", "z", "w", "roll", "pitch", "yaw", "pose", "position", "orientation",
    "quaternion", "coordinate", "coordinates", "path", "trajectory", "waypoints",
    "joint_positions", "joints", "controller", "controllers", "topic", "topics",
    "node", "port", "url", "endpoint", "command", "commands", "shell", "script",
    "debug", "diagnostic", "diagnostics", "raw", "secret", "token", "password",
})

# Host-managed numeric bounds, mirrored from the serving layer. Recorded here so
# a reviewer can see the model never picks an unbounded timeout.
HOST_TIMEOUT_BOUNDS = {
    "navigate_to_target": (0.1, 90.0),
    "grasp_object": (0.1, 600.0),
    "place_to_affordance": (0.1, 600.0),
    "search_and_pick": (1.0, 600.0),
}


def check_argument_names(name, arguments):
    """Reject a tool whose inputs leak coordinates, paths or debug handles."""
    leaked = sorted(key for key in arguments
                    if isinstance(key, str) and key.lower() in FORBIDDEN_ARGUMENT_NAMES)
    if leaked:
        raise ValueError(f"{name} exposes free-coordinate/debug arguments: {leaked}")


def project_schema(name, schema):
    """Return the model-facing JSON schema for one advertised MCP tool.

    Host-managed arguments are dropped, enum-like arguments are replaced with the
    registered values, and free text is length-bounded. A tool whose advertised
    arguments do not match the declared model-facing set is rejected rather than
    silently narrowed.
    """
    if name not in MODEL_FACING_ARGUMENTS:
        raise ValueError(f"{name} is not a model-facing tool")
    expected = set(MODEL_FACING_ARGUMENTS[name])
    properties = dict(schema.get("properties") or {})
    check_argument_names(name, properties)
    if name == "grasp_object" and "object_id" in properties:
        raise ValueError("grasp_object must accept a visual perception_track_id, never object_id")
    model_visible = {key: value for key, value in properties.items()
                     if key not in ("request_id", "timeout_sec")}
    if set(model_visible) != expected:
        raise ValueError(f"{name}: advertised model-facing arguments {sorted(model_visible)} "
                         f"do not match the declared surface {sorted(expected)}")
    narrowed = {}
    for key in MODEL_FACING_ARGUMENTS[name]:
        spec = deepcopy(model_visible[key])
        if key in REGISTERED_VALUES:
            spec["type"] = "string"
            spec["enum"] = list(REGISTERED_VALUES[key])
            spec.pop("anyOf", None)
        elif key in TEXT_BOUNDS:
            low, high = TEXT_BOUNDS[key]
            spec["type"] = "string"
            spec["minLength"] = low
            spec["maxLength"] = high
            spec.pop("anyOf", None)
        narrowed[key] = spec
    return {"type": "object", "properties": narrowed,
            "required": list(MODEL_FACING_ARGUMENTS[name]),
            "additionalProperties": False}


def model_facing_tools(advertised):
    """Project the advertised MCP schemas onto the model-facing surface.

    ``advertised`` maps tool name -> input JSON schema. The returned mapping has
    exactly the ``MODEL_FACING_TOOLS`` keys; a missing tool is an error, so the
    runner fails closed instead of quietly offering a smaller surface.
    """
    missing = [name for name in MODEL_FACING_TOOLS if name not in advertised]
    if missing:
        raise ValueError(f"model-facing tools missing from the server: {missing}")
    surface = {}
    for name in MODEL_FACING_TOOLS:
        surface[name] = project_schema(name, advertised[name])
    validate_model_facing_surface(surface)
    return surface


def validate_advertised_surface(advertised):
    """Check the *server* surface never leaks coordinates/paths/debug handles."""
    for name, schema in advertised.items():
        check_argument_names(name, schema.get("properties") or {})


def validate_model_facing_surface(surface):
    """Assert the projected surface cannot express an unauthorized request."""
    if set(surface) != set(MODEL_FACING_TOOLS):
        raise ValueError("model-facing surface does not match the declared allow-list")
    for name, schema in surface.items():
        if schema.get("additionalProperties") is not False:
            raise ValueError(f"{name}: model-facing schema must forbid extra arguments")
        expected = set(MODEL_FACING_ARGUMENTS[name])
        if set(schema.get("properties") or {}) != expected:
            raise ValueError(f"{name}: projected arguments do not match the declared surface")
        if set(schema.get("required") or []) != expected:
            raise ValueError(f"{name}: every model-facing argument must be required")
        for key, spec in (schema.get("properties") or {}).items():
            if key in REGISTERED_VALUES:
                if spec.get("enum") != list(REGISTERED_VALUES[key]):
                    raise ValueError(f"{name}.{key}: enum is not the registered value set")
        for value in unauthorized_values():
            if value in str(schema):
                raise ValueError(f"{name}: unauthorized delivery table leaked into the schema")
    return True


def unauthorized_values():
    """Values that must never appear in any model-facing enum."""
    return ("sim_delivery_top_right",)
