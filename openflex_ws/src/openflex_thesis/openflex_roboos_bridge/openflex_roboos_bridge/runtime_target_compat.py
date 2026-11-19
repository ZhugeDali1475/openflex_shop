"""Safe MCP-to-legacy target compatibility for the single-table runtime.

The static competition registry knows two logical tables, but the current
online ROS registry only knows ``delivery_zone`` (the left table).  This
module keeps that boundary explicit and deliberately refuses the unvalidated
right table before any navigation HTTP request is made.
"""

LEGACY_DELIVERY_ID = "delivery_zone"
LEFT_DELIVERY_ID = "sim_delivery_top_left"
RIGHT_DELIVERY_ID = "sim_delivery_top_right"
PICKUP_ID = "pickup_zone"


def resolve_navigation_target(target_id: str) -> str:
    """Map supported public IDs onto the currently deployed ROS registry."""
    if target_id in (LEGACY_DELIVERY_ID, LEFT_DELIVERY_ID):
        return LEGACY_DELIVERY_ID
    if target_id == PICKUP_ID:
        return PICKUP_ID
    if target_id == RIGHT_DELIVERY_ID:
        raise ValueError("sim_delivery_top_right is registered but not runtime route-validated")
    raise ValueError(f"Unknown registered target: {target_id}")


def navigate_compat(client, target_id: str, request_id: str, timeout_sec: float = 60.0):
    """Call the legacy runtime only after resolving a safe public target."""
    try:
        runtime_target = resolve_navigation_target(target_id)
    except ValueError as error:
        return {"status": "failed", "success": False, "error": str(error),
                "downstream_called": False}
    return client.navigation("/navigate", {"target_id": runtime_target,
        "request_id": request_id, "timeout_sec": timeout_sec})


def place_compat(client, object_id: str, affordance_id: str, request_id: str,
                 timeout_sec: float = 600.0):
    """Map only the validated legacy/left affordance; refuse right in MCP."""
    if affordance_id not in (LEGACY_DELIVERY_ID, LEFT_DELIVERY_ID):
        message = ("sim_delivery_top_right is registered but not runtime route-validated"
                   if affordance_id == RIGHT_DELIVERY_ID
                   else f"Unknown delivery affordance: {affordance_id}")
        return {"status": "failed", "success": False, "error": message,
                "downstream_called": False}
    return client.navigation("/place", {"object_id": object_id,
        "affordance_id": LEGACY_DELIVERY_ID, "request_id": request_id,
        "timeout_sec": timeout_sec})
