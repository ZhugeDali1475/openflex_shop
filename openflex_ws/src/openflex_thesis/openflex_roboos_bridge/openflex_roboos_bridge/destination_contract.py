"""Canonical destination IDs shared by planning and the simulation bridge."""

LEFT_DESTINATION_ID = "sim_delivery_top_left"
RIGHT_DESTINATION_ID = "sim_delivery_top_right"
LEGACY_DESTINATION_ID = "delivery_zone"
DESTINATION_IDS = (LEFT_DESTINATION_ID, RIGHT_DESTINATION_ID)


def canonical_destination_id(destination_id: str) -> str:
    if destination_id == LEGACY_DESTINATION_ID:
        return LEFT_DESTINATION_ID
    if destination_id in DESTINATION_IDS:
        return destination_id
    raise ValueError(f"Unknown delivery destination: {destination_id}")


def destination_geometry_parameters(destination_id: str) -> tuple[str, str]:
    canonical = canonical_destination_id(destination_id)
    if canonical == LEFT_DESTINATION_ID:
        return "thesis_delivery_table", "thesis_delivery_top"
    return "counter_top_right", "counter_top_right_top"
