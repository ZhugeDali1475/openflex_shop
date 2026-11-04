import math

from openflex_navigation_gui.goal_safety import (
    CostmapSnapshot,
    GoalSafetyGeometry,
    GoalSafetyMap,
    costmap_pose_problem,
)
from openflex_navigation_gui.map_editor_core import FREE, OCCUPIED, GridMap, MapMetadata


def _safety_map() -> GoalSafetyMap:
    grid = GridMap(41, 41, bytearray([FREE] * (41 * 41)))
    grid.pixels[20 * grid.width + 20] = OCCUPIED
    metadata = MapMetadata(resolution=0.05, origin=(-1.025, -1.025, 0.0))
    geometry = GoalSafetyGeometry(
        half_length_m=0.46,
        half_width_m=0.45,
        clearance_m=0.05,
    )
    return GoalSafetyMap(grid, metadata, geometry)


def test_goal_requires_five_centimetres_beyond_front_footprint():
    safety = _safety_map()

    # The occupied cell spans +/-2.5 cm. With a 46 cm front overhang, the
    # exact forbidden threshold is 46 + 2.5 + 5 = 53.5 cm from cell centre.
    assert not safety.is_safe(0.5349, 0.0, math.pi)
    assert safety.is_safe(0.5351, 0.0, math.pi)


def test_goal_mask_rotates_with_base_link_heading():
    safety = _safety_map()

    # A square-ish rectangular footprint reaches much farther along map X at
    # 45 degrees, so the same base_link cell becomes forbidden after rotation.
    assert safety.is_safe(0.575, 0.0, math.pi)
    assert not safety.is_safe(0.575, 0.0, math.pi / 4.0)


def test_goal_mask_contains_visible_outer_boundary():
    safety = _safety_map()
    forbidden, boundary = safety.masks(0.0)

    assert any(forbidden)
    assert any(boundary)
    assert all(not edge or forbidden[index] for index, edge in enumerate(boundary))


def test_costmap_pose_rejects_unsafe_robot_centre():
    free = CostmapSnapshot(0.1, -1.0, -1.0, 20, 20, [0] * 400, 0.0)
    assert not costmap_pose_problem(free, 0.0, 0.0)

    for value, expected in ((253, "inscribed"), (254, "lethal"), (255, "unknown")):
        data = list(free.data)
        data[10 * free.width + 10] = value
        snapshot = CostmapSnapshot(
            free.resolution,
            free.origin_x,
            free.origin_y,
            free.width,
            free.height,
            data,
            free.received_at,
        )
        assert expected in costmap_pose_problem(snapshot, 0.0, 0.0)


def test_costmap_pose_does_not_apply_footprint_to_inflated_map_twice():
    data = [0] * 400
    # This inscribed cell is inside the physical footprint but not at the
    # robot centre. The costmap already accounts for the footprint inflation.
    data[10 * 20 + 14] = 253
    snapshot = CostmapSnapshot(0.1, -1.0, -1.0, 20, 20, data, 0.0)

    assert not costmap_pose_problem(snapshot, 0.0, 0.0)
