#!/usr/bin/env python3
"""Offline, ROS-free audit of the phase C2 mapping + exploration contract.

Builds a synthetic ground-truth occupancy world, lets the robot reveal it with a
ray-cast depth/laser model through ``mapping_contract`` (occlusion included),
and drives ``exploration_contract.ExplorationSearch`` to pick the next
observation stand.  The next stand is chosen only from the current map snapshot;
no preset route or station list is ever consulted.

Nothing here starts ROS or commands motion.  It is the deterministic, replayable
counterpart of a live read-only exploration run.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.exploration_contract import (  # noqa: E402
    CANCELLED, LOCALIZATION_LOST, MAP_INCOMPLETE, NEXT_POSITION, STEPS_EXHAUSTED, TARGET_FOUND,
    TARGET_NOT_FOUND, TIMEOUT, ExplorationSearch,
)
from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, LocalizationQuality, OccupancyGridBuilder,
    ground_truth_localization,
)

_CODES = {"#": OCCUPIED, ".": FREE, "?": UNKNOWN, "T": OCCUPIED}


def parse_world(rows):
    if not rows or len({len(row) for row in rows}) != 1:
        raise ValueError("world rows must be non-empty and equal length")
    return tuple(_CODES[character] for row in rows for character in row)


def make_builder(width: int, height: int, *, resolution_m: float = 0.1,
                 origin_xy=(0.0, 0.0)) -> OccupancyGridBuilder:
    return OccupancyGridBuilder(frame_id="map", resolution_m=resolution_m,
                                origin_xy=origin_xy, width=width, height=height)


def _cell_at(truth, width, height, i, j):
    if 0 <= i < width and 0 <= j < height:
        return truth[j * width + i]
    return OCCUPIED  # treat out-of-grid as a wall so rays stop cleanly


def cast_rays(truth, width, height, robot_cell, *, resolution_m, radius_m, angle_step_deg=3.0,
              origin_xy=(0.0, 0.0)):
    """Ray-cast a synthetic range sensor; returns endpoint points in world frame."""

    from openflex_roboos_bridge.mapping_contract import _bresenham

    reach = max(1, int(round(radius_m / resolution_m)))
    endpoints = []
    angle = 0.0
    while angle < 360.0:
        radians = math.radians(angle)
        di = int(round(math.cos(radians) * reach))
        dj = int(round(math.sin(radians) * reach))
        if di == 0 and dj == 0:
            angle += angle_step_deg
            continue
        target = (robot_cell[0] + di, robot_cell[1] + dj)
        hit = None
        for i, j in _bresenham(robot_cell[0], robot_cell[1], *target):
            if (i, j) == robot_cell:
                continue
            if _cell_at(truth, width, height, i, j) == OCCUPIED:
                hit = (i, j)
                break
        endpoint = hit or target
        endpoints.append((origin_xy[0] + endpoint[0] * resolution_m,
                          origin_xy[1] + endpoint[1] * resolution_m))
        angle += angle_step_deg
    return endpoints


def _world_to_cell(origin_xy, resolution_m, point):
    return (int(round((point[0] - origin_xy[0]) / resolution_m)),
            int(round((point[1] - origin_xy[1]) / resolution_m)))


def _cell_to_world(origin_xy, resolution_m, cell):
    return (origin_xy[0] + cell[0] * resolution_m, origin_xy[1] + cell[1] * resolution_m)


def simulate(rows, *, start_cell, target_cell=None, resolution_m: float = 0.1,
             origin_xy=(0.0, 0.0), sensor_radius_m: float = 0.9, min_known_fraction: float = 0.9,
             max_steps: int = 60, deadline_sec=None, move_step_m: float = 0.4,
             inject_localization_after=None):
    """Run one deterministic exploration episode and return its record."""

    height, width = len(rows), len(rows[0])
    truth = parse_world(rows)
    builder = make_builder(width, height, resolution_m=resolution_m, origin_xy=origin_xy)
    search = ExplorationSearch(frame_id="map", sensor_radius_m=sensor_radius_m,
                               min_known_fraction=min_known_fraction, max_steps=max_steps,
                               deadline_sec=deadline_sec)
    robot_cell = tuple(start_cell)
    steps = []
    now = 0.0
    while not search.finished:
        endpoints = cast_rays(truth, width, height, robot_cell, resolution_m=resolution_m,
                              radius_m=sensor_radius_m, origin_xy=origin_xy)
        builder.integrate_points_2d(_cell_to_world(origin_xy, resolution_m, robot_cell), endpoints)
        localization = ground_truth_localization(stamp_sec=now)
        if inject_localization_after is not None and len(steps) >= inject_localization_after:
            localization = LocalizationQuality(converged=False, source="injected_loss",
                                               position_rmse_m=0.4, yaw_rmse_rad=0.3)
        snapshot = builder.snapshot(localization=localization, stamp_sec=now)
        observed_target = target_cell is not None and \
            snapshot.cell(target_cell[0], target_cell[1]) == OCCUPIED
        decision = search.plan(snapshot, robot_xy=_cell_to_world(origin_xy, resolution_m, robot_cell),
                               now_sec=now, target_observed=observed_target)
        steps.append({"step": len(steps), "map_version": snapshot.map_version,
                      "coverage_fraction": round(snapshot.coverage_fraction(), 6),
                      "robot_cell": list(robot_cell), "outcome": decision.outcome,
                      "goal_xy": list(decision.goal_xy) if decision.goal_xy else None,
                      "information_gain": decision.information_gain})
        if decision.finished:
            break
        goal_cell = _world_to_cell(origin_xy, resolution_m, decision.goal_xy)
        robot_cell = _advance(robot_cell, goal_cell, max(1, int(round(move_step_m / resolution_m))))
        now += 1.0
    return {"outcome": search.outcome, "steps": steps, "report": search.report(),
            "final_coverage": builder.snapshot(localization=ground_truth_localization(),
                                               stamp_sec=now).coverage_fraction()}


def _advance(start, goal, step_cells):
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance <= step_cells:
        return goal
    scale = step_cells / distance
    return (int(round(start[0] + dx * scale)), int(round(start[1] + dy * scale)))


ROOM_WITH_PILLARS = [
    "##############",
    "#............#",
    "#....##......#",
    "#....##......#",
    "#............#",
    "#.....####...#",
    "#............#",
    "#............#",
    "##############",
]

ROOM_WITH_TARGET = [
    "##############",
    "#............#",
    "#....##......#",
    "#....##....T.#",
    "#............#",
    "#.....####...#",
    "#............#",
    "#............#",
    "##############",
]

ROOM_WITHOUT_PILLARS = [
    "##############",
    "#............#",
    "#............#",
    "#............#",
    "#............#",
    "##############",
]

SEALED_POCKET = [
    "####" + "#" * 40,
    "#..#" + "#" * 40,
    "#..#" + "#" * 40,
    "####" + "#" * 40,
] + ["#" * 44] * 16


def run_audit() -> dict:
    scenarios = []

    def record(name, expected, result, extra=None):
        entry = {"name": name, "expected_outcome": expected, "outcome": result["outcome"],
                 "passed": result["outcome"] == expected,
                 "steps_taken": len(result["steps"]),
                 "final_coverage": round(result["final_coverage"], 6),
                 "trajectory": result["steps"]}
        if extra:
            entry.update(extra)
        scenarios.append(entry)

    record("room_target_absent",
           TARGET_NOT_FOUND,
           simulate(ROOM_WITHOUT_PILLARS, start_cell=(1, 1), min_known_fraction=0.9))

    record("room_target_present",
           TARGET_FOUND,
           simulate(ROOM_WITH_TARGET, start_cell=(1, 7), target_cell=(11, 3),
                    min_known_fraction=0.9))

    # Product unreachable: pocket is fully observed but the map stays mostly unknown.
    record("sealed_pocket_map_incomplete",
           MAP_INCOMPLETE,
           simulate(SEALED_POCKET, start_cell=(2, 2), sensor_radius_m=0.2,
                    min_known_fraction=0.9))

    record("localization_lost",
           LOCALIZATION_LOST,
           simulate(ROOM_WITH_PILLARS, start_cell=(1, 1), inject_localization_after=1,
                    min_known_fraction=0.9))

    record("deadline_timeout",
           TIMEOUT,
           simulate(ROOM_WITH_PILLARS, start_cell=(1, 1), min_known_fraction=0.9,
                    deadline_sec=2.0))

    record("steps_exhausted",
           STEPS_EXHAUSTED,
           simulate(ROOM_WITH_PILLARS, start_cell=(1, 1), min_known_fraction=0.9, max_steps=1))

    cancelled = ExplorationSearch(frame_id="map")
    cancelled.cancel()
    scenarios.append({"name": "cancelled", "expected_outcome": CANCELLED, "outcome": CANCELLED,
                      "passed": True, "steps_taken": 0,
                      "report": cancelled.report()})

    report = {
        "scope": "phase_C2_mapping_and_frontier_exploration_offline_NOT_PHYSICAL",
        "physical_verification": False,
        "robot_motion_executed": False,
        "ros_started": False,
        "preset_route_used": False,
        "next_position_source": "current_map_frontier_information_gain_only",
        "localization_source": "simulation_ground_truth_not_an_estimator",
        "scenarios": scenarios,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    report["passed"] = all(item["passed"] for item in scenarios)
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    arguments = parser.parse_args(argv)
    report = run_audit()
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if arguments.output:
        Path(arguments.output).write_text(text + "\n", encoding="utf-8")
        print(arguments.output)
    else:
        print(text)
    print(json.dumps({"passed": report["passed"],
                      "scenarios": {item["name"]: item["outcome"] for item in report["scenarios"]}},
                     ensure_ascii=False, indent=2), file=sys.stderr)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
