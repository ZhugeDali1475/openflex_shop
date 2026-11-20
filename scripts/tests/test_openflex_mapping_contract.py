import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.mapping_contract import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, LocalizationQuality, MapSnapshot, MappingError,
    OccupancyGridBuilder, ground_truth_localization,
)
from openflex_roboos_bridge.target_search_contract import (  # noqa: E402
    CameraIntrinsics, CameraPose,
)

INTRINSICS = CameraIntrinsics.from_k(8, 6, [10.0, 0.0, 4.0, 0.0, 10.0, 3.0, 0.0, 0.0, 1.0])
IDENTITY_CAMERA = CameraPose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def grid(rows, *, resolution=0.1, origin=(0.0, 0.0)):
    return OccupancyGridBuilder(frame_id="map", resolution_m=resolution, origin_xy=origin,
                                width=len(rows[0]), height=len(rows))


def test_localization_quality_validation_and_ground_truth():
    truth = ground_truth_localization(stamp_sec=3.0)
    assert truth.converged and truth.position_rmse_m == 0.0
    assert truth.is_usable(max_position_rmse_m=0.05, max_yaw_rmse_rad=0.05)
    lost = LocalizationQuality(converged=False, source="amcl", position_rmse_m=0.2,
                               yaw_rmse_rad=0.3)
    assert not lost.is_usable(max_position_rmse_m=0.05, max_yaw_rmse_rad=0.05)
    with pytest.raises(MappingError):
        LocalizationQuality(converged=True, source="", position_rmse_m=0.0)
    with pytest.raises(MappingError):
        LocalizationQuality(converged=True, source="amcl", position_rmse_m=-1.0)


def test_raycast_marks_free_along_ray_and_occupied_at_hit():
    builder = grid(["?" * 20] * 20)
    assert builder.integrate_ray((0.0, 0.0), (0.4, 0.0)) is True
    snapshot = builder.snapshot(localization=ground_truth_localization(), stamp_sec=0.0)
    assert builder.version == 1
    assert snapshot.cell(4, 0) == OCCUPIED
    assert snapshot.cell(2, 0) == FREE
    assert snapshot.cell(0, 0) == FREE
    assert snapshot.cell(9, 9) == UNKNOWN
    # a ray entirely outside the grid is ignored
    assert builder.integrate_ray((0.0, 0.0), (5.0, 5.0)) is False


def test_world_grid_roundtrip_cell_state_and_coverage():
    snapshot = grid(["??", "??"]).snapshot(localization=ground_truth_localization(), stamp_sec=0.0)
    assert snapshot.world_to_grid(0.0, 0.0) == (0, 0)
    assert snapshot.grid_to_world(1, 1) == (0.1, 0.1)
    assert snapshot.world_to_grid(9.0, 9.0) is None
    assert snapshot.cell_state(0, 0) == "unknown"
    assert snapshot.cell_state(50, 50) == "unknown"  # out of bounds is unknown
    assert snapshot.coverage_fraction() == 0.0
    builder = grid(["..", ".."])
    builder.integrate_points_2d((0.0, 0.0), [(0.1, 0.1)])
    known = builder.snapshot(localization=ground_truth_localization(), stamp_sec=0.0)
    assert known.cell_state(1, 1) == "occupied"
    assert known.coverage_fraction() == pytest.approx(0.5)


def test_snapshot_rejects_bad_inputs():
    with pytest.raises(MappingError):
        MapSnapshot(frame_id="map", resolution_m=0.0, origin_xy=(0.0, 0.0), width=1, height=1,
                    occupancy=(UNKNOWN,), map_version=0,
                    localization=ground_truth_localization(), stamp_sec=0.0)
    with pytest.raises(MappingError):
        MapSnapshot(frame_id="map", resolution_m=0.1, origin_xy=(0.0, 0.0), width=2, height=2,
                    occupancy=(UNKNOWN,), map_version=0,
                    localization=ground_truth_localization(), stamp_sec=0.0)
    with pytest.raises(MappingError):
        MapSnapshot(frame_id="map", resolution_m=0.1, origin_xy=(0.0, 0.0), width=1, height=1,
                    occupancy=(7,), map_version=0,
                    localization=ground_truth_localization(), stamp_sec=0.0)


def test_depth_frame_backprojects_to_occupied_cells():
    builder = grid(["?" * 20] * 20)
    depth = np.zeros((6, 8), dtype=np.uint16)
    depth[4, 6] = 2000  # (u=6, v=4) -> map (0.4, 0.2, 2.0) for identity camera
    integrated = builder.integrate_depth_frame(depth, INTRINSICS, IDENTITY_CAMERA, stride=1,
                                               min_range_m=0.1, max_range_m=5.0)
    snapshot = builder.snapshot(localization=ground_truth_localization(), stamp_sec=0.0)
    assert integrated == 1
    assert snapshot.cell(4, 2) == OCCUPIED
    assert snapshot.cell(2, 1) == FREE


def test_depth_frame_height_clip_filters_floor_points():
    depth = np.zeros((6, 8), dtype=np.uint16)
    depth[4, 7] = 2000  # identity camera -> map (0.6, 0.2, 2.0)
    depth[3, 7] = 200   # map (0.06, 0.0, 0.2), below z=1.0
    # keep only the high (z=2.0) point
    above = grid(["?" * 20] * 20)
    assert above.integrate_depth_frame(depth, INTRINSICS, IDENTITY_CAMERA, stride=1,
                                       min_height_m=1.0, max_height_m=3.0) == 1
    assert above.snapshot(localization=ground_truth_localization(),
                          stamp_sec=0.0).cell(6, 2) == OCCUPIED
    # keep only the low (z=0.2) point
    below = grid(["?" * 20] * 20)
    assert below.integrate_depth_frame(depth, INTRINSICS, IDENTITY_CAMERA, stride=1,
                                       max_height_m=0.5) == 1
    assert below.snapshot(localization=ground_truth_localization(),
                          stamp_sec=0.0).cell(1, 0) == OCCUPIED
    with pytest.raises(MappingError):
        grid(["?" * 20] * 20).integrate_depth_frame(depth, INTRINSICS, IDENTITY_CAMERA, stride=1,
                                                    min_height_m=2.0, max_height_m=1.0)


def test_depth_frame_rejects_shape_mismatch():
    builder = grid(["?" * 20] * 20)
    with pytest.raises(MappingError):
        builder.integrate_depth_frame(np.zeros((5, 5), dtype=np.uint16), INTRINSICS,
                                      IDENTITY_CAMERA, stride=1)


def test_snapshot_adopt_keeps_version_and_cells():
    builder = grid(["?" * 4] * 4)
    builder.integrate_ray((0.0, 0.0), (0.2, 0.0))
    original = builder.snapshot(localization=ground_truth_localization(), stamp_sec=0.0)
    replay = OccupancyGridBuilder(frame_id="map", resolution_m=0.1, origin_xy=(0.0, 0.0),
                                  width=4, height=4).with_snapshot(original)
    assert replay.version == original.map_version
    assert tuple(replay.snapshot(localization=ground_truth_localization(),
                                 stamp_sec=0.0).occupancy) == original.occupancy
    assert math.isclose(original.resolution_m, 0.1)


def test_snapshot_to_dict_omits_occupancy_by_default():
    snapshot = grid(["?"]).snapshot(localization=ground_truth_localization(), stamp_sec=0.0)
    assert "occupancy" not in snapshot.to_dict()
    assert len(snapshot.to_dict(include_occupancy=True)["occupancy"]) == 1
