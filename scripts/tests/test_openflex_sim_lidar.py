import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"
sys.path.insert(0, str(PACKAGE))

from openflex_thesis_mujoco.camera_spec import MID360_LIDAR  # noqa: E402
from openflex_thesis_mujoco.lidar_sim import (  # noqa: E402
    GEOM_GROUP_COUNT,
    cast_points,
    lidar_directions,
    lidar_geomgroup_mask,
    robot_geom_groups,
    xyzw_from_wxyz,
)
from openflex_thesis_mujoco.model_builder import build_thesis_model  # noqa: E402


def test_lidar_directions_shape_and_unit_length():
    directions = lidar_directions(MID360_LIDAR)
    assert directions.shape == (MID360_LIDAR.channels * MID360_LIDAR.azimuth_bins, 3)
    assert np.allclose(np.linalg.norm(directions, axis=1), 1.0)


def test_lidar_directions_vertical_span_matches_spec():
    directions = lidar_directions(channels=8, azimuth_bins=240)
    elevation = np.degrees(np.arcsin(np.clip(directions[:, 2], -1.0, 1.0)))
    assert math.isclose(elevation.min(), MID360_LIDAR.elevation_min_deg, abs_tol=1e-9)
    assert math.isclose(elevation.max(), MID360_LIDAR.elevation_max_deg, abs_tol=1e-9)


def test_lidar_directions_azimuth_bins_are_distinct():
    directions = lidar_directions(channels=1, azimuth_bins=8)
    azimuth = np.arctan2(directions[:, 1], directions[:, 0])
    assert len({round(float(value), 6) for value in azimuth}) == 8


def test_lidar_directions_reject_non_positive_grid():
    with pytest.raises(ValueError):
        lidar_directions(channels=0, azimuth_bins=10)
    with pytest.raises(ValueError):
        lidar_directions(channels=4, azimuth_bins=-1)


def test_cast_points_drops_misses_and_close_returns():
    directions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
    distances = np.array([0.2, 3.0, -1.0, 5.0])
    points, ranges = cast_points(directions, distances, min_range_m=0.5)
    assert ranges.tolist() == [3.0, 5.0]
    assert np.allclose(points, [[0.0, 3.0, 0.0], [5.0, 0.0, 0.0]])


def test_cast_points_matches_supplied_hit_mask():
    directions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    distances = np.array([2.0, 2.0])
    points, ranges = cast_points(directions, distances, min_range_m=1.0, hit_mask=[True, False])
    assert ranges.tolist() == [2.0]
    assert np.allclose(points, [[2.0, 0.0, 0.0]])


def test_cast_points_applies_reproducible_range_noise():
    directions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    distances = np.array([4.0, 4.0, 4.0])
    _, first = cast_points(directions, distances, min_range_m=0.5, noise_std_m=0.05, rng=np.random.default_rng(7))
    _, second = cast_points(directions, distances, min_range_m=0.5, noise_std_m=0.05, rng=np.random.default_rng(7))
    assert np.allclose(first, second)
    assert not np.allclose(first, distances)


def test_robot_geom_groups_include_descendants_only():
    # bodies: world(0); robot(1) under world; arm(2)/arm2(4) under robot; shelf(3) under world
    geom_group = [1, 1, 0, 0]
    geom_bodyid = [1, 2, 3, 4]
    body_parentid = [0, 0, 1, 0, 1]  # 1->0, 2->1, 3->0, 4->1
    assert robot_geom_groups(geom_group, geom_bodyid, body_parentid, 1) == {0, 1}
    assert robot_geom_groups(geom_group, geom_bodyid, body_parentid, 3) == {0}


def test_lidar_geomgroup_mask_disables_robot_groups():
    mask = lidar_geomgroup_mask({1}, {0})
    assert mask.shape == (GEOM_GROUP_COUNT,)
    assert mask.tolist() == [1, 0, 1, 1, 1, 1]


def test_lidar_geomgroup_mask_rejects_shared_group():
    with pytest.raises(ValueError):
        lidar_geomgroup_mask({0, 1}, {0, 2})


def test_xyzw_from_wxyz_normalises_and_reorders():
    assert np.allclose(xyzw_from_wxyz([2.0, 0.0, 0.0, 0.0]), (0.0, 0.0, 0.0, 1.0))


def thesis_model(tmp_path):
    import mujoco

    upstream = ROOT / "openflex_ws/src/openflex_mujoco/openflex_mujoco.xml"
    output = build_thesis_model(str(upstream), str(tmp_path / "scene.xml"))
    return mujoco.MjModel.from_xml_path(str(output))


def model_lidar_mask(mujoco, model):
    robot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "openflex_robot_yaw_root")
    robot_groups = robot_geom_groups(
        model.geom_group, model.geom_bodyid, model.body_parentid, robot
    )
    environment_groups = set(int(model.geom_group[g]) for g in range(model.ngeom)) - robot_groups
    return lidar_geomgroup_mask(robot_groups, environment_groups), robot_groups, environment_groups


def test_generated_scene_splits_robot_and_environment_geom_groups(tmp_path):
    import mujoco

    model = thesis_model(tmp_path)
    _, robot_groups, environment_groups = model_lidar_mask(mujoco, model)
    assert robot_groups == {1}
    assert environment_groups == {0}


def test_simulated_lidar_sees_scene_and_not_the_robot(tmp_path):
    import mujoco

    model = thesis_model(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mask, robot_groups, _ = model_lidar_mask(mujoco, model)

    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, MID360_LIDAR.parent_body)
    point = np.asarray(data.xpos[sensor], dtype=np.float64)
    rotation = np.asarray(data.xmat[sensor], dtype=np.float64).reshape(3, 3)
    directions = lidar_directions(MID360_LIDAR)
    world_vectors = directions @ rotation.T

    rays = directions.shape[0]
    geomid = np.zeros(rays, dtype=np.int32)
    distances = np.zeros(rays, dtype=np.float64)
    mujoco.mj_multiRay(model, data, point, world_vectors.reshape(-1), mask, 1, -1,
                       geomid, distances, rays, float(MID360_LIDAR.max_range_m))

    robot_geoms = {g for g in range(model.ngeom) if int(model.geom_group[g]) in robot_groups}
    hits = distances >= 0.0
    assert hits.any()
    assert not (set(int(g) for g in geomid[hits]) & robot_geoms)

    points, ranges = cast_points(directions, distances, min_range_m=MID360_LIDAR.min_range_m)
    assert points.shape[0] == ranges.shape[0] > 0
    assert np.allclose(np.linalg.norm(points, axis=1), ranges)
    assert ranges.min() >= MID360_LIDAR.min_range_m


def test_robot_bodyexclude_alone_would_self_occlude(tmp_path):
    """Document why the group mask is required: bodyexclude keeps descendant geoms."""

    import mujoco

    model = thesis_model(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, MID360_LIDAR.parent_body)
    robot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "openflex_robot_yaw_root")
    point = np.asarray(data.xpos[sensor], dtype=np.float64)
    rotation = np.asarray(data.xmat[sensor], dtype=np.float64).reshape(3, 3)
    directions = lidar_directions(MID360_LIDAR)
    world_vectors = directions @ rotation.T

    rays = directions.shape[0]
    geomid = np.zeros(rays, dtype=np.int32)
    distances = np.zeros(rays, dtype=np.float64)
    mujoco.mj_multiRay(model, data, point, world_vectors.reshape(-1), np.ones(6, dtype=np.uint8),
                       1, robot, geomid, distances, rays, float(MID360_LIDAR.max_range_m))
    hits = distances >= 0.0
    robot_geoms = {g for g in range(model.ngeom) if int(model.geom_group[g]) == 1}
    assert hits.any()
    assert set(int(g) for g in geomid[hits]) <= robot_geoms
    assert distances[hits].max() < MID360_LIDAR.min_range_m
