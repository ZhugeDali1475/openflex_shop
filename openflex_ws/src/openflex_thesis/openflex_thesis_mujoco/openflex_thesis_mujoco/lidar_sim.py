"""ROS-free geometry for the simulated MID-360 ray-cast sensor.

The simulation replaces the real, non-repetitive Livox MID-360 with a fixed
elevation/azimuth ray grid whose vertical span matches the unit's field of view
(-7 deg to +52 deg).  Nothing here imports ROS or MuJoCo, so the ray geometry
and the self-occlusion mask can be unit tested offline.

Self-occlusion note: ``mujoco.mj_multiRay``'s ``bodyexclude`` argument only
drops the geoms attached to the excluded body itself, not the geoms of its
descendants (verified against the generated thesis model).  Excluding the robot
root body therefore still lets every ray hit the robot mesh next to the sensor.
The generated scene instead puts the whole robot body tree on a dedicated geom
group and every environment geom on another, so the lidar casts with a group
mask that disables only the robot groups.
"""

from __future__ import annotations

import math

import numpy as np

from .camera_spec import MID360_LIDAR, Mid360LidarSpec

#: Geom group used by the robot body tree in the generated thesis model.
ROBOT_GEOM_GROUP = 1
#: Geom group used by the environment (floor, shelf, object) in the generated model.
ENVIRONMENT_GEOM_GROUP = 0
#: ``mj_multiRay`` accepts a mask of at most six geom groups (bits 0..5).
GEOM_GROUP_COUNT = 6


def lidar_directions(spec: Mid360LidarSpec = MID360_LIDAR, channels=None, azimuth_bins=None) -> np.ndarray:
    """Unit ray directions (sensor frame) ordered ring-major, shape ``(n, 3)``."""

    channels = spec.channels if channels is None else int(channels)
    azimuth_bins = spec.azimuth_bins if azimuth_bins is None else int(azimuth_bins)
    if channels < 1 or azimuth_bins < 1:
        raise ValueError("channels and azimuth_bins must be positive")
    low = math.radians(float(spec.elevation_min_deg))
    high = math.radians(float(spec.elevation_max_deg))
    span = max(1, channels - 1)
    elevation = (low + (high - low) * np.arange(channels)[:, None] / span)
    azimuth = 2.0 * math.pi * np.arange(azimuth_bins)[None, :] / azimuth_bins
    horizontal = np.cos(elevation)
    directions = np.stack(
        [
            horizontal * np.cos(azimuth),
            horizontal * np.sin(azimuth),
            np.sin(elevation) * np.ones_like(azimuth),
        ],
        axis=-1,
    )
    return directions.reshape(-1, 3)


def cast_points(directions, distances, *, min_range_m, hit_mask=None, noise_std_m=0.0, rng=None):
    """Convert per-ray distances into sensor-frame points ``(points, ranges)``.

    ``distances`` uses the MuJoCo convention where a negative value means the ray
    hit nothing.  Points below ``min_range_m`` (self-returns, near clutter) are
    dropped.  ``noise_std_m`` optionally adds Gaussian range noise for a
    reproducible ``rng``.
    """

    directions = np.asarray(directions, dtype=np.float64)
    distances = np.asarray(distances, dtype=np.float64)
    if directions.shape[0] != distances.shape[0]:
        raise ValueError("directions and distances must have the same length")
    if hit_mask is None:
        hit_mask = distances >= 0.0
    hit_mask = np.asarray(hit_mask, dtype=bool)
    ranges = distances[hit_mask]
    kept = directions[hit_mask]
    if noise_std_m > 0.0 and ranges.size:
        rng = np.random.default_rng() if rng is None else rng
        ranges = ranges + rng.normal(0.0, noise_std_m, size=ranges.shape)
    keep = np.isfinite(ranges) & (ranges >= float(min_range_m))
    ranges = ranges[keep]
    kept = kept[keep]
    return kept * ranges[:, None], ranges


def robot_geom_groups(geom_group, geom_bodyid, body_parentid, robot_bodyid) -> set:
    """Geom groups used by the body subtree rooted at ``robot_bodyid``."""

    groups = set()
    for geom in range(len(geom_group)):
        body = int(geom_bodyid[geom])
        cursor = body
        while cursor != 0 and cursor != int(robot_bodyid):
            cursor = int(body_parentid[cursor])
        if cursor == int(robot_bodyid):
            groups.add(int(geom_group[geom]))
    return groups


def lidar_geomgroup_mask(robot_groups, environment_groups, group_count: int = GEOM_GROUP_COUNT) -> np.ndarray:
    """Return a ``mj_multiRay`` geom-group mask that drops the robot and keeps the world.

    Raises ``ValueError`` when a group is shared between the robot and the
    environment: masking it would also hide part of the scene, so the scene
    groups must be split before the lidar can cast safely.
    """

    overlap = set(int(group) for group in robot_groups) & set(int(group) for group in environment_groups)
    if overlap:
        raise ValueError(
            "robot and environment share geom group(s) %s; the simulated lidar "
            "cannot exclude the robot without hiding part of the scene" % sorted(overlap)
        )
    mask = np.ones(group_count, dtype=np.uint8)
    for group in robot_groups:
        mask[int(group)] = 0
    return mask


def xyzw_from_wxyz(quaternion_wxyz):
    """Normalise a ``(w, x, y, z)`` quaternion and return it as ``(x, y, z, w)``."""

    w, x, y, z = (float(value) for value in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (x / norm, y / norm, z / norm, w / norm)
