"""Per-destination release-zone planning contract (offline only)."""

from dataclasses import dataclass
from typing import Tuple

from openflex_thesis_mujoco.release_verification import reachable_release_center


class ReleaseZoneError(ValueError):
    pass


@dataclass(frozen=True)
class ReleaseZoneCandidate:
    destination_id: str
    surface_center_xyz: Tuple[float, float, float]
    surface_half_size_xy: Tuple[float, float]
    base_xy: Tuple[float, float]
    release_center_xyz: Tuple[float, float, float]
    version: str


def plan_release_zone(destination_id: str, *, surface_center_xyz, surface_dimensions_xy,
                      base_xy, version: str) -> ReleaseZoneCandidate:
    if destination_id not in {"sim_delivery_top_left", "sim_delivery_top_right"}:
        raise ReleaseZoneError("release zone must name the left or right registered table")
    if not isinstance(version, str) or not version.strip():
        raise ReleaseZoneError("release zone requires a version")
    center = tuple(float(value) for value in surface_center_xyz)
    dimensions = tuple(float(value) for value in surface_dimensions_xy)
    base = tuple(float(value) for value in base_xy)
    if len(center) != 3 or len(dimensions) != 2 or len(base) != 2 or any(value <= 0 for value in dimensions):
        raise ReleaseZoneError("invalid release surface geometry")
    release = reachable_release_center(base, center, (dimensions[0] / 2, dimensions[1] / 2))
    if release is None:
        raise ReleaseZoneError("no reachable release center inside the destination footprint")
    candidate = ReleaseZoneCandidate(destination_id, center,
                                     (dimensions[0] / 2, dimensions[1] / 2), base,
                                     tuple(release), version)
    return candidate
