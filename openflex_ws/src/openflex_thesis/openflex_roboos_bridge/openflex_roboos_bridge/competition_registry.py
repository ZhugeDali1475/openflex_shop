"""Registered topology for the two-shelf/two-delivery-table competition half.

The registry is intentionally independent from ROS and execution.  It turns
the reviewed static single-red scene into stable logical IDs and refuses to
pretend that an unvalidated route is executable.
"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Optional, Tuple
import xml.etree.ElementTree as ET


TARGET_OBJECT_ID = "thesis_item_01"
SHELF_IDS = ("shelf_group_1", "shelf_group_2")
AFFORDANCE_IDS = ("sim_delivery_top_left", "sim_delivery_top_right")


def _finite(values, count, field):
    result = tuple(float(value) for value in values)
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field} must contain {count} finite values")
    return result


def _hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class ObjectRegistration:
    object_id: str
    color: str
    body: str
    geom: str
    free_joint: str
    shelf_id: str
    face: str
    level: int
    initial_pose_xyzyaw: Tuple[float, float, float, float]


@dataclass(frozen=True)
class StationRegistration:
    station_id: str
    role: str
    object_id: str
    navigation_pose_xyyaw: Optional[Tuple[float, float, float]]
    validated: bool
    validation_basis: str


@dataclass(frozen=True)
class AffordanceRegistration:
    affordance_id: str
    body: str
    surface_geom: str
    surface_center_xyz: Tuple[float, float, float]
    surface_dimensions_xy: Tuple[float, float]
    navigation_pose_xyyaw: Optional[Tuple[float, float, float]]
    validated: bool
    validation_basis: str


@dataclass(frozen=True)
class CompetitionHalfRegistry:
    scene_path: str
    scene_sha256: str
    shelves: Tuple[str, ...]
    objects: Mapping[str, ObjectRegistration]
    stations: Mapping[str, StationRegistration]
    affordances: Mapping[str, AffordanceRegistration]
    aliases: Mapping[str, str]

    def validate(self) -> None:
        if tuple(self.shelves) != SHELF_IDS:
            raise ValueError(f"competition half must contain exactly {SHELF_IDS}")
        if tuple(sorted(self.objects)) != (TARGET_OBJECT_ID,):
            raise ValueError("competition half must contain exactly thesis_item_01")
        target = self.objects[TARGET_OBJECT_ID]
        if target.color.casefold() != "red":
            raise ValueError("the registered competition target must be red")
        if target.shelf_id not in SHELF_IDS:
            raise ValueError("target shelf is outside the competition half")
        if tuple(sorted(self.affordances)) != tuple(sorted(AFFORDANCE_IDS)):
            raise ValueError(f"competition half must contain exactly {AFFORDANCE_IDS}")
        if "pickup_zone" not in self.stations:
            raise ValueError("pickup_zone station is required")
        for affordance_id, affordance in self.affordances.items():
            if not affordance.surface_geom or not affordance.body:
                raise ValueError(f"affordance {affordance_id} has incomplete geometry mapping")
            if affordance.navigation_pose_xyyaw is None and affordance.validated:
                raise ValueError(f"validated affordance {affordance_id} needs a navigation pose")

    def resolve_affordance(self, affordance_id: str) -> AffordanceRegistration:
        canonical = self.aliases.get(affordance_id, affordance_id)
        if canonical not in self.affordances:
            raise KeyError(f"unknown delivery affordance: {affordance_id}")
        affordance = self.affordances[canonical]
        if not affordance.validated:
            raise RuntimeError(f"delivery affordance is registered but not route-validated: {canonical}")
        return affordance

    def snapshot(self) -> dict:
        return {
            "scene_path": self.scene_path,
            "scene_sha256": self.scene_sha256,
            "shelves": list(self.shelves),
            "objects": {key: value.__dict__ for key, value in self.objects.items()},
            "stations": {key: value.__dict__ for key, value in self.stations.items()},
            "affordances": {key: value.__dict__ for key, value in self.affordances.items()},
            "aliases": dict(self.aliases),
        }


def _world_box(world, body_name, geom_name):
    body = world.find(f"body[@name='{body_name}']")
    geom = body.find(f"geom[@name='{geom_name}']") if body is not None else None
    if body is None or geom is None or geom.get("type") != "box":
        raise ValueError(f"missing axis-aligned table geometry: {body_name}/{geom_name}")
    if body.get("quat") or body.get("euler") or geom.get("quat") or geom.get("euler"):
        raise ValueError("rotated delivery geometry requires explicit calibration")
    body_pos = _finite(body.get("pos", "0 0 0").split(), 3, f"{body_name}.pos")
    geom_pos = _finite(geom.get("pos", "0 0 0").split(), 3, f"{geom_name}.pos")
    half = _finite(geom.get("size", "").split(), 3, f"{geom_name}.size")
    if any(value <= 0 for value in half):
        raise ValueError(f"{geom_name} must have positive half-size")
    center = tuple(body_pos[index] + geom_pos[index] for index in range(3))
    return center, half


def load_competition_half_registry(scene_path: str, manifest_path: Optional[str] = None) -> CompetitionHalfRegistry:
    scene = Path(scene_path).expanduser().resolve(strict=True)
    root = ET.parse(scene).getroot()
    world = root.find("worldbody")
    if world is None:
        raise ValueError("scene has no worldbody")
    manifest_file = Path(manifest_path).expanduser().resolve(strict=True) if manifest_path else scene.parent / "manifest.json"
    manifest = json.loads(manifest_file.read_text()) if manifest_file.exists() else {}
    products = manifest.get("products", [])
    if len(products) != 1:
        raise ValueError("registry source must describe exactly one manifest product")
    product = products[0]
    object_registration = ObjectRegistration(
        object_id=str(product.get("object_id", "")), color=str(product.get("color", "")),
        body=str(product.get("body", "")), geom=str(product.get("geom", "")),
        free_joint=str(product.get("free_joint", "")), shelf_id=str(product.get("shelf_id", "")),
        face=str(product.get("face", "")), level=int(product.get("level", -1)),
        initial_pose_xyzyaw=_finite((*product.get("position_world_m", ()), float(product.get("yaw_rad", 0.0))), 4,
                                    "product initial pose"),
    )
    object_body = world.find(f"body[@name='{object_registration.body}']")
    object_geom = (object_body.find(f"geom[@name='{object_registration.geom}']")
                   if object_body is not None else None)
    object_joint = (object_body.find(f"freejoint[@name='{object_registration.free_joint}']")
                    if object_body is not None else None)
    if object_body is None or object_geom is None or object_joint is None:
        raise ValueError("manifest object_id must map to an existing MJCF body/geom/freejoint")
    if object_geom.get("type") != "box":
        raise ValueError("registered red test object must use its declared box geometry")
    body_position = _finite(object_body.get("pos", "0 0 0").split(), 3,
                            f"{object_registration.body}.pos")
    geom_half_size = _finite(object_geom.get("size", "").split(), 3,
                             f"{object_registration.geom}.size")
    manifest_product = products[0]
    manifest_half_size = _finite(manifest_product.get("half_size_m", ()), 3,
                                 "product half_size_m")
    if any(abs(actual - expected) > 1e-8 for actual, expected in
           zip(body_position, object_registration.initial_pose_xyzyaw[:3])):
        raise ValueError("manifest initial object position differs from MJCF body position")
    if any(abs(actual - expected) > 1e-8 for actual, expected in
           zip(geom_half_size, manifest_half_size)):
        raise ValueError("manifest object dimensions differ from MJCF geometry")
    rgba = _finite(object_geom.get("rgba", "").split(), 4, "object rgba")
    if not (rgba[0] > rgba[1] * 2 and rgba[0] > rgba[2] * 2):
        raise ValueError("manifest red object does not map to a visibly red MJCF geom")
    shelf_names = tuple(sorted(name for name in (body.get("name", "") for body in world.findall("body"))
                              if name.startswith("shelf_group_")))
    if shelf_names != SHELF_IDS:
        raise ValueError(f"static registry scene must expose shelves {SHELF_IDS}, got {shelf_names}")
    table_map = {
        "sim_delivery_top_left": ("counter_top_left", "counter_top_left_top"),
        "sim_delivery_top_right": ("counter_top_right", "counter_top_right_top"),
    }
    affordances = {}
    for affordance_id, (body_name, geom_name) in table_map.items():
        center, half = _world_box(world, body_name, geom_name)
        top = center[2] + half[2]
        affordances[affordance_id] = AffordanceRegistration(
            affordance_id, body_name, geom_name, (center[0], center[1], top),
            (2 * half[0], 2 * half[1]), None, False,
            "geometry registered; navigation/release path not yet validated",
        )
    registry = CompetitionHalfRegistry(
        scene_path=str(scene), scene_sha256=_hash(scene), shelves=shelf_names,
        objects={object_registration.object_id: object_registration},
        stations={"pickup_zone": StationRegistration(
            "pickup_zone", "red_object_observation", object_registration.object_id,
            None, False, "station geometry requires runtime path validation",
        )},
        affordances=affordances,
        aliases={"delivery_zone": "sim_delivery_top_left"},
    )
    registry.validate()
    return registry
