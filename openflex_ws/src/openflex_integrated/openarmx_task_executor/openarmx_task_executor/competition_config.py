"""Validated configuration for the three-stage supermarket competition."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


PHASE_PICKING = 1
PHASE_RESTOCKING = 2
PHASE_RETURN_MISPLACED = 3
VALID_PHASES = (PHASE_PICKING, PHASE_RESTOCKING, PHASE_RETURN_MISPLACED)


def _mapping(value: Any, label: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _station_list(value: Any, label: str) -> List[str]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{label} must be a non-empty list")
    result = [str(item).strip() for item in value]
    if any(not item for item in result):
        raise ValueError(f"{label} contains an empty station id")
    return result


@dataclass(frozen=True)
class PhasePolicy:
    phase: int
    name: str
    required_items: int
    timeout_sec: float
    entry_station: str
    judging_station: str
    scan_tour: List[str] = field(default_factory=list)
    use_dream: bool = False
    verify_placement: bool = True


@dataclass(frozen=True)
class SkuRoute:
    sku: str
    source_station: str
    destination_station: str
    object_id: str
    source_slot: str = ""
    destination_slot: str = ""


@dataclass(frozen=True)
class InventoryPolicy:
    """How semantic shelf records are mapped to tested navigation stations."""

    enabled: bool = False
    database_path: str = ""
    default_destination_station: str = ""
    shelf_face_stations: Dict[Tuple[int, int], str] = field(default_factory=dict)
    prefer_inventory: bool = True

    def station_for(self, shelf_id: int, face: int) -> Optional[str]:
        return self.shelf_face_stations.get((int(shelf_id), int(face)))


@dataclass(frozen=True)
class CompetitionConfig:
    control_mode: str
    total_timeout_sec: float
    max_transport_retries: int
    min_order_confidence: float
    min_inspection_confidence: float
    perception_mode_control: bool
    phases: Dict[int, PhasePolicy]
    sku_routes: Dict[str, SkuRoute]
    inventory: InventoryPolicy = field(default_factory=InventoryPolicy)

    @classmethod
    def from_yaml(cls, path: str) -> "CompetitionConfig":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        root = _mapping(raw.get("competition", raw), "competition")
        control_mode = str(root.get("control_mode", "autonomous"))
        if control_mode != "autonomous":
            raise ValueError("competition.control_mode must be 'autonomous'")

        phase_data = _mapping(root.get("phases"), "competition.phases")
        phases: Dict[int, PhasePolicy] = {}
        for phase in VALID_PHASES:
            value = phase_data.get(phase, phase_data.get(str(phase)))
            value = _mapping(value, f"competition.phases.{phase}")
            scan_tour = value.get("scan_tour", [])
            if scan_tour:
                scan_tour = _station_list(
                    scan_tour, f"competition.phases.{phase}.scan_tour"
                )
            phases[phase] = PhasePolicy(
                phase=phase,
                name=str(value.get("name", f"phase_{phase}")),
                required_items=int(value.get("required_items", 2)),
                timeout_sec=float(value.get("timeout_sec", 360.0)),
                entry_station=str(value.get("entry_station", "")).strip(),
                judging_station=str(value.get("judging_station", "")).strip(),
                scan_tour=scan_tour,
                use_dream=bool(value.get("use_dream", False)),
                verify_placement=bool(value.get("verify_placement", True)),
            )
            if phases[phase].required_items <= 0:
                raise ValueError(f"phase {phase} required_items must be positive")
            if phases[phase].timeout_sec <= 0.0:
                raise ValueError(f"phase {phase} timeout_sec must be positive")
            if not phases[phase].entry_station or not phases[phase].judging_station:
                raise ValueError(f"phase {phase} requires entry_station and judging_station")
            if phase in (PHASE_RESTOCKING, PHASE_RETURN_MISPLACED) and not scan_tour:
                raise ValueError(f"phase {phase} requires a scan_tour")

        route_data = _mapping(root.get("sku_routes", {}), "competition.sku_routes")
        sku_routes: Dict[str, SkuRoute] = {}
        for route_key, route_value in route_data.items():
            route_value = _mapping(route_value, f"sku route {route_key}")
            sku = str(route_key).strip()
            route = SkuRoute(
                sku=sku,
                source_station=str(route_value.get("source_station", "")).strip(),
                destination_station=str(route_value.get("destination_station", "")).strip(),
                object_id=str(route_value.get("object_id", sku)).strip(),
                source_slot=str(route_value.get("source_slot", "")).strip(),
                destination_slot=str(route_value.get("destination_slot", "")).strip(),
            )
            if not sku or not route.source_station or not route.destination_station:
                raise ValueError(f"sku route '{route_key}' is incomplete")
            sku_routes[sku] = route

        inventory_value = root.get("inventory", {}) or {}
        if not isinstance(inventory_value, dict):
            raise ValueError("competition.inventory must be a mapping")
        inventory_enabled = bool(inventory_value.get("enabled", False))
        inventory_path = str(inventory_value.get("database_path", "")).strip()
        if inventory_path and not Path(inventory_path).expanduser().is_absolute():
            inventory_path = str((Path(path).resolve().parent / inventory_path).resolve())
        default_destination = str(
            inventory_value.get("default_destination_station", "")
        ).strip()
        raw_face_stations = inventory_value.get("shelf_face_stations", {}) or {}
        if not isinstance(raw_face_stations, dict):
            raise ValueError("competition.inventory.shelf_face_stations must be a mapping")
        face_stations: Dict[Tuple[int, int], str] = {}
        for key, station in raw_face_stations.items():
            text_key = str(key).replace("-", ":")
            parts = text_key.split(":")
            if len(parts) != 2:
                raise ValueError(
                    f"inventory shelf face key {key!r} must be 'shelf_id:face'"
                )
            shelf_id, face = int(parts[0]), int(parts[1])
            if shelf_id < 0 or face not in (0, 1):
                raise ValueError(f"invalid inventory shelf face key {key!r}")
            station_id = str(station).strip()
            if not station_id:
                raise ValueError(f"inventory shelf face {key!r} has empty station")
            face_stations[(shelf_id, face)] = station_id
        if inventory_enabled:
            if not inventory_path:
                raise ValueError("enabled competition.inventory requires database_path")
            if not face_stations:
                raise ValueError(
                    "enabled competition.inventory requires shelf_face_stations"
                )
            if not default_destination:
                raise ValueError(
                    "enabled competition.inventory requires default_destination_station"
                )

        return cls(
            control_mode=control_mode,
            total_timeout_sec=float(root.get("total_timeout_sec", 1200.0)),
            max_transport_retries=int(root.get("max_transport_retries", 1)),
            min_order_confidence=float(root.get("min_order_confidence", 0.80)),
            min_inspection_confidence=float(
                root.get("min_inspection_confidence", 0.65)
            ),
            perception_mode_control=bool(root.get("perception_mode_control", True)),
            phases=phases,
            sku_routes=sku_routes,
            inventory=InventoryPolicy(
                enabled=inventory_enabled,
                database_path=inventory_path,
                default_destination_station=default_destination,
                shelf_face_stations=face_stations,
                prefer_inventory=bool(inventory_value.get("prefer_inventory", True)),
            ),
        )

    def route_for(self, sku: str, specification: str = "") -> SkuRoute:
        exact_key = f"{sku}|{specification}" if specification else ""
        if exact_key and exact_key in self.sku_routes:
            return self.sku_routes[exact_key]
        if sku in self.sku_routes:
            return self.sku_routes[sku]
        raise KeyError(f"no configured source route for SKU '{sku}' spec '{specification}'")

    def try_route_for(self, sku: str, specification: str = "") -> Optional[SkuRoute]:
        """Like :meth:`route_for`, but useful when inventory is authoritative."""
        try:
            return self.route_for(sku, specification)
        except KeyError:
            return None
