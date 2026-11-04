"""Pure deterministic competition world and scoring model."""

from dataclasses import asdict, dataclass
from pathlib import Path
import random
from typing import Dict, List, Optional

import yaml


PHASE_PICKING = 1
PHASE_RESTOCKING = 2
PHASE_RETURN_MISPLACED = 3
POINTS_PER_ITEM = {PHASE_PICKING: 40, PHASE_RESTOCKING: 55, PHASE_RETURN_MISPLACED: 55}


@dataclass(frozen=True)
class OrderEntry:
    sku: str
    display_name: str
    specification: str
    source_station: str
    destination_station: str


@dataclass(frozen=True)
class WorkSpec:
    phase: int
    work_id: str
    object_id: str
    sku: str
    source_station: str
    destination_station: str
    destination_slot: str
    visible_from: str


@dataclass
class ObjectState:
    object_id: str
    sku: str
    phase: int
    station: str
    target_station: str
    target_slot: str = ""
    held: bool = False
    verified: bool = False


class CompetitionWorld:
    """Owns hidden truth. Production-like clients only receive action results."""

    def __init__(self, config_path: str, seed: int = 20260729):
        raw = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
        root = raw.get("world", raw)
        self.order_count = int(root.get("order_count", 2))
        self.restock_count = int(root.get("restock_count", 2))
        self.misplaced_count = int(root.get("misplaced_count", 2))
        self.map_bounds = tuple(float(value) for value in root["map_bounds"])
        if len(self.map_bounds) != 4:
            raise ValueError("map_bounds must contain xmin, xmax, ymin, ymax")
        self.order_catalog = {
            str(sku): OrderEntry(
                sku=str(sku),
                display_name=str(value["display_name"]),
                specification=str(value["specification"]),
                source_station=str(value["source_station"]),
                destination_station=str(value["destination_station"]),
            )
            for sku, value in root["order_catalog"].items()
        }
        self.restock_pool = self._work_pool(
            root["restock_cases"], PHASE_RESTOCKING, "restock"
        )
        self.misplaced_pool = self._work_pool(
            root["misplaced_cases"], PHASE_RETURN_MISPLACED, "return"
        )
        self.base_seed = int(seed)
        self.reset_index = -1
        self.reset()

    @staticmethod
    def _work_pool(values, phase: int, prefix: str) -> List[WorkSpec]:
        result = []
        for index, value in enumerate(values, start=1):
            result.append(
                WorkSpec(
                    phase=phase,
                    work_id=f"{prefix}:{value['object_id']}:{index}",
                    object_id=str(value["object_id"]),
                    sku=str(value["sku"]),
                    source_station=str(value["source_station"]),
                    destination_station=str(value["destination_station"]),
                    destination_slot=str(value.get("destination_slot", "")),
                    visible_from=str(value["visible_from"]),
                )
            )
        return result

    def reset(self, seed: Optional[int] = None) -> Dict:
        self.reset_index += 1
        self.seed = int(seed) if seed is not None else self.base_seed + self.reset_index
        rng = random.Random(self.seed)
        if self.order_count > len(self.order_catalog):
            raise ValueError("order_count exceeds order catalog")
        self.order = rng.sample(list(self.order_catalog.values()), self.order_count)
        self.restock = rng.sample(self.restock_pool, self.restock_count)
        self.misplaced = rng.sample(self.misplaced_pool, self.misplaced_count)
        self.objects: Dict[str, ObjectState] = {}
        for entry in self.order:
            self.objects[entry.sku] = ObjectState(
                object_id=entry.sku,
                sku=entry.sku,
                phase=PHASE_PICKING,
                station=entry.source_station,
                target_station=entry.destination_station,
            )
        for work in [*self.restock, *self.misplaced]:
            self.objects[work.object_id] = ObjectState(
                object_id=work.object_id,
                sku=work.sku,
                phase=work.phase,
                station=work.source_station,
                target_station=work.destination_station,
                target_slot=work.destination_slot,
            )
        self.held_object = ""
        self.verified_keys = set()
        self.phase_verified = {1: 0, 2: 0, 3: 0}
        self.collisions = 0
        self.out_of_bounds = 0
        self.drops = 0
        self.failed = False
        return self.snapshot()

    def work_visible(self, phase: int, station: str) -> List[WorkSpec]:
        pool = self.restock if phase == PHASE_RESTOCKING else self.misplaced
        return [
            work
            for work in pool
            if work.visible_from == station
            and not self.objects[work.object_id].verified
        ]

    def detect(self, object_id: str, station: str) -> bool:
        state = self.objects.get(object_id)
        return bool(state and not state.held and state.station == station)

    def pick(self, object_id: str, station: str) -> bool:
        if self.held_object or not self.detect(object_id, station):
            return False
        state = self.objects[object_id]
        state.held = True
        state.station = "robot"
        self.held_object = object_id
        return True

    def place(self, object_id: str, station: str) -> bool:
        if self.held_object != object_id:
            return False
        state = self.objects[object_id]
        state.held = False
        state.station = station
        self.held_object = ""
        return True

    def drop_held(self) -> bool:
        if not self.held_object:
            return False
        state = self.objects[self.held_object]
        state.held = False
        state.station = "floor"
        self.held_object = ""
        self.drops += 1
        return True

    def verify(self, object_id: str, station: str, slot: str = "") -> bool:
        state = self.objects.get(object_id)
        if state is None or state.held or state.station != station:
            return False
        if station != state.target_station:
            return False
        if state.target_slot and slot and slot != state.target_slot:
            return False
        key = (state.phase, object_id)
        if key not in self.verified_keys:
            self.verified_keys.add(key)
            self.phase_verified[state.phase] += 1
            state.verified = True
        return True

    def record_collision(self) -> None:
        self.collisions += 1

    def record_out_of_bounds(self) -> None:
        self.out_of_bounds += 1

    @property
    def raw_points(self) -> int:
        return sum(
            self.phase_verified[phase] * POINTS_PER_ITEM[phase]
            for phase in self.phase_verified
        )

    @property
    def penalty_points(self) -> int:
        return 5 * self.collisions + 10 * self.out_of_bounds + 5 * self.drops

    @property
    def score(self) -> int:
        return max(0, self.raw_points - self.penalty_points)

    def snapshot(self) -> Dict:
        return {
            "seed": self.seed,
            "order": [asdict(entry) for entry in self.order],
            "restock": [asdict(work) for work in self.restock],
            "misplaced": [asdict(work) for work in self.misplaced],
            "objects": {key: asdict(value) for key, value in self.objects.items()},
            "phase_verified": dict(self.phase_verified),
            "raw_points": self.raw_points,
            "penalty_points": self.penalty_points,
            "score": self.score,
            "collisions": self.collisions,
            "out_of_bounds": self.out_of_bounds,
            "drops": self.drops,
        }
