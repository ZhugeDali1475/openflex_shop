from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from uuid import UUID

import numpy as np

from .perception.rgbd import FrozenRgbdFrame


class DetectionStoreError(RuntimeError):
    pass


@dataclass(frozen=True)
class DetectionSnapshot:
    detection_id: str
    object_id: str
    arm_name: str
    frame: FrozenRgbdFrame
    provider: str
    model: str
    request_id: str
    box_xyxy: tuple[int, int, int, int]
    mask: np.ndarray
    product_points: np.ndarray
    scene_points: np.ndarray
    object_center: np.ndarray
    confidence: float
    expires_at: float


class DetectionStore:
    """Thread-safe TTL cache with one successful Pick consumer per detection UUID."""

    def __init__(self, *, clock=time.monotonic):
        self._clock = clock
        self._values: dict[str, DetectionSnapshot] = {}
        self._lock = threading.Lock()

    @staticmethod
    def validate_id(value: str) -> str:
        try:
            return str(UUID(str(value)))
        except (TypeError, ValueError) as error:
            raise DetectionStoreError("detection_id must be a UUID") from error

    def put(self, snapshot: DetectionSnapshot) -> None:
        detection_id = self.validate_id(snapshot.detection_id)
        if snapshot.expires_at <= self._clock():
            raise DetectionStoreError("cannot store an already expired detection")
        with self._lock:
            self._values[detection_id] = snapshot

    def get(self, detection_id: str, *, consume: bool = False) -> DetectionSnapshot:
        detection_id = self.validate_id(detection_id)
        with self._lock:
            snapshot = self._values.get(detection_id)
            if snapshot is None:
                raise DetectionStoreError("detection snapshot was not found or was already consumed")
            if snapshot.expires_at <= self._clock():
                self._values.pop(detection_id, None)
                raise DetectionStoreError("detection snapshot has expired")
            if consume:
                self._values.pop(detection_id, None)
            return snapshot

    def purge_expired(self) -> int:
        now = self._clock()
        with self._lock:
            keys = [key for key, value in self._values.items() if value.expires_at <= now]
            for key in keys:
                self._values.pop(key, None)
        return len(keys)
