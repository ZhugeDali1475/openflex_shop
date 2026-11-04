"""Short-lived truth detections used by the simulator PickObject contract."""

from dataclasses import dataclass
import time
from typing import Dict

from openarmx_task_executor.manipulation_contract import normalize_arm_name


@dataclass
class TruthDetection:
    object_id: str
    station_id: str
    arm_name: str
    expires_at: float
    consumed: bool = False


class TruthDetectionStore:
    def __init__(self, ttl_sec: float = 30.0) -> None:
        if ttl_sec <= 0.0:
            raise ValueError("truth detection TTL must be positive")
        self.ttl_sec = float(ttl_sec)
        self._sequence = 0
        self._records: Dict[str, TruthDetection] = {}

    @staticmethod
    def validate_detect_request(
        object_id: str, station_id: str, arm_name: str
    ) -> tuple[str, str, str]:
        normalized_arm = normalize_arm_name(arm_name)
        object_key = str(object_id).strip()
        station_key = str(station_id).strip()
        if not object_key or not station_key:
            raise ValueError("truth detection requires object_id and station_id")
        return object_key, station_key, normalized_arm

    def clear(self) -> None:
        self._records.clear()

    def create(
        self, object_id: str, station_id: str, arm_name: str, now: float = None
    ) -> str:
        object_key, station_key, normalized_arm = self.validate_detect_request(
            object_id, station_id, arm_name
        )
        timestamp = time.monotonic() if now is None else float(now)
        self._sequence += 1
        detection_id = f"truth:{normalized_arm}:{self._sequence}"
        self._records[detection_id] = TruthDetection(
            object_id=object_key,
            station_id=station_key,
            arm_name=normalized_arm,
            expires_at=timestamp + self.ttl_sec,
        )
        return detection_id

    def validate(
        self,
        detection_id: str,
        object_id: str,
        station_id: str,
        arm_name: str,
        now: float = None,
    ) -> TruthDetection:
        record = self._records.get(str(detection_id).strip())
        if record is None:
            raise ValueError("unknown detection_id")
        timestamp = time.monotonic() if now is None else float(now)
        if timestamp > record.expires_at:
            raise ValueError("detection_id has expired")
        if record.consumed:
            raise ValueError("detection_id was already consumed")
        if record.object_id != str(object_id).strip():
            raise ValueError("detection_id object does not match pick request")
        if record.station_id != str(station_id).strip():
            raise ValueError("detection_id station does not match current station")
        if record.arm_name != normalize_arm_name(arm_name):
            raise ValueError("detection_id arm does not match pick request")
        return record

    def consume(
        self,
        detection_id: str,
        object_id: str,
        station_id: str,
        arm_name: str,
        now: float = None,
    ) -> TruthDetection:
        record = self.validate(
            detection_id, object_id, station_id, arm_name, now=now
        )
        record.consumed = True
        return record
