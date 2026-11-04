"""Event-triggered object memory with conservative grasp authorization."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from .geometry import normalize_angle


EVENT_NONE = 0
EVENT_MOVED = 1
EVENT_LOST = 2
EVENT_REOBSERVED = 4
EVENT_AFFORDANCE_INVALID = 8


@dataclass(frozen=True)
class MemoryConfig:
    freshness_tau_sec: float = 2.0
    lost_timeout_sec: float = 1.0
    min_confidence: float = 0.50
    translation_move_threshold_m: float = 0.05
    yaw_move_threshold_rad: float = 0.15
    move_confirm_frames: int = 3
    default_variance: float = 0.0004

    def validate(self) -> None:
        if self.freshness_tau_sec <= 0.0 or self.lost_timeout_sec <= 0.0:
            raise ValueError("time constants must be positive")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if self.translation_move_threshold_m <= 0.0:
            raise ValueError("translation threshold must be positive")
        if self.yaw_move_threshold_rad <= 0.0:
            raise ValueError("yaw threshold must be positive")
        if self.move_confirm_frames <= 0:
            raise ValueError("move_confirm_frames must be positive")


@dataclass
class MemoryState:
    object_id: str
    frame_id: str
    position: Tuple[float, float, float]
    yaw: float
    variances: Tuple[float, float, float]
    yaw_variance: float
    confidence: float
    last_seen_sec: float
    source: str
    sequence: int = 0
    visible: bool = True
    map_version: int = 1
    event_flags: int = EVENT_NONE
    moved_streak: int = 0
    was_lost: bool = False


class ObjectMemoryFilter:
    def __init__(self, config: MemoryConfig):
        config.validate()
        self.config = config
        self.state: Optional[MemoryState] = None

    @staticmethod
    def _fuse(old: float, old_var: float, new: float, new_var: float) -> Tuple[float, float]:
        old_var = max(old_var, 1.0e-9)
        new_var = max(new_var, 1.0e-9)
        gain = old_var / (old_var + new_var)
        return old + gain * (new - old), (1.0 - gain) * old_var

    def observe(
        self,
        object_id: str,
        frame_id: str,
        position: Tuple[float, float, float],
        yaw: float,
        variances: Tuple[float, float, float],
        yaw_variance: float,
        confidence: float,
        stamp_sec: float,
        sequence: int = 0,
        source: str = "unknown",
    ) -> MemoryState:
        if not object_id or not frame_id:
            raise ValueError("object_id and frame_id are required")
        values = (*position, yaw, *variances, yaw_variance, confidence, stamp_sec)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("observation contains non-finite values")
        variances = tuple(value if value > 0.0 else self.config.default_variance for value in variances)
        yaw_variance = (
            yaw_variance if yaw_variance > 0.0 else self.config.default_variance
        )
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("observation confidence must be in [0, 1]")
        current = self.state
        if current is None or current.object_id != object_id or current.frame_id != frame_id:
            self.state = MemoryState(
                object_id, frame_id, position, normalize_angle(yaw), variances,
                yaw_variance,
                confidence, stamp_sec, source, sequence=sequence,
            )
            return self.state

        if stamp_sec <= current.last_seen_sec:
            raise ValueError("observation timestamp must be strictly increasing")
        if sequence > 0 and current.sequence > 0 and sequence <= current.sequence:
            raise ValueError("observation sequence must be strictly increasing")

        current.event_flags = EVENT_NONE
        if current.was_lost:
            current.event_flags |= EVENT_REOBSERVED | EVENT_AFFORDANCE_INVALID
            current.map_version += 1
            current.was_lost = False

        translation_jump = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, current.position)))
        yaw_jump = abs(normalize_angle(yaw - current.yaw))
        moved = (
            translation_jump >= self.config.translation_move_threshold_m
            or yaw_jump >= self.config.yaw_move_threshold_rad
        )
        if moved:
            current.moved_streak += 1
            if current.moved_streak >= self.config.move_confirm_frames:
                current.position = position
                current.yaw = normalize_angle(yaw)
                current.variances = variances
                current.yaw_variance = yaw_variance
                current.map_version += 1
                current.event_flags |= EVENT_MOVED | EVENT_AFFORDANCE_INVALID
                current.moved_streak = 0
        else:
            current.moved_streak = 0
            fused = [self._fuse(a, av, b, bv) for a, av, b, bv in zip(
                current.position, current.variances, position, variances
            )]
            current.position = tuple(item[0] for item in fused)
            current.variances = tuple(item[1] for item in fused)
            yaw_delta = normalize_angle(yaw - current.yaw)
            fused_yaw_delta, fused_yaw_variance = self._fuse(
                0.0, current.yaw_variance, yaw_delta, yaw_variance
            )
            current.yaw = normalize_angle(current.yaw + fused_yaw_delta)
            current.yaw_variance = fused_yaw_variance
        current.confidence = confidence
        current.last_seen_sec = stamp_sec
        current.sequence = sequence
        current.source = source
        current.visible = True
        return current

    def freshness(self, now_sec: float) -> float:
        if self.state is None:
            return 0.0
        age = max(0.0, now_sec - self.state.last_seen_sec)
        return self.state.confidence * math.exp(-age / self.config.freshness_tau_sec)

    def tick(self, now_sec: float) -> Optional[MemoryState]:
        current = self.state
        if current is None:
            return None
        age = max(0.0, now_sec - current.last_seen_sec)
        if age > self.config.lost_timeout_sec and current.visible:
            current.visible = False
            current.was_lost = True
            current.map_version += 1
            current.event_flags = EVENT_LOST | EVENT_AFFORDANCE_INVALID
        return current

    def grasp_authorized(self, now_sec: float) -> bool:
        return bool(
            self.state is not None
            and self.state.visible
            and self.freshness(now_sec) >= self.config.min_confidence
        )
