"""ROS-independent velocity ownership and handoff safety logic."""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple


HOLD = 0
NAV2 = 1
ACT = 2
RECOVERY = 3
PLACE = 4
MOVEIT = 5


@dataclass(frozen=True)
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class ArbiterConfig:
    input_timeout_sec: float = 0.20
    measured_timeout_sec: float = 0.25
    stop_confirm_sec: float = 0.50
    stopped_linear_mps: float = 0.02
    stopped_angular_rps: float = 0.03
    max_linear_mps: float = 0.25
    max_angular_rps: float = 0.40
    require_physical_affordance: bool = True


class VelocityArbiter:
    def __init__(self, config: ArbiterConfig):
        self.config = config
        self.mode = HOLD
        self.epoch = 0
        self.stopped_since: Optional[float] = None
        self.last_measured_sec: Optional[float] = None
        self.last_measured_velocity: Optional[Velocity] = None
        self.sources: Dict[int, Tuple[Velocity, float]] = {}
        self.affordance_valid = False
        self.affordance_physical = False
        self.object_visible = False
        self.grasp_authorized = False

    def update_measured_velocity(self, velocity: Velocity, now_sec: float) -> None:
        self.last_measured_velocity = velocity
        self.last_measured_sec = now_sec
        stopped = (
            abs(velocity.vx) <= self.config.stopped_linear_mps
            and abs(velocity.vy) <= self.config.stopped_linear_mps
            and abs(velocity.wz) <= self.config.stopped_angular_rps
        )
        if stopped:
            if self.stopped_since is None:
                self.stopped_since = now_sec
        else:
            self.stopped_since = None

    def update_safety(
        self,
        affordance_valid: bool,
        affordance_physical: bool,
        object_visible: bool,
        grasp_authorized: bool,
    ) -> None:
        self.affordance_valid = affordance_valid
        self.affordance_physical = affordance_physical
        self.object_visible = object_visible
        self.grasp_authorized = grasp_authorized
        if self.mode == ACT and not self.act_safety_valid():
            self.mode = HOLD
            self.epoch += 1

    def update_source(self, mode: int, velocity: Velocity, now_sec: float) -> None:
        if mode not in (NAV2, ACT, RECOVERY):
            raise ValueError("source mode must be NAV2, ACT or RECOVERY")
        self.sources[mode] = velocity, now_sec

    def confirmed_stopped(self, now_sec: float, stale_sec: Optional[float] = None) -> bool:
        return bool(
            self.last_measured_sec is not None
            and now_sec - self.last_measured_sec <= (
                self.config.measured_timeout_sec if stale_sec is None else stale_sec)
            and self.stopped_since is not None
            and now_sec - self.stopped_since >= self.config.stop_confirm_sec
        )

    def stop_diagnostics(self, now_sec: float) -> dict:
        """Raw fields behind confirmed_stopped(), for logging only."""
        return {
            "now_sec": round(float(now_sec), 3),
            "last_measured_sec": None if self.last_measured_sec is None else round(self.last_measured_sec, 3),
            "measured_age_sec": None if self.last_measured_sec is None else round(now_sec - self.last_measured_sec, 3),
            "stopped_since": None if self.stopped_since is None else round(self.stopped_since, 3),
            "measured_velocity": None if self.last_measured_velocity is None else [
                round(float(self.last_measured_velocity.vx), 4),
                round(float(self.last_measured_velocity.vy), 4),
                round(float(self.last_measured_velocity.wz), 4)],
        }

    def act_safety_valid(self) -> bool:
        return bool(
            self.affordance_valid
            and self.object_visible
            and self.grasp_authorized
            and (
                self.affordance_physical
                or not self.config.require_physical_affordance
            )
        )

    def request_mode(self, requested: int, now_sec: float) -> Tuple[bool, str]:
        if requested not in (HOLD, NAV2, ACT, RECOVERY, PLACE, MOVEIT):
            return False, "unknown control mode"
        if requested == ACT:
            if not self.confirmed_stopped(now_sec):
                return False, "measured base velocity has not remained stopped long enough"
            if not self.act_safety_valid():
                return False, "fresh physical TC-WAM and visible authorized object required"
        if requested in (MOVEIT, PLACE) and not self.confirmed_stopped(now_sec):
            return False, "base must remain measurably stopped before manipulation"
        if requested != self.mode:
            self.mode = requested
            self.epoch += 1
        return True, "accepted"

    @staticmethod
    def _limit(value: float, maximum: float) -> float:
        return max(-maximum, min(maximum, value))

    def selected_velocity(self, now_sec: float) -> Velocity:
        if self.mode not in (NAV2, ACT, RECOVERY):
            return Velocity()
        if self.mode == ACT and not self.act_safety_valid():
            self.mode = HOLD
            self.epoch += 1
            return Velocity()
        source = self.sources.get(self.mode)
        if source is None or now_sec - source[1] > self.config.input_timeout_sec:
            return Velocity()
        velocity = source[0]
        return Velocity(
            self._limit(velocity.vx, self.config.max_linear_mps),
            self._limit(velocity.vy, self.config.max_linear_mps),
            self._limit(velocity.wz, self.config.max_angular_rps),
        )
