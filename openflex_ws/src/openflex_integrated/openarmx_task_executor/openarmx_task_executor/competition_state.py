"""Pure referee-gated phase state used by the ROS competition coordinator."""

from dataclasses import dataclass

from .competition_config import VALID_PHASES


@dataclass
class CompetitionRunState:
    run_id: str = ""
    expected_phase: int = 1
    active_phase: int = 0
    failed_phase: int = 0

    def can_start(self, run_id: str, phase: int) -> bool:
        if not run_id or phase not in VALID_PHASES or self.active_phase:
            return False
        if self.run_id and run_id != self.run_id:
            return False
        if self.failed_phase:
            return False
        return phase == self.expected_phase

    def start(self, run_id: str, phase: int) -> None:
        if not self.can_start(run_id, phase):
            raise ValueError(
                f"cannot start run={run_id!r} phase={phase}; expected={self.expected_phase}, "
                f"active={self.active_phase}, failed={self.failed_phase}"
            )
        if not self.run_id:
            self.run_id = run_id
        self.active_phase = phase

    def complete(self) -> None:
        if not self.active_phase:
            raise ValueError("no active phase")
        completed = self.active_phase
        self.active_phase = 0
        self.expected_phase = min(4, completed + 1)

    def release_dry_run(self) -> None:
        if not self.active_phase:
            raise ValueError("no active phase")
        self.active_phase = 0
        if self.expected_phase == 1:
            self.run_id = ""

    def fail(self) -> None:
        if not self.active_phase:
            raise ValueError("no active phase")
        self.failed_phase = self.active_phase
        self.active_phase = 0

    def reset(self) -> None:
        if self.active_phase:
            raise ValueError("cannot reset while a phase is active")
        self.run_id = ""
        self.expected_phase = 1
        self.failed_phase = 0
