from dataclasses import dataclass
import threading
import time
from uuid import UUID

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory


@dataclass(frozen=True)
class PickPlan:
    plan_id: str
    detection_id: str
    object_id: str
    arm_name: str
    expires_at: float
    start_joints: dict[str, float]
    pregrasp_pose: PoseStamped
    grasp_pose: PoseStamped
    raise_trajectory: RobotTrajectory | None
    pregrasp_trajectory: RobotTrajectory
    approach_trajectory: RobotTrajectory
    retreat_trajectory: RobotTrajectory
    photo_return_trajectory: RobotTrajectory
    lower_trajectory: RobotTrajectory | None
    required_width_m: float
    score: float
    candidate_count: int


class PickPlanStore:
    def __init__(self, *, clock=time.monotonic):
        self._clock = clock
        self._values = {}
        self._lock = threading.Lock()

    @staticmethod
    def validate_id(value):
        try:
            return str(UUID(str(value)))
        except (TypeError, ValueError) as error:
            raise ValueError("plan_id must be a UUID") from error

    def put(self, plan):
        with self._lock:
            self._values[self.validate_id(plan.plan_id)] = plan

    def take(self, plan_id):
        with self._lock:
            plan = self._values.pop(self.validate_id(plan_id), None)
        if plan is None:
            raise ValueError("pick plan was not found or was already used")
        if plan.expires_at <= self._clock():
            raise ValueError("pick plan has expired")
        return plan
