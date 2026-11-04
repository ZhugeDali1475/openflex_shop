"""Write auditable JSONL events and one terminal result per real-robot trial."""

import json
from pathlib import Path
import time

from openflex_thesis_msgs.msg import (
    AffordanceState,
    ControlMode,
    GraspCandidateArray,
    GraspStatus,
    MissionState,
    ObjectMemory,
    PlaceStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class TrialLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("trial_logger")
        self.declare_parameter("output_root", "/tmp/openflex_thesis_runs")
        self.declare_parameter("trial_id", "UNSET")
        self.declare_parameter("experiment", "UNSET")
        self.declare_parameter("condition_json", "{}")
        self.declare_parameter("protocol_version", "v1")
        self.declare_parameter("code_version", "UNSET")
        self.declare_parameter("dataset_version", "UNSET")
        self.declare_parameter("checkpoint", "UNSET")
        trial_id = str(self.get_parameter("trial_id").value)
        if trial_id == "UNSET":
            raise ValueError("trial_id must be supplied from the frozen manifest")
        self.started_wall = time.time()
        self.started_monotonic = time.monotonic()
        self.run_dir = Path(str(self.get_parameter("output_root").value)) / trial_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        try:
            condition = json.loads(str(self.get_parameter("condition_json").value))
        except json.JSONDecodeError as error:
            raise ValueError(f"condition_json is invalid: {error}") from error
        metadata = {
            "trial_id": trial_id,
            "experiment": str(self.get_parameter("experiment").value),
            "condition": condition,
            "protocol_version": str(self.get_parameter("protocol_version").value),
            "code_version": str(self.get_parameter("code_version").value),
            "dataset_version": str(self.get_parameter("dataset_version").value),
            "checkpoint": str(self.get_parameter("checkpoint").value),
            "started_unix_sec": self.started_wall,
        }
        self._write_json(self.run_dir / "metadata.json", metadata)
        self.event_stream = (self.run_dir / "events.jsonl").open("w", encoding="utf-8")
        self.maximum_score = 0.0
        self.phase_entered = {}
        self.terminal_written = False
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(MissionState, "/thesis/mission_state", self.on_mission, qos)
        self.create_subscription(AffordanceState, "/thesis/tc_wam/state", self.on_affordance, qos)
        self.create_subscription(ObjectMemory, "/thesis/object_memory", self.on_memory, qos)
        self.create_subscription(ControlMode, "/thesis/control_mode", self.on_mode, qos)
        self.create_subscription(
            GraspCandidateArray,
            "/thesis/anygrasp/candidates",
            self.on_anygrasp,
            10,
        )
        self.create_subscription(GraspStatus, "/thesis/grasp_status", self.on_grasp, 20)
        self.create_subscription(PlaceStatus, "/thesis/place/status", self.on_place, 20)

    @staticmethod
    def _write_json(path: Path, value: dict) -> None:
        with path.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")

    def event(self, event_type: str, payload: dict) -> None:
        row = {
            "elapsed_sec": time.monotonic() - self.started_monotonic,
            "event": event_type,
            "payload": payload,
        }
        self.event_stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        self.event_stream.flush()

    def on_mission(self, msg: MissionState) -> None:
        self.phase_entered.setdefault(str(msg.phase), time.monotonic() - self.started_monotonic)
        self.event(
            "mission",
            {
                "task_id": msg.task_id,
                "object_id": msg.object_id,
                "phase": int(msg.phase),
                "attempt": int(msg.attempt),
                "map_version": int(msg.map_version),
                "grasp_id": int(msg.grasp_id),
                "ready_region_id": msg.ready_region_id,
                "score": float(msg.current_affordance_score),
                "detail": msg.detail,
                "error_code": int(msg.error_code),
            },
        )
        if msg.phase in (MissionState.SUCCEEDED, MissionState.FAILED) and not self.terminal_written:
            self.terminal_written = True
            self._write_json(
                self.run_dir / "result.json",
                {
                    "success": msg.phase == MissionState.SUCCEEDED,
                    "terminal_phase": int(msg.phase),
                    "error_code": int(msg.error_code),
                    "duration_sec": time.monotonic() - self.started_monotonic,
                    "recovery_attempts": int(msg.attempt),
                    "maximum_affordance_score": self.maximum_score,
                    "phase_entered_sec": self.phase_entered,
                    "detail": msg.detail,
                },
            )

    def on_affordance(self, msg: AffordanceState) -> None:
        self.maximum_score = max(self.maximum_score, float(msg.current_score))
        self.event(
            "affordance",
            {
                "object_id": msg.object_id,
                "map_version": int(msg.map_version),
                "active_arm": int(msg.active_arm),
                "selected_grasp_id": int(msg.selected_grasp_id),
                "selected_lift_position": float(msg.selected_lift_position),
                "planning_group": msg.planning_group,
                "ready_region_id": msg.ready_region_id,
                "current_score": float(msg.current_score),
                "best_score": float(msg.best_score),
                "ready": bool(msg.ready),
                "valid": bool(msg.valid),
                "physical": bool(msg.physical_evaluation),
            },
        )

    def on_memory(self, msg: ObjectMemory) -> None:
        self.event(
            "memory",
            {
                "object_id": msg.object_id,
                "map_version": int(msg.map_version),
                "visible": bool(msg.visible),
                "grasp_authorized": bool(msg.grasp_authorized),
                "freshness": float(msg.freshness),
                "event_flags": int(msg.event_flags),
            },
        )

    def on_mode(self, msg: ControlMode) -> None:
        self.event("control_mode", {"mode": int(msg.mode), "epoch": int(msg.epoch), "reason": msg.reason})

    def on_anygrasp(self, msg: GraspCandidateArray) -> None:
        self.event(
            "anygrasp",
            {
                "request_id": msg.request_id,
                "object_id": msg.object_id,
                "map_version": int(msg.map_version),
                "stage": int(msg.stage),
                "physical": bool(msg.physical_perception),
                "grasp_ids": [int(item.grasp_id) for item in msg.grasps],
                "qualities": [float(item.quality) for item in msg.grasps],
            },
        )

    def on_grasp(self, msg: GraspStatus) -> None:
        self.event(
            "grasp_status",
            {
                "verified": bool(msg.verified),
                "retained": bool(msg.object_retained),
                "transport_safe": bool(msg.transport_safe),
                "failure_code": int(msg.failure_code),
            },
        )

    def on_place(self, msg: PlaceStatus) -> None:
        self.event(
            "place_status",
            {
                "reached": bool(msg.target_reached),
                "released": bool(msg.released),
                "verified": bool(msg.verified),
                "failure_code": int(msg.failure_code),
            },
        )

    def destroy_node(self):
        if hasattr(self, "event_stream") and not self.event_stream.closed:
            self.event_stream.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrialLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
