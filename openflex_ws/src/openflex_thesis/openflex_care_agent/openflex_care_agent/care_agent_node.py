"""CARE-Agent ROS node: observation aggregation and skill execution.

This node integrates the CARE-Agent decision core with ROS infrastructure,
translating between ROS messages/actions and the agent's BeliefState/Decision model.
"""

from copy import deepcopy
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from openflex_care_agent import (
    BeliefState,
    CAREAgent,
    ObjectBelief,
    RobotContext,
    TaskPhase,
    TaskSpec,
)
from openflex_care_agent.failure import FailureCode
from openflex_care_agent.skills import SkillName
from openflex_care_agent.skill_executor import SkillExecutor, map_error_to_failure
from openflex_thesis_msgs.action import (
    ComputeTcWam,
    ExecuteFixedPlace,
    ExecuteLocalAlignment,
    ExecuteMoveItGrasp,
    GenerateGrasps,
    RunPickDeliver,
)
from openflex_thesis_msgs.msg import ControlMode, GraspStatus, ObjectMemory
from openflex_thesis_msgs.srv import SetControlMode
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class CAREAgentNode(Node):
    """ROS node that runs the CARE-Agent decision loop."""

    def __init__(self) -> None:
        super().__init__("care_agent_node")

        # Parameters
        self.declare_parameter("mission_action", "/thesis/run_care_mission")
        self.declare_parameter("nav2_action", "/navigate_to_pose")
        self.declare_parameter("anygrasp_action", "/thesis/generate_grasps")
        self.declare_parameter("tc_wam_action", "/thesis/compute_tc_wam")
        self.declare_parameter("local_alignment_action", "/thesis/execute_local_alignment")
        self.declare_parameter("moveit_grasp_action", "/thesis/execute_moveit_grasp")
        self.declare_parameter("fixed_place_action", "/thesis/execute_fixed_place")
        self.declare_parameter("set_mode_service", "/thesis/set_control_mode")
        self.declare_parameter("memory_topic", "/thesis/object_memory")
        self.declare_parameter("grasp_status_topic", "/thesis/grasp_status")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("measured_velocity_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("decision_rate_hz", 1.0)
        self.declare_parameter("stream_timeout_sec", 0.5)
        self.declare_parameter("stopped_linear_mps", 0.02)
        self.declare_parameter("stopped_angular_rps", 0.03)

        self.callback_group = ReentrantCallbackGroup()

        # Action clients (reuse existing infrastructure)
        self.nav_client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter("nav2_action").value),
            callback_group=self.callback_group,
        )
        self.anygrasp_client = ActionClient(
            self, GenerateGrasps,
            str(self.get_parameter("anygrasp_action").value),
            callback_group=self.callback_group,
        )
        self.tc_wam_client = ActionClient(
            self, ComputeTcWam,
            str(self.get_parameter("tc_wam_action").value),
            callback_group=self.callback_group,
        )
        self.alignment_client = ActionClient(
            self, ExecuteLocalAlignment,
            str(self.get_parameter("local_alignment_action").value),
            callback_group=self.callback_group,
        )
        self.moveit_client = ActionClient(
            self, ExecuteMoveItGrasp,
            str(self.get_parameter("moveit_grasp_action").value),
            callback_group=self.callback_group,
        )
        self.place_client = ActionClient(
            self, ExecuteFixedPlace,
            str(self.get_parameter("fixed_place_action").value),
            callback_group=self.callback_group,
        )

        # Service client
        self.mode_client = self.create_client(
            SetControlMode,
            str(self.get_parameter("set_mode_service").value),
            callback_group=self.callback_group,
        )

        # Subscribers
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            ObjectMemory,
            str(self.get_parameter("memory_topic").value),
            self.on_memory,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            GraspStatus,
            str(self.get_parameter("grasp_status_topic").value),
            self.on_grasp_status,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.on_joint_state,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("measured_velocity_topic").value),
            self.on_odometry,
            20,
            callback_group=self.callback_group,
        )

        # Action server (mission interface)
        self.server = ActionServer(
            self,
            RunPickDeliver,
            str(self.get_parameter("mission_action").value),
            execute_callback=self.execute_mission,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )

        # CARE-Agent core
        self.agent = CAREAgent(maximum_recovery_attempts=3)
        self.skill_executor = SkillExecutor(self)

        # State
        self.latest_memory: Optional[ObjectMemory] = None
        self.latest_memory_time = 0.0
        self.latest_grasp_status: Optional[GraspStatus] = None
        self.latest_grasp_status_time = 0.0
        self.latest_joint_state: Optional[JointState] = None
        self.latest_joint_state_time = 0.0
        self.latest_odometry: Optional[Odometry] = None
        self.latest_odometry_time = 0.0
        self.busy = False
        self.mission_start_time = 0.0

        self.get_logger().info("CARE-Agent node initialized")

    def on_memory(self, msg: ObjectMemory) -> None:
        self.latest_memory = deepcopy(msg)
        self.latest_memory_time = time.monotonic()

    def on_grasp_status(self, msg: GraspStatus) -> None:
        self.latest_grasp_status = deepcopy(msg)
        self.latest_grasp_status_time = time.monotonic()

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = deepcopy(msg)
        self.latest_joint_state_time = time.monotonic()

    def on_odometry(self, msg: Odometry) -> None:
        self.latest_odometry = deepcopy(msg)
        self.latest_odometry_time = time.monotonic()

    def goal_callback(self, request) -> GoalResponse:
        if self.busy:
            self.get_logger().warn("Rejecting goal: agent is busy")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def aggregate_belief(self, task: TaskSpec, phase: TaskPhase) -> BeliefState:
        """Build BeliefState from current ROS observations."""
        now = time.monotonic()
        timeout = float(self.get_parameter("stream_timeout_sec").value)

        # Object belief from memory
        memory_fresh = (now - self.latest_memory_time) < timeout
        target = ObjectBelief(
            object_id=task.object_id,
            visible=memory_fresh and self.latest_memory is not None,
            grasp_authorized=memory_fresh and self.latest_memory is not None,
            confidence=1.0 if memory_fresh else 0.0,
            map_version=self.latest_memory.map_version if self.latest_memory else 0,
            age_sec=now - self.latest_memory_time if memory_fresh else 999.9,
            position_m=(
                (
                    self.latest_memory.pose.pose.position.x,
                    self.latest_memory.pose.pose.position.y,
                    self.latest_memory.pose.pose.position.z,
                )
                if memory_fresh and self.latest_memory
                else None
            ),
        )

        # Robot context
        joint_fresh = (now - self.latest_joint_state_time) < timeout
        odom_fresh = (now - self.latest_odometry_time) < timeout

        base_stopped = False
        if odom_fresh and self.latest_odometry:
            lin = self.latest_odometry.twist.twist.linear
            ang = self.latest_odometry.twist.twist.angular
            lin_speed = (lin.x ** 2 + lin.y ** 2 + lin.z ** 2) ** 0.5
            ang_speed = abs(ang.z)
            base_stopped = (
                lin_speed < float(self.get_parameter("stopped_linear_mps").value)
                and ang_speed < float(self.get_parameter("stopped_angular_rps").value)
            )

        context = RobotContext(
            camera_fresh=memory_fresh,
            odometry_fresh=odom_fresh,
            joint_state_fresh=joint_fresh,
            localization_valid=odom_fresh,
            base_stopped=base_stopped,
            base_available=not self.busy,
            manipulation_available=not self.busy,
            perception_ready=True,  # Assume ready if node is running
            moveit_ready=True,      # Assume ready if node is running
            remaining_time_sec=max(0.0, task.total_time_budget_sec - (now - self.mission_start_time)),
            map_version=self.latest_memory.map_version if self.latest_memory else 0,
        )

        return BeliefState(
            task=task,
            phase=phase,
            target=target,
            context=context,
            attempt_count=0,
            latest_failure=None,
        )

    async def execute_mission(self, goal_handle):
        """Main CARE-Agent decision loop."""
        self.busy = True
        self.mission_start_time = time.monotonic()
        goal = goal_handle.request

        # Create task spec
        task = TaskSpec(
            instruction=f"pick {goal.object_id} and deliver",
            object_id=goal.object_id,
            total_time_budget_sec=300.0,
        )

        phase = TaskPhase.NAVIGATE_OBSERVE
        failure = FailureCode.NONE

        self.get_logger().info(f"CARE-Agent mission started: {task.object_id}")

        try:
            # Decision loop
            while phase not in (TaskPhase.DONE, TaskPhase.FAILED, TaskPhase.HOLD):
                # Aggregate current belief
                belief = self.aggregate_belief(task, phase)

                # Get decision from agent
                decision = self.agent.decide(belief, failure)

                self.get_logger().info(
                    f"Phase={phase.value}, Decision={decision.skill.value}, "
                    f"Reason={decision.reason}, Fallback={decision.fallback}"
                )

                # If fallback (abort), mission failed
                if decision.fallback:
                    self.get_logger().error(f"Mission aborted: {decision.reason}")
                    result = RunPickDeliver.Result()
                    result.success = False
                    result.error_code = 99
                    result.message = decision.reason
                    goal_handle.abort()
                    return result

                # Execute skill using skill executor
                success, next_phase, error_code = await self.skill_executor.execute(
                    decision.skill, phase, {"goal": goal, "belief": belief}
                )

                if success:
                    phase = next_phase
                    failure = FailureCode.NONE
                else:
                    # Map error to failure code
                    failure = map_error_to_failure(error_code, decision.skill)
                    self.get_logger().warn(
                        f"Skill {decision.skill.value} failed: {failure.value} (code={error_code})"
                    )

            # Mission complete
            if phase == TaskPhase.DONE:
                self.get_logger().info("CARE-Agent mission SUCCESS")
                result = RunPickDeliver.Result()
                result.success = True
                result.total_duration_sec = time.monotonic() - self.mission_start_time
                goal_handle.succeed()
                return result
            else:
                self.get_logger().error(f"Mission ended in terminal phase: {phase.value}")
                result = RunPickDeliver.Result()
                result.success = False
                result.error_code = 98
                result.message = f"terminal phase {phase.value}"
                goal_handle.abort()
                return result

        finally:
            self.busy = False



def main(args=None):
    rclpy.init(args=args)
    node = CAREAgentNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
