import asyncio
from copy import deepcopy
import json
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from moveit_msgs.msg import ContactInformation, RobotState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from openflex_thesis_control.moveit_grasp_executor_node import (
    MoveItGraspExecutorNode,
)
from openflex_thesis_control.validity_diagnostics import (
    failure_category,
    sample_pairing,
)


def test_simulation_and_joint_samples_pair_by_exact_stamp_and_object():
    joint_state = JointState()
    joint_state.header.stamp.sec = 12
    joint_state.header.stamp.nanosec = 34
    simulation = {
        "stamp": {"sec": 12, "nanosec": 34},
        "object_id": "item-1",
    }
    paired = sample_pairing(joint_state, simulation, 0.02, "item-1")
    assert paired["same_publish_stamp"]
    assert paired["same_object_id"]
    assert paired["stamp_delta_ns"] == 0
    assert not paired["synchronized_with_moveit"]


def test_mismatched_stamp_or_object_never_claims_pairing():
    joint_state = JointState()
    joint_state.header.stamp.sec = 12
    simulation = {"stamp": {"sec": 13, "nanosec": 0}, "object_id": "other"}
    paired = sample_pairing(joint_state, simulation, 0.02, "item-1")
    assert not paired["same_publish_stamp"]
    assert not paired["same_object_id"]


def test_failure_category_distinguishes_current_and_target_checks():
    assert failure_category([
        {"label": "current", "response": {"valid": False}},
        {"label": "transport", "response": {"valid": True}},
    ]) == "current_invalid"
    assert failure_category([
        {"label": "current", "response": {"valid": False}},
        {"label": "transport", "response": {"valid": False}},
    ]) == "current_and_target_invalid"
    assert failure_category([
        {"label": "current", "response": None}
    ]) == "validity_service_failure"


@pytest.fixture
def executor():
    logs = []
    joint_state = JointState()
    joint_state.header.stamp.sec = 12
    joint_state.name = ["lift_joint", "inactive_arm_joint"]
    joint_state.position = [0.17, -1.2]
    node = SimpleNamespace(
        joint_diagnostic_sample=(joint_state, time.monotonic()),
        simulation_diagnostic_sample=(None, 0.0),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
        get_logger=lambda: SimpleNamespace(
            error=logs.append, warning=logs.append
        ),
        active_object_id="item-1",
        active_map_version=5,
        failure_stage="transport",
        goal_target_diagnostic=None,
        object_memory=None,
        object_memory_receipt=0.0,
        attachment_diagnostic={
            "requested_attachment": {"object": {"id": "attached_item"}}
        },
        logs=logs,
    )

    async def record(snapshot):
        await MoveItGraspExecutorNode.record_validity_failure(node, snapshot)
    node.record_validity_failure = record
    return node


def test_invalid_state_retains_scene_and_contacts_before_cleanup(executor):
    requests = []
    original_joints = deepcopy(executor.joint_diagnostic_sample[0])
    contact = ContactInformation()
    contact.contact_body_1 = "shelf_board"
    contact.contact_body_2 = "attached_item"
    contact.depth = 0.000083
    contact.position.z = 0.84

    async def checked(request):
        requests.append(request)
        executor.joint_diagnostic_sample[0].position[0] = 0.9
        if len(requests) == 1:
            return GetStateValidity.Response(valid=False, contacts=[contact])
        return GetStateValidity.Response(valid=True)

    def readback(request):
        future = asyncio.get_running_loop().create_future()
        response = GetPlanningScene.Response()
        response.scene.name = "shelf_scene"
        future.set_result(response)
        return future

    executor.validity_client = SimpleNamespace(call_async=checked)
    executor.scene_readback_client = SimpleNamespace(call_async=readback)
    valid, detail = asyncio.run(
        MoveItGraspExecutorNode.transportation_start_valid(
            executor, "right_arm_with_lift", RobotState()
        )
    )
    assert not valid and "shelf_board<->attached_item" in detail
    assert requests[0].robot_state.joint_state == original_joints
    assert all(request.group_name == "" for request in requests)
    snapshot = json.loads(executor.logs[0].split(" ", 1)[1])
    assert snapshot["failure_category"] == "current_invalid"
    contact_json = snapshot["checks"][0]["response"]["contacts"][0]
    assert contact_json["depth"] == pytest.approx(0.000083)
    assert contact_json["position"]["z"] == 0.84
    assert snapshot["scene_readback"]["scene"]["name"] == "shelf_scene"
    attached_id = snapshot["attachment_input"]["requested_attachment"]
    assert attached_id["object"]["id"] == "attached_item"
    assert not snapshot["simulation_pairing"]["synchronized_with_moveit"]


def test_missing_scene_readback_does_not_hide_validity_failure(executor):
    async def checked(request):
        return None

    def readback(request):
        raise RuntimeError("scene service offline")

    executor.validity_client = SimpleNamespace(call_async=checked)
    executor.scene_readback_client = SimpleNamespace(call_async=readback)
    valid, detail = asyncio.run(
        MoveItGraspExecutorNode.transportation_start_valid(
            executor, "right_arm_with_lift", RobotState()
        )
    )
    snapshot = json.loads(executor.logs[0].split(" ", 1)[1])
    assert not valid and "returned nothing" in detail
    assert snapshot["failure_category"] == "validity_service_failure"
    assert snapshot["scene_readback_error"] == "scene service offline"


def test_scene_readback_timeout_is_bounded(executor, monkeypatch):
    pending = []

    def readback(request):
        future = asyncio.get_running_loop().create_future()
        pending.append(future)
        return future

    elapsed = [0.0]

    async def sleep(node, duration):
        elapsed[0] += 1.1

    monkeypatch.setattr(
        "openflex_thesis_control.moveit_grasp_executor_node.ros_sleep", sleep)
    monkeypatch.setattr(
        "openflex_thesis_control.moveit_grasp_executor_node.time.monotonic",
        lambda: elapsed[0],
    )
    executor.scene_readback_client = SimpleNamespace(call_async=readback)
    asyncio.run(MoveItGraspExecutorNode.record_validity_failure(executor, {}))
    assert pending[0].cancelled()
    snapshot = json.loads(executor.logs[0].split(" ", 1)[1])
    assert "exceeded 1 second" in snapshot["scene_readback_error"]


@pytest.mark.parametrize("payload", [
    "{",
    "[]",
    '{"schema_version": 2}',
    '{"schema_version": 1, "source": '
    '"mujoco_diagnostic_only_NOT_PHYSICAL", "stamp": {}}',
])
def test_malformed_simulation_sample_keeps_previous(executor, payload):
    original = executor.simulation_diagnostic_sample
    message = String(data=payload)
    MoveItGraspExecutorNode.on_simulation_diagnostic(executor, message)
    assert executor.simulation_diagnostic_sample == original
    assert "ignoring simulation diagnostic" in executor.logs[0]


def test_ros_pubsub_delivers_same_frame_diagnostic_to_real_callbacks():
    context = Context()
    rclpy.init(context=context, domain_id=173)
    publisher_node = Node("diagnostic_probe_publisher", context=context)
    subscriber_node = Node("diagnostic_probe_subscriber", context=context)
    executor = SingleThreadedExecutor(context=context)
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    receipt_node = SimpleNamespace(
        joint_state=None,
        joint_state_receipt=0.0,
        joint_diagnostic_sample=(None, 0.0),
        simulation_diagnostic_sample=(None, 0.0),
        get_logger=lambda: SimpleNamespace(warning=lambda message: None),
    )
    joint_subscriber = subscriber_node.create_subscription(
        JointState,
        "/diagnostic_probe/joint_states",
        lambda message: MoveItGraspExecutorNode.on_joint_state(
            receipt_node, message
        ),
        qos,
    )
    diagnostic_subscriber = subscriber_node.create_subscription(
        String,
        "/diagnostic_probe/state",
        lambda message: MoveItGraspExecutorNode.on_simulation_diagnostic(
            receipt_node, message
        ),
        qos,
    )
    joint_publisher = publisher_node.create_publisher(
        JointState, "/diagnostic_probe/joint_states", qos
    )
    diagnostic_publisher = publisher_node.create_publisher(
        String, "/diagnostic_probe/state", qos
    )
    executor.add_node(subscriber_node)
    joint_message = JointState()
    joint_message.header.stamp.sec = 17
    joint_message.header.stamp.nanosec = 23
    joint_message.name = ["lift_joint"]
    joint_message.position = [0.17]
    diagnostic_message = String(data=json.dumps({
        "schema_version": 1,
        "source": "mujoco_diagnostic_only_NOT_PHYSICAL",
        "stamp": {"sec": 17, "nanosec": 23},
        "object_id": "item-1",
        "object_pose": {"position_m": [1.0, 2.0, 3.0]},
        "contacts": [{"distance_m": -0.001}],
    }))
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            joint_publisher.publish(joint_message)
            diagnostic_publisher.publish(diagnostic_message)
            executor.spin_once(timeout_sec=0.05)
            joint_sample, _ = receipt_node.joint_diagnostic_sample
            simulation, _ = receipt_node.simulation_diagnostic_sample
            if joint_sample is not None and simulation is not None:
                break
        joint_sample, _ = receipt_node.joint_diagnostic_sample
        simulation, simulation_receipt = (
            receipt_node.simulation_diagnostic_sample
        )
        assert joint_sample is not None
        assert simulation is not None
        assert simulation["contacts"][0]["distance_m"] == pytest.approx(-0.001)
        paired = sample_pairing(
            joint_sample,
            simulation,
            time.monotonic() - simulation_receipt,
            "item-1",
        )
        assert paired["same_publish_stamp"]
        assert paired["same_object_id"]
        assert not paired["synchronized_with_moveit"]

        joint_message.header.stamp.nanosec = 24
        joint_publisher.publish(joint_message)
        mismatch_deadline = time.monotonic() + 1.0
        while time.monotonic() < mismatch_deadline:
            executor.spin_once(timeout_sec=0.02)
            joint_sample, _ = receipt_node.joint_diagnostic_sample
            if joint_sample.header.stamp.nanosec == 24:
                break
        paired = sample_pairing(joint_sample, simulation, 0.0, "item-1")
        assert not paired["same_publish_stamp"]
    finally:
        executor.remove_node(subscriber_node)
        executor.shutdown()
        publisher_node.destroy_publisher(joint_publisher)
        publisher_node.destroy_publisher(diagnostic_publisher)
        subscriber_node.destroy_subscription(joint_subscriber)
        subscriber_node.destroy_subscription(diagnostic_subscriber)
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown(context=context)
