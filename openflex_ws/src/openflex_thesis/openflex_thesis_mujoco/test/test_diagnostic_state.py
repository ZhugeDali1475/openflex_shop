import json
import threading
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
import mujoco
from openflex_thesis_msgs.msg import ControlMode, GraspStatus, ObjectObservation
import pytest

from openflex_thesis_mujoco.bridge_node import MujocoBridgeNode


@pytest.fixture
def state():
    model = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <worldbody>
            <geom name="shelf_board" type="plane" size="1 1 0.1"/>
            <body name="object" pos="0 0 0.04">
              <joint name="lift_joint" type="slide" axis="0 0 1"/>
              <geom name="object_box" type="box" size="0.05 0.05 0.05"/>
            </body>
            <body name="free_payload_fixture" pos="5 5 5">
              <freejoint name="thesis_object_free"/>
              <geom name="thesis_object_geom" type="sphere" size="0.01"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    data.time = 12.0
    return model, data


def test_diagnostic_contacts_keep_both_geometries_and_signed_distance(state):
    model, data = state
    before = data.qpos.copy()
    bridge = SimpleNamespace(model=model, data=data)
    contacts = MujocoBridgeNode._diagnostic_contact_pairs(bridge)
    assert contacts
    first = contacts[0]
    assert {first["first"]["geom"], first["second"]["geom"]} == {
        "shelf_board", "object_box"
    }
    assert first["distance_m"] < 0.0
    assert (before == data.qpos).all()


def _publish_bridge(state, control_mode=None):
    """Minimal bridge stub that drives ``MujocoBridgeNode._publish_state``.

    ``control_mode`` is None for the no-acknowledgement case and a stub with
    ``mode``/``epoch``/``inputs_healthy`` when an acknowledgement is active.
    """
    model, data = state
    published = {}

    def publisher(name):
        def publish(message):
            published[name] = message
        return SimpleNamespace(publish=publish)

    def observation(stamp, position, orientation):
        message = ObjectObservation()
        message.header.stamp = stamp
        message.pose.pose.position.x = 999.0
        return message

    bridge = SimpleNamespace(
        lock=threading.RLock(), control_mode_lock=threading.RLock(), model=model, data=data,
        scalar_joint_names=["lift_joint"], base_pose=[0.0, 0.0, 0.0],
        actual_twist=[0.0, 0.0, 0.0], object_body_id=1,
        attached_arm="right", safety_stop=None, place_status=None,
        hand_body_ids={"right": 1},
        control_mode=control_mode,
        control_mode_receipt=time.monotonic(), commanded_twist=[0.0, 0.0, 0.0],
        control_mode_gap_sec=None, control_mode_gap_max_sec=0.0,
        control_mode_stamp=None, control_mode_stamp_gap_sec=None,
        control_mode_stamp_gap_max_sec=0.0,
        control_mode_entry=0.0, control_mode_entry_gap_sec=None,
        control_mode_entry_gap_max_sec=0.0, control_mode_lock_wait_sec=None,
        control_mode_lock_wait_max_sec=0.0,
        gil_heartbeat_sec=0.02, gil_heartbeat_max_sec=0.02,
        gil_heartbeat_max_steady_sec=0.02,
        trajectories={},
        joint_ids={'thesis_object_free': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'thesis_object_free')},
        _delivery_geometry=lambda: (0, [0.035, 0.03, 0.08], [0, 0, 0], [1, 1, 0.1]),
        place_status_until=0.0, viewer=None,
        _next_publish_stamp=lambda: Time(sec=12, nanosec=34),
        _joint_position=lambda name: float(data.qpos[0]),
        _id=lambda kind, name: mujoco.mj_name2id(model, kind, name),
        _mujoco_body_pose=lambda sample, body: (
            (0.0, 0.0, 0.04), (0.0, 0.0, 0.0, 1.0)
        ),
        _object_observation=observation,
        _grasp_status=lambda stamp, position: GraspStatus(),
        _publish_tf=lambda stamp, pose: None,
        get_parameter=lambda name: SimpleNamespace(value="item-1"),
    )
    bridge._diagnostic_contact_pairs = lambda: (
        MujocoBridgeNode._diagnostic_contact_pairs(bridge)
    )
    bridge._arm_control_valid = lambda: MujocoBridgeNode._arm_control_valid(bridge)
    bridge._place_guard_valid = lambda: MujocoBridgeNode._place_guard_valid(bridge)
    for name in ("clock", "joint", "odom", "observation", "grasp",
                 "diagnostic_state"):
        setattr(bridge, name + "_pub", publisher(name))
    return bridge, published
def test_publish_state_keeps_truth_joints_and_contacts_in_one_sample(state):
    bridge, published = _publish_bridge(state)
    MujocoBridgeNode._publish_state(bridge)
    snapshot = json.loads(published["diagnostic_state"].data)
    stamp = published["joint"].header.stamp
    assert snapshot["stamp"] == {"sec": stamp.sec, "nanosec": stamp.nanosec}
    assert snapshot["joint_state"]["position_rad_or_m"] == list(
        published["joint"].position
    )
    assert snapshot["object_pose"]["position_m"] == [0.0, 0.0, 0.04]
    assert published["observation"].pose.pose.position.x == 999.0
    assert snapshot["contacts"]
    assert snapshot["attached_arm"] == "right"
    assert snapshot["attached_hand_pose"]["frame_id"] == "map"
    assert snapshot["attachment_transform"]["parent_frame"] == "map"
    assert snapshot["attachment_transform"]["child_frame"] == "openarmx_right_hand"
    assert snapshot["attachment_transform"]["translation_m"] == [0.0, 0.0, 0.0]
    assert snapshot['control_ack'] is None
    assert snapshot['object_velocity_linear_angular'] == [0.0] * 6
    assert snapshot['delivery_support_contact'] is False
    assert snapshot["source"] == "mujoco_diagnostic_only_NOT_PHYSICAL"


def test_publish_state_publishes_control_ack_age_and_twist(state):
    """Diagnostics must expose *why* an arm acknowledgement is revoked."""
    bridge, published = _publish_bridge(
        state, control_mode=SimpleNamespace(mode=ControlMode.MOVEIT, epoch=7,
                                            inputs_healthy=True))
    MujocoBridgeNode._publish_state(bridge)
    snapshot = json.loads(published["diagnostic_state"].data)
    acknowledgement = snapshot["control_ack"]
    assert acknowledgement["mode"] == int(ControlMode.MOVEIT)
    assert acknowledgement["epoch"] == 7
    assert acknowledgement["inputs_healthy"] is True
    assert acknowledgement["fresh"] is True
    assert acknowledgement["arm_control_valid"] is True
    assert 0.0 <= acknowledgement["mode_receipt_age_sec"] < 1.0
    assert acknowledgement["actual_twist"] == [0.0, 0.0, 0.0]
    assert acknowledgement["receipt_gap_sec"] is None
    assert acknowledgement["receipt_gap_max_sec"] == 0.0
    assert acknowledgement["publisher_stamp_gap_sec"] is None
    assert acknowledgement["publisher_stamp_gap_max_sec"] == 0.0


def test_control_mode_receipt_gap_is_measured_apart_from_publisher_gap(monkeypatch):
    """A long receipt gap with a short publisher stamp gap is bridge-side dispatch delay.

    The gate compares the receipt age against a monotonic window, so the
    discrimination must not depend on the publisher's clock domain: both gaps are
    measured with each side's own clock and only their *difference* is interpreted.
    """
    # Two monotonic reads per message: the callback entry, then the in-lock
    # receipt.  Both invocations keep their own clock so the *difference* between
    # the two receipts (1.4 s) is compared with the publisher stamp gap (0.1 s).
    clock = iter([100.0, 100.0, 101.4, 101.4])
    monkeypatch.setattr('openflex_thesis_mujoco.bridge_node.time.monotonic', lambda: next(clock))
    warnings = []
    bridge = SimpleNamespace(
        control_mode_lock=threading.RLock(), control_mode=None, control_mode_receipt=0.0,
        control_mode_gap_sec=None, control_mode_gap_max_sec=0.0,
        control_mode_stamp=None, control_mode_stamp_gap_sec=None,
        control_mode_stamp_gap_max_sec=0.0,
        control_mode_entry=0.0, control_mode_entry_gap_sec=None,
        control_mode_entry_gap_max_sec=0.0, control_mode_lock_wait_sec=None,
        control_mode_lock_wait_max_sec=0.0,
        gil_heartbeat_sec=0.02, gil_heartbeat_max_steady_sec=0.0,
        get_logger=lambda: SimpleNamespace(warn=warnings.append),
    )
    first = ControlMode()
    first.header.stamp = Time(sec=0, nanosec=0)
    first.mode = ControlMode.HOLD
    second = ControlMode()
    second.header.stamp = Time(sec=0, nanosec=100000000)
    second.mode = ControlMode.MOVEIT
    MujocoBridgeNode._on_control_mode(bridge, first)
    assert bridge.control_mode_gap_sec is None and bridge.control_mode_stamp_gap_sec is None
    MujocoBridgeNode._on_control_mode(bridge, second)
    assert bridge.control_mode_gap_sec == 1.4
    assert bridge.control_mode_stamp_gap_sec == 0.1
    assert bridge.control_mode_gap_max_sec == 1.4
    assert bridge.control_mode_stamp_gap_max_sec == 0.1
    assert warnings and 'receipt gap 1.400s' in warnings[0]
    assert 'publisher stamped only 0.100s apart' in warnings[0]
    # The failure instant must be self-attributing: a steady-state heartbeat
    # spike means a process-wide interpreter stall, ~0.02 s means the receipt
    # thread alone was held up.
    assert 'gil_heartbeat=0.020' in warnings[0]


def test_control_mode_receipt_never_takes_the_simulation_lock(monkeypatch):
    """The 0.5 s exclusivity heartbeat must not wait on the sim loop's lock.

    Regression guard for the grasp dispatch fix: the simulation loop acquires
    ``self.lock`` ~500x/s, which starved the reentrant receipt callback and aged
    it past the window on an idle stack (``entry_gap`` up to 0.98 s with
    ``lock_wait`` near zero).  The receipt now uses its own ``control_mode_lock``
    only, so a poisoned ``self.lock`` must never be entered.
    """

    class PoisonLock:
        def __enter__(self):
            raise AssertionError('control-mode receipt must not take the simulation lock')

        def __exit__(self, *exc):
            return False

    clock = iter([500.0, 500.001])
    monkeypatch.setattr('openflex_thesis_mujoco.bridge_node.time.monotonic', lambda: next(clock))
    bridge = SimpleNamespace(
        lock=PoisonLock(), control_mode_lock=threading.RLock(),
        control_mode=None, control_mode_receipt=0.0,
        control_mode_gap_sec=None, control_mode_gap_max_sec=0.0,
        control_mode_stamp=None, control_mode_stamp_gap_sec=None,
        control_mode_stamp_gap_max_sec=0.0, control_mode_entry=0.0,
        control_mode_entry_gap_sec=None, control_mode_entry_gap_max_sec=0.0,
        control_mode_lock_wait_sec=None, control_mode_lock_wait_max_sec=0.0,
        gil_heartbeat_sec=0.02, gil_heartbeat_max_steady_sec=0.0,
        get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None),
    )
    message = ControlMode()
    message.header.stamp = Time(sec=0, nanosec=0)
    message.mode = ControlMode.MOVEIT
    MujocoBridgeNode._on_control_mode(bridge, message)
    assert bridge.control_mode is not None
    assert bridge.control_mode_receipt == 500.001
    assert bridge.control_mode_lock_wait_sec == 0.001
