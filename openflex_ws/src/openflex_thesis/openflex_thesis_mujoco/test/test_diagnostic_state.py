import json
import threading
from types import SimpleNamespace

from builtin_interfaces.msg import Time
import mujoco
from openflex_thesis_msgs.msg import GraspStatus, ObjectObservation
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


def test_publish_state_keeps_truth_joints_and_contacts_in_one_sample(state):
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
        lock=threading.RLock(), model=model, data=data,
        scalar_joint_names=["lift_joint"], base_pose=[0.0, 0.0, 0.0],
        actual_twist=[0.0, 0.0, 0.0], object_body_id=1,
        attached_arm="right", safety_stop=None, place_status=None,
        control_mode=None,
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
    for name in ("clock", "joint", "odom", "observation", "grasp",
                 "diagnostic_state"):
        setattr(bridge, name + "_pub", publisher(name))
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
    assert snapshot['control_ack'] is None
    assert snapshot['object_velocity_linear_angular'] == [0.0] * 6
    assert snapshot['delivery_support_contact'] is False
    assert snapshot["source"] == "mujoco_diagnostic_only_NOT_PHYSICAL"
