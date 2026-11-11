from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
pytest.importorskip('mujoco')
from openflex_roboos_bridge.navigation_guard import NavigationGuard


@pytest.fixture
def guard(tmp_path):
    model = tmp_path / 'payload.xml'
    model.write_text('''<mujoco><worldbody>
      <geom type="plane" size="10 10 .1"/>
      <body name="openflex_robot_yaw_root" pos="0 0 1">
        <joint name="base_x" type="slide" axis="1 0 0"/>
        <joint name="base_y" type="slide" axis="0 1 0"/>
        <joint name="base_yaw" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size=".1" mass="1"/>
        <body name="openarmx_right_hand" pos=".6 0 0"><geom type="sphere" size=".05" mass="1"/></body>
      </body>
      <body name="thesis_shelf" pos="3 0 1"><geom type="box" size=".1 .1 .3"/></body>
      <body name="thesis_delivery_table" pos="-3 0 1"><geom type="box" size=".1 .1 .3"/></body>
      <body name="thesis_object" pos="0 0 5"><freejoint name="thesis_object_free"/>
        <geom type="box" size=".035 .03 .08" mass=".1"/></body>
    </worldbody></mujoco>''')
    return NavigationGuard(model)


def payload(position):
    return {'frame_id': 'map', 'position_m': position, 'orientation_xyzw': [0, 0, 0, 1]}


def test_payload_path_rejected_when_robot_alone_is_clear(guard):
    joints = SimpleNamespace(name=[], position=[])
    assert guard.check(joints, [0, 0, 0], [2, 0, 0])['accepted']
    result = guard.check(joints, [0, 0, 0], [2, 0, 0], payload([1, 0, 1]))
    assert not result['accepted']
    assert 'thesis_object' in result['bodies']


def test_clear_payload_path_is_labeled_rigid_discrete_not_physical(guard):
    result = guard.check(SimpleNamespace(name=[], position=[]), [0, 0, 0], [.2, 0, .5], payload([1, 0, 1]))
    assert result['accepted']
    assert result['scope'].startswith('measured_rigid_payload')
    assert result['not_continuous_collision_proof'] and not result['physical_navigation_validated']


def test_payload_floor_collision_is_not_support_contact(guard):
    result = guard.check(SimpleNamespace(name=[], position=[]), [0, 0, 0], [0, 0, 0], payload([1, 0, .01]))
    assert not result['accepted']


def test_payload_collision_with_robot_non_touch_link_rejected(guard):
    result = guard.check(SimpleNamespace(name=[], position=[]), [0, 0, 0], [0, 0, 0], payload([0, 0, 1]))
    assert not result['accepted']


@pytest.mark.parametrize('update', [{'frame_id': 'odom'}, {'position_m': [float('nan'), 0, 1]},
                                  {'orientation_xyzw': [0, 0, 0, 0]}, {'position_m': [0, 1]}])
def test_bad_measured_payload_rejected(guard, update):
    with pytest.raises(ValueError):
        guard.check(SimpleNamespace(name=[], position=[]), [0, 0, 0], [0, 0, 0], dict(payload([1, 0, 1]), **update))
