from pathlib import Path
import sys

import pytest


pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from probe_openflex_place_trajectory import NAMES, TOUCH, joint_goal_constraints, payload_contact_permitted


def test_postplace_retraction_accepts_integer_seed_values_as_ros_floats():
    positions = dict(zip(NAMES, [-0.10, 0, 0, 0, 1.8, 0, 0, 0]))
    constraints = joint_goal_constraints(positions).joint_constraints
    assert [item.joint_name for item in constraints] == NAMES
    assert [item.position for item in constraints] == [-0.10, 0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0]
    assert all(type(item.position) is float for item in constraints)


@pytest.mark.parametrize('body_name', sorted(TOUCH))
def test_grasp_touch_allowance_never_applies_after_releasing_payload(body_name):
    assert payload_contact_permitted(body_name, released=False)
    assert not payload_contact_permitted(body_name, released=True)


def test_other_robot_links_cannot_touch_payload_before_or_after_release():
    assert not payload_contact_permitted('openarmx_right_link7', released=False)
    assert not payload_contact_permitted('openarmx_right_link7', released=True)
