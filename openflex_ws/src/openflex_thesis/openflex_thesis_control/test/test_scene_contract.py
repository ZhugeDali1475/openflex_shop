import asyncio
from copy import deepcopy
import math
from types import SimpleNamespace
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
import pytest

from openflex_thesis_control.moveit_grasp_executor_node import MoveItGraspExecutorNode
from openflex_thesis_control.scene_contract import (
    primitive_pose,
    scene_readback_matches,
    shelf_collision_objects,
)


def scene():
    return {"frame_id": "map", "collision_objects": [
        {"id": "board", "shape": "box", "dimensions": [0.5, 1.3, 0.036],
         "position": [1.24, 0.0, 1.18], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}
    ]}


def transform():
    output = TransformStamped()
    output.transform.translation.x = -0.4
    output.transform.translation.z = -0.17
    output.transform.rotation.w = 1.0
    return output


def test_shelf_geometry_uses_measured_translation_and_mount_height():
    boxes = shelf_collision_objects(scene(), transform(), "base_link")
    pose = primitive_pose(boxes[0])
    assert pose.header.frame_id == "base_link"
    assert pose.pose.position.x == pytest.approx(0.84)
    assert pose.pose.position.z == pytest.approx(1.01)


def test_shelf_geometry_applies_measured_rotation():
    measured = transform()
    measured.transform.rotation.z = math.sin(math.pi / 4)
    measured.transform.rotation.w = math.cos(math.pi / 4)
    box = shelf_collision_objects(scene(), measured, "base_link")[0]
    assert primitive_pose(box).pose.position.y == pytest.approx(1.24)


@pytest.mark.parametrize("field,value", [
    ("dimensions", [0.0, 1.0, 1.0]), ("dimensions", [1.0, 1.0]),
    ("position", [float("nan"), 0.0, 0.0]),
    ("orientation_xyzw", [0.0, 0.0, 0.0, 0.0]), ("shape", "mesh"), ("id", ""),
])
def test_invalid_geometry_fails_closed(field, value):
    configuration = scene()
    configuration["collision_objects"][0][field] = value
    with pytest.raises(ValueError):
        shelf_collision_objects(configuration, transform(), "base_link")


def test_duplicate_scene_ids_fail_closed():
    configuration = scene()
    configuration["collision_objects"] *= 2
    with pytest.raises(ValueError):
        shelf_collision_objects(configuration, transform(), "base_link")


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize("component", ["translation", "rotation"])
def test_nonfinite_measured_transform_fails_closed(value, component):
    measured = transform()
    getattr(measured.transform, component).x = value
    with pytest.raises(ValueError):
        shelf_collision_objects(scene(), measured, "base_link")


@pytest.mark.parametrize("component", ["pose", "primitive"])
def test_nonfinite_readback_quaternion_is_rejected(component):
    expected = shelf_collision_objects(scene(), transform(), "base_link")
    observed = deepcopy(expected)
    pose = observed[0].pose if component == "pose" else observed[0].primitive_poses[0]
    pose.orientation.w = float("nan")
    assert not scene_readback_matches(expected, observed, lambda pose: pose)


def test_scene_readback_requires_geometry_not_just_object_ids():
    expected = shelf_collision_objects(scene(), transform(), "base_link")
    assert scene_readback_matches(expected, deepcopy(expected), lambda pose: pose)
    assert not scene_readback_matches(expected, [], lambda pose: pose)
    observed = deepcopy(expected)
    observed[0].primitives[0].dimensions[0] += 0.1
    assert not scene_readback_matches(expected, observed, lambda pose: pose)
    observed = deepcopy(expected)
    observed[0].primitive_poses[0].position.x += 0.1
    assert not scene_readback_matches(expected, observed, lambda pose: pose)


def test_scene_readback_accepts_equivalent_quaternion_sign():
    expected = shelf_collision_objects(scene(), transform(), "base_link")
    observed = deepcopy(expected)
    observed[0].primitive_poses[0].orientation.w = -1.0
    assert scene_readback_matches(expected, observed, lambda pose: pose)


def test_scene_readback_composes_object_and_primitive_poses():
    expected = shelf_collision_objects(scene(), transform(), "base_link")
    observed = deepcopy(expected)
    observed[0].pose.position.x = 0.1
    observed[0].primitive_poses[0].position.x -= 0.1
    assert scene_readback_matches(expected, observed, lambda pose: pose)


def test_missing_scene_prevents_motion_request():
    async def unavailable():
        return False
    executor = SimpleNamespace(hardware_guard_valid=lambda: True, apply_shelf_scene=unavailable)
    assert not asyncio.run(MoveItGraspExecutorNode.move_to_state(executor, "right_arm_with_lift", RobotState()))
    assert executor.last_move_failure == "scene_unverified"


def test_current_and_goal_validity_checks_include_inactive_arm():
    requests = []
    async def checked(request):
        requests.append(request)
        return GetStateValidity.Response(valid=True)
    executor = SimpleNamespace(
        joint_diagnostic_sample=(None, 0.0), simulation_diagnostic_sample=(None, 0.0),
        validity_client=SimpleNamespace(call_async=checked),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        active_object_id="item", active_map_version=1, failure_stage="pregrasp",
        goal_target_diagnostic=None, object_memory=None, object_memory_receipt=time.monotonic(),
        attachment_diagnostic=None,
    )
    valid, detail = asyncio.run(MoveItGraspExecutorNode.transportation_start_valid(
        executor, "right_arm_with_lift", RobotState()))
    assert valid and not detail
    assert len(requests) == 2
    assert all(request.group_name == "" for request in requests)
