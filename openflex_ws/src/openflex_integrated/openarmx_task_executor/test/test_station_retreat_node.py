from openarmx_task_msgs.action import RetreatFromStation
from rclpy.action import GoalResponse
from rclpy.serialization import deserialize_message, serialize_message

from openarmx_task_executor.station_retreat_node import (
    StationRetreatNode,
    _as_float32,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_successful_retreat_releases_fine_alignment_guard():
    node = object.__new__(StationRetreatNode)
    node.localization_guard_pub = _Publisher()

    node._release_fine_alignment_guard()

    assert len(node.localization_guard_pub.messages) == 1
    assert node.localization_guard_pub.messages[0].data == 0


def test_float32_maximum_retreat_distance_is_accepted():
    request = RetreatFromStation.Goal()
    request.station_id = "SHELF-A"
    request.distance = 0.30
    request.max_speed = 0.03
    request.timeout_sec = 13.0
    request = deserialize_message(
        serialize_message(request), RetreatFromStation.Goal
    )

    node = object.__new__(StationRetreatNode)
    node.maximum_distance = _as_float32(0.30)
    node.maximum_speed = _as_float32(0.06)
    node._active_goal = False

    assert request.distance > 0.30
    assert node.goal_callback(request) == GoalResponse.ACCEPT


def test_retreat_distance_above_float32_maximum_is_rejected():
    request = RetreatFromStation.Goal()
    request.station_id = "SHELF-A"
    request.distance = 0.301
    request.max_speed = 0.03
    request.timeout_sec = 13.0

    node = object.__new__(StationRetreatNode)
    node.maximum_distance = _as_float32(0.30)
    node.maximum_speed = _as_float32(0.06)
    node._active_goal = False

    assert node.goal_callback(request) == GoalResponse.REJECT
