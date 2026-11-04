import threading
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Image

from openarmx_task_executor.shopping_ticket_reader_node import (
    ShoppingTicketReaderNode,
)


class FakeHandle:
    is_cancel_requested = False


def _reader_with_image(sequence: int, image: Image, timeout: float = 0.01):
    node = object.__new__(ShoppingTicketReaderNode)
    node._state_lock = threading.Lock()
    node._image_sequence = sequence
    node._latest_image = image
    parameters = {
        "camera_wait_timeout_sec": timeout,
        "expected_image_width": 1280,
        "expected_image_height": 720,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    return node


def test_fresh_image_requires_a_frame_after_pitch_settles():
    image = Image()
    image.width = 1280
    image.height = 720
    node = _reader_with_image(8, image)

    frozen = ShoppingTicketReaderNode._fresh_image(node, 7, FakeHandle())

    assert frozen is not image
    assert frozen.width == 1280
    assert frozen.height == 720


def test_fresh_image_rejects_pre_settle_frame():
    image = Image()
    image.width = 1280
    image.height = 720
    node = _reader_with_image(8, image, timeout=0.001)

    with pytest.raises(RuntimeError, match="no fresh head RGB frame"):
        ShoppingTicketReaderNode._fresh_image(node, 8, FakeHandle())


def test_model_future_has_no_local_inference_deadline():
    node = object.__new__(ShoppingTicketReaderNode)

    class DeferredFuture:
        def __init__(self):
            self.polls = 0

        def done(self):
            self.polls += 1
            return self.polls >= 4

        @staticmethod
        def result():
            return "completed"

    future = DeferredFuture()
    result = ShoppingTicketReaderNode._wait_future(
        node, future, FakeHandle()
    )

    assert result == "completed"
    assert future.polls == 4
