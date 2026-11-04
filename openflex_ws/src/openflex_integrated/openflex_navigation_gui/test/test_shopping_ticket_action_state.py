from types import SimpleNamespace

from openflex_navigation_gui.main_window import NavigationRosNode


class DeferredFuture:
    def __init__(self, result):
        self._result = result
        self._callback = None

    def add_done_callback(self, callback):
        self._callback = callback

    def result(self):
        return self._result

    def complete(self):
        assert self._callback is not None
        self._callback(self)


class FakeGoalHandle:
    accepted = True

    def __init__(self, result):
        self.result_future = DeferredFuture(result)
        self.cancel_future = DeferredFuture(
            SimpleNamespace(goals_canceling=[object()])
        )
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class FakeActionClient:
    def __init__(self, goal_handle):
        self.goal_handle = goal_handle
        self.send_future = DeferredFuture(goal_handle)
        self.goal = None

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goal = goal
        return self.send_future


class FakeNode:
    request_shopping_ticket = NavigationRosNode.request_shopping_ticket
    _shopping_ticket_feedback = NavigationRosNode._shopping_ticket_feedback
    _shopping_ticket_response = NavigationRosNode._shopping_ticket_response
    _shopping_ticket_result = NavigationRosNode._shopping_ticket_result
    cancel_shopping_ticket = NavigationRosNode.cancel_shopping_ticket
    _request_shopping_ticket_cancel = (
        NavigationRosNode._request_shopping_ticket_cancel
    )
    _shopping_ticket_cancel_response = (
        NavigationRosNode._shopping_ticket_cancel_response
    )
    _clear_shopping_ticket_state = NavigationRosNode._clear_shopping_ticket_state

    def __init__(self, goal_handle):
        self.read_order_action_client = FakeActionClient(goal_handle)
        self.shopping_ticket_goal_handle = None
        self._shopping_ticket_sequence = 0
        self._active_shopping_ticket_sequence = 0
        self._shopping_ticket_cancel_requested_sequence = None
        self.shopping_ticket_status = "idle"
        self.shopping_ticket_phase = ""
        self.shopping_ticket_message = ""
        self.shopping_ticket_lines = ()


def test_read_order_waits_for_result_and_preserves_zero_timeout():
    line = SimpleNamespace(sku="milk", quantity=1)
    payload = SimpleNamespace(success=True, lines=[line], message="ok")
    handle = FakeGoalHandle(SimpleNamespace(status=4, result=payload))
    node = FakeNode(handle)

    assert node.request_shopping_ticket("delivery_counter")
    assert node.read_order_action_client.goal.timeout_sec == 0.0
    assert node.shopping_ticket_status == "sending"

    node.read_order_action_client.send_future.complete()
    assert node.shopping_ticket_status == "running"
    # The result is deliberately completed later; there is no local 5-second
    # fallback state that can turn this into a manual order.
    node.shopping_ticket_goal_handle.result_future.complete()

    assert node.shopping_ticket_status == "succeeded"
    assert node.shopping_ticket_lines[0].sku == "milk"
    assert node._active_shopping_ticket_sequence == 0


def test_cancel_before_goal_acceptance_cancels_read_order():
    payload = SimpleNamespace(success=False, lines=[], message="cancelled")
    handle = FakeGoalHandle(SimpleNamespace(status=5, result=payload))
    node = FakeNode(handle)

    assert node.request_shopping_ticket("delivery_counter")
    node.cancel_shopping_ticket()
    assert node.shopping_ticket_status == "canceling"

    node.read_order_action_client.send_future.complete()
    assert handle.cancel_calls == 1
    handle.cancel_future.complete()

    assert node.shopping_ticket_status == "canceled"
    assert node._active_shopping_ticket_sequence == 0
