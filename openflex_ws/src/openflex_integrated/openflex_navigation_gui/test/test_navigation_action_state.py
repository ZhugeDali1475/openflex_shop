from types import SimpleNamespace

from openflex_navigation_gui.main_window import NavigationRosNode, Telemetry


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

    def __init__(self):
        self.cancel_future = DeferredFuture(
            SimpleNamespace(goals_canceling=[object()])
        )
        self.result_future = DeferredFuture(SimpleNamespace(status=5))
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


class FakeNavigationNode:
    def __init__(self, status="running", goal_handle=None):
        self.telemetry = Telemetry(action_status=status)
        self.goal_handle = goal_handle
        self._goal_sequence = 1
        self._active_goal_sequence = 1
        self._cancel_requested_sequence = None
        self._cancel_future = None
        self._cancel_deadline = None

    cancel_navigation_goal = NavigationRosNode.cancel_navigation_goal
    _request_goal_cancel = NavigationRosNode._request_goal_cancel
    _cancel_response = NavigationRosNode._cancel_response
    _check_cancel_timeout = NavigationRosNode._check_cancel_timeout
    _clear_goal_state = NavigationRosNode._clear_goal_state
    _goal_feedback = NavigationRosNode._goal_feedback
    _goal_response = NavigationRosNode._goal_response
    _goal_result = NavigationRosNode._goal_result

    CANCEL_RESPONSE_TIMEOUT_SEC = NavigationRosNode.CANCEL_RESPONSE_TIMEOUT_SEC


def test_cancel_acceptance_releases_goal_and_ignores_late_feedback():
    handle = FakeGoalHandle()
    node = FakeNavigationNode(goal_handle=handle)

    node.cancel_navigation_goal()

    assert node.telemetry.action_status == "canceling"
    assert handle.cancel_calls == 1
    feedback = SimpleNamespace(feedback=SimpleNamespace(distance_remaining=3.0))
    node._goal_feedback(feedback, 1)
    assert node.telemetry.action_status == "canceling"

    handle.cancel_future.complete()

    assert node.telemetry.action_status == "canceled"
    assert node.goal_handle is None
    node._goal_feedback(feedback, 1)
    assert node.telemetry.action_status == "canceled"


def test_cancel_while_goal_is_sending_runs_after_acceptance():
    node = FakeNavigationNode(status="sending", goal_handle=None)
    handle = FakeGoalHandle()

    node.cancel_navigation_goal()
    node._goal_response(DeferredFuture(handle), 1)

    assert node.telemetry.action_status == "canceling"
    assert handle.cancel_calls == 1
    handle.cancel_future.complete()
    assert node.telemetry.action_status == "canceled"


def test_cancel_timeout_releases_stale_local_goal():
    handle = FakeGoalHandle()
    node = FakeNavigationNode(goal_handle=handle)

    node.cancel_navigation_goal()
    node._cancel_deadline = 0.0
    node._check_cancel_timeout()

    assert node.telemetry.action_status == "cancel_timeout"
    assert node.goal_handle is None
    assert node._active_goal_sequence == 0


def test_late_result_from_old_goal_does_not_overwrite_new_goal():
    node = FakeNavigationNode(status="running", goal_handle=FakeGoalHandle())
    node._active_goal_sequence = 2
    node.telemetry.action_status = "sending"

    old_result = DeferredFuture(SimpleNamespace(status=5))
    node._goal_result(old_result, 1)

    assert node.telemetry.action_status == "sending"


def test_shelf_alignment_feedback_and_success_are_independent_from_nav_status():
    node = SimpleNamespace(
        telemetry=Telemetry(action_status="succeeded", shelf_align_status="running"),
        shelf_align_goal_handle=object(),
        _active_shelf_align_sequence=3,
    )
    feedback = SimpleNamespace(
        feedback=SimpleNamespace(
            phase="APPROACH",
            measured_clearance=0.072,
            yaw_error=0.01,
            stable_samples=2,
        )
    )
    NavigationRosNode._shelf_align_feedback(node, feedback, 3)
    assert node.telemetry.action_status == "succeeded"
    assert node.telemetry.shelf_clearance == 0.072
    assert node.telemetry.shelf_stable_samples == 2

    payload = SimpleNamespace(
        success=True,
        final_clearance=0.052,
        final_yaw_error=0.012,
        message="complete",
    )
    completed = DeferredFuture(SimpleNamespace(status=4, result=payload))
    node._clear_shelf_align_state = lambda sequence: setattr(
        node, "_active_shelf_align_sequence", 0
    )
    NavigationRosNode._shelf_align_result(node, completed, 3)
    assert node.telemetry.shelf_align_status == "succeeded"
    assert node.telemetry.shelf_clearance == 0.052
    assert node.telemetry.shelf_yaw_error == 0.012


class FakeAuxiliaryActionNode:
    def __init__(self, kind, handle):
        self.telemetry = Telemetry()
        self.mock_pick_goal_handle = handle if kind == "pick" else None
        self._active_pick_test_sequence = 7 if kind == "pick" else 0
        self.retreat_goal_handle = handle if kind == "retreat" else None
        self._active_retreat_test_sequence = 9 if kind == "retreat" else 0

    _clear_pick_test_state = NavigationRosNode._clear_pick_test_state
    _mock_pick_cancel_response = NavigationRosNode._mock_pick_cancel_response
    _clear_retreat_test_state = NavigationRosNode._clear_retreat_test_state
    _retreat_cancel_response = NavigationRosNode._retreat_cancel_response


def test_mock_pick_cancel_releases_action_state():
    handle = FakeGoalHandle()
    node = FakeAuxiliaryActionNode("pick", handle)

    NavigationRosNode.cancel_mock_pick_goal(node)
    assert node.telemetry.pick_test_status == "canceling"
    assert handle.cancel_calls == 1
    handle.cancel_future.complete()

    assert node.telemetry.pick_test_status == "canceled"
    assert node.mock_pick_goal_handle is None
    assert node._active_pick_test_sequence == 0


def test_retreat_cancel_releases_action_state():
    handle = FakeGoalHandle()
    node = FakeAuxiliaryActionNode("retreat", handle)

    NavigationRosNode.cancel_station_retreat_goal(node)
    assert node.telemetry.retreat_test_status == "canceling"
    assert handle.cancel_calls == 1
    handle.cancel_future.complete()

    assert node.telemetry.retreat_test_status == "canceled"
    assert node.retreat_goal_handle is None
    assert node._active_retreat_test_sequence == 0
