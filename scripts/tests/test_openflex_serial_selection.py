from pathlib import Path
import sys
from types import SimpleNamespace


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_openflex_roboos_delivery import failure_detail, serial_selection


def test_only_first_model_selected_call_is_dispatched_without_reordering():
    first = SimpleNamespace(id='actual_first', function=SimpleNamespace(name='get_robot_state'))
    second = SimpleNamespace(id='speculative_second', function=SimpleNamespace(name='grasp_object'))
    raw_calls = [first, second]
    message = SimpleNamespace(tool_calls=raw_calls)
    assert serial_selection(message).tool_calls == [first]
    assert raw_calls == [first, second]


def test_no_call_does_not_invent_an_action():
    assert serial_selection(SimpleNamespace(tool_calls=None)).tool_calls is None


def test_taskgroup_failure_keeps_actual_robot_failure_in_summary():
    nested = SimpleNamespace(exceptions=[SimpleNamespace(exceptions=[RuntimeError('Released payload moved')])])
    assert failure_detail(nested) == 'RuntimeError: Released payload moved'
