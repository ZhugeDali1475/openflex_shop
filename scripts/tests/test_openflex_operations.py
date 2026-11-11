import asyncio
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))

from openflex_roboos_bridge.mcp_server import create_server
from openflex_roboos_bridge.operation_store import OperationStore


@pytest.fixture
def store(tmp_path):
    return OperationStore(tmp_path / "operations.jsonl")


def test_identical_request_is_not_executed_twice(store):
    record, started = store.begin("request_1", "navigate", {"target": "pickup"})
    assert started is True
    repeated, started = store.begin("request_1", "navigate", {"target": "pickup"})
    assert started is False
    assert repeated == record
    store.finish("request_1", "succeeded", success=True)
    repeated, started = store.begin("request_1", "navigate", {"target": "pickup"})
    assert started is False and repeated["success"] is True


def test_busy_different_request_rejected(store):
    store.begin("request_1", "navigate", {})
    result, started = store.begin("request_2", "navigate", {})
    assert started is False and result["status"] == "rejected_busy"


def test_same_id_different_arguments_rejected(store):
    store.begin("request_1", "navigate", {"target": "pickup"})
    with pytest.raises(ValueError):
        store.begin("request_1", "navigate", {"target": "delivery"})


@pytest.mark.parametrize("request_id", ["", "中文", "bad id", "../path", "a" * 81, None, 123])
def test_invalid_request_id_rejected(store, request_id):
    with pytest.raises(ValueError):
        store.begin(request_id, "navigate", {})


def test_unknown_holds_exclusive_lock(store):
    store.begin("request_1", "navigate", {})
    store.finish("request_1", "unknown", success=False)
    assert store.begin("request_2", "navigate", {})[0]["status"] == "rejected_busy"


def test_restart_running_operation_is_unknown_not_reexecuted(store):
    store.begin("request_1", "navigate", {})
    recovered = OperationStore(store.path)
    assert recovered.read("request_1")["status"] == "unknown"
    assert recovered.begin("request_1", "navigate", {})[1] is False
    assert recovered.begin("request_2", "navigate", {})[0]["status"] == "rejected_busy"


def test_cancel_never_claims_success_or_stop(store):
    store.begin("request_1", "navigate", {})
    assert store.cancel("other")["status"] == "not_active"
    result = store.cancel("request_1")
    assert result["status"] == "cancel_requested" and result["success"] is False
    assert store.cancel_event.is_set()
    assert store.read("request_1")["status"] == "running"


def test_event_journal_records_real_transitions(store):
    store.begin("request_1", "navigate", {})
    store.event("request_1", "hold", {"epoch": 2})
    result = store.finish("request_1", "canceled", success=False)
    assert result["events"][0]["data"] == {"epoch": 2}
    assert json.loads(store.path.read_text().splitlines()[-1])["status"] == "canceled"


def test_navigation_tools_are_explicit_opt_in():
    tools = asyncio.run(create_server(enable_navigation=True).list_tools())
    names = {tool.name for tool in tools}
    assert names == {"get_robot_state", "observe_scene", "navigate_to_target", "get_navigation_result", "cancel_current_task"}
    assert "grasp_object" not in names and "place_to_affordance" not in names
    navigate = next(tool for tool in tools if tool.name == "navigate_to_target")
    assert navigate.annotations.readOnlyHint is False
    assert navigate.annotations.destructiveHint is True
    assert navigate.inputSchema["properties"]["target_id"]["enum"] == ["pickup_zone", "delivery_zone"]


def test_manipulation_requires_explicit_shared_gateway():
    with pytest.raises(ValueError):
        create_server(enable_manipulation=True)
    tools = asyncio.run(create_server(enable_navigation=True, enable_manipulation=True).list_tools())
    names = {tool.name for tool in tools}
    assert 'grasp_object' in names and 'get_skill_result' in names
    assert 'place_to_affordance' not in names


def test_placement_requires_separate_explicit_opt_in():
    with pytest.raises(ValueError):
        create_server(enable_placement=True)
    tools = asyncio.run(create_server(enable_navigation=True, enable_manipulation=True, enable_placement=True).list_tools())
    place = next(tool for tool in tools if tool.name == 'place_to_affordance')
    assert place.annotations.destructiveHint is True
    assert place.inputSchema['properties']['affordance_id']['const'] == 'delivery_zone'
