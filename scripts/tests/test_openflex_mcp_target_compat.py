from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"))
from openflex_roboos_bridge.runtime_target_compat import (  # noqa: E402
    navigate_compat,
    place_compat,
    resolve_navigation_target,
)


class RecordingClient:
    def __init__(self):
        self.calls = []

    def navigation(self, route, arguments):
        self.calls.append((route, arguments))
        return {"status": "running", "success": False}


def test_left_canonical_target_maps_to_legacy_station():
    client = RecordingClient()
    result = navigate_compat(client, "sim_delivery_top_left", "req-left", 12)
    assert result["status"] == "running"
    assert client.calls == [("/navigate", {"target_id": "delivery_zone",
                                             "request_id": "req-left", "timeout_sec": 12})]


def test_right_target_is_rejected_without_downstream_request():
    client = RecordingClient()
    result = navigate_compat(client, "sim_delivery_top_right", "req-right")
    assert result["status"] == "failed"
    assert result["downstream_called"] is False
    assert "not runtime route-validated" in result["error"]
    assert client.calls == []


def test_right_affordance_is_rejected_without_place_request():
    client = RecordingClient()
    result = place_compat(client, "thesis_item_01", "sim_delivery_top_right", "req-place")
    assert result["status"] == "failed"
    assert result["downstream_called"] is False
    assert client.calls == []


def test_left_affordance_maps_to_legacy_place_contract():
    client = RecordingClient()
    place_compat(client, "thesis_item_01", "sim_delivery_top_left", "req-place", 30)
    assert client.calls == [("/place", {"object_id": "thesis_item_01",
                                          "affordance_id": "delivery_zone",
                                          "request_id": "req-place", "timeout_sec": 30})]


@pytest.mark.parametrize("target", ["free_coordinate", "delivery_zone_b"])
def test_unknown_target_is_rejected_before_network(target):
    client = RecordingClient()
    result = navigate_compat(client, target, "req-invalid")
    assert result["status"] == "failed"
    assert client.calls == []


def test_explicit_mapping_function_rejects_right():
    assert resolve_navigation_target("sim_delivery_top_left") == "delivery_zone"
    with pytest.raises(ValueError, match="not runtime route-validated"):
        resolve_navigation_target("sim_delivery_top_right")
