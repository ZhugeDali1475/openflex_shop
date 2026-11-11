import ast
import asyncio
import base64
from io import BytesIO
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from PIL import Image
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "openflex_ws/src/openflex_thesis/openflex_roboos_bridge"
sys.path.insert(0, str(PACKAGE))

from openflex_roboos_bridge import mcp_server
from openflex_roboos_bridge.snapshot import SnapshotCache, encode_image
from openflex_roboos_bridge.scene_registry import load_scene_registry


@pytest.fixture
def clock():
    return SimpleNamespace(wall=100.0, ros=10_000_000_000)


@pytest.fixture
def cache(clock):
    return SnapshotCache(lambda: clock.ros, lambda: clock.wall)


def test_absent_sources_remain_unknown(cache):
    state = cache.snapshot()
    assert state["status"] == "unavailable"
    assert state["holding"] is None
    assert state["physical_verification"] is False
    assert all(source["data"] is None for source in state["sources"].values())


def test_fresh_telemetry_is_not_task_success(cache, clock):
    cache.update("mission_state", {"phase": 15}, clock.ros)
    result = cache.snapshot()
    assert result["status"] == "partial"
    assert result["sources"]["mission_state"]["data"]["phase"] == 15
    assert "success" not in result
    assert result["holding"] is None


def test_expired_receive_hides_verified_holding(cache, clock):
    cache.update("grasp_status", {"object_id": "item", "verified": True,
                 "contact_detected": True, "gripper_closed": True, "object_retained": True}, clock.ros)
    assert cache.snapshot()["holding"] == "item"
    clock.wall += 3.1
    result = cache.snapshot()
    assert result["holding"] is None
    assert result["sources"]["grasp_status"]["status"] == "stale"
    assert result["sources"]["grasp_status"]["data"] is None


def test_recent_receive_does_not_refresh_old_source(cache, clock):
    cache.update("odometry", {"pose": {}}, clock.ros - 4_000_000_000)
    assert cache.snapshot()["sources"]["odometry"]["status"] == "stale"


def test_repeated_stamp_stales_even_when_ros_clock_freezes(cache, clock):
    cache.update("odometry", {"pose": {}}, clock.ros)
    clock.wall += 4
    cache.update("odometry", {"pose": {}}, clock.ros)
    assert cache.snapshot()["sources"]["odometry"]["status"] == "stale"


@pytest.mark.parametrize("stamp,status", [(0, "clock_unavailable"), (11_000_000_000, "future_timestamp")])
def test_bad_clock_not_fresh(cache, stamp, status):
    cache.update("odometry", {}, stamp)
    assert cache.snapshot()["sources"]["odometry"]["status"] == status


def test_invalid_new_sample_replaces_old_fresh_camera(cache, clock):
    cache.update("camera", {"data": "old"}, clock.ros)
    cache.invalidate("camera", "broken frame")
    camera = cache.snapshot(True)["sources"]["camera"]
    assert camera["status"] == "invalid"
    assert camera["data"] is None


def test_nan_normalized_to_null(cache, clock):
    cache.update("odometry", {"value": float("nan")}, clock.ros)
    assert cache.snapshot()["sources"]["odometry"]["data"]["value"] is None
    json.dumps(cache.snapshot(), allow_nan=False)


def image_message(encoding="bgr8", width=2, height=1, step=8, data=None):
    return SimpleNamespace(encoding=encoding, width=width, height=height, step=step,
                           data=data if data is not None else bytes([0, 0, 255, 0, 0, 255, 0, 0]))


def test_bgr_image_stride_and_real_jpeg():
    encoded = encode_image(image_message())
    image = Image.open(BytesIO(base64.b64decode(encoded["data"])))
    assert image.size == (2, 1)
    red, green, blue = image.getpixel((0, 0))
    assert red > 240 and green < 10 and blue < 10


@pytest.mark.parametrize("changes", [
    {"encoding": "32FC1"}, {"step": 5}, {"height": 0}, {"data": b"bad"},
])
def test_malformed_camera_rejected(changes):
    message = image_message()
    for key, value in changes.items():
        setattr(message, key, value)
    with pytest.raises(ValueError):
        encode_image(message)


def test_large_camera_resized():
    encoded = encode_image(image_message("rgb8", 800, 600, 2400, bytes(800 * 600 * 3)))
    assert (encoded["width"], encoded["height"]) == (640, 480)


def test_only_two_readonly_tools_registered():
    tools = asyncio.run(mcp_server.create_server().list_tools())
    assert {tool.name for tool in tools} == {"get_robot_state", "observe_scene"}
    for tool in tools:
        assert tool.annotations.readOnlyHint is True
        assert tool.annotations.destructiveHint is False
        assert tool.inputSchema.get("properties", {}) == {}


def test_mcp_observation_contains_image_not_local_path(cache, clock, monkeypatch):
    camera = encode_image(image_message())
    cache.update("camera", camera, clock.ros)
    monkeypatch.setattr(mcp_server.SnapshotClient, "read", lambda self, path: cache.snapshot(True))
    result = asyncio.run(mcp_server.create_server().call_tool("observe_scene", {}))
    assert [block.type for block in result] == ["text", "image"]
    metadata = json.loads(result[0].text)
    assert "data" not in metadata["sources"]["camera"]["data"]
    assert result[1].mimeType == "image/jpeg"
    assert base64.b64decode(result[1].data).startswith(b"\xff\xd8")
    assert "oracle" in metadata["provenance"]["object_observation"]


def test_stale_observation_has_no_image(cache, clock, monkeypatch):
    cache.update("camera", encode_image(image_message()), clock.ros)
    clock.wall += 4
    monkeypatch.setattr(mcp_server.SnapshotClient, "read", lambda self, path: cache.snapshot(True))
    result = asyncio.run(mcp_server.create_server().call_tool("observe_scene", {}))
    assert [block.type for block in result] == ["text"]
    assert json.loads(result[0].text)["camera_status"] == "stale"


def test_stopped_ros_snapshot_reports_unavailable(monkeypatch):
    def unavailable(*args, **kwargs):
        raise OSError("Connection refused")

    client = mcp_server.SnapshotClient()
    monkeypatch.setattr(client.opener, "open", unavailable)
    result = client.read("/state")
    assert result["status"] == "unavailable"
    assert result["holding"] is None


def test_no_robot_command_clients_or_publishers_in_bridge():
    source = (PACKAGE / "openflex_roboos_bridge/ros_snapshot.py").read_text()
    tree = ast.parse(source)
    calls = [node.func for node in ast.walk(tree) if isinstance(node, ast.Call)]
    forbidden = {"create_publisher", "create_client", "ActionClient", "ActionServer"}
    assert not any(isinstance(call, ast.Attribute) and call.attr in forbidden for call in calls)
    assert not any(isinstance(call, ast.Name) and call.id in forbidden for call in calls)


@pytest.fixture
def registry_files(tmp_path):
    config = {
        "openflex_thesis_mujoco_bridge": {"ros__parameters": {
            "object_id": "fixture_item", "object_nominal_pose": [1.12, 0, 0.84, 3.141592653589793],
            "delivery_surface_z_m": 0.56,
        }},
        "thesis_mujoco_mission_client": {"ros__parameters": {
            "object_id": "fixture_item", "pickup_observation_pose": [0, 0, 0],
            "delivery_pose": [-0.85, -0.8, 0],
        }},
    }
    model = """<mujoco><worldbody>
      <body name="thesis_delivery_table"><geom name="thesis_delivery_top" type="box"
        pos="-0.85 -0.8 0.455" size="0.35 0.35 0.025"/></body>
      <body name="thesis_object" pos="1.12 0 0.84"><geom name="thesis_object_geom"
        type="box" size="0.035 0.03 0.08"/></body>
    </worldbody></mujoco>"""
    config_path, model_path = tmp_path / "config.yaml", tmp_path / "scene.xml"
    config_path.write_text(yaml.safe_dump(config))
    model_path.write_text(model)
    return config_path, model_path


def test_registry_binds_semantic_ids_without_executable_goals(registry_files):
    result = load_scene_registry(*registry_files)
    assert result["execution_enabled"] is False
    assert result["runtime_geometry_verified"] is False
    assert result["objects"]["fixture_item"]["visual_identity_verified"] is False
    assert set(result["stations"]) == {"pickup_zone", "delivery_zone"}
    assert all(value["navigation_pose"] is None for value in result["stations"].values())


def test_table_center_goal_rejected_and_center_height_not_confused_with_surface(registry_files):
    result = load_scene_registry(*registry_files)
    assert result["stations"]["delivery_zone"]["validation"] == "rejected_inside_table_footprint"
    surface = result["affordances"]["delivery_zone"]
    assert surface["surface_center_xyz"][2] == pytest.approx(0.48)
    assert surface["configured_release_center_z_m"] == pytest.approx(0.56)
    assert surface["expected_upright_object_center_z_m"] == pytest.approx(0.56)
    assert surface["upright_center_height_consistent"] is True
    assert "teleport" in surface["legacy_place_mechanism"]


@pytest.mark.parametrize("change", ["object_id", "object_position", "nonfinite", "parent_transform", "invalid_yaml", "invalid_xml"])
def test_invalid_registry_is_rejected(registry_files, change):
    config_path, model_path = registry_files
    config = yaml.safe_load(config_path.read_text())
    if change == "object_id":
        config["thesis_mujoco_mission_client"]["ros__parameters"]["object_id"] = "wrong"
    elif change == "object_position":
        config["openflex_thesis_mujoco_bridge"]["ros__parameters"]["object_nominal_pose"][0] += 1
    elif change == "nonfinite":
        config["thesis_mujoco_mission_client"]["ros__parameters"]["delivery_pose"][0] = float("nan")
    elif change == "parent_transform":
        tree = ET.parse(model_path)
        tree.find(".//body[@name='thesis_delivery_table']").set("pos", "1 0 0")
        tree.write(model_path)
    config_path.write_text(yaml.safe_dump(config))
    if change == "invalid_yaml":
        config_path.write_text("foo: [broken")
    elif change == "invalid_xml":
        model_path.write_text("<broken")
    with pytest.raises(ValueError):
        load_scene_registry(config_path, model_path)


def test_no_fresh_pose_means_unknown_binding_even_with_static_registry(cache, clock, registry_files):
    cache.set_registry(load_scene_registry(*registry_files))
    assert cache.snapshot()["object_binding"]["status"] == "unknown"
    cache.update("object_observation", {"object_id": "fixture_item"}, clock.ros)
    assert cache.snapshot()["object_binding"]["status"] == "registered"
    assert cache.snapshot()["object_binding"]["visual_identity_verified"] is False
    clock.wall += 4
    assert cache.snapshot()["object_binding"]["status"] == "unknown"


def test_wrong_live_object_is_not_bound_to_registry(cache, clock, registry_files):
    cache.set_registry(load_scene_registry(*registry_files))
    cache.update("object_observation", {"object_id": "unregistered_item"}, clock.ros)
    assert cache.snapshot()["object_binding"]["status"] == "unregistered"
