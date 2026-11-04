import json
from types import SimpleNamespace

from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QFileDialog

from openflex_navigation_gui.media_store import MediaStore
from openflex_navigation_gui.web_bridge import WebSceneBridge


class ImmediateFuture:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._response


class FakeClient:
    def __init__(self, response):
        self.response = response
        self.request = None

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.request = request
        return ImmediateFuture(self.response)


class FakeService:
    class Request:
        pass


class FakeBatchService:
    class Request:
        def __init__(self):
            self.skus = []


class FakeImportSku:
    pass


class FakeCompressedImage:
    pass


class FakeMediaStore:
    def bytes(self, asset_id):
        assert asset_id == "asset-1"
        return b"compressed", "image/jpeg"


def bridge_response(bridge):
    values = []
    bridge.response.connect(
        lambda request_id, success, payload: values.append(
            (request_id, success, json.loads(payload))
        )
    )
    return values


def test_inspection_request_carries_visible_gui_config():
    bridge = WebSceneBridge(None)
    bridge.media_store = FakeMediaStore()
    bridge._compressed_image_type = FakeCompressedImage
    bridge._types["inspect_shelf"] = FakeService
    client = FakeClient(SimpleNamespace(
        success=True,
        error="",
        observed_scene_revision=12,
        shelf_id=1,
        face=0,
        anomalies=[],
    ))
    bridge._clients["inspect_shelf"] = client

    bridge._inspect_shelf("request-1", {
        "asset_id": "asset-1",
        "config": {
            "min_current_coverage": 0.11,
            "analysis_center_ratio": 0.71,
            "lab_distance_threshold": 18,
            "slot_change_ratio_threshold": 0.21,
            "dino_confidence_threshold": 0.82,
            "ambiguity_margin": 0.03,
            "vlm_fallback": True,
            "vlm_top_k": 7,
        },
    })

    request = client.request
    assert request.use_request_config is True
    assert request.min_current_coverage == 0.11
    assert request.analysis_center_ratio == 0.71
    assert request.lab_distance_threshold == 18
    assert request.slot_change_ratio_threshold == 0.21
    assert request.dino_confidence_threshold == 0.82
    assert request.ambiguity_margin == 0.03
    assert request.vlm_fallback is True
    assert request.vlm_top_k == 7


def test_sku_query_request_carries_provider_model_and_config():
    bridge = WebSceneBridge(None)
    bridge.media_store = FakeMediaStore()
    bridge._compressed_image_type = FakeCompressedImage
    bridge._types["find_sku"] = FakeService
    client = FakeClient(SimpleNamespace(
        success=True,
        error="",
        sku="sprite",
        reference_slot_id="1-0-1-20",
        provider="dashscope",
        model="qwen3-vl-plus",
        request_seconds=0.5,
        total_seconds=0.7,
        boxes=[],
    ))
    bridge._clients["find_sku"] = client

    bridge._find_sku("request-2", {
        "asset_id": "asset-1",
        "query": "sprite",
        "provider": "dashscope",
        "model": "qwen3-vl-plus",
        "config": {
            "max_boxes": 6,
            "dino_fallback": True,
            "dino_confidence_threshold": 0.83,
            "owlv2_score_threshold": 0.24,
        },
    })

    request = client.request
    assert request.use_request_options is True
    assert request.provider == "dashscope"
    assert request.model == "qwen3-vl-plus"
    assert request.max_boxes == 6
    assert request.dino_fallback is True
    assert request.dino_confidence_threshold == 0.83
    assert request.owlv2_score_threshold == 0.24


def test_mutation_refreshes_snapshot_and_reports_revision_conflict():
    bridge = WebSceneBridge(None)
    snapshots = []
    bridge._snapshot = lambda request_id, details=None, slot_id="": snapshots.append(
        (request_id, details, slot_id)
    )
    bridge._clients["update_slot"] = FakeClient(SimpleNamespace(success=True, error=""))

    bridge._mutate("request-3", "update_slot", object(), {"changed": True}, "slot-1")

    assert snapshots == [("request-3", {"changed": True}, "slot-1")]

    responses = bridge_response(bridge)
    bridge._clients["update_slot"] = FakeClient(SimpleNamespace(
        success=False,
        error="scene revision conflict: expected 2, current 3",
    ))
    bridge._mutate("request-4", "update_slot", object())

    assert responses == [(
        "request-4",
        False,
        {"error": "scene revision conflict: expected 2, current 3"},
    )]


def test_nullable_slot_update_maps_to_explicit_ros_flags():
    bridge = WebSceneBridge(None)
    bridge._types["update_slot"] = FakeService
    captured = []
    bridge._mutate = lambda request_id, service_name, request, details=None, slot_id="": (
        captured.append((service_name, request, details))
    )

    bridge._dispatch("request-5", "PUT", "/api/slots/1-0-1-20", {
        "actual_sku": None,
        "width_cm": None,
        "height_cm": 24.5,
        "image_dir": "",
    })

    service_name, request, details = captured[0]
    assert service_name == "update_slot"
    assert request.update_actual_sku is True
    assert request.actual_sku_is_null is True
    assert request.update_width_cm is True
    assert request.width_cm_is_null is True
    assert request.update_height_cm is True
    assert request.height_cm_is_null is False
    assert request.height_cm == 24.5
    assert request.update_image_dir is True
    assert request.image_dir == ""
    assert details == {"_slot_id": "1-0-1-20"}


def test_sku_batch_maps_qwen_metadata_to_one_ros_request():
    bridge = WebSceneBridge(None)
    bridge._types["upsert_skus_batch"] = FakeBatchService
    bridge._import_sku_type = FakeImportSku
    captured = []
    bridge._mutate = lambda request_id, service_name, request, details=None, slot_id="": (
        captured.append((service_name, request))
    )

    bridge._dispatch("request-batch", "POST", "/api/skus/batch", {"skus": [{
        "sku": "绿豆冰沙",
        "category": "饮料",
        "mesh_file": "",
        "tex_file": "",
        "owlv2_prompt": "",
        "qwen_grounding_prompt": "绿色；绿豆；杯装",
        "reference_image_path": "data/sku_images/绿豆冰沙.png",
        "grasp_method": "夹爪",
    }]})

    service_name, request = captured[0]
    assert service_name == "upsert_skus_batch"
    assert len(request.skus) == 1
    assert request.skus[0].sku == "绿豆冰沙"
    assert request.skus[0].qwen_grounding_prompt == "绿色；绿豆；杯装"
    assert request.skus[0].grasp_method == "夹爪"


def test_media_store_returns_metadata_without_absolute_path(tmp_path, monkeypatch):
    path = tmp_path / "selected.png"
    image = QImage(4, 4, QImage.Format_RGB32)
    image.fill(0x228844)
    assert image.save(str(path), "PNG")
    monkeypatch.setattr(
        QFileDialog,
        "getOpenFileName",
        lambda *args, **kwargs: (str(path), ""),
    )

    store = MediaStore()
    asset = store.choose(False)[0]

    assert asset["original_name"] == "selected.png"
    assert "data" not in asset
    assert "path" not in asset
    assert str(tmp_path) not in json.dumps(asset)
    assert store.bytes(asset["asset_id"])[0] == path.read_bytes()
