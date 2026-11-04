"""QWebChannel bridge from the embedded React manager to typed ROS services."""

from __future__ import annotations

import json
import re
import uuid
from urllib.parse import unquote

from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot

from .scene_client import SceneRosClient
from .media_store import MediaStore


class WebSceneBridge(QObject):
    response = pyqtSignal(str, bool, str)

    def __init__(self, ros_node, parent=None) -> None:
        super().__init__(parent)
        self.ros_node = ros_node
        self.scene_client = SceneRosClient(ros_node) if ros_node is not None else None
        self._pending = set()
        self._clients = {}
        self._types = {}
        self.media_store = MediaStore(parent)
        self._vision_runs = {}
        self._vision_config = {
            "min_current_coverage": 0.05,
            "analysis_center_ratio": 0.8,
            "lab_distance_threshold": 12.0,
            "slot_change_ratio_threshold": 0.15,
            "dino_confidence_threshold": 0.72,
            "ambiguity_margin": 0.05,
            "vlm_fallback": False,
            "vlm_top_k": 4,
        }
        self._sku_query_config = {
            "max_boxes": 1,
            "dino_fallback": False,
            "dino_confidence_threshold": 0.72,
            "owlv2_score_threshold": 0.10,
        }
        if ros_node is not None:
            self._create_clients()

    def _create_clients(self) -> None:
        from supermarket_scene_interfaces.srv import (
            ApplyInspectionResults,
            ClearShelfInventory,
            CreateDeliveryTable,
            CreateShelf,
            CreateShelfType,
            CreateSku,
            CreateSlot,
            DeleteDeliveryTable,
            DeleteShelf,
            DeleteShelfType,
            DeleteSku,
            DeleteSlot,
            FindSku,
            InspectShelf,
            RestockSlot,
            TakeSlot,
            UpdateDeliveryTable,
            UpdateShelf,
            UpdateShelfType,
            UpdateSku,
            UpdateSlot,
            UpsertSkusBatch,
        )

        definitions = {
            "create_shelf": CreateShelf,
            "update_shelf": UpdateShelf,
            "delete_shelf": DeleteShelf,
            "create_delivery_table": CreateDeliveryTable,
            "update_delivery_table": UpdateDeliveryTable,
            "delete_delivery_table": DeleteDeliveryTable,
            "create_shelf_type": CreateShelfType,
            "update_shelf_type": UpdateShelfType,
            "delete_shelf_type": DeleteShelfType,
            "create_sku": CreateSku,
            "update_sku": UpdateSku,
            "upsert_skus_batch": UpsertSkusBatch,
            "delete_sku": DeleteSku,
            "create_slot": CreateSlot,
            "update_slot": UpdateSlot,
            "delete_slot": DeleteSlot,
            "take_slot": TakeSlot,
            "restock_slot": RestockSlot,
            "clear_shelf_inventory": ClearShelfInventory,
            "apply_inspection_results": ApplyInspectionResults,
            "inspect_shelf": InspectShelf,
            "find_sku": FindSku,
        }
        from sensor_msgs.msg import CompressedImage
        from supermarket_scene_interfaces.msg import ImportSku, SlotObservation

        self._compressed_image_type = CompressedImage
        self._slot_observation_type = SlotObservation
        self._import_sku_type = ImportSku
        self._types = definitions
        self._clients = {
            name: self.ros_node.create_client(service_type, name)
            for name, service_type in definitions.items()
        }

    @staticmethod
    def _json(value) -> str:
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))

    def _emit_error(self, request_id: str, error) -> None:
        self.response.emit(request_id, False, self._json({"error": str(error)}))

    def _track(self, future, callback) -> None:
        self._pending.add(future)

        def finished(value):
            self._pending.discard(value)
            callback(value)

        future.add_done_callback(finished)

    @staticmethod
    def _dashboard_state(response) -> dict:
        state = SceneRosClient.snapshot_from_response(response)
        slots = state["slots"]
        state["stats"] = {
            "shelf_types": len(state["shelf_types"]),
            "shelf_groups": len(state["shelves"]),
            "delivery_tables": len(state["delivery_tables"]),
            "sku_catalog": len(state["skus"]),
            "total_positions": len(slots),
            "actual_items": sum(item["actual_sku"] is not None for item in slots),
            "shortages": sum(item["actual_sku"] is None for item in slots),
            "misplacements": sum(
                item["actual_sku"] is not None
                and item["actual_sku"] != item["expected_sku"]
                for item in slots
            ),
        }
        state["shelf_images"] = []
        state["delivery_table_spec"] = {
            "length": 1.2,
            "width": 0.8,
            "height": 0.75,
            "top_thickness": 0.03,
        }
        return state

    def _snapshot(self, request_id: str, details=None, slot_id: str = "") -> None:
        if self.scene_client is None:
            self._emit_error(request_id, "ROS 场景客户端不可用")
            return
        if not self.scene_client.snapshot_service_ready:
            self._emit_error(request_id, "get_scene_snapshot 服务尚未就绪")
            return
        future = self.scene_client.request_snapshot()

        def finished(value):
            try:
                state = self._dashboard_state(value.result())
                result = dict(details or {})
                if details is None:
                    result = state
                else:
                    result["state"] = state
                    if slot_id:
                        result["slot"] = next(
                            (item for item in state["slots"] if item["slot_id"] == slot_id),
                            None,
                        )
                self.response.emit(request_id, True, self._json(result))
            except Exception as error:
                self._emit_error(request_id, error)

        self._track(future, finished)

    def _slot_world_position(self, request_id: str, slot_id: str) -> None:
        if self.scene_client is None or not self.scene_client.snapshot_service_ready:
            self._emit_error(request_id, "get_scene_snapshot 服务尚未就绪")
            return
        future = self.scene_client.request_snapshot()

        def finished(value):
            try:
                state = self._dashboard_state(value.result())
                slot = next(
                    (item for item in state["slots"] if item["slot_id"] == slot_id),
                    None,
                )
                if slot is None:
                    raise ValueError(f"Slot {slot_id} does not exist")
                self.response.emit(
                    request_id,
                    True,
                    self._json({"slot": {**slot, "frame": "map"}}),
                )
            except Exception as error:
                self._emit_error(request_id, error)

        self._track(future, finished)

    def _mutate(self, request_id: str, service_name: str, request,
                details=None, slot_id: str = "") -> None:
        client = self._clients.get(service_name)
        if client is None or not client.service_is_ready():
            self._emit_error(request_id, f"{service_name} 服务尚未就绪")
            return
        future = client.call_async(request)

        def finished(value):
            try:
                response = value.result()
                if not response.success:
                    raise RuntimeError(response.error)
                resolved = details(response) if callable(details) else dict(details or {})
                self._snapshot(request_id, resolved, slot_id or resolved.pop("_slot_id", ""))
            except Exception as error:
                self._emit_error(request_id, error)

        self._track(future, finished)

    def _request(self, name: str):
        return self._types[name].Request()

    def _compressed_image(self, asset_id: str):
        data, mime_type = self.media_store.bytes(asset_id)
        message = self._compressed_image_type()
        message.format = mime_type
        message.data = list(data)
        return message

    def _inspect_shelf(self, request_id: str, payload: dict) -> None:
        if not payload.get("asset_id"):
            raise ValueError("嵌入模式必须通过 Qt 文件选择器选择巡检图片")
        client = self._clients["inspect_shelf"]
        if not client.service_is_ready():
            raise RuntimeError("inspect_shelf 视觉服务尚未就绪")
        request = self._request("inspect_shelf")
        request.image = self._compressed_image(payload["asset_id"])
        config = {**self._vision_config, **payload.get("config", {})}
        request.use_request_config = True
        for field in (
            "min_current_coverage", "analysis_center_ratio",
            "lab_distance_threshold", "slot_change_ratio_threshold",
            "dino_confidence_threshold", "ambiguity_margin",
        ):
            setattr(request, field, float(config[field]))
        request.vlm_fallback = bool(config["vlm_fallback"])
        request.vlm_top_k = int(config["vlm_top_k"])
        future = client.call_async(request)

        def finished(value):
            try:
                response = value.result()
                if not response.success:
                    raise RuntimeError(response.error)
                run_id = uuid.uuid4().hex
                rows = []
                for item in response.anomalies:
                    slot = item.slot
                    rows.append({
                        "slot_id": slot.slot_id,
                        "expected_sku": slot.expected_sku,
                        "actual_sku": slot.actual_sku if slot.has_actual_sku else None,
                        "status": slot.status,
                        "source": item.source,
                        "confidence": item.confidence if item.has_confidence else None,
                        "reason": item.reason,
                        "selected": True,
                        "difference_ratio": 0.0,
                    })
                report = {
                    "run_id": run_id,
                    "shelf_id": response.shelf_id,
                    "face": response.face,
                    "slots": rows,
                    "artifacts": {},
                }
                self._vision_runs[run_id] = {
                    "scene_revision": response.observed_scene_revision,
                    "rows": rows,
                }
                self.response.emit(
                    request_id, True, self._json({"report": report})
                )
            except Exception as error:
                self._emit_error(request_id, error)

        self._track(future, finished)

    def _apply_inspection(self, request_id: str, run_id: str, payload: dict) -> None:
        run = self._vision_runs.get(run_id)
        if run is None:
            raise ValueError("巡检结果不存在或已经失效")
        selected = set(map(str, payload.get("slot_ids", [])))
        rows = [row for row in run["rows"] if row["slot_id"] in selected]
        if not rows:
            raise ValueError("没有选中的巡检结果")
        request = self._request("apply_inspection_results")
        request.expected_scene_revision = run["scene_revision"]
        for row in rows:
            observation = self._slot_observation_type()
            observation.slot_id = row["slot_id"]
            observation.has_actual_sku = row["actual_sku"] is not None
            observation.actual_sku = row["actual_sku"] or ""
            request.observations.append(observation)
        self._mutate(
            request_id,
            "apply_inspection_results",
            request,
            {"slots": rows},
        )

    def _find_sku(self, request_id: str, payload: dict) -> None:
        if not payload.get("asset_id"):
            raise ValueError("嵌入模式必须通过 Qt 文件选择器选择查询图片")
        client = self._clients["find_sku"]
        if not client.service_is_ready():
            raise RuntimeError("find_sku 视觉服务尚未就绪")
        request = self._request("find_sku")
        request.image = self._compressed_image(payload["asset_id"])
        request.query = str(payload.get("query", ""))
        config = {**self._sku_query_config, **payload.get("config", {})}
        request.use_request_options = True
        request.provider = str(payload.get("provider", "ark"))
        request.model = str(payload.get("model", ""))
        request.max_boxes = int(config["max_boxes"])
        request.dino_fallback = bool(config["dino_fallback"])
        request.dino_confidence_threshold = float(
            config["dino_confidence_threshold"]
        )
        request.owlv2_score_threshold = float(config["owlv2_score_threshold"])
        future = client.call_async(request)

        def finished(value):
            try:
                response = value.result()
                if not response.success:
                    raise RuntimeError(response.error)
                boxes = [
                    {
                        "index": item.index,
                        "label": item.sku,
                        "normalized": [0, 0, 0, 0],
                        "pixels": [item.x, item.y, item.x + item.width, item.y + item.height],
                    }
                    for item in response.boxes
                ]
                report = {
                    "query": request.query,
                    "sku": response.sku,
                    "reference_slot_id": response.reference_slot_id,
                    "provider": response.provider,
                    "model": response.model,
                    "request_seconds": response.request_seconds,
                    "total_seconds": response.total_seconds,
                    "detected_boxes": boxes,
                    "config": config,
                    "artifacts": {},
                }
                self.response.emit(
                    request_id, True, self._json({"report": report})
                )
            except Exception as error:
                self._emit_error(request_id, error)

        self._track(future, finished)

    @staticmethod
    def _set_shelf_type(request, payload: dict) -> None:
        for field in (
            "shelf_length", "shelf_width", "shelf_height", "num_levels",
            "bottom_clearance", "level_spacing", "panel_thick", "back_thick",
            "shelf_depth_normal", "shelf_depth_bottom",
        ):
            setattr(request, field, payload[field])

    @pyqtSlot(str, str, str, str)
    def invoke(self, request_id: str, method: str, path: str, payload_json: str) -> None:
        try:
            payload = json.loads(payload_json) if payload_json else {}
            if not isinstance(payload, dict):
                raise ValueError("请求内容必须是 JSON 对象")
            self._dispatch(request_id, method.upper(), path, payload)
        except Exception as error:
            self._emit_error(request_id, error)

    def _dispatch(self, request_id: str, method: str, path: str, payload: dict) -> None:
        if method == "POST" and path == "/api/media/choose":
            assets = self.media_store.choose(bool(payload.get("multiple", False)))
            self.response.emit(request_id, True, self._json({"assets": assets}))
            return

        if path == "/api/vision/config" and method == "GET":
            self.response.emit(request_id, True, self._json(self._vision_config))
            return
        if path == "/api/vision/config" and method == "PUT":
            self._vision_config.update(payload)
            self.response.emit(
                request_id, True, self._json({"inspection": self._vision_config})
            )
            return
        if path == "/api/vision/inspect" and method == "POST":
            self._inspect_shelf(request_id, payload)
            return
        match = re.fullmatch(r"/api/vision/runs/([A-Za-z0-9_-]+)/apply", path)
        if match and method == "POST":
            self._apply_inspection(request_id, match.group(1), payload)
            return

        if path == "/api/sku-query/config" and method == "GET":
            self.response.emit(request_id, True, self._json(self._sku_query_config))
            return
        if path == "/api/sku-query/config" and method == "PUT":
            self._sku_query_config.update(payload)
            self.response.emit(
                request_id, True, self._json({"sku_query": self._sku_query_config})
            )
            return
        if path == "/api/sku-query" and method == "POST":
            self._find_sku(request_id, payload)
            return

        if method == "GET" and path == "/api/state":
            self._snapshot(request_id)
            return

        match = re.fullmatch(r"/api/slots/(.+)/world-position", path)
        if method == "GET" and match:
            self._slot_world_position(request_id, unquote(match.group(1)))
            return

        if method == "POST" and path == "/api/shelves":
            request = self._request("create_shelf")
            request.name = str(payload.get("name", "New shelf"))
            request.world_x = float(payload.get("world_x", 0.0))
            request.world_y = float(payload.get("world_y", 0.0))
            request.yaw = float(payload.get("yaw", 0.0))
            request.has_shelf_type_id = payload.get("shelf_type_id") is not None
            request.shelf_type_id = int(payload.get("shelf_type_id") or 0)
            self._mutate(
                request_id, "create_shelf", request,
                lambda response: {"id": response.shelf_id},
            )
            return

        match = re.fullmatch(r"/api/shelves/(\d+)/inventory", path)
        if method == "DELETE" and match:
            request = self._request("clear_shelf_inventory")
            request.shelf_id = int(match.group(1))
            request.scope = str(payload.get("scope", ""))
            request.face = int(payload.get("face", 0))
            request.level = int(payload.get("level", 0))
            self._mutate(
                request_id, "clear_shelf_inventory", request,
                lambda response: {"removed": response.removed_slots},
            )
            return

        match = re.fullmatch(r"/api/shelves/(\d+)", path)
        if match and method == "PUT":
            request = self._request("update_shelf")
            request.shelf_id = int(match.group(1))
            request.update_name = "name" in payload
            request.name = str(payload.get("name", ""))
            request.update_pose = any(key in payload for key in ("world_x", "world_y", "yaw"))
            request.world_x = float(payload.get("world_x", 0.0))
            request.world_y = float(payload.get("world_y", 0.0))
            request.yaw = float(payload.get("yaw", 0.0))
            request.update_shelf_type_id = "shelf_type_id" in payload
            request.has_shelf_type_id = payload.get("shelf_type_id") is not None
            request.shelf_type_id = int(payload.get("shelf_type_id") or 0)
            self._mutate(request_id, "update_shelf", request)
            return
        if match and method == "DELETE":
            request = self._request("delete_shelf")
            request.shelf_id = int(match.group(1))
            self._mutate(
                request_id, "delete_shelf", request,
                lambda response: {"removed": response.removed_slots},
            )
            return

        if method == "POST" and path == "/api/delivery-tables":
            request = self._request("create_delivery_table")
            request.name = str(payload.get("name", "新交付桌"))
            for field in ("world_x", "world_y", "yaw"):
                setattr(request, field, float(payload.get(field, 0.0)))
            request.has_dimensions = all(
                field in payload for field in ("length", "width", "height", "top_thickness")
            )
            for field, default in (
                ("length", 1.2), ("width", 0.8), ("height", 0.75),
                ("top_thickness", 0.03),
            ):
                setattr(request, field, float(payload.get(field, default)))
            self._mutate(
                request_id, "create_delivery_table", request,
                lambda response: {"id": response.table_id},
            )
            return

        match = re.fullmatch(r"/api/delivery-tables/(\d+)", path)
        if match and method == "PUT":
            request = self._request("update_delivery_table")
            request.table_id = int(match.group(1))
            request.update_name = "name" in payload
            request.name = str(payload.get("name", ""))
            request.update_pose = any(key in payload for key in ("world_x", "world_y", "yaw"))
            for field in ("world_x", "world_y", "yaw"):
                setattr(request, field, float(payload.get(field, 0.0)))
            request.update_dimensions = any(
                field in payload for field in ("length", "width", "height", "top_thickness")
            )
            for field, default in (
                ("length", 1.2), ("width", 0.8), ("height", 0.75),
                ("top_thickness", 0.03),
            ):
                setattr(request, field, float(payload.get(field, default)))
            self._mutate(request_id, "update_delivery_table", request)
            return
        if match and method == "DELETE":
            request = self._request("delete_delivery_table")
            request.table_id = int(match.group(1))
            self._mutate(request_id, "delete_delivery_table", request)
            return

        if method == "POST" and path == "/api/shelf-types":
            request = self._request("create_shelf_type")
            request.name = str(payload.get("name", ""))
            self._set_shelf_type(request, payload)
            self._mutate(
                request_id, "create_shelf_type", request,
                lambda response: {"id": response.shelf_type_id},
            )
            return

        match = re.fullmatch(r"/api/shelf-types/(\d+)", path)
        if match and method == "PUT":
            request = self._request("update_shelf_type")
            request.shelf_type_id = int(match.group(1))
            request.name = str(payload.get("name", ""))
            self._set_shelf_type(request, payload)
            self._mutate(request_id, "update_shelf_type", request)
            return
        if match and method == "DELETE":
            request = self._request("delete_shelf_type")
            request.shelf_type_id = int(match.group(1))
            self._mutate(request_id, "delete_shelf_type", request)
            return

        if method == "POST" and path == "/api/skus":
            request = self._request("create_sku")
            for field in (
                "sku", "category", "mesh_file", "tex_file", "owlv2_prompt",
                "qwen_grounding_prompt", "reference_image_path", "grasp_method",
            ):
                setattr(request, field, str(payload.get(field, "")))
            if not request.grasp_method:
                request.grasp_method = "夹爪"
            self._mutate(request_id, "create_sku", request)
            return

        if method == "POST" and path == "/api/skus/batch":
            rows = payload.get("skus")
            if not isinstance(rows, list) or not rows:
                raise ValueError("skus 必须是非空数组")
            request = self._request("upsert_skus_batch")
            for row in rows:
                if not isinstance(row, dict):
                    raise ValueError("每个 SKU 必须是对象")
                message = self._import_sku_type()
                for field in (
                    "sku", "category", "mesh_file", "tex_file", "owlv2_prompt",
                    "qwen_grounding_prompt", "reference_image_path", "grasp_method",
                ):
                    setattr(message, field, str(row.get(field, "")))
                if not message.grasp_method:
                    message.grasp_method = "夹爪"
                request.skus.append(message)
            self._mutate(request_id, "upsert_skus_batch", request)
            return

        match = re.fullmatch(r"/api/skus/(.+)", path)
        if match and method == "PUT":
            request = self._request("update_sku")
            request.sku = unquote(match.group(1))
            for field in (
                "category", "mesh_file", "tex_file", "owlv2_prompt",
                "qwen_grounding_prompt", "reference_image_path", "grasp_method",
            ):
                setattr(request, field, str(payload.get(field, "")))
            if not request.grasp_method:
                request.grasp_method = "夹爪"
            self._mutate(request_id, "update_sku", request)
            return
        if match and method == "DELETE":
            request = self._request("delete_sku")
            request.sku = unquote(match.group(1))
            self._mutate(
                request_id, "delete_sku", request,
                {"removed": request.sku},
            )
            return

        if method == "POST" and path == "/api/slots":
            request = self._request("create_slot")
            for field in ("shelf_id", "face", "level"):
                setattr(request, field, int(payload[field]))
            request.y_cm = float(payload["y_cm"])
            request.expected_sku = str(payload["expected_sku"])
            request.has_actual_sku = payload.get("actual_sku") is not None
            request.actual_sku = str(payload.get("actual_sku") or "")
            request.has_width_cm = payload.get("width_cm") is not None
            request.width_cm = float(payload.get("width_cm") or 0.0)
            request.has_height_cm = payload.get("height_cm") is not None
            request.height_cm = float(payload.get("height_cm") or 0.0)
            request.image_dir = str(payload.get("image_dir", ""))
            self._mutate(
                request_id, "create_slot", request,
                lambda response: {"_slot_id": response.slot.slot_id},
            )
            return

        match = re.fullmatch(r"/api/slots/(.+)/(take|restock)", path)
        if method == "POST" and match:
            service_name = f"{match.group(2)}_slot"
            request = self._request(service_name)
            request.slot_id = unquote(match.group(1))
            self._mutate(
                request_id, service_name, request,
                lambda response: {"_slot_id": response.slot.slot_id},
            )
            return

        match = re.fullmatch(r"/api/slots/(.+)", path)
        if match and method == "PUT":
            request = self._request("update_slot")
            request.slot_id = unquote(match.group(1))
            request.update_expected_sku = "expected_sku" in payload
            request.expected_sku = str(payload.get("expected_sku", ""))
            request.update_actual_sku = "actual_sku" in payload
            request.actual_sku_is_null = payload.get("actual_sku") is None
            request.actual_sku = str(payload.get("actual_sku") or "")
            request.update_width_cm = "width_cm" in payload
            request.width_cm_is_null = payload.get("width_cm") is None
            request.width_cm = float(payload.get("width_cm") or 0.0)
            request.update_height_cm = "height_cm" in payload
            request.height_cm_is_null = payload.get("height_cm") is None
            request.height_cm = float(payload.get("height_cm") or 0.0)
            request.update_image_dir = "image_dir" in payload
            request.image_dir = str(payload.get("image_dir", ""))
            self._mutate(
                request_id, "update_slot", request,
                {"_slot_id": request.slot_id},
            )
            return
        if match and method == "DELETE":
            request = self._request("delete_slot")
            request.slot_id = unquote(match.group(1))
            self._mutate(request_id, "delete_slot", request)
            return

        raise ValueError(f"嵌入模式尚不支持 {method} {path}")
