"""OWLv2 provider implemented solely through supermarket_scene's FindSku service."""

from __future__ import annotations

import time

import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from supermarket_scene_interfaces.srv import FindSku

from openflex_grasp.perception.providers.base import (
    BoxRequest,
    BoxResponse,
    DetectedBox,
    ProviderError,
    elapsed_since,
)


class OwlV2SceneProvider:
    """Translate FindSku's local OWLv2/DINO response into the grasp box contract.

    This class intentionally lives outside the core grasp package. The caller supplies
    provider options; DINO is valid only for this OWLv2 route.
    """

    name = "owlv2_local"
    model = "google/owlv2-large-patch14-ensemble"

    def __init__(self, node: Node, *, service_name: str = "/find_sku"):
        self._node = node
        self._client = node.create_client(FindSku, service_name)

    @staticmethod
    def _compressed(rgb: np.ndarray) -> CompressedImage:
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if not ok:
            raise ProviderError("failed to JPEG-encode RGB image for FindSku")
        message = CompressedImage()
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    def detect(self, request: BoxRequest) -> BoxResponse:
        if not request.owlv2_prompt:
            raise ProviderError(
                f"OWLv2 provider requires a reviewed owlv2_prompt for SKU {request.sku_name!r}"
            )
        if not self._client.wait_for_service(timeout_sec=2.0):
            raise ProviderError("FindSku service is unavailable; do not automatically switch provider")
        options = request.parameters
        started = time.monotonic()
        goal = FindSku.Request()
        goal.image = self._compressed(request.rgb)
        goal.query = request.sku_name
        goal.use_request_options = True
        goal.provider = "local"
        goal.model = self.model
        goal.max_boxes = int(request.max_boxes)
        goal.dino_fallback = bool(options.get("dino_enabled", False))
        goal.dino_confidence_threshold = float(options.get("dino_confidence_threshold", 0.72))
        goal.owlv2_score_threshold = float(options.get("owlv2_score_threshold", 0.10))
        future = self._client.call_async(goal)
        timeout_s = float(options.get("timeout_s", 15.0))
        deadline = time.monotonic() + max(0.1, timeout_s)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise ProviderError(f"FindSku OWLv2 request timed out after {timeout_s:.3f}s")
        response = future.result()
        if response is None or not response.success:
            raise ProviderError(
                "FindSku OWLv2 request failed: "
                f"{'' if response is None else response.error}"
            )
        elapsed = elapsed_since(started)
        boxes = []
        for item in response.boxes[: int(request.max_boxes)]:
            if item.width <= 0 or item.height <= 0:
                continue
            boxes.append(
                DetectedBox(
                    box_xyxy=(item.x, item.y, item.x + item.width, item.y + item.height),
                    label=item.sku or request.sku_name,
                    confidence=float(item.confidence) if item.has_confidence else None,
                    provider=self.name,
                    model=response.model or self.model,
                    request_id=response.request_id or request.request_id,
                    elapsed_s=elapsed,
                )
            )
        return BoxResponse(
            boxes=tuple(boxes),
            provider=self.name,
            model=response.model or self.model,
            request_id=response.request_id or request.request_id,
            elapsed_s=elapsed,
        )
