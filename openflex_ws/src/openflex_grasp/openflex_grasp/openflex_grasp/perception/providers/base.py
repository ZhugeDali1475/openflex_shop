from dataclasses import dataclass, field
import time
from typing import Any, Protocol
from uuid import uuid4

import numpy as np


class ProviderError(RuntimeError):
    """Stable error raised when a selected box provider cannot answer."""


@dataclass(frozen=True)
class BoxRequest:
    rgb: np.ndarray
    sku_name: str
    owlv2_prompt: str = ""
    qwen_grounding_prompt: str = ""
    max_boxes: int = 8
    parameters: dict[str, Any] = field(default_factory=dict)
    request_id: str = field(default_factory=lambda: str(uuid4()))

    def __post_init__(self):
        image = np.array(self.rgb, dtype=np.uint8, order="C", copy=True)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("BoxRequest.rgb must have shape (H, W, 3)")
        if not self.sku_name.strip():
            raise ValueError("BoxRequest.sku_name is required")
        if int(self.max_boxes) <= 0:
            raise ValueError("BoxRequest.max_boxes must be positive")
        image.setflags(write=False)
        object.__setattr__(self, "rgb", image)
        object.__setattr__(self, "sku_name", self.sku_name.strip())
        object.__setattr__(self, "owlv2_prompt", self.owlv2_prompt.strip())
        object.__setattr__(
            self, "qwen_grounding_prompt", self.qwen_grounding_prompt.strip()
        )
        if not self.request_id.strip():
            raise ValueError("BoxRequest.request_id is required")


@dataclass(frozen=True)
class DetectedBox:
    box_xyxy: tuple[int, int, int, int]
    label: str
    confidence: float | None
    provider: str
    model: str
    request_id: str
    elapsed_s: float


@dataclass(frozen=True)
class BoxResponse:
    boxes: tuple[DetectedBox, ...]
    provider: str
    model: str
    request_id: str
    elapsed_s: float


class BoxProvider(Protocol):
    name: str
    model: str

    def detect(self, request: BoxRequest) -> BoxResponse:
        ...


def elapsed_since(start: float) -> float:
    return round(max(0.0, time.monotonic() - start), 6)
