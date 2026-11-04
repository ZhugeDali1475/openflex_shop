from pathlib import Path
import time

import numpy as np
import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

from ...grounding import Detection, parse_detections
from .base import BoxRequest, BoxResponse, DetectedBox, ProviderError, elapsed_since


class QwenGrounder:
    """Legacy Qwen grounding implementation behind a provider boundary."""

    def __init__(self, model_path: str, device: str = "cuda"):
        self.name = "qwen_local"
        self.model_id = Path(model_path).name
        self.device = torch.device(device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but PyTorch cannot access the GPU")
        if not Path(model_path).is_dir():
            raise FileNotFoundError(f"model directory does not exist: {model_path}")
        dtype = torch.bfloat16 if self.device.type == "cuda" else torch.float32
        self.processor = AutoProcessor.from_pretrained(model_path, local_files_only=True)
        self.model = AutoModelForImageTextToText.from_pretrained(
            model_path,
            dtype=dtype,
            device_map=str(self.device),
            local_files_only=True,
            low_cpu_mem_usage=True,
        ).eval()

    @staticmethod
    def grounding_prompt(target: str, qwen_grounding_prompt: str = "") -> str:
        visual_hint = (
            f" Database visual features for this SKU: {qwen_grounding_prompt}."
            if qwen_grounding_prompt else ""
        )
        return (
            "Locate every distinct visible instance of the requested product in the image. "
            f"Requested product Chinese SKU name: {target!r}.{visual_hint} "
            "Return only compact JSON in exactly this schema: "
            '{"boxes":[{"label":"product","bbox_2d":[x1,y1,x2,y2],"confidence":0.0}]}. '
            "bbox_2d must be integers normalized to 0..1000, with the top-left and bottom-right "
            "corners. Include one boxes entry per physical instance, including repeated identical "
            "products. The confidence field must represent semantic confidence that this specific "
            "instance matches the requested product, not visibility or ease of grasping. Do not "
            "merge adjacent products. Do not include explanations or markdown. "
            "If packaging is transparent, bound the complete package silhouette."
        )

    def locate_all(
        self, rgb: np.ndarray, target: str, qwen_grounding_prompt: str = ""
    ) -> list[Detection]:
        response = self.generate_text(
            rgb,
            self.grounding_prompt(target, qwen_grounding_prompt),
            max_new_tokens=512,
        )
        height, width = rgb.shape[:2]
        return parse_detections(response, target, width, height)

    def generate_text(
        self,
        rgb: np.ndarray,
        prompt: str,
        *,
        max_new_tokens: int = 256,
    ) -> str:
        """Run deterministic image-text generation on the resident model."""
        prompt = str(prompt).strip()
        if not prompt:
            raise ValueError("Qwen prompt cannot be empty")
        max_new_tokens = int(max_new_tokens)
        if not 1 <= max_new_tokens <= 2048:
            raise ValueError("max_new_tokens must be within 1..2048")
        image = Image.fromarray(np.asarray(rgb, dtype=np.uint8), mode="RGB")
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": prompt},
                ],
            }
        ]
        inputs = self.processor.apply_chat_template(
            messages,
            tokenize=True,
            add_generation_prompt=True,
            return_dict=True,
            return_tensors="pt",
            enable_thinking=False,
        ).to(self.device)
        with torch.inference_mode():
            output_ids = self.model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                do_sample=False,
                use_cache=True,
            )
        generated = output_ids[:, inputs["input_ids"].shape[1] :]
        return self.processor.batch_decode(
            generated, skip_special_tokens=True
        )[0].strip()

    def detect(self, request: BoxRequest) -> BoxResponse:
        started = time.monotonic()
        try:
            boxes = self.locate_all(
                request.rgb, request.sku_name, request.qwen_grounding_prompt
            )
        except Exception as error:
            raise ProviderError(
                f"Qwen provider failed for SKU {request.sku_name!r}: {error}"
            ) from error
        boxes = boxes[: int(request.max_boxes)]
        elapsed = elapsed_since(started)
        timeout_s = float(request.parameters.get("timeout_s", 0.0))
        if timeout_s > 0.0 and elapsed > timeout_s:
            raise ProviderError(
                f"Qwen provider exceeded timeout_s={timeout_s:.3f} "
                f"(elapsed={elapsed:.3f})"
            )
        return BoxResponse(
            boxes=tuple(
                DetectedBox(
                    box_xyxy=box.box_xyxy,
                    label=box.label,
                    confidence=box.confidence,
                    provider=self.name,
                    model=self.model_id,
                    request_id=request.request_id,
                    elapsed_s=elapsed,
                )
                for box in boxes
            ),
            provider=self.name,
            model=self.model_id,
            request_id=request.request_id,
            elapsed_s=elapsed,
        )

    def locate(self, rgb: np.ndarray, target: str) -> Detection:
        return self.locate_all(rgb, target)[0]
