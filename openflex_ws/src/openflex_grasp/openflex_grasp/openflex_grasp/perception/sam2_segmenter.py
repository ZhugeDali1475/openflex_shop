from pathlib import Path

import numpy as np
import torch
from PIL import Image
from transformers import Sam2Model, Sam2Processor


class Sam2BoxSegmenter:
    def __init__(self, model_path: str, device: str = "cuda"):
        self.device = torch.device(device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but PyTorch cannot access the GPU")
        if not Path(model_path).is_dir():
            raise FileNotFoundError(f"model directory does not exist: {model_path}")
        dtype = torch.bfloat16 if self.device.type == "cuda" else torch.float32
        self.processor = Sam2Processor.from_pretrained(model_path, local_files_only=True)
        self.model = Sam2Model.from_pretrained(
            model_path,
            dtype=dtype,
            local_files_only=True,
        ).to(self.device).eval()

    def segment_box(self, rgb: np.ndarray, box) -> tuple[np.ndarray, float]:
        image = Image.fromarray(np.asarray(rgb, dtype=np.uint8), mode="RGB")
        inputs = self.processor(
            images=image,
            input_boxes=[[list(box)]],
            return_tensors="pt",
        ).to(self.device)
        with torch.inference_mode(), torch.autocast(
            device_type=self.device.type,
            dtype=torch.bfloat16,
            enabled=self.device.type == "cuda",
        ):
            outputs = self.model(**inputs, multimask_output=True)
        masks = self.processor.post_process_masks(
            outputs.pred_masks.detach().cpu(),
            inputs["original_sizes"].detach().cpu(),
        )[0]
        scores = outputs.iou_scores.detach().float().cpu()[0, 0]
        x1, y1, x2, y2 = box
        box_area = max(1, (x2 - x1) * (y2 - y1))
        areas = masks[0].reshape(masks.shape[1], -1).sum(dim=1)
        plausible = [
            index
            for index, area in enumerate(areas.tolist())
            if max(64, box_area * 0.03) <= area <= box_area * 6.0
        ]
        best = (
            max(plausible, key=lambda index: float(scores[index]))
            if plausible
            else int(torch.argmax(scores).item())
        )
        return masks[0, best].numpy() > 0, float(scores[best].item())
