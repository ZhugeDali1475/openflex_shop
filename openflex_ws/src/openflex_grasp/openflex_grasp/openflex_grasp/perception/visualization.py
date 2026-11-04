import cv2
import numpy as np


def render_overlay(rgb: np.ndarray, mask: np.ndarray, box, label: str) -> np.ndarray:
    overlay = rgb.copy()
    binary = np.asarray(mask, dtype=bool)
    tint = np.zeros_like(overlay)
    tint[:, :, 1] = 255
    overlay[binary] = cv2.addWeighted(overlay, 0.45, tint, 0.55, 0)[binary]
    x1, y1, x2, y2 = box
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 80, 0), 2)
    cv2.putText(
        overlay,
        label,
        (x1, max(18, y1 - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 80, 0),
        2,
        cv2.LINE_AA,
    )
    return overlay
