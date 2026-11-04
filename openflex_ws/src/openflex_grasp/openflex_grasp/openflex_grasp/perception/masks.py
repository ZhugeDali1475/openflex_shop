import cv2
import numpy as np


def largest_component(mask: np.ndarray) -> np.ndarray:
    binary = np.asarray(mask, dtype=np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return binary.astype(bool)
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == index


def clean_mask(mask: np.ndarray, close_kernel: int = 5) -> np.ndarray:
    binary = (np.asarray(mask) > 0).astype(np.uint8)
    if close_kernel > 1:
        size = close_kernel if close_kernel % 2 else close_kernel + 1
        kernel = np.ones((size, size), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(
            binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )
    return largest_component(binary)
