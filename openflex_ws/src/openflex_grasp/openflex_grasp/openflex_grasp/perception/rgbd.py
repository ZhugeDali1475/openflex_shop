from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self):
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(np.isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")

    def scaled(self, width: int, height: int):
        if width <= 0 or height <= 0:
            raise ValueError("scaled camera dimensions must be positive")
        sx = width / float(self.width)
        sy = height / float(self.height)
        return CameraIntrinsics(
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
            width=width,
            height=height,
        )


@dataclass(frozen=True)
class FrozenRgbdFrame:
    rgb: np.ndarray
    depth: np.ndarray
    intrinsics: CameraIntrinsics
    arm_name: str
    camera_serial: str
    frame_id: str
    stamp_ns: int
    depth_scale: float

    def __post_init__(self):
        rgb = np.array(self.rgb, dtype=np.uint8, order="C", copy=True)
        depth = np.array(self.depth, order="C", copy=True)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("RGB image must have shape (H, W, 3)")
        if depth.ndim != 2 or depth.shape != rgb.shape[:2]:
            raise ValueError("aligned depth must match the RGB image dimensions")
        if self.arm_name not in {"left", "right"}:
            raise ValueError("arm_name must be left or right")
        if not self.camera_serial.strip() or not self.frame_id.strip():
            raise ValueError("camera serial and frame_id are required")
        if int(self.stamp_ns) < 0:
            raise ValueError("stamp_ns must be non-negative")
        if not np.isfinite(self.depth_scale) or self.depth_scale <= 0.0:
            raise ValueError("depth_scale must be finite and positive")
        rgb.setflags(write=False)
        depth.setflags(write=False)
        object.__setattr__(self, "rgb", rgb)
        object.__setattr__(self, "depth", depth)
