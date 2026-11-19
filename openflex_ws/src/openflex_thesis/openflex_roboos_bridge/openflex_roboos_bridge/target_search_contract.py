"""Image-derived target search and fail-closed resolution (P4.3).

Plain-Python/numpy contract layer: it turns one live head-camera frame
(RGB + aligned depth + intrinsics + ``map``<-camera pose) into colour
*candidates* and resolves a natural-language query into either an authorized
grasp target or a named, fail-closed refusal.

Design rules
------------
* The MuJoCo oracle object observation (``/thesis/object_observation``) is
  **never** used to select the target.  Selection is image-derived only; the
  oracle stays separate as an audit cross-check (see ``RecognitionAudit``).
* An empty/ambiguous/occluded/wrong-class/uncertain observation never yields a
  grasp authorization: the caller must end safely or re-observe.
* The module imports no ROS and performs no motion.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import math
from typing import Mapping, Optional, Sequence, Tuple

from .competition_registry import TARGET_OBJECT_ID

DEFAULT_MIN_CONFIDENCE = 0.8
DEFAULT_MIN_BLOB_PIXELS = 24
DEFAULT_OCCLUSION_DEPTH_MARGIN_M = 0.04
DEFAULT_REGION_RADIUS_PX = 9
DEFAULT_MAX_ASPECT_RATIO = 4.0
DEFAULT_SLOT_RADIUS_PX = 26
DEFAULT_SLOT_DEPTH_TOLERANCE_M = 0.25

# Saturated hue buckets (degrees).  Neutral surfaces (floor, shelf, robot,
# black dividers) stay unclassified so they can never become a candidate.
_HUE_BUCKETS = (
    ("red", ((0.0, 22.0), (338.0, 360.0))),
    ("yellow", ((38.0, 70.0),)),
    ("green", ((80.0, 165.0),)),
    ("blue", ((190.0, 265.0),)),
    ("magenta", ((285.0, 335.0),)),
)


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, width: int, height: int, k: Sequence[float]) -> "CameraIntrinsics":
        if len(k) != 9:
            raise ValueError("camera K must contain 9 values")
        fx, cx = float(k[0]), float(k[2])
        fy, cy = float(k[4]), float(k[5])
        if not all(math.isfinite(value) and value > 0 for value in (fx, fy)):
            raise ValueError("camera focal lengths must be positive and finite")
        return cls(int(width), int(height), fx, fy, cx, cy)


@dataclass(frozen=True)
class CameraPose:
    """Pose of the camera optical frame expressed in ``map``."""

    translation_xyz: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]


@dataclass(frozen=True)
class BlobCandidate:
    candidate_id: str
    color: str
    confidence: float
    depth_m: Optional[float]
    pose_xyzyaw: Optional[Tuple[float, float, float, float]]
    visible: bool
    pixel_count: int
    bbox_px: Tuple[int, int, int, int]


@dataclass(frozen=True)
class RegionVerdict:
    """Verdict for the registered target slot, derived from the frame only."""

    expected_px: Optional[Tuple[int, int]]
    expected_depth_m: Optional[float]
    observed_depth_m: Optional[float]
    state: str  # 'target' | 'occluded' | 'empty' | 'unknown'
    detail: str


@dataclass(frozen=True)
class FrameAnalysis:
    candidates: Tuple[BlobCandidate, ...]
    region: RegionVerdict
    frame_sha256: str
    model_output: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class SearchResolution:
    query: str
    requested_color: str
    outcome: str  # 'authorized' | 'failed'
    reason: str  # stable machine-readable code
    selection_reason: str
    selected_object_id: Optional[str]
    selected_candidate_id: Optional[str]
    selected_perception_track_id: Optional[str]
    candidates: Tuple[BlobCandidate, ...]
    region: RegionVerdict
    frame_sha256: str
    model_output: Mapping[str, object]

    def to_dict(self) -> dict:
        return {
            "query": self.query,
            "requested_color": self.requested_color,
            "outcome": self.outcome,
            "reason": self.reason,
            "selection_reason": self.selection_reason,
            "selected_object_id": self.selected_object_id,
            "selected_candidate_id": self.selected_candidate_id,
            "selected_perception_track_id": self.selected_perception_track_id,
            "candidates": [candidate.__dict__ for candidate in self.candidates],
            "region": self.region.__dict__,
            "frame_sha256": self.frame_sha256,
            "model_output": dict(self.model_output),
            "selection_basis": "image_derived_candidates_not_oracle_object_id",
            "physical_verification": False,
        }


def _rgb_to_hsv(rgb) -> Tuple[object, object, object]:
    import numpy as np

    maximum = rgb.max(axis=-1)
    minimum = rgb.min(axis=-1)
    delta = maximum - minimum
    hue = np.zeros_like(maximum)
    nonzero = delta > 1e-6
    safe_delta = np.where(nonzero, delta, 1.0)
    red, green, blue = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    red_max = nonzero & (maximum == red)
    green_max = nonzero & (maximum == green) & ~red_max
    blue_max = nonzero & (maximum == blue) & ~red_max & ~green_max
    hue[red_max] = (60.0 * ((green - blue) / safe_delta))[red_max] % 360.0
    hue[green_max] = (60.0 * ((blue - red) / safe_delta) + 120.0)[green_max]
    hue[blue_max] = (60.0 * ((red - green) / safe_delta) + 240.0)[blue_max]
    return hue, delta, maximum


def color_mask(rgb, color: str):
    """Boolean mask of pixels classified as ``color`` (saturated hues only)."""
    import numpy as np

    hue, delta, maximum = _rgb_to_hsv(rgb.astype(np.float32) / 255.0)
    saturated = (maximum >= 0.25) & (delta >= 0.25 * np.maximum(maximum, 1e-6))
    mask = np.zeros(hue.shape, dtype=bool)
    for name, buckets in _HUE_BUCKETS:
        if name != color:
            continue
        for low, high in buckets:
            mask |= (hue >= low) & (hue < high)
    return mask & saturated


def _connected_blobs(mask):
    """Label 4-connected components; returns list of pixel-coordinate arrays."""
    import numpy as np

    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    blobs = []
    for seed_y, seed_x in zip(*np.nonzero(mask)):
        if visited[seed_y, seed_x]:
            continue
        stack = [(int(seed_y), int(seed_x))]
        visited[seed_y, seed_x] = True
        pixels = []
        while stack:
            y, x = stack.pop()
            pixels.append((y, x))
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True
                    stack.append((ny, nx))
        blobs.append(np.asarray(pixels, dtype=int))
    return blobs


def _backproject(u: float, v: float, depth_m: float, intrinsics: CameraIntrinsics) -> Tuple[float, float, float]:
    return ((u - intrinsics.cx) * depth_m / intrinsics.fx,
            (v - intrinsics.cy) * depth_m / intrinsics.fy,
            depth_m)


def _quaternion_matrix(quaternion_xyzw) -> Tuple[Tuple[float, ...], ...]:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("camera orientation quaternion has near-zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def camera_to_map(camera_pose: CameraPose, point_camera: Sequence[float]) -> Tuple[float, float, float]:
    rotation = _quaternion_matrix(camera_pose.quaternion_xyzw)
    translation = camera_pose.translation_xyz
    return tuple(translation[row] + sum(rotation[row][k] * float(point_camera[k]) for k in range(3))
                 for row in range(3))  # type: ignore[return-value]


def map_to_camera(camera_pose: CameraPose, point_map: Sequence[float]) -> Tuple[float, float, float]:
    rotation = _quaternion_matrix(camera_pose.quaternion_xyzw)
    delta = [float(point_map[k]) - camera_pose.translation_xyz[k] for k in range(3)]
    # inverse rotation == transpose for a rotation matrix
    return tuple(sum(rotation[k][row] * delta[k] for k in range(3)) for row in range(3))  # type: ignore[return-value]


def project_point(camera_pose: CameraPose, intrinsics: CameraIntrinsics,
                  point_map: Sequence[float]) -> Optional[Tuple[float, float, float]]:
    """Project a map point into the image; None when behind the camera."""
    x, y, z = map_to_camera(camera_pose, point_map)
    if z <= 1e-4:
        return None
    return (intrinsics.fx * x / z + intrinsics.cx, intrinsics.fy * y / z + intrinsics.cy, z)


def extract_color_candidates(rgb, depth_mm, intrinsics: CameraIntrinsics, camera_pose: CameraPose, *,
                             colors: Sequence[str], min_blob_pixels: int = DEFAULT_MIN_BLOB_PIXELS,
                             min_confidence: float = DEFAULT_MIN_CONFIDENCE,
                             max_aspect_ratio: float = DEFAULT_MAX_ASPECT_RATIO,
                             slot_yaw_rad: Optional[float] = None) -> Tuple[BlobCandidate, ...]:
    """Segment saturated colour blobs into candidates with depth + map pose.

    Long thin saturated stripes (fixture trim) are rejected by the aspect-ratio
    gate so decoration cannot masquerade as an object.
    """
    import numpy as np

    if rgb.shape[:2] != (intrinsics.height, intrinsics.width):
        raise ValueError("rgb shape does not match camera intrinsics")
    depth_m = depth_mm.astype(np.float32) / 1000.0
    candidates = []
    for color in colors:
        mask = color_mask(rgb, color)
        for blob_index, pixels in enumerate(_connected_blobs(mask)):
            pixel_count = int(pixels.shape[0])
            if pixel_count < min_blob_pixels:
                continue
            ys, xs = pixels[:, 0], pixels[:, 1]
            extent_x = int(xs.max()) - int(xs.min()) + 1
            extent_y = int(ys.max()) - int(ys.min()) + 1
            if max(extent_x, extent_y) / max(1, min(extent_x, extent_y)) > max_aspect_ratio:
                continue
            center_u = float(xs.mean())
            center_v = float(ys.mean())
            patch = depth_m[max(0, int(center_v) - 2):int(center_v) + 3,
                            max(0, int(center_u) - 2):int(center_u) + 3]
            valid = patch[(patch > 0.05) & np.isfinite(patch)]
            depth_value = float(np.median(valid)) if valid.size else None
            pose = None
            if depth_value is not None:
                position = camera_to_map(camera_pose,
                                         _backproject(center_u, center_v, depth_value, intrinsics))
                yaw = float(slot_yaw_rad) if slot_yaw_rad is not None else 0.0
                pose = (position[0], position[1], position[2], yaw)
            # Confidence blends blob coverage and depth support; a blob with no
            # valid depth return is not confidently localised.
            coverage = min(1.0, pixel_count / (2.0 * min_blob_pixels))
            confidence = 0.0 if depth_value is None else round(min(0.99, 0.6 + 0.4 * coverage), 4)
            candidates.append(BlobCandidate(
                candidate_id=f"color[{color}]#{blob_index}",
                color=color,
                confidence=confidence,
                depth_m=None if depth_value is None else round(depth_value, 5),
                pose_xyzyaw=pose,
                visible=bool(depth_value is not None and confidence >= min_confidence),
                pixel_count=pixel_count,
                bbox_px=(int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())),
            ))
    return tuple(candidates)


def classify_slot_region(rgb, depth_mm, intrinsics: CameraIntrinsics, camera_pose: CameraPose,
                         expected_point_map: Sequence[float], *, color: str = "red",
                         radius_px: int = DEFAULT_REGION_RADIUS_PX,
                         margin_m: float = DEFAULT_OCCLUSION_DEPTH_MARGIN_M) -> RegionVerdict:
    """Image-only verdict for the registered target slot.

    Distinguishes the three failure modes that must never be grasped:
    a foreign surface in front (``occluded``), background with no return
    (``empty``), and an undecidable reading (``unknown``).
    """
    import numpy as np

    projection = project_point(camera_pose, intrinsics, expected_point_map)
    if projection is None:
        return RegionVerdict(None, None, None, "unknown", "target slot not in camera frustum")
    u, v, expected_depth = projection
    if not (0 <= u < intrinsics.width and 0 <= v < intrinsics.height):
        return RegionVerdict((int(u), int(v)), round(expected_depth, 5), None, "unknown",
                             "target slot outside image bounds")
    mask = color_mask(rgb, color)
    x0, x1 = max(0, int(u) - radius_px), min(intrinsics.width, int(u) + radius_px + 1)
    y0, y1 = max(0, int(v) - radius_px), min(intrinsics.height, int(v) + radius_px + 1)
    if bool(mask[y0:y1, x0:x1].any()):
        return RegionVerdict((int(u), int(v)), round(expected_depth, 5), round(expected_depth, 5),
                             "target", "target-colour pixels present at the registered slot")
    patch = depth_mm.astype(np.float32)[y0:y1, x0:x1] / 1000.0
    valid = patch[(patch > 0.05) & np.isfinite(patch)]
    if valid.size == 0:
        return RegionVerdict((int(u), int(v)), round(expected_depth, 5), None, "unknown",
                             "no depth return at the registered slot")
    observed = float(np.median(valid))
    if observed < expected_depth - margin_m:
        return RegionVerdict((int(u), int(v)), round(expected_depth, 5), round(observed, 5), "occluded",
                             "nearer surface occludes the registered slot")
    if observed > expected_depth + margin_m:
        return RegionVerdict((int(u), int(v)), round(expected_depth, 5), round(observed, 5), "empty",
                             "background surface seen where the object should be")
    return RegionVerdict((int(u), int(v)), round(expected_depth, 5), round(observed, 5), "unknown",
                         "surface at the slot depth but without target colour")


def analyze_frame(rgb, depth_mm, intrinsics: CameraIntrinsics, camera_pose: CameraPose, *,
                  expected_point_map: Optional[Sequence[float]] = None, color: str = "red",
                  colors: Sequence[str] = ("red", "blue", "green", "yellow", "magenta"),
                  min_blob_pixels: int = DEFAULT_MIN_BLOB_PIXELS,
                  min_confidence: float = DEFAULT_MIN_CONFIDENCE,
                  max_aspect_ratio: float = DEFAULT_MAX_ASPECT_RATIO,
                  slot_yaw_rad: Optional[float] = None) -> FrameAnalysis:
    import numpy as np

    frame_sha = hashlib.sha256(np.ascontiguousarray(rgb).tobytes()
                               + np.ascontiguousarray(depth_mm).tobytes()).hexdigest()
    candidates = extract_color_candidates(
        rgb, depth_mm, intrinsics, camera_pose, colors=colors, min_blob_pixels=min_blob_pixels,
        min_confidence=min_confidence, max_aspect_ratio=max_aspect_ratio, slot_yaw_rad=slot_yaw_rad)
    if expected_point_map is None:
        region = RegionVerdict(None, None, None, "unknown", "no registered slot supplied")
    else:
        region = classify_slot_region(rgb, depth_mm, intrinsics, camera_pose, expected_point_map,
                                      color=color)
    model_output = {
        "extractor": "saturated_hue_blob_segmenter_v1",
        "frame_sha256": frame_sha,
        "color_masks": {name: int(color_mask(rgb, name).sum()) for name in colors},
        "candidate_count": len(candidates),
        "max_aspect_ratio": max_aspect_ratio,
        "slot_verdict": region.state,
        "pose_source": "image_depth_backprojection_map_frame",
        "yaw_source": "registered_slot_geometry_not_perception" if expected_point_map is not None
                      else "unobserved_default_zero_not_object_orientation",
    }
    return FrameAnalysis(candidates=candidates, region=region, frame_sha256=frame_sha,
                         model_output=model_output)


def resolve_search_target(analysis: FrameAnalysis, query: str, *,
                          color: str = "red", object_id: Optional[str] = TARGET_OBJECT_ID,
                          min_confidence: float = DEFAULT_MIN_CONFIDENCE,
                          slot_radius_px: float = DEFAULT_SLOT_RADIUS_PX,
                          depth_tolerance_m: float = DEFAULT_SLOT_DEPTH_TOLERANCE_M) -> SearchResolution:
    """Resolve a query into an authorized target or a named fail-closed refusal.

    When a registered expected point is supplied, retain the legacy slot-bound
    behavior for regression comparisons.  Without it, resolve across the full
    frame: the result carries a visual candidate/track identity and never a
    MuJoCo object id.  Online search must use this no-slot mode.
    """

    def failed(reason: str, detail: str) -> SearchResolution:
        return SearchResolution(query=query, requested_color=color, outcome="failed", reason=reason,
                                selection_reason=detail, selected_object_id=None,
                                selected_candidate_id=None, selected_perception_track_id=None,
                                candidates=analysis.candidates,
                                region=analysis.region, frame_sha256=analysis.frame_sha256,
                                model_output=dict(analysis.model_output))

    expected_px = analysis.region.expected_px
    expected_depth = analysis.region.expected_depth_m

    def at_slot(candidate: BlobCandidate) -> bool:
        x0, y0, x1, y1 = candidate.bbox_px
        if expected_px is not None:
            center_x, center_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            if math.hypot(center_x - expected_px[0], center_y - expected_px[1]) > slot_radius_px:
                return False
        if expected_depth is not None and candidate.depth_m is not None:
            if abs(candidate.depth_m - expected_depth) > depth_tolerance_m:
                return False
        return True

    slot_constrained = expected_px is not None or expected_depth is not None
    matching = [candidate for candidate in analysis.candidates
                if candidate.color == color and candidate.visible
                and candidate.confidence >= min_confidence
                and (not slot_constrained or at_slot(candidate))]
    foreign = [candidate for candidate in analysis.candidates
               if candidate.color != color and candidate.visible
               and candidate.confidence >= min_confidence
               and (not slot_constrained or at_slot(candidate))]

    if len(matching) > 1:
        return failed("ambiguous_candidates",
                      f"{len(matching)} visible {color} candidates; multi-instance disambiguation is out of scope")
    if not matching:
        if foreign:
            return failed("wrong_class",
                          f"visible object(s) of colour {sorted({c.color for c in foreign})} but requested {color}")
        if analysis.region.state == "occluded":
            return failed("target_occluded", analysis.region.detail)
        if analysis.region.state == "empty":
            return failed("empty_slot", analysis.region.detail)
        weak = [candidate for candidate in analysis.candidates if candidate.color == color]
        if weak:
            return failed("below_confidence",
                          f"only sub-threshold {color} evidence (max confidence "
                          f"{max(candidate.confidence for candidate in weak)})")
        return failed("target_absent", "no target-colour candidate and the slot is not occluded")

    candidate = matching[0]
    if candidate.depth_m is None or candidate.pose_xyzyaw is None:
        return failed("not_authorizable", "candidate lacks RGB-D depth or map pose")
    # A frame-local observation token is not a simulator identity and is not a
    # substitute for temporal multi-frame tracking.  The execution layer must
    # persist/track this observation before requesting a grasp.
    perception_track_id = f"track_{analysis.frame_sha256[:12]}_{candidate.candidate_id}"
    selected_object_id = object_id if slot_constrained and object_id is not None else None
    return SearchResolution(query=query, requested_color=color, outcome="authorized",
                            reason="target_visible",
                            selection_reason=("unique visible target-colour candidate in the full frame; "
                                              "depth and pose confirmed from the live frame"
                                              if not slot_constrained else
                                              "unique candidate at legacy registered slot; not valid as unknown-slot evidence"),
                            selected_object_id=selected_object_id,
                            selected_candidate_id=candidate.candidate_id,
                            selected_perception_track_id=perception_track_id,
                            candidates=analysis.candidates, region=analysis.region,
                            frame_sha256=analysis.frame_sha256,
                            model_output={**dict(analysis.model_output),
                                          "identity_source": "visual_frame_candidate",
                                          "mujoco_object_id_used": False,
                                          "legacy_slot_binding": slot_constrained})
