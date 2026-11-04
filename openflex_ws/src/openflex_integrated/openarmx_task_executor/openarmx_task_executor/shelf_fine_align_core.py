"""Pure geometry and filtering helpers for shelf fine alignment."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Iterable, Optional


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class ShelfMeasurement:
    clearance: float
    point_x: float
    point_y: float
    sample_count: int


@dataclass(frozen=True)
class ShelfCornerMeasurement:
    """Clearances observed near both front corners of the robot body."""

    left_clearance: float
    right_clearance: float
    shelf_yaw_error: float
    left_point_count: int
    right_point_count: int
    lateral_span: float

    @property
    def minimum_corner_clearance(self) -> float:
        return min(self.left_clearance, self.right_clearance)

    @property
    def mean_corner_clearance(self) -> float:
        return 0.5 * (self.left_clearance + self.right_clearance)

    @property
    def corner_clearance_difference(self) -> float:
        return abs(self.left_clearance - self.right_clearance)


@dataclass(frozen=True)
class ShelfPlaneMeasurement:
    """A front-facing shelf plane expressed in ``base_link``.

    ``normal_*`` points from the robot toward the shelf. ``left_clearance``
    and ``right_clearance`` are the signed perpendicular distances from the
    respective front body corners to that plane.
    """

    normal_x: float
    normal_y: float
    offset: float
    left_clearance: float
    right_clearance: float
    point_count: int
    lateral_span: float

    @property
    def shelf_yaw_error(self) -> float:
        return math.atan2(self.normal_y, self.normal_x)

    @property
    def minimum_corner_clearance(self) -> float:
        return min(self.left_clearance, self.right_clearance)

    @property
    def mean_corner_clearance(self) -> float:
        return 0.5 * (self.left_clearance + self.right_clearance)

    @property
    def corner_clearance_difference(self) -> float:
        return abs(self.left_clearance - self.right_clearance)


def rectangular_clearance(
    point_x: float,
    point_y: float,
    half_length: float,
    half_width: float,
) -> float:
    """Euclidean distance from a point to an axis-aligned robot rectangle."""
    dx = max(point_x - half_length, 0.0)
    dy = max(abs(point_y) - half_width, 0.0)
    return math.hypot(dx, dy)


def measure_scan_clearance(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    *,
    half_length: float,
    half_width: float,
    roi_forward_min: float,
    roi_forward_max: float,
    roi_half_width: float,
    maximum_clearance: float,
    point_transform: Optional[Callable[[float, float], tuple[float, float]]] = None,
) -> Optional[ShelfMeasurement]:
    """Return the nearest finite scan return in the expected shelf ROI.

    Distance is measured to the complete physical footprint, not to a single
    centre ray.  If supplied, ``point_transform`` projects sensor-frame points
    into ``base_link`` before the ROI and footprint checks. A temporal filter in
    :class:`ClearanceFilter` supplies the multi-frame validation required for
    sparse Livox scans.
    """
    closest: Optional[tuple[float, float, float]] = None
    count = 0
    for index, raw_range in enumerate(ranges):
        radius = float(raw_range)
        if not math.isfinite(radius) or radius <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        point_x = radius * math.cos(angle)
        point_y = radius * math.sin(angle)
        if point_transform is not None:
            point_x, point_y = point_transform(point_x, point_y)
        if not roi_forward_min <= point_x <= roi_forward_max:
            continue
        if abs(point_y) > roi_half_width:
            continue
        clearance = rectangular_clearance(
            point_x, point_y, half_length, half_width
        )
        if clearance > maximum_clearance:
            continue
        count += 1
        if closest is None or clearance < closest[0]:
            closest = (clearance, point_x, point_y)
    if closest is None:
        return None
    return ShelfMeasurement(closest[0], closest[1], closest[2], count)


def measure_scan_shelf_corners(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    *,
    half_length: float,
    half_width: float,
    roi_forward_min: float,
    roi_forward_max: float,
    roi_half_width: float,
    maximum_clearance: float,
    minimum_lateral_offset: float,
    minimum_points_per_side: int,
    point_transform: Optional[Callable[[float, float], tuple[float, float]]] = None,
) -> Optional[ShelfCornerMeasurement]:
    """Measure the nearest shelf return independently at each front corner.

    Real supermarket shelving produces sparse returns from uprights and front
    edges rather than one continuous planar face. Restricting observations to
    lateral corner bands prevents a farther back panel from hiding a close
    front edge. Multiple points are required on each side before a frame is
    accepted; temporal filtering provides the final outlier rejection.
    """
    side_points: tuple[list[tuple[float, float, float]], ...] = ([], [])
    for index, raw_range in enumerate(ranges):
        radius = float(raw_range)
        if not math.isfinite(radius) or radius <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        point_x = radius * math.cos(angle)
        point_y = radius * math.sin(angle)
        if point_transform is not None:
            point_x, point_y = point_transform(point_x, point_y)
        if not roi_forward_min <= point_x <= roi_forward_max:
            continue
        absolute_y = abs(point_y)
        if not minimum_lateral_offset <= absolute_y <= roi_half_width:
            continue
        clearance = rectangular_clearance(
            point_x, point_y, half_length, half_width
        )
        if clearance > maximum_clearance:
            continue
        side_points[0 if point_y >= 0.0 else 1].append(
            (clearance, point_x, point_y)
        )

    left_points, right_points = side_points
    if min(len(left_points), len(right_points)) < minimum_points_per_side:
        return None
    left = min(left_points)
    right = min(right_points)
    normal_y = (right[0] - left[0]) / (2.0 * half_width)
    yaw_error = math.asin(max(-1.0, min(1.0, normal_y)))
    all_y = [point[2] for points in side_points for point in points]
    return ShelfCornerMeasurement(
        left_clearance=left[0],
        right_clearance=right[0],
        shelf_yaw_error=yaw_error,
        left_point_count=len(left_points),
        right_point_count=len(right_points),
        lateral_span=max(all_y) - min(all_y),
    )


def _fit_line_normal(points: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Fit a 2-D total-least-squares line and return its forward normal."""
    count = len(points)
    mean_x = sum(point[0] for point in points) / count
    mean_y = sum(point[1] for point in points) / count
    xx = sum((point[0] - mean_x) ** 2 for point in points)
    yy = sum((point[1] - mean_y) ** 2 for point in points)
    xy = sum((point[0] - mean_x) * (point[1] - mean_y) for point in points)
    tangent_yaw = 0.5 * math.atan2(2.0 * xy, xx - yy)
    normal_x = -math.sin(tangent_yaw)
    normal_y = math.cos(tangent_yaw)
    if normal_x < 0.0:
        normal_x = -normal_x
        normal_y = -normal_y
    offset = normal_x * mean_x + normal_y * mean_y
    return normal_x, normal_y, offset


def fit_shelf_plane(
    points: Iterable[tuple[float, float]],
    *,
    half_length: float,
    half_width: float,
    minimum_points: int,
    minimum_lateral_span: float,
    minimum_points_per_side: int,
    inlier_tolerance: float,
    maximum_normal_yaw: float,
) -> Optional[ShelfPlaneMeasurement]:
    """Robustly fit a front shelf face that spans both front body corners.

    A sparse Livox-derived scan can contain shelf legs, edges, and occasional
    unrelated returns. Candidate lines are formed only from pairs separated in
    the lateral direction, scored by inlier count, then re-fitted by total
    least squares. Requiring inliers on both sides of ``base_link`` prevents a
    single close corner or a small side object from masquerading as a shelf.
    """
    usable = [
        (float(point_x), float(point_y))
        for point_x, point_y in points
        if math.isfinite(point_x) and math.isfinite(point_y)
    ]
    if len(usable) < minimum_points:
        return None

    best_inliers: list[tuple[float, float]] = []
    best_span = 0.0
    best_score = -1
    pair_span = max(0.10, 0.5 * minimum_lateral_span)
    for first_index, first in enumerate(usable[:-1]):
        for second in usable[first_index + 1:]:
            delta_x = second[0] - first[0]
            delta_y = second[1] - first[1]
            length = math.hypot(delta_x, delta_y)
            if length <= 1e-6 or abs(delta_y) < pair_span:
                continue
            normal_x = delta_y / length
            normal_y = -delta_x / length
            if normal_x < 0.0:
                normal_x = -normal_x
                normal_y = -normal_y
            if abs(math.atan2(normal_y, normal_x)) > maximum_normal_yaw:
                continue
            offset = normal_x * first[0] + normal_y * first[1]
            inliers = [
                point
                for point in usable
                if abs(normal_x * point[0] + normal_y * point[1] - offset)
                <= inlier_tolerance
            ]
            if len(inliers) < minimum_points:
                continue
            lateral_span = max(point[1] for point in inliers) - min(
                point[1] for point in inliers
            )
            side_count = sum(point[1] >= 0.0 for point in inliers), sum(
                point[1] < 0.0 for point in inliers
            )
            if (
                lateral_span < minimum_lateral_span
                or min(side_count) < minimum_points_per_side
            ):
                continue
            score = len(inliers)
            if score > best_score or (score == best_score and lateral_span > best_span):
                best_inliers = inliers
                best_span = lateral_span
                best_score = score

    if not best_inliers:
        return None

    normal_x, normal_y, offset = _fit_line_normal(best_inliers)
    if abs(math.atan2(normal_y, normal_x)) > maximum_normal_yaw:
        return None
    inliers = [
        point
        for point in usable
        if abs(normal_x * point[0] + normal_y * point[1] - offset)
        <= inlier_tolerance
    ]
    if len(inliers) < minimum_points:
        return None
    lateral_span = max(point[1] for point in inliers) - min(point[1] for point in inliers)
    side_count = sum(point[1] >= 0.0 for point in inliers), sum(
        point[1] < 0.0 for point in inliers
    )
    if lateral_span < minimum_lateral_span or min(side_count) < minimum_points_per_side:
        return None

    left_clearance = offset - normal_x * half_length - normal_y * half_width
    right_clearance = offset - normal_x * half_length + normal_y * half_width
    return ShelfPlaneMeasurement(
        normal_x=normal_x,
        normal_y=normal_y,
        offset=offset,
        left_clearance=left_clearance,
        right_clearance=right_clearance,
        point_count=len(inliers),
        lateral_span=lateral_span,
    )


def measure_scan_shelf_plane(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    *,
    half_length: float,
    half_width: float,
    roi_forward_min: float,
    roi_forward_max: float,
    roi_half_width: float,
    maximum_clearance: float,
    minimum_points: int,
    minimum_lateral_span: float,
    minimum_points_per_side: int,
    inlier_tolerance: float,
    maximum_normal_yaw: float,
    point_transform: Optional[Callable[[float, float], tuple[float, float]]] = None,
) -> Optional[ShelfPlaneMeasurement]:
    """Fit a shelf face from scan returns inside the guarded forward ROI."""
    points: list[tuple[float, float]] = []
    for index, raw_range in enumerate(ranges):
        radius = float(raw_range)
        if not math.isfinite(radius) or radius <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        point_x = radius * math.cos(angle)
        point_y = radius * math.sin(angle)
        if point_transform is not None:
            point_x, point_y = point_transform(point_x, point_y)
        if not roi_forward_min <= point_x <= roi_forward_max:
            continue
        if abs(point_y) > roi_half_width:
            continue
        if (
            rectangular_clearance(point_x, point_y, half_length, half_width)
            > maximum_clearance
        ):
            continue
        points.append((point_x, point_y))
    return fit_shelf_plane(
        points,
        half_length=half_length,
        half_width=half_width,
        minimum_points=minimum_points,
        minimum_lateral_span=minimum_lateral_span,
        minimum_points_per_side=minimum_points_per_side,
        inlier_tolerance=inlier_tolerance,
        maximum_normal_yaw=maximum_normal_yaw,
    )


class ClearanceFilter:
    """Conservative multi-frame clearance estimator for sparse scan returns."""

    def __init__(
        self,
        window_size: int,
        minimum_samples: int,
        quantile: float = 0.25,
    ) -> None:
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        if minimum_samples <= 0 or minimum_samples > window_size:
            raise ValueError("minimum_samples must be in [1, window_size]")
        if not 0.0 <= quantile <= 1.0:
            raise ValueError("quantile must be in [0, 1]")
        self._values: deque[float] = deque(maxlen=window_size)
        self.minimum_samples = minimum_samples
        self.quantile = quantile

    def clear(self) -> None:
        self._values.clear()

    def add(self, value: float) -> Optional[float]:
        if math.isfinite(value) and value >= 0.0:
            self._values.append(float(value))
        return self.value

    @property
    def sample_count(self) -> int:
        return len(self._values)

    @property
    def value(self) -> Optional[float]:
        if len(self._values) < self.minimum_samples:
            return None
        values = sorted(self._values)
        position = self.quantile * (len(values) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return values[lower]
        ratio = position - lower
        return values[lower] * (1.0 - ratio) + values[upper] * ratio


class ShelfPlaneFilter:
    """Temporal filter for two-corner shelf-plane measurements.

    Clearance sides use the configured conservative quantile. The plane yaw
    uses the median of the recent normals so one sparse scan cannot command a
    large steering correction.
    """

    def __init__(
        self,
        window_size: int,
        minimum_samples: int,
        *,
        half_length: float,
        quantile: float = 0.25,
    ):
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        if minimum_samples <= 0 or minimum_samples > window_size:
            raise ValueError("minimum_samples must be in [1, window_size]")
        if not 0.0 <= quantile <= 1.0:
            raise ValueError("quantile must be in [0, 1]")
        self._values: deque[ShelfPlaneMeasurement] = deque(maxlen=window_size)
        self.minimum_samples = minimum_samples
        self.half_length = float(half_length)
        self.quantile = quantile

    def clear(self) -> None:
        self._values.clear()

    def add(self, value: ShelfPlaneMeasurement) -> Optional[ShelfPlaneMeasurement]:
        if (
            math.isfinite(value.left_clearance)
            and math.isfinite(value.right_clearance)
            and math.isfinite(value.shelf_yaw_error)
        ):
            self._values.append(value)
        return self.value

    @property
    def sample_count(self) -> int:
        return len(self._values)

    @staticmethod
    def _quantile(values: list[float], quantile: float) -> float:
        ordered = sorted(values)
        position = quantile * (len(ordered) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return ordered[lower]
        ratio = position - lower
        return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio

    @property
    def value(self) -> Optional[ShelfPlaneMeasurement]:
        if len(self._values) < self.minimum_samples:
            return None
        left = self._quantile(
            [measurement.left_clearance for measurement in self._values],
            self.quantile,
        )
        right = self._quantile(
            [measurement.right_clearance for measurement in self._values],
            self.quantile,
        )
        yaw = self._quantile(
            [measurement.shelf_yaw_error for measurement in self._values], 0.5
        )
        normal_x = math.cos(yaw)
        normal_y = math.sin(yaw)
        mean_clearance = 0.5 * (left + right)
        offset = mean_clearance + normal_x * self.half_length
        return ShelfPlaneMeasurement(
            normal_x=normal_x,
            normal_y=normal_y,
            offset=offset,
            left_clearance=left,
            right_clearance=right,
            point_count=min(measurement.point_count for measurement in self._values),
            lateral_span=min(measurement.lateral_span for measurement in self._values),
        )


class ShelfCornerFilter:
    """Conservative temporal filter for independent front-corner returns."""

    def __init__(
        self,
        window_size: int,
        minimum_samples: int,
        *,
        half_width: float,
        quantile: float = 0.25,
    ) -> None:
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        if minimum_samples <= 0 or minimum_samples > window_size:
            raise ValueError("minimum_samples must be in [1, window_size]")
        if half_width <= 0.0:
            raise ValueError("half_width must be positive")
        if not 0.0 <= quantile <= 1.0:
            raise ValueError("quantile must be in [0, 1]")
        self._values: deque[ShelfCornerMeasurement] = deque(maxlen=window_size)
        self.minimum_samples = minimum_samples
        self.half_width = float(half_width)
        self.quantile = quantile

    def clear(self) -> None:
        self._values.clear()

    def add(self, value: ShelfCornerMeasurement) -> Optional[ShelfCornerMeasurement]:
        if (
            math.isfinite(value.left_clearance)
            and value.left_clearance >= 0.0
            and math.isfinite(value.right_clearance)
            and value.right_clearance >= 0.0
        ):
            self._values.append(value)
        return self.value

    @property
    def sample_count(self) -> int:
        return len(self._values)

    @property
    def value(self) -> Optional[ShelfCornerMeasurement]:
        if len(self._values) < self.minimum_samples:
            return None
        left = ShelfPlaneFilter._quantile(
            [measurement.left_clearance for measurement in self._values],
            self.quantile,
        )
        right = ShelfPlaneFilter._quantile(
            [measurement.right_clearance for measurement in self._values],
            self.quantile,
        )
        normal_y = (right - left) / (2.0 * self.half_width)
        yaw_error = math.asin(max(-1.0, min(1.0, normal_y)))
        return ShelfCornerMeasurement(
            left_clearance=left,
            right_clearance=right,
            shelf_yaw_error=yaw_error,
            left_point_count=min(
                measurement.left_point_count for measurement in self._values
            ),
            right_point_count=min(
                measurement.right_point_count for measurement in self._values
            ),
            lateral_span=min(
                measurement.lateral_span for measurement in self._values
            ),
        )


@dataclass(frozen=True)
class FineAlignCommand:
    linear_x: float
    angular_z: float
    phase: str
    within_tolerance: bool


@dataclass(frozen=True)
class ShelfTraverseCommand:
    linear_x: float
    linear_y: float
    angular_z: float
    phase: str
    within_tolerance: bool


def effective_angular_command(
    yaw_error: float,
    yaw_tolerance: float,
    rotate_kp: float,
    maximum_angular_speed: float,
    minimum_angular_speed: float,
) -> float:
    """Keep corrective rotation above the deployed swerve module deadband."""
    angular = max(
        -maximum_angular_speed,
        min(maximum_angular_speed, rotate_kp * yaw_error),
    )
    minimum = min(maximum_angular_speed, max(0.0, minimum_angular_speed))
    if abs(yaw_error) > yaw_tolerance and abs(angular) < minimum:
        angular = math.copysign(minimum, yaw_error)
    return angular


def compute_shelf_traverse_command(
    lateral_error: float,
    lateral_tolerance: float,
    clearance: float,
    target_clearance: float,
    clearance_tolerance: float,
    yaw_error: float,
    yaw_tolerance: float,
    *,
    clearance_hysteresis: float,
    yaw_align_gate: float,
    rotate_kp: float,
    approach_kp: float,
    lateral_kp: float,
    maximum_angular_speed: float,
    maximum_forward_speed: float,
    minimum_forward_speed: float,
    maximum_lateral_speed: float,
    minimum_lateral_speed: float,
    lateral_only: bool = False,
    minimum_angular_speed: float = 0.0,
) -> ShelfTraverseCommand:
    """Follow a shelf face laterally, optionally without longitudinal correction."""
    lower_clearance = target_clearance - clearance_tolerance
    upper_clearance = target_clearance + clearance_tolerance
    in_clearance_band = (
        lower_clearance - clearance_hysteresis
        <= clearance
        <= upper_clearance + clearance_hysteresis
    )
    if not lateral_only and clearance < lower_clearance - clearance_hysteresis:
        return ShelfTraverseCommand(0.0, 0.0, 0.0, "TOO_CLOSE", False)

    angular = effective_angular_command(
        yaw_error,
        yaw_tolerance,
        rotate_kp,
        maximum_angular_speed,
        minimum_angular_speed,
    )
    if abs(yaw_error) > yaw_align_gate:
        return ShelfTraverseCommand(0.0, 0.0, angular, "ALIGN_HEADING", False)

    if not lateral_only and clearance > upper_clearance + clearance_hysteresis:
        forward = min(
            maximum_forward_speed,
            approach_kp * (clearance - target_clearance),
        )
        forward = max(minimum_forward_speed, forward)
        return ShelfTraverseCommand(
            forward, 0.0, angular, "RESTORE_CLEARANCE", False
        )

    within = (
        abs(lateral_error) <= lateral_tolerance
        and (lateral_only or in_clearance_band)
        and abs(yaw_error) <= yaw_tolerance
    )
    if within:
        return ShelfTraverseCommand(0.0, 0.0, 0.0, "STABILIZING", True)

    if abs(lateral_error) <= lateral_tolerance:
        return ShelfTraverseCommand(
            0.0, 0.0, angular, "ALIGN_HEADING_CLOSE", False
        )
    lateral = max(
        -maximum_lateral_speed,
        min(maximum_lateral_speed, lateral_kp * lateral_error),
    )
    if abs(lateral) < minimum_lateral_speed:
        lateral = math.copysign(minimum_lateral_speed, lateral_error)
    return ShelfTraverseCommand(0.0, lateral, angular, "TRAVERSE", False)


def compute_shelf_corner_command(
    left_clearance: float,
    right_clearance: float,
    target_clearance: float,
    clearance_tolerance: float,
    shelf_yaw_error: float,
    yaw_tolerance: float,
    *,
    target_clearance_hysteresis: float,
    yaw_align_gate: float,
    rotate_kp: float,
    approach_kp: float,
    maximum_angular_speed: float,
    maximum_linear_speed: float,
    minimum_linear_speed: float,
    minimum_angular_speed: float = 0.0,
) -> FineAlignCommand:
    """Approach using two-corner ranges with robust close-range settling."""
    minimum_clearance = min(left_clearance, right_clearance)
    lower_target = target_clearance - clearance_tolerance
    upper_target = target_clearance + clearance_tolerance
    in_near_target_band = (
        minimum_clearance >= lower_target - target_clearance_hysteresis
        and minimum_clearance <= upper_target + target_clearance_hysteresis
    )
    if in_near_target_band and abs(shelf_yaw_error) <= yaw_tolerance:
        return FineAlignCommand(0.0, 0.0, "STABILIZING", True)

    if minimum_clearance < lower_target - target_clearance_hysteresis:
        return FineAlignCommand(0.0, 0.0, "TOO_CLOSE", False)

    angular = effective_angular_command(
        shelf_yaw_error,
        yaw_tolerance,
        rotate_kp,
        maximum_angular_speed,
        minimum_angular_speed,
    )
    # Large errors are corrected in place. Once inside the yaw gate, a safe
    # forward command keeps every swerve module above its wheel-speed deadband
    # while angular correction continues. Near the target, translation is
    # held again so the closer corner cannot be walked into the shelf.
    if abs(shelf_yaw_error) > yaw_align_gate or minimum_clearance <= upper_target:
        return FineAlignCommand(0.0, angular, "ALIGN_SHELF_FACE", False)

    clearance_error = minimum_clearance - target_clearance
    if clearance_error <= clearance_tolerance:
        return FineAlignCommand(0.0, angular, "ALIGN_HEADING_CLOSE", False)

    scale_denominator = max(yaw_align_gate, yaw_tolerance)
    heading_scale = max(0.20, 1.0 - abs(shelf_yaw_error) / scale_denominator)
    linear = min(maximum_linear_speed, approach_kp * clearance_error)
    linear = max(minimum_linear_speed, linear * heading_scale)
    linear = min(maximum_linear_speed, linear)
    return FineAlignCommand(linear, angular, "APPROACH", False)


# Retain the original helper name for callers that construct corner
# clearances from a fitted plane.
compute_shelf_plane_command = compute_shelf_corner_command


def compute_command(
    clearance: float,
    target_clearance: float,
    clearance_tolerance: float,
    yaw_error: float,
    yaw_tolerance: float,
    *,
    yaw_align_gate: float,
    rotate_kp: float,
    approach_kp: float,
    maximum_angular_speed: float,
    maximum_linear_speed: float,
    minimum_linear_speed: float,
) -> FineAlignCommand:
    """Angle-first controller used only after Nav2 has stopped at the shelf."""
    absolute_yaw_error = abs(yaw_error)
    clearance_error = clearance - target_clearance
    within = (
        abs(clearance_error) <= clearance_tolerance
        and absolute_yaw_error <= yaw_tolerance
    )
    if within:
        return FineAlignCommand(0.0, 0.0, "STABILIZING", True)

    angular = max(
        -maximum_angular_speed,
        min(maximum_angular_speed, rotate_kp * yaw_error),
    )
    if absolute_yaw_error > yaw_align_gate:
        return FineAlignCommand(0.0, angular, "ALIGN_HEADING", False)

    # The deployment safety gate intentionally forbids reverse motion.  If a
    # shelf is already too close, stop and fail safely instead of backing up.
    if clearance_error < -clearance_tolerance:
        return FineAlignCommand(0.0, 0.0, "TOO_CLOSE", False)
    if clearance_error <= clearance_tolerance:
        return FineAlignCommand(0.0, angular, "ALIGN_HEADING_CLOSE", False)

    # Slow translation as heading approaches the gate; never advance while a
    # large heading correction is still required. Apply the effective-command
    # floor *after* this scaling: the deployed swerve controller suppresses
    # chassis requests below 0.015 m/s and wheel requests below 0.020 m/s, so
    # scaling an earlier floor can silently turn a valid approach request into
    # zero physical motion.
    linear = min(maximum_linear_speed, approach_kp * clearance_error)
    heading_scale = max(0.20, 1.0 - absolute_yaw_error / yaw_align_gate)
    linear = max(minimum_linear_speed, linear * heading_scale)
    linear = min(maximum_linear_speed, linear)
    return FineAlignCommand(
        linear,
        angular,
        "APPROACH",
        False,
    )
