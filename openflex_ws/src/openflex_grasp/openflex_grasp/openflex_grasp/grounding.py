import json
import re
from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    label: str
    box_xyxy: tuple[int, int, int, int]
    confidence: float
    raw_response: str


def _json_candidates(text: str):
    cleaned = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL)
    cleaned = re.sub(r"```(?:json)?", "", cleaned, flags=re.IGNORECASE)
    cleaned = cleaned.replace("```", "").strip()
    yield cleaned
    start = cleaned.find("{")
    end = cleaned.rfind("}")
    if 0 <= start < end:
        yield cleaned[start : end + 1]


def _extract_box(payload):
    if isinstance(payload, dict):
        for key in ("bbox_2d", "bbox", "box", "box_xyxy"):
            value = payload.get(key)
            if isinstance(value, (list, tuple)) and len(value) == 4:
                return value, payload
        for key in ("boxes", "detections", "objects", "results"):
            value = payload.get(key)
            if isinstance(value, list) and value:
                for item in value:
                    found = _extract_box(item)
                    if found:
                        return found
            elif isinstance(value, dict):
                found = _extract_box(value)
                if found:
                    return found
    elif isinstance(payload, list):
        for item in payload:
            found = _extract_box(item)
            if found:
                return found
    return None


def _extract_boxes(payload):
    found = []
    if isinstance(payload, dict):
        for key in ("bbox_2d", "bbox", "box", "box_xyxy"):
            value = payload.get(key)
            if isinstance(value, (list, tuple)) and len(value) == 4:
                return [(value, payload)]
        for key in ("boxes", "detections", "objects", "results"):
            value = payload.get(key)
            if isinstance(value, (list, dict)):
                found.extend(_extract_boxes(value))
    elif isinstance(payload, list):
        for item in payload:
            found.extend(_extract_boxes(item))
    return found


def _scale_box(values, width: int, height: int) -> tuple[int, int, int, int]:
    coords = [float(value) for value in values]
    maximum = max(abs(value) for value in coords)
    if maximum <= 1.5:
        coords = [coords[0] * width, coords[1] * height, coords[2] * width, coords[3] * height]
    else:
        # The grounding prompt requires coordinates normalized to 0..1000.
        coords = [
            coords[0] * width / 1000.0,
            coords[1] * height / 1000.0,
            coords[2] * width / 1000.0,
            coords[3] * height / 1000.0,
        ]
    x1, y1, x2, y2 = coords
    x1, x2 = sorted((round(x1), round(x2)))
    y1, y2 = sorted((round(y1), round(y2)))
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(x1 + 1, min(width, x2))
    y2 = max(y1 + 1, min(height, y2))
    if (x2 - x1) * (y2 - y1) < 16:
        raise ValueError(f"grounding box is too small: {(x1, y1, x2, y2)}")
    return x1, y1, x2, y2


def parse_detection(response: str, target: str, width: int, height: int) -> Detection:
    payload = None
    for candidate in _json_candidates(response):
        try:
            payload = json.loads(candidate)
            break
        except json.JSONDecodeError:
            continue

    found = _extract_box(payload) if payload is not None else None
    if found is None:
        match = re.search(
            r"(?:bbox_2d|bbox|box)?[^\d-]*"
            r"(-?\d+(?:\.\d+)?)\D+(-?\d+(?:\.\d+)?)\D+"
            r"(-?\d+(?:\.\d+)?)\D+(-?\d+(?:\.\d+)?)",
            response,
            flags=re.IGNORECASE,
        )
        if match is None:
            raise ValueError(f"Qwen did not return a parseable bounding box: {response!r}")
        values = [float(value) for value in match.groups()]
        metadata = {}
    else:
        values, metadata = found

    label = str(metadata.get("label", target))
    try:
        confidence = float(metadata.get("confidence", metadata.get("score", 1.0)))
    except (TypeError, ValueError):
        confidence = 1.0
    confidence = max(0.0, min(1.0, confidence))
    return Detection(label, _scale_box(values, width, height), confidence, response)


def _box_iou(first, second) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0 else 0.0


def parse_detections(
    response: str,
    target: str,
    width: int,
    height: int,
    *,
    duplicate_iou: float = 0.85,
) -> list[Detection]:
    payload = None
    for candidate in _json_candidates(response):
        try:
            payload = json.loads(candidate)
            break
        except json.JSONDecodeError:
            continue

    records = _extract_boxes(payload) if payload is not None else []
    if not records:
        return [parse_detection(response, target, width, height)]

    detections = []
    for values, metadata in records:
        label = str(metadata.get("label", target))
        try:
            confidence = float(
                metadata.get("confidence", metadata.get("score", 1.0))
            )
        except (TypeError, ValueError):
            confidence = 1.0
        detections.append(
            Detection(
                label,
                _scale_box(values, width, height),
                max(0.0, min(1.0, confidence)),
                response,
            )
        )

    selected = []
    for detection in sorted(detections, key=lambda item: item.confidence, reverse=True):
        if all(
            _box_iou(detection.box_xyxy, existing.box_xyxy) < duplicate_iou
            for existing in selected
        ):
            selected.append(detection)
    return selected
