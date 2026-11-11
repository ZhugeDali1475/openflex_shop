import base64
from copy import deepcopy
from io import BytesIO
import math
import threading
import time

from PIL import Image


TOPICS = {
    "odometry": "/swerve_drive_controller/odom",
    "joint_states": "/joint_states",
    "grasp_status": "/thesis/grasp_status",
    "place_status": "/thesis/place/status",
    "object_observation": "/thesis/object_observation",
    "object_memory": "/thesis/object_memory",
    "control_mode": "/thesis/control_mode",
    "mission_state": "/thesis/mission_state",
    "affordance": "/thesis/tc_wam/state",
    "camera": "/thesis_sim/head/color/image_raw",
    "simulation_diagnostic": "/thesis_sim/diagnostic_state",
}


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def encode_image(message):
    modes = {"rgb8": ("RGB", "RGB"), "bgr8": ("RGB", "BGR"),
             "rgba8": ("RGBA", "RGBA"), "bgra8": ("RGBA", "BGRA"),
             "mono8": ("L", "L")}
    if message.encoding not in modes:
        raise ValueError(f"Unsupported image encoding: {message.encoding}")
    if not (0 < message.width <= 4096 and 0 < message.height <= 4096):
        raise ValueError("Invalid image dimensions")
    mode, raw_mode = modes[message.encoding]
    channels = len(mode)
    if message.step < message.width * channels or len(message.data) != message.step * message.height:
        raise ValueError("Invalid image stride or buffer length")
    image = Image.frombytes(mode, (message.width, message.height), bytes(message.data),
                            "raw", raw_mode, message.step, 1).convert("RGB")
    image.thumbnail((640, 480))
    output = BytesIO()
    image.save(output, format="JPEG", quality=85)
    return {"mime_type": "image/jpeg", "width": image.width, "height": image.height,
            "data": base64.b64encode(output.getvalue()).decode("ascii")}


class SnapshotCache:
    def __init__(self, ros_now, monotonic=time.monotonic, max_age_sec=3.0, registry=None):
        self.ros_now = ros_now
        self.monotonic = monotonic
        self.max_age_sec = max_age_sec
        self.samples = {}
        self.lock = threading.Lock()
        self.registry = deepcopy(registry) if registry is not None else {
            "status": "unavailable", "execution_enabled": False,
        }

    def set_registry(self, registry):
        with self.lock:
            self.registry = deepcopy(registry)

    def update(self, key, data, source_stamp_ns):
        if key not in TOPICS:
            raise KeyError(key)
        now = self.monotonic()
        with self.lock:
            previous = self.samples.get(key)
            advanced = now
            if previous and previous["source_stamp_ns"] == source_stamp_ns:
                advanced = previous["advanced"]
            self.samples[key] = {"data": json_safe(deepcopy(data)), "source_stamp_ns": source_stamp_ns,
                                 "received": now, "advanced": advanced, "error": None}

    def invalidate(self, key, error):
        with self.lock:
            self.samples[key] = {"data": None, "source_stamp_ns": None,
                                 "received": self.monotonic(), "advanced": self.monotonic(),
                                 "error": str(error)}

    def _sample(self, key, now, ros_now):
        sample = self.samples.get(key)
        result = {"topic": TOPICS[key], "status": "missing", "data": None,
                  "received_age_ms": None, "source_age_ms": None, "source_stamp_ns": None,
                  "frame_id": None}
        if sample is None:
            return result
        stamp = sample["source_stamp_ns"]
        result.update(source_stamp_ns=stamp,
                      received_age_ms=round(max(0.0, now - sample["received"]) * 1000, 1),
                      source_advance_age_ms=round(max(0.0, now - sample["advanced"]) * 1000, 1))
        if sample["error"]:
            result.update(status="invalid", error=sample["error"])
            return result
        result["frame_id"] = sample["data"].get("header", {}).get("frame_id")
        if stamp is None or stamp <= 0 or ros_now <= 0:
            result["status"] = "clock_unavailable"
            return result
        source_age_sec = (ros_now - stamp) / 1e9
        result["source_age_ms"] = round(source_age_sec * 1000, 1)
        if source_age_sec < -0.25:
            result["status"] = "future_timestamp"
        elif max(now - sample["received"], now - sample["advanced"], source_age_sec) > self.max_age_sec:
            result["status"] = "stale"
        else:
            result.update(status="fresh", data=deepcopy(sample["data"]))
        return result

    def snapshot(self, observation=False):
        with self.lock:
            now, ros_now = self.monotonic(), self.ros_now()
            keys = ["camera", "object_observation", "object_memory"] if observation else [
                key for key in TOPICS if key != "camera"]
            sources = {key: self._sample(key, now, ros_now) for key in keys}
            registry = deepcopy(self.registry)
        fresh = [key for key, value in sources.items() if value["status"] == "fresh"]
        result = {"schema_version": 1, "robot": "openflex", "read_only": True,
                  "environment": "mujoco_simulation", "physical_verification": False,
                  "status": "available" if len(fresh) == len(keys) else "partial" if fresh else "unavailable",
                  "max_age_ms": self.max_age_sec * 1000, "sources": sources,
                  "warning": "Telemetry is not action completion. Simulation state/object poses are not physical evidence."}
        result["scene_registry"] = registry
        observation_sample = sources.get("object_observation", {})
        observed_object = (observation_sample.get("data") or {}).get("object_id")
        registered = registry.get("objects", {})
        result["object_binding"] = {
            "observed_object_id": observed_object,
            "status": "registered" if observed_object in registered else "unregistered" if observed_object else "unknown",
            "basis": "fresh_ROS_object_id_not_visual_identity",
            "visual_identity_verified": False,
        }
        if observation:
            result["camera_status"] = sources["camera"]["status"]
            result["provenance"] = {
                "camera": "MuJoCo head RGB; mesh-estimated mount and synthetic intrinsics, not physically calibrated",
                "object_observation": "MuJoCo ground truth/oracle, not image-derived perception",
                "object_memory": "Upstream memory; preserve source field, not independent visual proof",
            }
        else:
            grasp = sources["grasp_status"]["data"]
            holding = None
            if grasp and all(grasp.get(field) is True for field in (
                    "verified", "contact_detected", "gripper_closed", "object_retained")):
                holding = grasp.get("object_id") or None
            result["holding"] = holding
            result["holding_evidence"] = "reported_simulation_grasp" if holding else "unknown_or_unverified"
        return result
