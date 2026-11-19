"""Offline search planning for the isolated multi-product retail fixture."""

from dataclasses import dataclass, asdict
import math
import re


PRODUCT_ALIASES = {
    "cracker_box": ("cracker", "盒装饼干", "饼干"),
    "sugar_box": ("sugar", "糖盒", "砂糖盒"),
    "tomato_soup_can": ("番茄汤罐", "番茄汤", "番茄罐头"),
    "tuna_fish_can": ("tuna", "金枪鱼罐", "金枪鱼罐头"),
    "potted_meat_can": ("午餐肉罐", "午餐肉罐头", "肉罐头"),
    "banana": ("banana", "香蕉"),
    "fruit_snacks_grape": ("grape", "葡萄软糖", "葡萄果汁软糖"),
    "fruit_snacks_juicy": ("juicy", "果汁软糖", "水果软糖"),
    "mustard_bottle": ("mustard", "芥末瓶", "芥末"),
    "pitcher_base": ("pitcher", "水壶", "壶"),
    "mug": ("mug", "马克杯", "杯子"),
    "coffee_mug": ("coffee mug", "咖啡杯"),
    "milk_frother": ("milk frother", "奶泡器", "打奶泡器"),
    "porcelain_bowl": ("porcelain bowl", "瓷碗"),
    "plastic_bowl": ("plastic bowl", "塑料碗"),
    "elderberry_syrup": ("elderberry", "接骨木莓糖浆", "莓果糖浆"),
    "brisk_tea": ("brisk tea", "冰茶", "茶饮"),
    "bleach_cleanser": ("bleach", "漂白清洁剂", "清洁剂"),
    "large_marker": ("marker", "大号记号笔", "马克笔"),
}


@dataclass(frozen=True)
class BasePose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class HeadObservation:
    base_pose: BasePose
    head_yaw_rad: float
    head_pitch_rad: float
    phase: str
    target_instance_ids: tuple
    attempt: int = 0
    lift_m: float = 0.0


def _finite_vector(values, size, field):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f"{field} must contain {size} values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field} must be finite")
    return result


def _center(bounds):
    lower = _finite_vector(bounds["min"], 3, "bounds.min")
    upper = _finite_vector(bounds["max"], 3, "bounds.max")
    if any(high <= low for low, high in zip(lower, upper)):
        raise ValueError("Product bounds must have positive extent")
    return tuple((low + high) / 2 for low, high in zip(lower, upper))


def _normalise(text):
    return re.sub(r"[\s_\-]+", "", str(text).casefold())


def _aliases(product):
    sku = str(product.get("simulation_sku", ""))
    return (sku, str(product.get("display_name", ""))) + PRODUCT_ALIASES.get(sku, ())


def resolve_product_instances(products, query):
    """Resolve a natural-language query to fixture candidates, without visual claims."""
    normalised_query = _normalise(query)
    if not normalised_query:
        raise ValueError("target query must not be empty")
    exact = []
    partial = []
    for product in products:
        instance_id = product.get("instance_id")
        if not instance_id:
            raise ValueError("Every product must have an instance_id")
        aliases = [_normalise(alias) for alias in _aliases(product) if alias]
        if normalised_query in aliases:
            exact.append(product)
        elif any(normalised_query in alias or alias in normalised_query for alias in aliases):
            partial.append(product)
    matches = exact or partial
    if not matches:
        raise LookupError(f"No simulation product matches target: {query}")
    return tuple(matches)


def _shelf_normal(shelf_id, face):
    if not str(shelf_id).startswith("shelf_group_") or face not in {"neg", "pos"}:
        raise ValueError("Product shelf identity is incomplete")
    return -1.0 if face == "neg" else 1.0


def _station_for_product(product, clearance_m):
    support = product.get("support_bounds")
    if not isinstance(support, dict):
        raise ValueError("Product support bounds are required")
    center = _center(support)
    normal = _shelf_normal(product.get("shelf_id"), product.get("face"))
    x = center[0] + normal * (abs(support["max"][0] - support["min"][0]) / 2 + clearance_m)
    target = _center(product.get("world_mesh_bounds"))
    yaw = math.atan2(target[1] - center[1], target[0] - x)
    return BasePose(x_m=x, y_m=target[1], yaw_rad=yaw)


def _head_angles(base_pose, target_center, camera_height_m=1.312):
    dx = target_center[0] - base_pose.x_m
    dy = target_center[1] - base_pose.y_m
    horizontal = math.hypot(dx, dy)
    if horizontal < 1e-6:
        raise ValueError("Observation station is coincident with target")
    desired_yaw = math.atan2(dy, dx)
    head_yaw = math.atan2(math.sin(desired_yaw - base_pose.yaw_rad),
                          math.cos(desired_yaw - base_pose.yaw_rad))
    head_pitch = math.atan2(target_center[2] - camera_height_m, horizontal)
    return head_yaw, head_pitch


def _dedupe_observations(observations):
    result = []
    seen = set()
    for observation in observations:
        key = tuple(round(value, 6) for value in (
            observation.base_pose.x_m, observation.base_pose.y_m,
            observation.base_pose.yaw_rad, observation.head_yaw_rad,
            observation.head_pitch_rad, observation.lift_m)) + observation.target_instance_ids
        if key not in seen:
            seen.add(key)
            result.append(observation)
    return tuple(result)


def build_search_plan(products, query, current_base=(0.0, 0.0, 0.0),
                      clearance_m=0.75, max_head_yaw_rad=1.45,
                      max_head_pitch_rad=1.4):
    """Build a perception search sequence; it does not command a base or head."""
    if clearance_m <= 0 or not math.isfinite(clearance_m):
        raise ValueError("clearance_m must be positive and finite")
    current = BasePose(*_finite_vector(current_base, 3, "current_base"))
    candidates = resolve_product_instances(products, query)
    observations = []
    target_observations = []
    for product in candidates:
        target = _center(product.get("world_mesh_bounds"))
        station = _station_for_product(product, clearance_m)
        station_variants = (
            station,
            BasePose(station.x_m, station.y_m - 0.35, station.yaw_rad),
            BasePose(station.x_m, station.y_m + 0.35, station.yaw_rad),
        )
        attempt = 0
        lift_variants = (0.0, 0.15, 0.3) if product.get("level", 0) >= 3 else (0.0,)
        for lift_m in lift_variants:
            for variant in station_variants:
                direct_yaw, direct_pitch = _head_angles(
                    variant, target, camera_height_m=1.312 + lift_m)
                offsets = ((0.0, 0.0), (-0.45, 0.0), (0.45, 0.0),
                           (0.0, -0.25), (0.0, 0.25))
                for yaw_offset, pitch_offset in offsets:
                    head_yaw = math.atan2(math.sin(direct_yaw + yaw_offset),
                                          math.cos(direct_yaw + yaw_offset))
                    head_pitch = direct_pitch + pitch_offset
                    if abs(head_yaw) > max_head_yaw_rad or abs(head_pitch) > max_head_pitch_rad:
                        continue
                    target_observations.append(HeadObservation(
                        variant, head_yaw, head_pitch, "target_observation",
                        (product["instance_id"],), attempt, lift_m))
                    attempt += 1
    observations.sort(key=lambda item: math.hypot(
        item.base_pose.x_m - current.x_m, item.base_pose.y_m - current.y_m))
    head_scan = tuple(HeadObservation(
        current, yaw, pitch, "head_scan", tuple(product["instance_id"] for product in candidates))
        for yaw, pitch in ((-1.2, -0.45), (-0.6, -0.45), (0.0, -0.45),
                           (0.6, -0.45), (1.2, -0.45), (0.0, 0.0)))
    observations = _dedupe_observations(head_scan + tuple(target_observations))
    stations = []
    for observation in observations:
        if observation.base_pose not in stations:
            stations.append(observation.base_pose)
    return {
        "query": str(query),
        "candidate_instance_ids": [product["instance_id"] for product in candidates],
        "identity_source": "simulation_manifest_candidate_not_live_visual_recognition",
        "requires_visual_confirmation": True,
        "execution_enabled": False,
        "current_base": asdict(current),
        "stations": [asdict(station) for station in stations],
        "observations": [asdict(observation) for observation in observations],
        "perception_contract": {
            "required_fields": ["instance_id", "simulation_sku", "confidence", "bbox_xyxy",
                                "depth_m", "observation_id"],
            "acceptance": "model_result_and_depth_must_be_fresh_and_match_target_before_motion",
            "segmentation_role": "MuJoCo_instance_segmentation_is_evaluation_ground_truth_only",
        },
        "safety_gates": ["fresh_joint_state", "base_control_lease", "observation_reachable",
                         "lift_and_head_path_validated", "approach_and_retreat_validated",
                         "grasp_precheck", "result_readback"],
    }
