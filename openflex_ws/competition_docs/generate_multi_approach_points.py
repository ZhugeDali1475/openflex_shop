#!/usr/bin/env python3
"""
基于数据库自动生成多接近点配置

根据货架库存数据库，为每个货架面生成多个接近点，
确保机器人右手能够覆盖所有商品位置。

使用方法：
    python3 generate_multi_approach_points.py [--coverage 50] [--clearance 0.05]
"""

import sqlite3
import math
import yaml
import argparse
from pathlib import Path
from typing import List, Tuple, Dict


def normalize_angle(angle):
    """归一化角度到[-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def calculate_approach_point(
    shelf_center_x: float,
    shelf_center_y: float,
    shelf_yaw: float,
    shelf_width: float,
    target_y_cm: float,
    face: int,
    clearance: float = 0.05
) -> Tuple[float, float, float]:
    """
    计算货架接近点坐标

    参数:
        shelf_center_x, shelf_center_y: 货架中心世界坐标（m）
        shelf_yaw: 货架朝向（rad）
        shelf_width: 货架宽度（m）
        target_y_cm: 目标Y坐标（沿货架长度，cm）
        face: 0（面0）或1（面1）
        clearance: 精调目标净距（m）

    返回:
        (x, y, yaw) 接近点世界坐标和朝向
    """
    robot_front = 0.46  # 机器人前端到base_link距离

    # 货架坐标系 → 世界坐标系变换
    cos_yaw = math.cos(shelf_yaw)
    sin_yaw = math.sin(shelf_yaw)

    # 货架坐标系中的位置
    local_x = 0.0 if face == 0 else shelf_width
    local_y = target_y_cm / 100.0  # cm转m

    # 面的世界坐标
    face_world_x = shelf_center_x + local_x * cos_yaw - local_y * sin_yaw
    face_world_y = shelf_center_y + local_x * sin_yaw + local_y * cos_yaw

    # 面的法线朝向（朝外）
    face_yaw = shelf_yaw if face == 0 else normalize_angle(shelf_yaw + math.pi)

    # 接近点距离面前沿的距离
    approach_distance = clearance + robot_front

    # 接近点坐标
    approach_x = face_world_x + approach_distance * math.cos(face_yaw)
    approach_y = face_world_y + approach_distance * math.sin(face_yaw)
    approach_yaw = normalize_angle(face_yaw + math.pi)  # 机器人面向货架

    return approach_x, approach_y, approach_yaw


def generate_approach_points(
    shelf_id: int,
    shelf_name: str,
    shelf_x: float,
    shelf_y: float,
    shelf_yaw: float,
    shelf_width: float,
    shelf_length: float,
    face: int,
    coverage_cm: float = 50.0,
    clearance: float = 0.05,
    edge_margin_cm: float = 10.0
) -> List[Dict]:
    """
    为一个货架面生成多个接近点

    参数:
        coverage_cm: 每个接近点的有效覆盖范围（cm）
        edge_margin_cm: 边缘裕度（cm），防止接近墙壁

    返回:
        接近点列表
    """
    shelf_length_cm = shelf_length * 100.0

    # 计算需要的接近点数量
    effective_length = shelf_length_cm - 2 * edge_margin_cm
    num_points = max(2, math.ceil(effective_length / coverage_cm))

    # 均匀分布接近点
    points = []
    if num_points == 1:
        # 只需一个点，放在中心
        positions = [shelf_length_cm / 2.0]
    else:
        # 多个点，从左到右均匀分布
        start = edge_margin_cm + coverage_cm / 2.0
        end = shelf_length_cm - edge_margin_cm - coverage_cm / 2.0
        step = (end - start) / (num_points - 1) if num_points > 1 else 0
        positions = [start + i * step for i in range(num_points)]

    # 生成接近点
    position_labels = ["LEFT", "MID_L", "MID_R", "RIGHT", "P5", "P6"]
    for i, y_cm in enumerate(positions):
        x, y, yaw = calculate_approach_point(
            shelf_x, shelf_y, shelf_yaw, shelf_width,
            y_cm, face, clearance
        )

        # 生成站点ID和标签
        label = position_labels[i] if i < len(position_labels) else f"P{i+1}"
        station_id = f"SHELF_{shelf_id}_FACE_{face}_{label}"
        station_label = f"{shelf_name}-面{face}-{label}"

        points.append({
            "id": station_id,
            "label": station_label,
            "role": "shelf_approach",
            "pose": {
                "x": round(x, 6),
                "y": round(y, 6),
                "yaw": round(yaw, 6)
            },
            "coverage_start_cm": max(0, y_cm - coverage_cm / 2.0),
            "coverage_end_cm": min(shelf_length_cm, y_cm + coverage_cm / 2.0),
            "action": {
                "fine_align": {
                    "enabled": True,
                    "mode": "front_surface_scan",
                    "target_clearance_m": clearance,
                    "clearance_tolerance_m": 0.01,
                    "yaw_tolerance_deg": 1.0,
                    "timeout_sec": 20.0
                },
                "retreat": {
                    "after_pick": True,
                    "after_place": False,
                    "distance_m": 0.15,
                    "speed_mps": 0.04
                }
            }
        })

    return points


def main():
    parser = argparse.ArgumentParser(description="生成多接近点配置")
    parser.add_argument("--db", default="/home/openflex/openflex_shop/supermarket_scene/shelf_inventory.db",
                       help="货架库存数据库路径")
    parser.add_argument("--coverage", type=float, default=50.0,
                       help="每个接近点的覆盖范围（cm）")
    parser.add_argument("--clearance", type=float, default=0.05,
                       help="精调目标净距（m）")
    parser.add_argument("--edge-margin", type=float, default=10.0,
                       help="边缘裕度（cm）")
    parser.add_argument("--output-dir", default="/home/openflex/old/openflex_maps/2d/supermarket_test_030",
                       help="输出目录")
    args = parser.parse_args()

    print("=" * 70)
    print("多接近点配置生成器")
    print("=" * 70)
    print(f"数据库: {args.db}")
    print(f"覆盖范围: {args.coverage}cm")
    print(f"精调净距: {args.clearance}m")
    print(f"边缘裕度: {args.edge_margin}cm")
    print()

    # 连接数据库
    conn = sqlite3.connect(args.db)
    cursor = conn.cursor()

    # 查询货架信息
    cursor.execute("""
        SELECT sg.id, sg.name, sg.world_x, sg.world_y, sg.yaw,
               st.shelf_width, st.shelf_length
        FROM shelf_groups sg
        JOIN shelf_types st ON sg.shelf_type_id = st.id
        ORDER BY sg.id
    """)

    shelves = cursor.fetchall()

    # 为每个货架生成接近点
    all_stations = []
    shelf_face_mapping = {}

    for shelf_id, name, world_x, world_y, yaw, width, length in shelves:
        print(f"处理 {name} (ID={shelf_id}):")
        print(f"  位置: ({world_x:.4f}, {world_y:.4f})")
        print(f"  朝向: {math.degrees(yaw):.1f}°")
        print(f"  尺寸: {length:.2f}m × {width:.2f}m")

        # 为面0和面1生成接近点
        for face in [0, 1]:
            points = generate_approach_points(
                shelf_id, name, world_x, world_y, yaw,
                width, length, face,
                coverage_cm=args.coverage,
                clearance=args.clearance,
                edge_margin_cm=args.edge_margin
            )

            print(f"  面{face}: 生成 {len(points)} 个接近点")
            for point in points:
                print(f"    - {point['id']}: ({point['pose']['x']:.4f}, {point['pose']['y']:.4f}) "
                      f"覆盖Y {point['coverage_start_cm']:.0f}~{point['coverage_end_cm']:.0f}cm")

            all_stations.extend(points)

            # 记录映射
            key = f"{shelf_id}:{face}"
            shelf_face_mapping[key] = [p["id"] for p in points]

        print()

    conn.close()

    # 添加固定站点
    fixed_stations = [
        {
            "id": "START",
            "label": "起点",
            "role": "start_position",
            "pose": {"x": 0.002498, "y": -0.278702, "yaw": 1.5687},
            "action": {
                "fine_align": {"enabled": False, "mode": "front_surface_scan",
                              "target_clearance_m": 0.2, "clearance_tolerance_m": 0.01,
                              "yaw_tolerance_deg": 1.0, "timeout_sec": 20.0},
                "retreat": {"after_pick": False, "after_place": False,
                           "distance_m": 0.15, "speed_mps": 0.04}
            }
        },
        {
            "id": "PRE_START",
            "label": "起点缓冲区",
            "role": "waypoint",
            "pose": {"x": 0.0031, "y": 0.0213, "yaw": 1.5687},
            "action": {
                "fine_align": {"enabled": False, "mode": "front_surface_scan",
                              "target_clearance_m": 0.2, "clearance_tolerance_m": 0.01,
                              "yaw_tolerance_deg": 1.0, "timeout_sec": 20.0},
                "retreat": {"after_pick": False, "after_place": False,
                           "distance_m": 0.15, "speed_mps": 0.04}
            }
        }
    ]

    # 生成 competition_mission.yaml
    mission = {
        "schema_version": 2,
        "map_name": "supermarket_test_030",
        "stations": fixed_stations + all_stations
    }

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    mission_path = output_dir / "competition_mission.yaml"
    with open(mission_path, 'w', encoding='utf-8') as f:
        yaml.dump(mission, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

    print(f"✅ 已生成: {mission_path}")
    print(f"   总站点数: {len(mission['stations'])}")
    print(f"   货架接近点: {len(all_stations)}")
    print()

    # 生成 shelf_face_stations 映射配置
    print("=" * 70)
    print("competition.yaml 配置片段（复制到 inventory 部分）:")
    print("=" * 70)
    print("inventory:")
    print("  enabled: true")
    print(f"  database_path: {args.db}")
    print("  prefer_inventory: true")
    print("  shelf_face_stations:")
    for key, station_ids in sorted(shelf_face_mapping.items()):
        print(f"    \"{key}\": {station_ids}")
    print()

    print("=" * 70)
    print("完成！")
    print("=" * 70)
    print("下一步:")
    print("1. 在GUI中加载地图验证接近点位置")
    print("2. 更新 competition.yaml 中的 shelf_face_stations 配置")
    print("3. 在临时环境测试导航和抓取")
    print()


if __name__ == "__main__":
    main()
