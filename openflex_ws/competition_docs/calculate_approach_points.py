#!/usr/bin/env python3
"""
计算货架和交付台的安全接近点坐标

使用方法:
    python3 calculate_approach_points.py

按照提示输入货架/交付台的位置信息，自动计算接近点坐标
"""

import math
import yaml
from pathlib import Path


def normalize_angle(angle):
    """归一化角度到[-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def calculate_approach_point(target_x, target_y, target_yaw, clearance, robot_front=0.46):
    """
    计算接近点坐标

    参数:
        target_x, target_y: 目标前沿中心点坐标
        target_yaw: 目标朝向（法线方向，朝外）
        clearance: 精调目标净距（货架0.05m，交付台0.20m）
        robot_front: 机器人前端到base_link距离（默认0.46m）

    返回:
        (x, y, yaw) 接近点坐标
    """
    # 接近点在目标前方 (clearance + robot_front) 距离
    approach_distance = clearance + robot_front

    # 计算接近点位置
    x = target_x + approach_distance * math.cos(target_yaw)
    y = target_y + approach_distance * math.sin(target_yaw)

    # 机器人朝向目标（反向）
    yaw = normalize_angle(target_yaw + math.pi)

    return x, y, yaw


def main():
    print("=" * 70)
    print("OpenFlex 比赛站点坐标计算工具")
    print("=" * 70)
    print()
    print("机器人参数:")
    print("  尺寸: 长0.92m × 宽0.90m")
    print("  前端到base_link: 0.46m")
    print("  碰撞监控足迹: ±0.46m × ±0.45m")
    print()
    print("距离规则:")
    print("  货架接近点: 精调净距0.05m → 导航目标0.51m")
    print("  交付台接近点: 精调净距0.20m → 导航目标0.66m")
    print()

    stations = []

    # 固定的起点和缓冲点
    start_x, start_y, start_yaw = 0.00249837, -0.278702, 1.5687
    pre_start_x = start_x + 0.30 * math.cos(start_yaw)
    pre_start_y = start_y + 0.30 * math.sin(start_yaw)

    stations.append({
        "id": "START",
        "label": "起点",
        "role": "start_position",
        "pose": {"x": start_x, "y": start_y, "yaw": start_yaw},
        "action": {
            "fine_align": {
                "enabled": False,
                "mode": "front_surface_scan",
                "target_clearance_m": 0.2,
                "clearance_tolerance_m": 0.01,
                "yaw_tolerance_deg": 1.0,
                "timeout_sec": 20.0
            },
            "retreat": {
                "after_pick": False,
                "after_place": False,
                "distance_m": 0.15,
                "speed_mps": 0.04
            }
        }
    })

    stations.append({
        "id": "PRE_START",
        "label": "起点缓冲区",
        "role": "waypoint",
        "pose": {"x": pre_start_x, "y": pre_start_y, "yaw": start_yaw},
        "action": {
            "fine_align": {
                "enabled": False,
                "mode": "front_surface_scan",
                "target_clearance_m": 0.2,
                "clearance_tolerance_m": 0.01,
                "yaw_tolerance_deg": 1.0,
                "timeout_sec": 20.0
            },
            "retreat": {
                "after_pick": False,
                "after_place": False,
                "distance_m": 0.15,
                "speed_mps": 0.04
            }
        }
    })

    print("\n已添加固定站点:")
    print(f"  START: ({start_x:.4f}, {start_y:.4f}, {math.degrees(start_yaw):.1f}°)")
    print(f"  PRE_START: ({pre_start_x:.4f}, {pre_start_y:.4f}, {math.degrees(start_yaw):.1f}°)")
    print()

    while True:
        print("=" * 70)
        print("添加新站点")
        print("=" * 70)

        station_type = input("站点类型 (1=货架, 2=交付台, q=完成): ").strip()
        if station_type.lower() == 'q':
            break

        if station_type not in ['1', '2']:
            print("⚠️  无效输入，请输入1、2或q")
            continue

        is_shelf = (station_type == '1')
        type_name = "货架" if is_shelf else "交付台"
        clearance = 0.05 if is_shelf else 0.20

        try:
            station_id = input(f"{type_name}站点ID (如SHELF_A, DELIVERY_B): ").strip().upper()
            if not station_id:
                print("⚠️  ID不能为空")
                continue

            label = input(f"站点标签 (默认={station_id}): ").strip() or station_id

            print(f"\n输入{type_name}前沿中心点坐标和朝向（法线方向，朝外）:")
            target_x = float(input("  X坐标 (m): "))
            target_y = float(input("  Y坐标 (m): "))

            yaw_input = input("  朝向角度 (输入度数或弧度，如90°或1.57): ").strip()
            if '°' in yaw_input or 'deg' in yaw_input.lower():
                target_yaw = math.radians(float(yaw_input.replace('°', '').replace('deg', '')))
            else:
                target_yaw = float(yaw_input)

            # 计算接近点
            x, y, yaw = calculate_approach_point(target_x, target_y, target_yaw, clearance)

            print(f"\n✅ 计算结果:")
            print(f"  {type_name}位置: ({target_x:.4f}, {target_y:.4f})")
            print(f"  {type_name}朝向: {math.degrees(target_yaw):.1f}°")
            print(f"  接近点坐标: ({x:.4f}, {y:.4f})")
            print(f"  接近点朝向: {math.degrees(yaw):.1f}°")
            print(f"  导航目标距离: {clearance + 0.46:.2f}m")
            print(f"  精调后净距: {clearance:.2f}m")

            station = {
                "id": station_id,
                "label": label,
                "role": "shelf_approach" if is_shelf else "delivery_table",
                "pose": {"x": round(x, 6), "y": round(y, 6), "yaw": round(yaw, 6)},
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
                        "after_pick": is_shelf,
                        "after_place": not is_shelf,
                        "distance_m": 0.15,
                        "speed_mps": 0.04
                    }
                }
            }

            stations.append(station)
            print(f"✅ 已添加站点 {station_id}")

        except ValueError as e:
            print(f"⚠️  输入错误: {e}")
            continue
        except KeyboardInterrupt:
            print("\n\n操作取消")
            break

    if len(stations) <= 2:
        print("\n没有添加新站点，退出")
        return

    # 生成配置文件
    print("\n" + "=" * 70)
    print("生成配置文件")
    print("=" * 70)

    mission = {
        "schema_version": 2,
        "map_name": "supermarket_test_030",
        "stations": stations
    }

    output_path = Path("/home/openflex/old/openflex_maps/2d/supermarket_test_030/competition_mission.yaml")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(mission, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

    print(f"✅ 已保存到: {output_path}")
    print()
    print("站点列表:")
    for station in stations:
        pose = station["pose"]
        print(f"  {station['id']:15s} ({pose['x']:7.4f}, {pose['y']:7.4f}, {math.degrees(pose['yaw']):6.1f}°) - {station['label']}")

    print()
    print("=" * 70)
    print("下一步:")
    print("=" * 70)
    print("1. 在GUI中加载 supermarket_test_030 地图")
    print("2. 验证站点位置是否合理")
    print("3. 必要时手动微调站点坐标")
    print("4. 保存最终配置")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序已退出")
