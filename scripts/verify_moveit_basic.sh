#!/usr/bin/env bash
# MoveIt基础验证脚本
# 验证MoveIt核心组件是否可用

set -eo pipefail

workspace="/home/wgb/openflex_shop/openflex_ws"
cd "${workspace}"
set +u
source /opt/ros/humble/setup.bash
source install_v2/setup.bash
set -u

echo "=========================================="
echo "MoveIt 基础环境验证"
echo "=========================================="

echo ""
echo "1. 检查MoveIt包..."
ros2 pkg list | grep -c moveit || true

echo ""
echo "2. 检查MoveIt配置包..."
if ros2 pkg prefix openarmx_integrated_moveit_config >/dev/null 2>&1; then
    echo "✓ openarmx_integrated_moveit_config 已安装"
else
    echo "✗ openarmx_integrated_moveit_config 未找到"
    exit 1
fi

echo ""
echo "3. 检查MoveIt launch文件..."
LAUNCH_FILE="/home/wgb/openflex_shop/openflex_ws/src/openflex_integrated/openarmx_integrated_moveit_config/launch/sim_moveit.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✓ sim_moveit.launch.py 存在"
else
    echo "✗ sim_moveit.launch.py 未找到"
    exit 1
fi

echo ""
echo "4. 检查pick-ik插件..."
if ros2 pkg prefix pick_ik >/dev/null 2>&1; then
    echo "✓ pick-ik 已安装"
else
    echo "✗ pick-ik 未安装"
    exit 1
fi

echo ""
echo "5. 检查URDF描述包..."
if ros2 pkg prefix openarmx_integrated_description >/dev/null 2>&1; then
    echo "✓ openarmx_integrated_description 已安装"
else
    echo "✗ openarmx_integrated_description 未找到"
    exit 1
fi

echo ""
echo "=========================================="
echo "✓ MoveIt基础环境验证通过"
echo "=========================================="
echo ""
echo "下一步："
echo "  1. 启动MuJoCo仿真: bash scripts/run_thesis_offline.sh"
echo "  2. 另一个终端启动MoveIt: bash scripts/start_moveit_sim.sh"
echo "  3. 验证IK和规划: bash scripts/test_moveit_ik.sh"
