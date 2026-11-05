#!/bin/bash
# G6 测试结果分析脚本

LOG_FILE="$1"

if [ -z "$LOG_FILE" ] || [ ! -f "$LOG_FILE" ]; then
    echo "用法: $0 <log_file>"
    exit 1
fi

echo "=== G6 测试结果分析 ==="
echo ""

echo "1. Mission 最终状态:"
grep -E "MUJOCO_MISSION_(SUCCESS|FAILED)" "$LOG_FILE" | tail -1
echo ""

echo "2. 各阶段进度:"
grep "phase=" "$LOG_FILE" | tail -20
echo ""

echo "3. 放置相关日志:"
grep -E "execute fixed delivery|removed attached object|no object is attached|place_status" "$LOG_FILE" | tail -10
echo ""

echo "4. 错误和警告:"
grep -E "ERROR|WARN|Ignoring unexpected" "$LOG_FILE" | tail -10
echo ""

echo "5. 关键事件时间线:"
grep -E "phase=13|phase=14|phase=15|phase=16|phase=17" "$LOG_FILE"
echo ""

echo "6. 附着体状态变化:"
grep -E "attached|detach|removed attached" "$LOG_FILE" | tail -15
