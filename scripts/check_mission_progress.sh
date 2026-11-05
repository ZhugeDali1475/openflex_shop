#!/bin/bash
# 快速检查 mission 测试进度

LOG_FILE="$1"

if [ -z "$LOG_FILE" ]; then
    # 查找最新的测试日志
    LOG_FILE=$(ls -t /tmp/g6_test_*.log 2>/dev/null | head -1)
    if [ -z "$LOG_FILE" ]; then
        echo "未找到测试日志文件"
        exit 1
    fi
fi

echo "检查日志: $LOG_FILE"
echo "文件大小: $(du -h "$LOG_FILE" | cut -f1)"
echo "行数: $(wc -l < "$LOG_FILE")"
echo ""

echo "=== 最新进度 ==="
tail -30 "$LOG_FILE" | grep -E "phase=|MISSION|ERROR" || echo "暂无关键进度信息"
