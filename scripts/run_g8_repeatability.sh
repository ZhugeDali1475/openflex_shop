#!/bin/bash
# G8 重复性测试脚本 - 运行 10 次完整 mission

set -e

# 配置
TOTAL_RUNS=10
TIMEOUT_SEC=300
LOG_DIR="./openflex_ws/migration_records/2026-11-05/g8_repeatability"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY_FILE="${LOG_DIR}/g8_summary_${TIMESTAMP}.txt"

# 创建日志目录
mkdir -p "${LOG_DIR}"

echo "=== G8 Repeatability Test ===" | tee "${SUMMARY_FILE}"
echo "Start time: $(date)" | tee -a "${SUMMARY_FILE}"
echo "Total runs: ${TOTAL_RUNS}" | tee -a "${SUMMARY_FILE}"
echo "Timeout per run: ${TIMEOUT_SEC}s" | tee -a "${SUMMARY_FILE}"
echo "" | tee -a "${SUMMARY_FILE}"

# 统计变量
SUCCESS_COUNT=0
FAIL_COUNT=0
TIMEOUT_COUNT=0
declare -a DURATIONS
declare -a RESULTS

# 主循环
for i in $(seq 1 ${TOTAL_RUNS}); do
    echo "========================================" | tee -a "${SUMMARY_FILE}"
    echo "Run ${i}/${TOTAL_RUNS} - $(date)" | tee -a "${SUMMARY_FILE}"
    echo "========================================" | tee -a "${SUMMARY_FILE}"

    RUN_LOG="${LOG_DIR}/run_${i}_${TIMESTAMP}.log"
    START_TIME=$(date +%s)

    # 运行 mission
    if timeout ${TIMEOUT_SEC} bash scripts/run_thesis_moveit_mission.sh > "${RUN_LOG}" 2>&1; then
        EXIT_CODE=$?
    else
        EXIT_CODE=$?
    fi

    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))
    DURATIONS+=("${DURATION}")

    # 分析结果
    if [ ${EXIT_CODE} -eq 124 ]; then
        # Timeout
        RESULT="TIMEOUT"
        TIMEOUT_COUNT=$((TIMEOUT_COUNT + 1))
        echo "Result: TIMEOUT (>${TIMEOUT_SEC}s)" | tee -a "${SUMMARY_FILE}"
    elif grep -q "MUJOCO_MISSION_SUCCESS" "${RUN_LOG}"; then
        # Success
        RESULT="SUCCESS"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
        MISSION_DURATION=$(grep "MUJOCO_MISSION_SUCCESS" "${RUN_LOG}" | grep -oP 'duration=\K[0-9.]+')
        echo "Result: SUCCESS (mission duration: ${MISSION_DURATION}s, total: ${DURATION}s)" | tee -a "${SUMMARY_FILE}"
    elif grep -q "MUJOCO_MISSION_FAILED" "${RUN_LOG}"; then
        # Failed
        RESULT="FAILED"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        ERROR_CODE=$(grep "MUJOCO_MISSION_FAILED" "${RUN_LOG}" | grep -oP 'code=\K[0-9]+' | head -1)
        ERROR_MSG=$(grep "MUJOCO_MISSION_FAILED" "${RUN_LOG}" | grep -oP 'message=\K.*' | head -1)
        echo "Result: FAILED (code=${ERROR_CODE}, msg=${ERROR_MSG}, total: ${DURATION}s)" | tee -a "${SUMMARY_FILE}"
    else
        # Unknown
        RESULT="UNKNOWN"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        echo "Result: UNKNOWN (check log: ${RUN_LOG}, total: ${DURATION}s)" | tee -a "${SUMMARY_FILE}"
    fi

    RESULTS+=("${RESULT}")

    # 提取关键信息
    echo "Key phases:" | tee -a "${SUMMARY_FILE}"
    grep "phase=" "${RUN_LOG}" | tail -5 | tee -a "${SUMMARY_FILE}"
    echo "" | tee -a "${SUMMARY_FILE}"

    # 短暂休息，避免资源竞争
    if [ ${i} -lt ${TOTAL_RUNS} ]; then
        echo "Cooling down for 5 seconds..." | tee -a "${SUMMARY_FILE}"
        sleep 5
    fi
done

# 生成统计报告
echo "" | tee -a "${SUMMARY_FILE}"
echo "========================================" | tee -a "${SUMMARY_FILE}"
echo "G8 REPEATABILITY TEST SUMMARY" | tee -a "${SUMMARY_FILE}"
echo "========================================" | tee -a "${SUMMARY_FILE}"
echo "End time: $(date)" | tee -a "${SUMMARY_FILE}"
echo "" | tee -a "${SUMMARY_FILE}"

echo "Total runs: ${TOTAL_RUNS}" | tee -a "${SUMMARY_FILE}"
echo "Success: ${SUCCESS_COUNT} ($(awk "BEGIN {printf \"%.1f\", ${SUCCESS_COUNT}*100/${TOTAL_RUNS}")%)" | tee -a "${SUMMARY_FILE}"
echo "Failed: ${FAIL_COUNT} ($(awk "BEGIN {printf \"%.1f\", ${FAIL_COUNT}*100/${TOTAL_RUNS}")%)" | tee -a "${SUMMARY_FILE}"
echo "Timeout: ${TIMEOUT_COUNT} ($(awk "BEGIN {printf \"%.1f\", ${TIMEOUT_COUNT}*100/${TOTAL_RUNS}")%)" | tee -a "${SUMMARY_FILE}"
echo "" | tee -a "${SUMMARY_FILE}"

# 计算平均耗时
if [ ${#DURATIONS[@]} -gt 0 ]; then
    TOTAL_DURATION=0
    for d in "${DURATIONS[@]}"; do
        TOTAL_DURATION=$((TOTAL_DURATION + d))
    done
    AVG_DURATION=$(awk "BEGIN {printf \"%.1f\", ${TOTAL_DURATION}/${#DURATIONS[@]}}")
    echo "Average duration: ${AVG_DURATION}s" | tee -a "${SUMMARY_FILE}"
    echo "" | tee -a "${SUMMARY_FILE}"
fi

# 详细结果
echo "Run-by-run results:" | tee -a "${SUMMARY_FILE}"
for i in $(seq 0 $((${#RESULTS[@]} - 1))); do
    RUN_NUM=$((i + 1))
    echo "  Run ${RUN_NUM}: ${RESULTS[$i]} (${DURATIONS[$i]}s)" | tee -a "${SUMMARY_FILE}"
done
echo "" | tee -a "${SUMMARY_FILE}"

# 判定 G8 是否通过
echo "========================================" | tee -a "${SUMMARY_FILE}"
if [ ${SUCCESS_COUNT} -ge 8 ]; then
    echo "G8 VERDICT: ✅ PASSED (≥80% success rate)" | tee -a "${SUMMARY_FILE}"
    EXIT_STATUS=0
else
    echo "G8 VERDICT: ❌ FAILED (<80% success rate)" | tee -a "${SUMMARY_FILE}"
    EXIT_STATUS=1
fi
echo "========================================" | tee -a "${SUMMARY_FILE}"

echo "" | tee -a "${SUMMARY_FILE}"
echo "Full summary saved to: ${SUMMARY_FILE}" | tee -a "${SUMMARY_FILE}"
echo "Individual logs in: ${LOG_DIR}/" | tee -a "${SUMMARY_FILE}"

exit ${EXIT_STATUS}
