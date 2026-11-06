#!/bin/bash
# G8 快速验证脚本 - 运行 3 次测试

set -e

TOTAL_RUNS=3
TIMEOUT_SEC=300
LOG_DIR="./openflex_ws/migration_records/2026-11-05/g8_repeatability"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY_FILE="${LOG_DIR}/g8_quick_validation_${TIMESTAMP}.txt"

echo "=== G8 Quick Validation (3 runs) ===" | tee "${SUMMARY_FILE}"
echo "Start time: $(date)" | tee -a "${SUMMARY_FILE}"
echo "" | tee -a "${SUMMARY_FILE}"

SUCCESS_COUNT=0
declare -a RESULTS
declare -a DURATIONS

for i in $(seq 1 ${TOTAL_RUNS}); do
    echo "========================================"
    echo "Run ${i}/${TOTAL_RUNS} - $(date)"
    echo "========================================"

    RUN_LOG="${LOG_DIR}/quick_run_${i}_${TIMESTAMP}.log"
    START_TIME=$(date +%s)

    if timeout ${TIMEOUT_SEC} bash scripts/run_thesis_moveit_mission.sh > "${RUN_LOG}" 2>&1; then
        EXIT_CODE=$?
    else
        EXIT_CODE=$?
    fi

    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))
    DURATIONS+=("${DURATION}")

    if [ ${EXIT_CODE} -eq 124 ]; then
        RESULT="TIMEOUT"
        echo "❌ Run ${i}: TIMEOUT (>${TIMEOUT_SEC}s)" | tee -a "${SUMMARY_FILE}"
    elif grep -q "MUJOCO_MISSION_SUCCESS" "${RUN_LOG}"; then
        RESULT="SUCCESS"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
        MISSION_DURATION=$(grep "MUJOCO_MISSION_SUCCESS" "${RUN_LOG}" | grep -oP 'duration=\K[0-9.]+' || echo "N/A")
        echo "✅ Run ${i}: SUCCESS (mission: ${MISSION_DURATION}s, total: ${DURATION}s)" | tee -a "${SUMMARY_FILE}"
    else
        RESULT="FAILED"
        ERROR_CODE=$(grep "MUJOCO_MISSION_FAILED" "${RUN_LOG}" | grep -oP 'code=\K[0-9]+' | head -1 || echo "N/A")
        echo "❌ Run ${i}: FAILED (code=${ERROR_CODE}, total: ${DURATION}s)" | tee -a "${SUMMARY_FILE}"
    fi

    RESULTS+=("${RESULT}")

    if [ ${i} -lt ${TOTAL_RUNS} ]; then
        echo "Cooling down 5s..."
        sleep 5
    fi
done

echo "" | tee -a "${SUMMARY_FILE}"
echo "========================================" | tee -a "${SUMMARY_FILE}"
echo "QUICK VALIDATION SUMMARY" | tee -a "${SUMMARY_FILE}"
echo "========================================" | tee -a "${SUMMARY_FILE}"
echo "Success: ${SUCCESS_COUNT}/${TOTAL_RUNS} ($(awk "BEGIN {printf \"%.0f\", ${SUCCESS_COUNT}*100/${TOTAL_RUNS}")%)" | tee -a "${SUMMARY_FILE}"

if [ ${#DURATIONS[@]} -gt 0 ]; then
    TOTAL_DURATION=0
    for d in "${DURATIONS[@]}"; do
        TOTAL_DURATION=$((TOTAL_DURATION + d))
    done
    AVG_DURATION=$(awk "BEGIN {printf \"%.1f\", ${TOTAL_DURATION}/${#DURATIONS[@]}}")
    echo "Average duration: ${AVG_DURATION}s" | tee -a "${SUMMARY_FILE}"
fi

echo "" | tee -a "${SUMMARY_FILE}"
if [ ${SUCCESS_COUNT} -eq ${TOTAL_RUNS} ]; then
    echo "🎉 Result: ALL PASSED (100% success)" | tee -a "${SUMMARY_FILE}"
    echo "✅ System appears stable - ready for full G8 (10 runs)" | tee -a "${SUMMARY_FILE}"
    EXIT_STATUS=0
elif [ ${SUCCESS_COUNT} -ge 2 ]; then
    echo "⚠️  Result: MOSTLY PASSED (${SUCCESS_COUNT}/${TOTAL_RUNS})" | tee -a "${SUMMARY_FILE}"
    echo "⚠️  May need investigation before full G8" | tee -a "${SUMMARY_FILE}"
    EXIT_STATUS=1
else
    echo "❌ Result: FAILED (${SUCCESS_COUNT}/${TOTAL_RUNS})" | tee -a "${SUMMARY_FILE}"
    echo "❌ System unstable - investigate before continuing" | tee -a "${SUMMARY_FILE}"
    EXIT_STATUS=2
fi
echo "========================================" | tee -a "${SUMMARY_FILE}"

echo "" | tee -a "${SUMMARY_FILE}"
echo "Summary saved to: ${SUMMARY_FILE}"
echo "Logs in: ${LOG_DIR}/"

exit ${EXIT_STATUS}
