#include "safety.h"

namespace {
  const WheelCommand STOP = {0, 0};

  // "앞으로 가는 중"인지: 두 바퀴 합이 양수면 차체가 앞으로 움직인다.
  // 제자리 회전(left = -right)은 전진이 아니라서 비상정지 대상이 아니다.
  bool isMovingForward(WheelCommand c) {
    return (c.left + c.right) > 0;
  }
}

SafetyResult applySafety(WheelCommand requested,
                         bool hasCommand,
                         unsigned long msSinceCommand,
                         int distanceMm,
                         int rangeStatus,
                         unsigned long watchdogMs,
                         int emergencyMm) {
  // 1) 부팅 후 첫 명령 전에는 무조건 정지
  if (!hasCommand) {
    return {STOP, SAFETY_NO_COMMAND};
  }
  // 1) watchdog: Python이 멈추거나 통신이 끊기면 명령이 안 온다
  if (msSinceCommand > watchdogMs) {
    return {STOP, SAFETY_WATCHDOG};
  }
  // 2) 센서를 믿을 수 없으면 정지
  if (rangeStatus == RANGE_ERROR) {
    return {STOP, SAFETY_SENSOR_ERROR};
  }
  // 3) 전진 중 앞이 너무 가까우면 정지 (후진·회전은 막지 않음 → 벽에서 빠져나올 수 있게)
  if (rangeStatus == RANGE_OK && distanceMm < emergencyMm && isMovingForward(requested)) {
    return {STOP, SAFETY_EMERGENCY};
  }
  // 4) 명령 그대로
  return {requested, SAFETY_PASS};
}

const char* safetyReasonName(SafetyReason reason) {
  switch (reason) {
    case SAFETY_PASS:         return "PASS";
    case SAFETY_NO_COMMAND:   return "NO_COMMAND";
    case SAFETY_WATCHDOG:     return "WATCHDOG";
    case SAFETY_SENSOR_ERROR: return "SENSOR_ERROR";
    case SAFETY_EMERGENCY:    return "EMERGENCY";
  }
  return "UNKNOWN";
}

void classifyRange(int vlStatus, int rawMm, int &distanceMm, int &rangeStatus) {
  distanceMm = -1;
  switch (vlStatus) {
    case 0:
      distanceMm = rawMm;
      rangeStatus = RANGE_OK;
      return;
    case 1:
    case 2:
    case 4:
      rangeStatus = RANGE_OUT_OF_RANGE;
      return;
    case 3:
      distanceMm = 0;  // 너무 가까움 → 가장 가까운 값으로 보고 정지하게 한다
      rangeStatus = RANGE_OK;
      return;
    default:
      rangeStatus = RANGE_ERROR;
      return;
  }
}
