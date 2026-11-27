#ifndef SAFETY_H
#define SAFETY_H

// =====================================================
//  안전장치: Python 명령을 그대로 실행해도 되는지 마지막으로 확인한다.
//  하드웨어를 직접 다루지 않는 순수 계산이라 노트북에서도 테스트할 수 있다.
//  (apps/qrover_drive/tests/test_safety.cpp)
// =====================================================

// 거리 센서 상태 코드 (docs/interfaces.md, #19)
const int RANGE_OK = 0;            // 정상 측정
const int RANGE_OUT_OF_RANGE = 1;  // 범위 밖 (약 2m 이상) → 앞이 비어 있음
const int RANGE_ERROR = 2;         // 측정 실패·센서 응답 없음 → 믿지 말고 정지

// 정지한 이유 (로그용). 우선순위 순서와 같다.
enum SafetyReason {
  SAFETY_PASS = 0,        // 명령 그대로 실행
  SAFETY_NO_COMMAND,      // 부팅 후 아직 명령을 받은 적 없음
  SAFETY_WATCHDOG,        // 마지막 명령 후 WATCHDOG_MS 경과
  SAFETY_SENSOR_ERROR,    // 거리 센서 오류
  SAFETY_EMERGENCY        // 전진 중 앞 EMERGENCY_MM 이내에 물체
};

struct WheelCommand {
  int left;   // -255 ~ 255, + = 앞으로
  int right;
};

struct SafetyResult {
  WheelCommand wheels;    // 최종으로 실행할 바퀴 값
  SafetyReason reason;
};

// 우선순위: 명령 없음/watchdog > 센서 오류 > 비상정지 > 명령 그대로
SafetyResult applySafety(WheelCommand requested,
                         bool hasCommand,
                         unsigned long msSinceCommand,
                         int distanceMm,
                         int rangeStatus,
                         unsigned long watchdogMs,
                         int emergencyMm);

const char* safetyReasonName(SafetyReason reason);

// VL53L0X 측정 결과(RangeStatus, 원래 거리)를 우리 상태 코드로 바꾼다. (Arduino 없이 테스트 가능)
//   0 정상                        → RANGE_OK, 측정 거리
//   1 Sigma fail / 2 Signal fail  → RANGE_OUT_OF_RANGE (잡히는 물체가 없거나 신호가 약함)
//   3 Min range fail (너무 가까움) → RANGE_OK, 0mm  → 전진이면 비상정지
//   4 Phase fail (범위 밖)         → RANGE_OUT_OF_RANGE
//   그 밖 (5 Hardware fail 등)     → RANGE_ERROR
// 2026-11-27 실측: 앞이 비었거나 손을 대고 떼는 순간 1·2가 한 번씩 섞여 나왔고,
// 예전처럼 1~3을 오류로 보면 그 한 번에 주행 전체가 ABORT됐다.
void classifyRange(int vlStatus, int rawMm, int &distanceMm, int &rangeStatus);

#endif
