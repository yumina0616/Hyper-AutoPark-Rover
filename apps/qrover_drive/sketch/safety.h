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

#endif
