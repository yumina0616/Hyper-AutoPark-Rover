// safety.cpp 테스트 — Arduino 없이 노트북 C++ 컴파일러로 실행한다.
//
// 실행 (레포 최상위 폴더에서):
//   g++ -std=c++11 -I apps/qrover_drive/sketch apps/qrover_drive/tests/test_safety.cpp apps/qrover_drive/sketch/safety.cpp -o test_safety
//   ./test_safety

#include <cstdio>
#include "safety.h"

namespace {
  int failures = 0;
  const unsigned long WD = 1000;
  const int EM = 80;

  void check(const char* name, SafetyResult r, int left, int right, SafetyReason reason) {
    bool ok = r.wheels.left == left && r.wheels.right == right && r.reason == reason;
    std::printf("%s %s (got L=%d R=%d %s)\n", ok ? "PASS" : "FAIL", name,
                r.wheels.left, r.wheels.right, safetyReasonName(r.reason));
    if (!ok) failures++;
  }
}

int main() {
  WheelCommand fwd = {100, 100};
  WheelCommand back = {-100, -100};
  WheelCommand spin = {120, -120};

  check("부팅 후 명령 없음 → 정지",
        applySafety(fwd, false, 0, 500, RANGE_OK, WD, EM), 0, 0, SAFETY_NO_COMMAND);
  check("정상 명령 → 그대로",
        applySafety(fwd, true, 100, 500, RANGE_OK, WD, EM), 100, 100, SAFETY_PASS);
  check("watchdog 경계(1000ms) → 아직 통과",
        applySafety(fwd, true, 1000, 500, RANGE_OK, WD, EM), 100, 100, SAFETY_PASS);
  check("watchdog 초과(1001ms) → 정지",
        applySafety(fwd, true, 1001, 500, RANGE_OK, WD, EM), 0, 0, SAFETY_WATCHDOG);
  check("센서 오류 → 정지",
        applySafety(back, true, 100, -1, RANGE_ERROR, WD, EM), 0, 0, SAFETY_SENSOR_ERROR);
  check("범위 밖(앞이 비어 있음) → 그대로",
        applySafety(fwd, true, 100, -1, RANGE_OUT_OF_RANGE, WD, EM), 100, 100, SAFETY_PASS);
  check("전진 중 앞 50mm → 비상정지",
        applySafety(fwd, true, 100, 50, RANGE_OK, WD, EM), 0, 0, SAFETY_EMERGENCY);
  check("후진 중 앞 50mm → 막지 않음",
        applySafety(back, true, 100, 50, RANGE_OK, WD, EM), -100, -100, SAFETY_PASS);
  check("제자리 회전 중 앞 50mm → 막지 않음",
        applySafety(spin, true, 100, 50, RANGE_OK, WD, EM), 120, -120, SAFETY_PASS);
  check("전진 중 앞 80mm(경계) → 통과",
        applySafety(fwd, true, 100, 80, RANGE_OK, WD, EM), 100, 100, SAFETY_PASS);
  check("watchdog이 센서 오류보다 우선",
        applySafety(fwd, true, 5000, -1, RANGE_ERROR, WD, EM), 0, 0, SAFETY_WATCHDOG);

  std::printf("\n%s (%d failed)\n", failures ? "FAILED" : "ALL PASSED", failures);
  return failures ? 1 : 0;
}
