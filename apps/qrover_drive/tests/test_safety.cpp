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

  // ── VL53L0X 상태 코드 분류 ──
  struct Case { const char* name; int vl; int raw; int mm; int status; };
  const Case cases[] = {
    {"range 0 정상",                0, 245, 245, RANGE_OK},
    {"range 1 sigma fail → 범위 밖", 1, 30,  -1,  RANGE_OUT_OF_RANGE},
    {"range 2 signal fail → 범위 밖", 2, 8190, -1, RANGE_OUT_OF_RANGE},
    {"range 3 너무 가까움 → 0mm",     3, 20,  0,   RANGE_OK},
    {"range 4 phase fail → 범위 밖", 4, 8190, -1, RANGE_OUT_OF_RANGE},
    {"range 5 hardware fail → 오류", 5, 0,   -1,  RANGE_ERROR},
  };
  for (const Case& c : cases) {
    int mm = 123, st = 99;
    classifyRange(c.vl, c.raw, mm, st);
    bool ok = mm == c.mm && st == c.status;
    std::printf("%s %s (got mm=%d status=%d)\n", ok ? "PASS" : "FAIL", c.name, mm, st);
    if (!ok) failures++;
  }
  // 너무 가까움(0mm)이 실제로 전진 비상정지로 이어지는지
  {
    int mm, st;
    classifyRange(3, 15, mm, st);
    check("range 3 + 전진 → 비상정지",
          applySafety(WheelCommand{100, 100}, true, 0, mm, st, WD, EM), 0, 0, SAFETY_EMERGENCY);
  }

  std::printf("\n%s (%d failed)\n", failures ? "FAILED" : "ALL PASSED", failures);
  return failures ? 1 : 0;
}
