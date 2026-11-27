// =====================================================
//  Q-ROVER 주행 실행부 (MCU) — #19
//  보드: Arduino UNO Q (Zephyr core), App Lab 앱
//
//  MCU는 스스로 판단하지 않는다. (docs/architecture.md "판단·실행 분리")
//    ① 거리를 읽어 Python에 보낸다          Bridge.notify("distance", dist_mm, status)
//    ② Python이 보낸 바퀴 값을 받는다       Bridge.provide("drive", onDriveCommand)
//    ③ 안전장치로 마지막 확인 후 실행한다     watchdog > 센서 오류 > 비상정지 > 명령
// =====================================================

#include <Arduino_RouterBridge.h>
#include "config.h"
#include "safety.h"
#include "sensor.h"
#include "motor.h"
#include "command.h"

SafetyReason lastReason = SAFETY_NO_COMMAND;

void setup() {
  Monitor.begin(115200);
  Bridge.begin();

  initializeMotors();   // 시작은 무조건 정지 상태

  // 이 문자열이 Python의 Bridge.notify("drive", ...)와 정확히 같아야 한다
  Bridge.provide("drive", onDriveCommand);

  if (initializeSensor()) {
    Monitor.println("VL53L0X ready");
  } else {
    Monitor.println("VL53L0X not found - running with SENSOR_ERROR (wheels stay stopped)");
  }
  Monitor.println("Q-ROVER drive executor ready");
}

void loop() {
  // ① 거리 읽기 → Python으로 보고
  int distanceMm;
  int rangeStatus;
  readDistance(distanceMm, rangeStatus);
  Bridge.notify("distance", distanceMm, rangeStatus);

  // ② 마지막 명령 꺼내기
  bool hasCommand;
  unsigned long msSinceCommand;
  WheelCommand requested = latestCommand(millis(), hasCommand, msSinceCommand);

  // ③ 안전장치 확인 후 실행
  SafetyResult result = applySafety(requested, hasCommand, msSinceCommand,
                                    distanceMm, rangeStatus, WATCHDOG_MS, EMERGENCY_MM);
  Bridge.notify("safety", (int)result.reason);
  setWheels(result.wheels.left, result.wheels.right);

  // 정지 이유가 바뀔 때만 출력 (시리얼 모니터에서 확인용)
  if (result.reason != lastReason) {
    Monitor.print("safety: ");
    Monitor.println(safetyReasonName(result.reason));
    lastReason = result.reason;
  }

  delay(LOOP_DELAY_MS);
}
