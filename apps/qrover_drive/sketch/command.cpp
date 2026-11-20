#include <Arduino.h>
#include "command.h"

// Bridge 콜백은 loop()와 다른 스레드에서 실행된다 (세미나 2차시 설명).
// 왼쪽·오른쪽 값을 따로 저장하면 loop()가 "새 왼쪽 + 옛 오른쪽"을 읽을 수 있어서,
// 32비트 정수 하나에 묶어 한 번에 쓰고 한 번에 읽는다.
namespace {
  volatile uint32_t packedCommand = 0;     // 상위 16비트 = left, 하위 16비트 = right
  volatile unsigned long lastCommandMs = 0;
  volatile bool commandReceived = false;

  uint32_t pack(int left, int right) {
    return ((uint32_t)(uint16_t)(int16_t)left << 16) | (uint16_t)(int16_t)right;
  }
}

void onDriveCommand(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  packedCommand = pack(left, right);
  lastCommandMs = millis();
  commandReceived = true;
}

WheelCommand latestCommand(unsigned long nowMs, bool &hasCommand, unsigned long &msSinceCommand) {
  uint32_t packed = packedCommand;
  unsigned long last = lastCommandMs;
  hasCommand = commandReceived;
  msSinceCommand = nowMs - last;

  WheelCommand c;
  c.left = (int16_t)(packed >> 16);
  c.right = (int16_t)(packed & 0xFFFF);
  return c;
}
