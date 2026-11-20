#ifndef COMMAND_H
#define COMMAND_H

#include "safety.h"

// Bridge.provide("drive", onDriveCommand)로 등록한다.
// Python 상태 기계가 Bridge.notify("drive", left, right)로 부른다.
void onDriveCommand(int left, int right);

// 메인 루프에서 마지막 명령을 꺼낸다.
// hasCommand: 부팅 후 명령을 한 번이라도 받았는지
// msSinceCommand: 마지막 명령 후 지난 시간 (ms)
WheelCommand latestCommand(unsigned long nowMs, bool &hasCommand, unsigned long &msSinceCommand);

#endif
