#ifndef SENSOR_H
#define SENSOR_H

// VL53L0X 초기화. SENSOR_INIT_TRIES번 안에 못 찾으면 false를 돌려주고,
// 이후 readDistance()는 계속 RANGE_ERROR를 돌려준다 (안전장치가 정지시킴).
bool initializeSensor();

// 거리(mm)와 상태 코드를 읽는다. 상태가 RANGE_OK가 아니면 distanceMm = -1.
void readDistance(int &distanceMm, int &rangeStatus);

#endif
