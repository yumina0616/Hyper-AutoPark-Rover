#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"
#include "safety.h"
#include "sensor.h"

// 출처: apps/qrover_obstacle_avoidance/sensor.cpp (#17)
// 바뀐 점 (#19):
//  - "-1" 하나로만 표시하던 것을 상태 코드 3종으로 구분
//  - 센서를 못 찾으면 무한 대기하지 않고 오류 상태로 시작

namespace {
  Adafruit_VL53L0X distanceSensor;
  bool sensorReady = false;
}

bool initializeSensor() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  delay(200);  // 센서 부팅 대기

  for (int i = 0; i < SENSOR_INIT_TRIES; i++) {
    if (distanceSensor.begin(SENSOR_I2C_ADDRESS, false, &Wire)) {
      sensorReady = true;
      return true;
    }
    delay(500);
  }
  sensorReady = false;
  return false;
}

void readDistance(int &distanceMm, int &rangeStatus) {
  distanceMm = -1;
  if (!sensorReady) {
    rangeStatus = RANGE_ERROR;
    return;
  }

  VL53L0X_RangingMeasurementData_t measurement;
  distanceSensor.rangingTest(&measurement, false);

  // VL53L0X RangeStatus: 0 = 정상, 4 = 범위 밖(phase fail), 1~3 = 신호 약함 등 측정 실패
  if (measurement.RangeStatus == 0) {
    distanceMm = measurement.RangeMilliMeter;
    rangeStatus = RANGE_OK;
  } else if (measurement.RangeStatus == 4) {
    rangeStatus = RANGE_OUT_OF_RANGE;
  } else {
    rangeStatus = RANGE_ERROR;
  }
}
