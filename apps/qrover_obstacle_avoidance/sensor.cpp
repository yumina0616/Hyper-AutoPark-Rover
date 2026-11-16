#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"
#include "sensor.h"

namespace {
  Adafruit_VL53L0X distanceSensor;
}

void initializeSensor() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  delay(200);

  while (!distanceSensor.begin(SENSOR_I2C_ADDRESS, false, &Wire)) {
    delay(500);
  }
}

int readDistance() {
  VL53L0X_RangingMeasurementData_t measurement;
  distanceSensor.rangingTest(&measurement, false);

  if (measurement.RangeStatus == 4) {
    return -1;
  }

  return measurement.RangeMilliMeter;
}

bool isObstacleDetected(int distance) {
  return distance >= 0 && distance <= STOP_DISTANCE_MM;
}
