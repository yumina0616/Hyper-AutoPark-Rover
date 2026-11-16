#include "sensor.h"
#include "motor.h"
#include "behavior.h"
#include "config.h"

void setup() {
  initializeMotors();
  initializeSensor();
}

void loop() {
  // PART 1: Read distance from the sensor
  int distance = readDistance();

  // PART 2 & 3: Decide and execute the movement
  updateMovement(distance);

  delay(SENSOR_INTERVAL);
}
