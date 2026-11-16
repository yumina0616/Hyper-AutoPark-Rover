#include "config.h"
#include "sensor.h"
#include "motor.h"
#include "behavior.h"

void updateMovement(int distance) {
  // An invalid/out-of-range reading (-1) is not treated as an obstacle,
  // so the rover keeps driving forward (same as the original code).
  if (isObstacleDetected(distance)) {
    turnAround();
  } else {
    moveForward();
  }
}
