#ifndef SENSOR_H
#define SENSOR_H

void initializeSensor();

// Returns distance in mm. Returns -1 when the reading is invalid.
int readDistance();

// Returns true when an obstacle is within STOP_DISTANCE_MM.
bool isObstacleDetected(int distance);

#endif
