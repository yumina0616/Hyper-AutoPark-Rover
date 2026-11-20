// Front parking for Arduino UNO Q (MCU / STM32)
// The Linux camera side calls set_marker(id, pixelSide, offsetPx).
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Arduino_RouterBridge.h>

// DRV8833, unchanged from original rover code
const int LEFT_MOTOR_IN1 = 5;
const int LEFT_MOTOR_IN2 = 3;
const int RIGHT_MOTOR_IN1 = 6;
const int RIGHT_MOTOR_IN2 = 9;

// Obstacle / ranging sensors
const int FRONT_IR_LEFT = 2;   // LOW == obstacle
const int FRONT_IR_RIGHT = 4;  // LOW == obstacle
const int REAR_IR_LEFT = 7;    // currently unused in front parking
const int REAR_IR_RIGHT = 8;   // currently unused in front parking
const int VL53L0X_ADDRESS = 0x29;

// TUNE ON THE ACTUAL ROBOT. Distances are to the ToF sensor, NOT robot bumper.
const int PARK_DISTANCE_MM = 150;       // combined parking condition
const int EMERGENCY_DISTANCE_MM = 90;   // unconditional safety stop
const int MARKER_STOP_SIDE_PX = 140;    // pixel width, 640x480, calibrate
const int CENTER_TOLERANCE_PX = 45;
const int SLOWDOWN_DISTANCE_MM = 300;
const int DRIVE_SPEED = 115;
const int SLOW_SPEED = 95;
const int ALIGN_SPEED = 90;
const unsigned long MARKER_TIMEOUT_MS = 600;
const unsigned long SENSOR_PERIOD_MS = 60;
const int REQUIRED_STOP_SAMPLES = 3;

// State codes: 0 WAIT, 1 ALIGN, 2 APPROACH, 3 PARKED,
//              4 SAFETY_STOP, 5 MARKER_LOST, 6 SENSOR_ERROR.
enum ParkingState { WAIT=0, ALIGN=1, APPROACH=2, PARKED=3,
                    SAFETY_STOP=4, MARKER_LOST=5, SENSOR_ERROR=6 };
ParkingState state = WAIT;
Adafruit_VL53L0X tof;
bool tofInitialized = false;
int latestDistance = -1;
int markerId = -1;
int markerSidePx = 0;
int markerOffsetPx = 0;
bool markerReceived = false;
unsigned long lastMarkerMs = 0;
unsigned long lastSensorMs = 0;
int parkingSamples = 0;

void controlWheel(int in1, int in2, int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  // Preserve the original DRV8833 input convention.
  if (motorSpeed > 0) {
    analogWrite(in1, 255);
    analogWrite(in2, 255 - motorSpeed);
  } else if (motorSpeed < 0) {
    analogWrite(in2, 255);
    analogWrite(in1, 255 + motorSpeed);
  } else {
    analogWrite(in1, 255); // electrical brake
    analogWrite(in2, 255);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  controlWheel(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, leftSpeed);
  controlWheel(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, rightSpeed);
}

void brake() { drive(0, 0); }

int readDistance() {
  if (!tofInitialized) return -1;
  VL53L0X_RangingMeasurementData_t measurement;
  tof.rangingTest(&measurement, false);
  if (measurement.RangeStatus != 0) return -1;
  return (int)measurement.RangeMilliMeter;
}

// RPC called from Python. Main-loop safe because it updates shared state.
// Return value is a status code; state 3 stays latched until MCU reset.
int set_marker(int id, int sidePx, int offsetPx) {
  if (state == PARKED || state == SAFETY_STOP || state == SENSOR_ERROR) {
    return (int)state;
  }
  markerId = id;
  markerSidePx = max(0, sidePx);
  markerOffsetPx = offsetPx;
  markerReceived = true;
  lastMarkerMs = millis();
  return (int)state;
}

int get_parking_state() { return (int)state; }
int get_distance_mm() { return latestDistance; }

void setup() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(FRONT_IR_LEFT, INPUT);
  pinMode(FRONT_IR_RIGHT, INPUT);
  pinMode(REAR_IR_LEFT, INPUT);
  pinMode(REAR_IR_RIGHT, INPUT);
  brake();

  Wire.begin();
  Wire.setClock(100000);
  delay(200);
  tofInitialized = tof.begin(VL53L0X_ADDRESS, false, &Wire);
  if (!tofInitialized) state = SENSOR_ERROR;

  Bridge.begin();
  Bridge.provide_safe("set_marker", set_marker);
  Bridge.provide_safe("get_parking_state", get_parking_state);
  Bridge.provide_safe("get_distance_mm", get_distance_mm);
}

void loop() {
  // Terminal stop states require an MCU reset to resume.
  if (state == PARKED || state == SAFETY_STOP || state == SENSOR_ERROR) {
    brake();
    delay(20);
    return;
  }

  // Always poll front IR sensors first.
  if (digitalRead(FRONT_IR_LEFT) == LOW ||
      digitalRead(FRONT_IR_RIGHT) == LOW) {
    state = SAFETY_STOP;
    brake();
    return;
  }

  if (millis() - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = millis();
    latestDistance = readDistance();
  }

  // Fail closed: stop rather than driving blind.
  if (latestDistance < 0) {
    state = SENSOR_ERROR;
    brake();
    return;
  }
  if (latestDistance <= EMERGENCY_DISTANCE_MM) {
    state = SAFETY_STOP;
    brake();
    return;
  }

  if (!markerReceived) {
    state = WAIT;
    brake();
    return;
  }
  if (millis() - lastMarkerMs > MARKER_TIMEOUT_MS || markerId < 0) {
    state = MARKER_LOST;
    parkingSamples = 0;
    brake();
    return;
  }

  // Require both visual and physical distance thresholds, plus alignment.
  bool centered = abs(markerOffsetPx) <= CENTER_TOLERANCE_PX;
  bool visionClose = markerSidePx >= MARKER_STOP_SIDE_PX;
  bool tofClose = latestDistance <= PARK_DISTANCE_MM;
  if (centered && visionClose && tofClose) {
    parkingSamples++;
    brake();
    if (parkingSamples >= REQUIRED_STOP_SAMPLES) state = PARKED;
    else state = APPROACH;
    return;
  }
  parkingSamples = 0;

  // If close enough physically but visual condition isn't met, do not creep.
  if (tofClose) {
    state = SAFETY_STOP;
    brake();
    return;
  }

  if (!centered) {
    state = ALIGN;
    // offset>0 means marker is right of image center. Spin right.
    if (markerOffsetPx > 0) drive(ALIGN_SPEED, -ALIGN_SPEED);
    else drive(-ALIGN_SPEED, ALIGN_SPEED);
  } else {
    state = APPROACH;
    int speed = (latestDistance <= SLOWDOWN_DISTANCE_MM ||
                 markerSidePx >= MARKER_STOP_SIDE_PX * 3 / 4)
                    ? SLOW_SPEED : DRIVE_SPEED;
    drive(speed, speed);
  }
  delay(20);
}
