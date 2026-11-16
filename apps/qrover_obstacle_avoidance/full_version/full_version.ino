// Q-Rover Obstacle Avoidance - single-file version
// Arduino UNO Q + VL53L0X distance sensor + DRV8833 motor driver
//
// 분리된 버전(config / sensor / motor / behavior)과 동작이 같습니다.
// 전체 흐름을 한 파일로 보고 싶을 때 참고하세요.

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================
// Config
// =====================================================

// DRV8833 motor driver pins
const int LEFT_MOTOR_IN1  = 5;
const int LEFT_MOTOR_IN2  = 3;
const int RIGHT_MOTOR_IN1 = 6;
const int RIGHT_MOTOR_IN2 = 9;

// VL53L0X sensor settings
const int SENSOR_I2C_ADDRESS = 0x29;
const int I2C_CLOCK = 100000;
const int STOP_DISTANCE_MM = 50;

// Motor speed (0 ~ 255)
const int DRIVE_SPEED = 130;
const int TURN_SPEED = 140;

// Timing (milliseconds)
const int MOTOR_START_TIME = 120;
const int TURN_180_TIME = 650;
const int STOP_TIME = 300;
const int SENSOR_INTERVAL = 50;

// =====================================================
// Sensor: obstacle detection
// =====================================================
Adafruit_VL53L0X distanceSensor;

void initializeSensor() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  delay(200);

  while (!distanceSensor.begin(SENSOR_I2C_ADDRESS, false, &Wire)) {
    delay(500);
  }
}

// Returns distance in mm. Returns -1 when the reading is invalid.
int readDistance() {
  VL53L0X_RangingMeasurementData_t measurement;
  distanceSensor.rangingTest(&measurement, false);

  if (measurement.RangeStatus == 4) {
    return -1;
  }

  return measurement.RangeMilliMeter;
}

// Returns true when an obstacle is within STOP_DISTANCE_MM.
bool isObstacleDetected(int distance) {
  return distance >= 0 && distance <= STOP_DISTANCE_MM;
}

// =====================================================
// Motor control
// =====================================================
bool isMovingForward = false;

// Positive: forward / Negative: backward / Zero: coast
void controlWheel(int in1, int in2, int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);

  if (motorSpeed > 0) {
    analogWrite(in1, 255);
    analogWrite(in2, 255 - motorSpeed);
  } else if (motorSpeed < 0) {
    analogWrite(in2, 255);
    analogWrite(in1, 255 + motorSpeed);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

int getKickSpeed(int motorSpeed) {
  if (motorSpeed > 0) return 255;
  if (motorSpeed < 0) return -255;
  return 0;
}

void drive(int leftSpeed, int rightSpeed) {
  controlWheel(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, leftSpeed);
  controlWheel(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, rightSpeed);
}

void brake() {
  // Electrical brake: all motor-driver inputs HIGH
  analogWrite(LEFT_MOTOR_IN1, 255);
  analogWrite(LEFT_MOTOR_IN2, 255);
  analogWrite(RIGHT_MOTOR_IN1, 255);
  analogWrite(RIGHT_MOTOR_IN2, 255);
  isMovingForward = false;
}

void initializeMotors() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  brake();
}

void startDrive(int leftSpeed, int rightSpeed) {
  // Full-power burst to overcome static friction
  drive(getKickSpeed(leftSpeed), getKickSpeed(rightSpeed));
  delay(MOTOR_START_TIME);
  drive(leftSpeed, rightSpeed);
}

void moveForward() {
  if (!isMovingForward) {
    startDrive(DRIVE_SPEED, DRIVE_SPEED);
    isMovingForward = true;
  }
}

void turnAround() {
  brake();
  delay(STOP_TIME);

  // Left wheel forward, right wheel backward
  startDrive(TURN_SPEED, -TURN_SPEED);
  delay(TURN_180_TIME);

  brake();
  delay(STOP_TIME);
}

// =====================================================
// Behavior: what to do after detection
// =====================================================
void updateMovement(int distance) {
  // An invalid/out-of-range reading (-1) is not treated as an obstacle,
  // so the rover keeps driving forward.
  if (isObstacleDetected(distance)) {
    turnAround();
  } else {
    moveForward();
  }
}

// =====================================================
// Main
// =====================================================
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
