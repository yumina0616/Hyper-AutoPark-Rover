#include <Arduino.h>
#include "config.h"
#include "motor.h"

namespace {
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
}

void initializeMotors() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  brake();
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

void startDrive(int leftSpeed, int rightSpeed) {
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
