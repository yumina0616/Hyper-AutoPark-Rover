#include <Arduino.h>
#include "config.h"
#include "motor.h"

// 출처: apps/qrover_obstacle_avoidance/motor.cpp (#17)
// 바뀐 점 (#19):
//  - moveForward(), turnAround() 제거: "언제 전진·회전할지"는 Python 상태 기계가 정한다
//  - setWheels() 추가: 정지↔주행이 바뀔 때만 출발 보정/급정지 (세미나 3주차 코드와 같은 방식)

namespace {
  int prevLeft = 0;
  int prevRight = 0;

  // + = 앞으로, - = 뒤로, 0 = 힘 빼기(coast). 반전 PWM(slow decay)
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

  int kickOf(int motorSpeed) {
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
  prevLeft = 0;
  prevRight = 0;
}

void drive(int leftSpeed, int rightSpeed) {
  controlWheel(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, leftSpeed);
  controlWheel(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, rightSpeed);
}

void brake() {
  analogWrite(LEFT_MOTOR_IN1, 255);
  analogWrite(LEFT_MOTOR_IN2, 255);
  analogWrite(RIGHT_MOTOR_IN1, 255);
  analogWrite(RIGHT_MOTOR_IN2, 255);
}

void setWheels(int leftSpeed, int rightSpeed) {
  bool wasStopped = (prevLeft == 0 && prevRight == 0);
  bool stopNow = (leftSpeed == 0 && rightSpeed == 0);

  if (stopNow) {
    if (!wasStopped) brake();          // 주행 → 정지: 급정지
  } else if (wasStopped) {
    // 정지 → 주행: 짧게 최대 출력으로 밀어 출발 (120ms 동안 루프가 멈추는 건 감수)
    drive(kickOf(leftSpeed), kickOf(rightSpeed));
    delay(MOTOR_START_TIME);
    drive(leftSpeed, rightSpeed);
  } else {
    drive(leftSpeed, rightSpeed);      // 주행 중 값만 바뀜
  }

  prevLeft = leftSpeed;
  prevRight = rightSpeed;
}
