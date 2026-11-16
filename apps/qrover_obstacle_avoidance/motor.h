#ifndef MOTOR_H
#define MOTOR_H

void initializeMotors();
void drive(int leftSpeed, int rightSpeed);
void brake();
void startDrive(int leftSpeed, int rightSpeed);
void moveForward();
void turnAround();

#endif
