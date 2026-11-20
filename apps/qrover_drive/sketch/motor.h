#ifndef MOTOR_H
#define MOTOR_H

void initializeMotors();

// 양쪽 바퀴 속도 (-255 ~ 255, + = 앞으로). 출발 보정 없이 바로 적용
void drive(int leftSpeed, int rightSpeed);

// 급정지: 네 핀 모두 HIGH
void brake();

// 메인 루프에서 매번 부르는 함수.
// 정지 → 주행으로 바뀔 때만 출발 보정(120ms 최대 출력), 주행 → 정지일 때는 brake.
void setWheels(int leftSpeed, int rightSpeed);

#endif
