#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
//  핀·센서·안전 설정값 (모두 여기서만 바꾼다)
//  출처: apps/qrover_obstacle_avoidance/config.h (#17) + #19 추가값
// =====================================================

// DRV8833 모터 드라이버 핀 (UNO Q에서 PWM이 되는 핀은 3·5·6·9뿐이라 변경 불가)
const int LEFT_MOTOR_IN1  = 5;
const int LEFT_MOTOR_IN2  = 3;
const int RIGHT_MOTOR_IN1 = 6;
const int RIGHT_MOTOR_IN2 = 9;

// VL53L0X 거리 센서
const int SENSOR_I2C_ADDRESS = 0x29;
const long I2C_CLOCK = 100000;       // 400kHz에서는 초기화가 실패한다
const int SENSOR_INIT_TRIES = 5;     // 이 횟수 안에 못 찾으면 "센서 오류" 상태로 시작 (무한 대기하지 않음)

// 출발 보정: 멈춰 있던 바퀴는 정지 마찰 때문에 낮은 PWM으로 출발하지 못한다
const int MOTOR_START_TIME = 120;    // ms

// ── 안전장치 (#19) ─────────────────────────────────
const unsigned long WATCHDOG_MS = 1000;  // 마지막 drive 명령 후 이 시간이 지나면 정지   TODO: 실험으로 결정
const int EMERGENCY_MM = 80;             // 전진 중 앞 거리가 이 값보다 작으면 정지    TODO: 실험으로 결정

// 메인 루프 주기. 거리 측정 자체에 약 30ms가 걸려서 실제 주기는 약 50ms
const int LOOP_DELAY_MS = 20;

#endif
