#ifndef CONFIG_H
#define CONFIG_H

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

#endif
