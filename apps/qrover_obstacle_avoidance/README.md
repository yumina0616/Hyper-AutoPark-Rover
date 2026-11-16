# Q-Rover Obstacle Avoidance

VL53L0X ToF 거리 센서로 전방 장애물을 감지하고, 장애물이 설정 거리 이내에 있으면 정지 후 제자리 회전하는 프로젝트입니다.

## Hardware
- Arduino UNO Q
- Q-Rover platform
- VL53L0X ToF distance sensor
- DRV8833 motor driver

## Pin Configuration

| Component | UNO Q pin |
|---|---:|
| Left motor IN1 | D5 |
| Left motor IN2 | D3 |
| Right motor IN1 | D6 |
| Right motor IN2 | D9 |
| Sensor SDA | D20 / SDA |
| Sensor SCL | D21 / SCL |

I2C address: `0x29`

## File Structure

```text
obstacle_avoidance/
├── qrover_obstacle_avoidance.ino  # Main program
├── config.h                       # Pins and tuning parameters
├── sensor.h                       # Sensor function declarations
├── sensor.cpp                     # VL53L0X initialization and reading
├── motor.h                        # Motor function declarations
├── motor.cpp                      # Motor movement control
├── behavior.h                     # Movement decision declaration
├── behavior.cpp                   # Obstacle response logic
├── full_version/
│   └── full_version.ino           # Single-file version (same behavior)
└── README.md
```

## Code Responsibilities

### 1. Sensor (`sensor.cpp`, `sensor.h`)
- Initialize VL53L0X over I2C.
- Read distance in millimeters.
- Return `-1` for an invalid/out-of-range reading.
- Determine whether an obstacle is within the configured threshold.

### 2. Motor (`motor.cpp`, `motor.h`)
- Initialize DRV8833 motor pins.
- Control left and right wheel speeds.
- Move forward, brake, and rotate approximately 180 degrees.

### 3. Behavior (`behavior.cpp`, `behavior.h`)
- Receive the distance measurement.
- Turn around when an obstacle is within the threshold.
- Otherwise (including an invalid/out-of-range reading), move forward.

### 4. Main (`qrover_obstacle_avoidance.ino`)
- Initialize the motor and sensor modules.
- Repeatedly read the distance and pass it to the behavior module.

## Adjustable Parameters (`config.h`)

| Parameter | Meaning | Default |
|---|---|---:|
| `STOP_DISTANCE_MM` | Obstacle detection threshold | 50 mm |
| `DRIVE_SPEED` | Forward speed | 130 |
| `TURN_SPEED` | Turning speed | 140 |
| `MOTOR_START_TIME` | Initial motor boost duration | 120 ms |
| `TURN_180_TIME` | Approximate 180-degree turn duration | 650 ms |
| `STOP_TIME` | Stop duration before/after turning | 300 ms |
| `SENSOR_INTERVAL` | Sensor polling interval | 50 ms |

## Dependencies
Install these libraries in Arduino IDE:
- Adafruit VL53L0X
- Adafruit BusIO (dependency)

## Upload
Open `qrover_obstacle_avoidance.ino` in Arduino IDE and upload the sketch to Arduino UNO Q. Keep all `.h` and `.cpp` files in the same sketch folder.

### Single-file version
`full_version/full_version.ino` contains the same code merged into one file. Open it as a separate sketch; do not copy it into the modular sketch folder, or the duplicated definitions will fail to compile.

## Calibration & Safety
- `TURN_180_TIME` is a time-based approximation, not a guaranteed 180-degree turn. Calibrate it on the actual floor.
- Test motor direction with the wheels lifted before placing the rover on the floor.
- Confirm the sensor module's VIN voltage rating and board pin compatibility.
- An invalid/out-of-range reading (`-1`) is treated as "no obstacle", so the rover keeps moving forward.
