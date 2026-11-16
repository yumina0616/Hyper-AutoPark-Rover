#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// --- Pin Configuration (Arduino UNO Q -> DRV8833) ---
const int AIN1 = 5;  // Left motor
const int AIN2 = 3;  // Left motor
const int BIN1 = 6;  // Right motor
const int BIN2 = 9;  // Right motor

// --- Tuning Parameters ---
const int STOP_DISTANCE_MM = 50;   // 5 cm threshold
const int DRIVE_SPEED      = 130;  // Forward & Reverse speed (0 - 255)
const int REVERSE_MS       = 880;  // 120 ms kickoff + 880 ms = 1000 ms (1 second)
const int TURN_SPEED       = 140;  // In-place spin speed (0 - 255)
const int TURN_180_MS      = 650;  // Spin duration for 180° (calibrate on your floor!)

Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X();
bool isMovingForward = false;

// --- Motor Control Functions ---
void wheel(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(in1, 255);
    analogWrite(in2, 255 - speed); // Slow-decay PWM
  } else if (speed < 0) {
    analogWrite(in2, 255);
    analogWrite(in1, 255 + speed);
  } else {
    analogWrite(in1, 0);           // Coast
    analogWrite(in2, 0);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  wheel(AIN1, AIN2, leftSpeed);
  wheel(BIN1, BIN2, rightSpeed);
}

void brake() {
  // Electrical brake: set all inputs HIGH
  analogWrite(AIN1, 255);
  analogWrite(AIN2, 255);
  analogWrite(BIN1, 255);
  analogWrite(BIN2, 255);
}

int kickOf(int v) {
  return (v > 0) ? 255 : ((v < 0) ? -255 : 0);
}

void startDrive(int leftSpeed, int rightSpeed) {
  // 120 ms full-power burst to overcome static friction
  drive(kickOf(leftSpeed), kickOf(rightSpeed));
  delay(120);
  drive(leftSpeed, rightSpeed);
}

// --- Setup ---
void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  brake();

  Wire.begin();
  Wire.setClock(100000); // 100 kHz I2C clock
  delay(200);

  // Initialize VL53L0X at default address 0x29
  while (!distanceSensor.begin(0x29, false, &Wire)) {
    delay(500);
  }
}

// --- Main Control Loop ---
void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  distanceSensor.rangingTest(&measure, false);

  // RangeStatus == 4 indicates out-of-range/invalid reading -> treat as -1
  int distance_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  // Check if a valid object is within 50 mm (5 cm)
  bool obstacleDetected = (distance_mm >= 0) && (distance_mm <= STOP_DISTANCE_MM);

  if (obstacleDetected) {
    // 1. Immediate electrical stop
    brake();
    isMovingForward = false;
    delay(250);

    // 2. Reverse at the same speed for 1 second (120 ms kickoff + 880 ms)
    startDrive(-DRIVE_SPEED, -DRIVE_SPEED);
    delay(REVERSE_MS);

    // 3. Brake before turning so the wheels don't slip
    brake();
    delay(250);

    // 4. Spin 180 degrees in place (left wheel forward, right wheel backward)
    startDrive(TURN_SPEED, -TURN_SPEED);
    delay(TURN_180_MS);

    // 5. Stop briefly before resuming forward motion
    brake();
    delay(250);
  } else {
    // Resume driving forward if not already moving
    if (!isMovingForward) {
      startDrive(DRIVE_SPEED, DRIVE_SPEED);
      isMovingForward = true;
    }
  }

  delay(50); // Sensor polling interval
}