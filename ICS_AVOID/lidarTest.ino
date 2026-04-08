#include <RPLidar.h>

// ===================================================
// RPLIDAR + TB6612FNG OBSTACLE AVOIDANCE
// Arduino Uno R4 WiFi using Serial for lidar
// IMPORTANT:
//   - Disconnect lidar TX/RX during upload
//   - Reconnect after upload
// ===================================================

RPLidar lidar;

// ---------------- LIDAR PIN ----------------
const uint8_t LIDAR_MOTOR_PIN = 11;   // PWM pin for RPLIDAR motor
// Use pin 11 so pin 3 can stay assigned to PWMB from your motor code

// ---------------- MOTOR DRIVER PINS ----------------
const uint8_t PWMA = 9;    // Left motor speed (PWM)
const uint8_t AIN1 = 8;
const uint8_t AIN2 = 7;

const uint8_t PWMB = 3;    // Right motor speed (PWM)
const uint8_t BIN1 = 5;
const uint8_t BIN2 = 4;

const uint8_t STBY = 6;    // Standby pin (must be HIGH)

// ---------------- DRIVE SETTINGS ----------------
const uint8_t BASE_SPEED   = 100;
const uint8_t TURN_SPEED   = 100;
const uint8_t BACKUP_SPEED = 90;

// Distances in mm
const float FRONT_STOP_MM   = 500.0;
const float FRONT_WARN_MM   = 800.0;
const float SIDE_CAUTION_MM = 350.0;

// ---------------- SECTOR STATE ----------------
float nearestFront = 99999.0;
float nearestLeft  = 99999.0;
float nearestRight = 99999.0;

// ---------------- Motor Helper ----------------
inline uint8_t clampPWM(int v) {
  return (uint8_t)constrain(v, 0, 255);
}

void leftMotorForward(uint8_t pwm) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, clampPWM(pwm));
}

void leftMotorBackward(uint8_t pwm) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, clampPWM(pwm));
}

void leftMotorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

void rightMotorForward(uint8_t pwm) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, clampPWM(pwm));
}

void rightMotorBackward(uint8_t pwm) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, clampPWM(pwm));
}

void rightMotorStop() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

void motorsBrake() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void motorsForward(uint8_t leftPWM, uint8_t rightPWM) {
  leftMotorForward(leftPWM);
  rightMotorForward(rightPWM);
}

void motorsBackward(uint8_t leftPWM, uint8_t rightPWM) {
  leftMotorBackward(leftPWM);
  rightMotorBackward(rightPWM);
}

void pivotLeft(uint8_t leftPWM, uint8_t rightPWM) {
  leftMotorBackward(leftPWM);
  rightMotorForward(rightPWM);
}

void pivotRight(uint8_t leftPWM, uint8_t rightPWM) {
  leftMotorForward(leftPWM);
  rightMotorBackward(rightPWM);
}

// ---------------- LIDAR HELPERS ----------------
void resetSectors() {
  nearestFront = 99999.0;
  nearestLeft  = 99999.0;
  nearestRight = 99999.0;
}

bool validReading(float distance, uint8_t quality) {
  if (quality == 0) return false;
  if (distance <= 0.0) return false;
  if (distance > 6000.0) return false;
  return true;
}

void updateSectors(float angle, float distance, uint8_t quality) {
  if (!validReading(distance, quality)) return;

  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;

  // Front: 330..360 and 0..30
  if (angle >= 330.0 || angle <= 30.0) {
    if (distance < nearestFront) nearestFront = distance;
  }
  // Left: 30..120
  else if (angle > 30.0 && angle <= 120.0) {
    if (distance < nearestLeft) nearestLeft = distance;
  }
  // Right: 240..330
  else if (angle >= 240.0 && angle < 330.0) {
    if (distance < nearestRight) nearestRight = distance;
  }
}

// ---------------- AVOIDANCE ----------------
void chooseMotion() {
  // Immediate obstacle ahead
  if (nearestFront < FRONT_STOP_MM) {
    motorsBrake();
    delay(80);

    motorsBackward(BACKUP_SPEED, BACKUP_SPEED);
    delay(180);

    motorsBrake();
    delay(60);

    if (nearestLeft > nearestRight) {
      pivotLeft(TURN_SPEED, TURN_SPEED);
      delay(280);
    } else {
      pivotRight(TURN_SPEED, TURN_SPEED);
      delay(280);
    }

    motorsBrake();
    delay(40);
    return;
  }

  // Front is somewhat close: steer toward more open side
  if (nearestFront < FRONT_WARN_MM) {
    if (nearestLeft > nearestRight) {
      // turn a little left
      motorsForward(BASE_SPEED - 10, BASE_SPEED + 20);
    } else {
      // turn a little right
      motorsForward(BASE_SPEED + 20, BASE_SPEED - 10);
    }
    return;
  }

  // Side bias
  if (nearestLeft < SIDE_CAUTION_MM && nearestRight >= SIDE_CAUTION_MM) {
    motorsForward(BASE_SPEED + 10, BASE_SPEED - 25);
    return;
  }

  if (nearestRight < SIDE_CAUTION_MM && nearestLeft >= SIDE_CAUTION_MM) {
    motorsForward(BASE_SPEED - 25, BASE_SPEED + 10);
    return;
  }

  // Clear
  motorsForward(BASE_SPEED, BASE_SPEED);
}

void startLidarIfPossible() {
  rplidar_response_device_info_t info;

  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
    lidar.startScan();
    analogWrite(LIDAR_MOTOR_PIN, 180);
    delay(300);
  } else {
    analogWrite(LIDAR_MOTOR_PIN, 0);
    motorsBrake();
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  analogWrite(LIDAR_MOTOR_PIN, 0);

  motorsBrake();
  delay(500);

  // RPLIDAR uses Serial
  lidar.begin(Serial);

  resetSectors();
  startLidarIfPossible();
}

// ---------------- LOOP ----------------
void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle    = lidar.getCurrentPoint().angle;
    uint8_t quality = lidar.getCurrentPoint().quality;
    bool startBit   = lidar.getCurrentPoint().startBit;

    updateSectors(angle, distance, quality);

    // On each new scan, make one driving decision
    if (startBit) {
      chooseMotion();
      resetSectors();
    }
  } else {
    // Lost lidar data: stop for safety and retry
    motorsBrake();
    analogWrite(LIDAR_MOTOR_PIN, 0);
    delay(100);

    startLidarIfPossible();
  }
}
