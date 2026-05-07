/*
 * Giga R1 — Main Control Loop
 * RF joystick → servo + motors + IMU odometry + slip detection
 * DataPacket matches path_reciever.ino
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// RF — joystick receiver
RF24 radio(2, 3);
const byte address[6] = "00001";

// RF — path transmitter (leader-follow)
RF24 path_radio(27, 29);
const byte address2[6] = "00100";

Servo myservo;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Joystick state
int xVal = 512;
int yVal = 512;

bool flag = 0;

// World-frame position (m)
float pos_x = 0.0f;
float pos_y = 0.0f;

// Control state
int   theta      = 90;
int   theta_old  = 90;
int   servoAngle = 90;
float velocity   = 0.0f;
float vehicle_heading = 0.0f;

// Motor pins
const int mainMotor1     = 48;
const int mainMotor2     = 49;
const int mainMotor1Dir  = 36;
const int mainMotor2Dir  = 37;
const int mainMotor1Stop = 33;
const int mainMotor2Stop = 32;

int   pwmMotor1   = 0;
int   pwmMotor2   = 0;
float r_RPM_left  = 0.0f;
float r_RPM_right = 0.0f;
float avgRpm      = 0.0f;
float avgVel      = 0.0f;
bool  dir1 = false;
bool  dir2 = false;

// IMU velocity / acceleration / yaw rate
float vel_imu   = 0.0f;
float accel_enc = 0.0f;
float accel_imu = 0.0f;
float gyro_z    = 0.0f;  // yaw rate (deg/s)

// Leaky-integrator time constant: prevents accelerometer bias drift.
const float IMU_VEL_TAU          = 2.0f;
// Accel readings below this (m/s^2) are treated as vibration noise.
const float IMU_ACCEL_DEADBAND   = 0.25f;
// Max allowed accel difference (enc vs IMU) before slip is flagged (m/s^2).
const float SLIP_ACCEL_THRESHOLD = 0.1f;
// Minimum encoder speed (m/s) before slip detection activates.
const float SLIP_MIN_SPEED       = 0.3f;

bool is_slipping = false;

// Timing
unsigned long lastRFTime     = 0;
unsigned long lastSampleTime = 0;
unsigned long lastSampleT = 0;
unsigned long lastRampTime   = 0;
bool stop_loop = 0;

// Encoder
volatile unsigned long pulseCountR = 0;
volatile unsigned long pulseCountL = 0;

const float pulsesPerRevolution = 187.79855f;
const int   scPinR = 6;
const int   scPinL = 7;

float rpmBuffer[5]      = {0};
float accelEncBuffer[5] = {0};
float accelImuBuffer[5] = {0};
int   rpmIndex     = 0;
bool  bufferFilled = false;

// Test/characterization variables
bool  flag2    = false;
int   counter  = 0;
float rampRPM  = 0.0f;
float rampRate = 6.5f;
float maxRPM   = 130.0f;
float maxPWM   = 130.0f;
float minPWM   = 0.0f;
unsigned long period = 10000;
float period2        = 500.0f;
int maxServoAngle = 158;
int minServoAngle = 22;
int currentAngle  = 90;

// ── Packet structures ────────────────────────────────────────────────────────

// Must match path_reciever.ino exactly (field order + types).
// 7 × float = 28 bytes — nRF24L01 max payload.
struct DataPacket {
  float x;
  float y;
  float heading;
  float speed;
  float vel_imu;
  float gyro_z;
  float servo_angle;
};

struct joy_stick_packet {
  float x = 0;
  float y = 0;
  bool  e_stop;
};

// ── ISRs ─────────────────────────────────────────────────────────────────────

void scISRR() { pulseCountR++; }
void scISRL() { pulseCountL++; }

// ── RF helpers ───────────────────────────────────────────────────────────────

void unpackJoystickData(joy_stick_packet &jdata, int &joyX, int &joyY) {
  joyX = (int)jdata.x;
  joyY = (int)jdata.y;
}

void send_path() {
  DataPacket pkt;
  pkt.x         = pos_x;
  pkt.y         = pos_y;
  pkt.heading   = vehicle_heading;
  pkt.speed     = avgVel;
  pkt.vel_imu   = vel_imu;
  pkt.gyro_z      = gyro_z;
  pkt.servo_angle = (float)servoAngle;

  if (!path_radio.write(&pkt, sizeof(pkt))) {
    //Serial.println("TX FAIL");
  } else {
    /*
    Serial.print("TX OK | X: ");  Serial.print(pkt.x, 2);
    Serial.print(" Y: ");         Serial.print(pkt.y, 2);
    Serial.print(" H: ");         Serial.print(pkt.heading, 1);
    Serial.print(" V: ");         Serial.print(pkt.speed, 2);
    Serial.print(" VI: ");        Serial.print(pkt.vel_imu, 2);
    Serial.print(" A_enc: ");     Serial.print(pkt.accel_enc, 2);
    Serial.print(" A_imu: ");     Serial.print(pkt.accel_imu, 2);
    Serial.print(" SLIP: ");      Serial.println(pkt.slipping);
    */
  }
}

// Returns true when a fresh joystick packet was received.
// lastRFTime is updated here; loop() uses it for the 500 ms failsafe.
bool joy_stick_controls() {
  if (radio.available()) {
    joy_stick_packet jdata;
    radio.read(&jdata, sizeof(jdata));  // always drain buffer
    if (flag) {
      unpackJoystickData(jdata, xVal, yVal);
    } else {
      xVal = 500;
      yVal = 100;
    }
    lastRFTime = millis();
    //send_path();
    return true;
  }
  return false;
}

// ── Control ──────────────────────────────────────────────────────────────────

void set_control_params() {
  // STEERING (X)
  if (xVal < 488)
    theta = map(xVal, 0, 488, 22, 90);
  else if (xVal > 535)
    theta = map(xVal, 535, 1023, 90, 158);
  else
    theta = 90;

  // Slew-rate limit (5 deg per call)
  if (theta - theta_old >=  5) theta = theta_old + 5;
  if (theta - theta_old <= -5) theta = theta_old - 5;

  if(flag)
  myservo.write(theta);

  if(flag)
  servoAngle = theta;

  theta_old  = theta;

  // THROTTLE (Y)
  if (yVal < 488)
    velocity = (float)yVal * 2.0f / 488.0f - 2.0f;
  else if (yVal > 535)
    velocity = (float)(yVal - 535) * 2.0f / (1023.0f - 535.0f);
  else
    velocity = 0.0f;

  // DIFFERENTIAL DRIVE
  const float wheelBaseFactor = 1.515f;
  const float offsetFactor    = 0.1875f;
  float theta_rad = theta * PI / 180.0f;

  if (theta == 90) {
    r_RPM_left  = velocity;
    r_RPM_right = velocity;
  } else {
    float turnFactor = tan(theta_rad - (PI/2));
    r_RPM_left  = velocity * (1.0f + ((turnFactor * offsetFactor) / (wheelBaseFactor)));
    r_RPM_right = velocity * (1.0f - ((turnFactor * offsetFactor) / (wheelBaseFactor)));
  }
}

void drive() {
  if (r_RPM_left < 0) {
    dir1 = false;
    r_RPM_left = fabsf(r_RPM_left);
  } else {
    dir1 = true;
  }
  if (r_RPM_right < 0) {
    dir2 = true;
    r_RPM_right = fabsf(r_RPM_right);
  } else {
    dir2 = false;
  }

  pwmMotor1 = constrain((int)((r_RPM_left  / 2.6f) * 130.0f), 0, 130);
  pwmMotor2 = constrain((int)((r_RPM_right / 2.6f) * 130.0f), 0, 130);

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
}

// ── Test / characterization drive modes ──────────────────────────────────────

void ramp_drive() {
  if (!flag2)
    rampRPM = constrain(rampRate * counter, 0.0f, maxRPM);
  else
    rampRPM = maxRPM - constrain(rampRate * counter, 0.0f, maxRPM);

  if (rampRPM >= maxRPM) { flag2 = true;  counter = 0; }
  if (rampRPM <= 0.0f  ) { flag2 = false; counter = 0; }

  dir1 = true; dir2 = false;
  pwmMotor1 = pwmMotor2 = constrain((int)rampRPM, 0, 130);

  if ((millis() - lastRampTime) > 1000) { counter++; lastRampTime = millis(); }

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
  //Serial.print("rampRPM: "); Serial.println(rampRPM);
}

void step_drive() {
  unsigned long t = millis() % period;
  float pwmValue  = (t < 5000) ? 0.0f : maxPWM;
  dir1 = true; dir2 = false;
  pwmMotor1 = pwmMotor2 = (int)pwmValue;
  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
  //Serial.print("step PWM: "); Serial.println(pwmValue);
}

void sin_drive() {
  float normalized = (sinf(2.0f * PI * counter / period2) + 1.0f) / 2.0f;
  if ((millis() - lastRampTime) > 100) { counter++; lastRampTime = millis(); }

  pwmMotor1 = constrain((int)(minPWM + normalized * (maxPWM - minPWM)), 0, 130);
  dir1 = true; dir2 = false;

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor1);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
  //Serial.print("sin PWM: "); Serial.println(pwmMotor1);
}

// Continuously sweeps servo: 90→min→90→max→90→ (repeat).
// Holds each angle for holdMs while driving both motors forward at testPwm.
// Logs CSV rows (step_angle, heading, gyro_z, vel) to Serial at 20 Hz.
// Never returns — loops indefinitely until the sketch is reset.
//
// Sweep order per cycle:
//   90 → minServoAngle  (stepping by -stepDeg)
//   minServoAngle → 90  (stepping by +stepDeg)
//   90 → maxServoAngle  (stepping by +stepDeg)
//   maxServoAngle → 90  (stepping by -stepDeg)
//
// Usage — replace drive() in loop() with:
//   servo_step_test(80, 5, 2000);   // PWM=80, 5-deg steps, 2 s per step
void servo_step_test(int testPwm, int stepDeg, unsigned long holdMs) {
  // Sweep phases: 0=90→min  1=min→90  2=90→max  3=max→90
  static uint8_t       phase     = 0;
  static int           curAngle  = 90;
  static unsigned long stepStart = 0;
  static unsigned long lastLog   = 0;
  static bool          running   = false;

  if (!running && !stop_loop) {
    phase     = 0;
    curAngle  = 90;
    stepStart = millis();
    lastLog   = millis();
    running   = true;
    dir1 = true; dir2 = false;
    digitalWrite(mainMotor1Dir, dir1);
    digitalWrite(mainMotor2Dir, dir2);
    if(!flag)
    myservo.write(curAngle);
    servoAngle = curAngle;
    //Serial.println("step_angle,heading,gyro_z,vel");
  }

  //analogWrite(mainMotor1, testPwm);
  //analogWrite(mainMotor2, testPwm);

  unsigned long now = millis();

  // Advance step when hold time elapses
  if (now - stepStart >= holdMs) {
    int nextAngle = curAngle;

    if      (phase == 0) nextAngle -= stepDeg;  // 90 → min
    else if (phase == 1) nextAngle += stepDeg;  // min → 90
    else if (phase == 2) nextAngle += stepDeg;  // 90 → max
    else                 nextAngle -= stepDeg;  // max → 90

    // Check phase transition
    if (phase == 0 && nextAngle < minServoAngle) {
      nextAngle = minServoAngle;
      phase = 1;
    } else if (phase == 1 && nextAngle > 90) {
      nextAngle = 90;
      phase = 2;
    } else if (phase == 2 && nextAngle > maxServoAngle) {
      nextAngle = maxServoAngle;
      phase = 3;
    } else if (phase == 3 && nextAngle < 90) {
      nextAngle = 90;
      if(!flag)
      myservo.write(90);

      servoAngle = 90;
      flag = true;
      stop_loop = 1;
      running = false;
      return;
    }

    curAngle   = nextAngle;
    servoAngle = curAngle;
    if(!flag)
    myservo.write(curAngle);

    stepStart = now;
  }
}

// ── Sensors ──────────────────────────────────────────────────────────────────

void update_position() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  const float circumference = 2.0f * PI * 0.0762f;

  float prevVel = avgVel;
  avgVel    = (avgRpm * circumference) / 60.0f;
  accel_enc = (dt > 0.0f) ? (avgVel - prevVel) / dt : 0.0f;

  float heading_rad = vehicle_heading * PI / 180.0f;
  pos_x += avgVel * cosf(heading_rad) * dt;
  pos_y += avgVel * sinf(heading_rad) * dt;
}

void updateImuVelocity() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  sensors_event_t accelData;
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  float fwd_accel = accelData.acceleration.x;
  if (fabsf(fwd_accel) < IMU_ACCEL_DEADBAND) fwd_accel = 0.0f;
  accel_imu = fwd_accel;

  // Leaky integrator — decays toward zero when stationary
  vel_imu = vel_imu * (1.0f - dt / IMU_VEL_TAU) + fwd_accel * dt;

  sensors_event_t gyroData;
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // BNO055 gyroscope returns rad/s; convert z-axis to deg/s for yaw rate.
  gyro_z = gyroData.gyro.z * 180.0f / PI;
}

// Quaternion-based yaw — avoids gimbal lock from Euler angles.
float getHeading() {
  static float heading_smooth = 0.0f;
  static bool  initialized    = false;

  imu::Quaternion q = bno.getQuat();
  float sinYaw = 2.0f * (q.w() * q.z() + q.x() * q.y());
  float cosYaw = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
  float raw    = atan2f(sinYaw, cosYaw) * 180.0f / PI;

  if (raw < 0.0f)    raw += 360.0f;
  if (raw >= 360.0f) raw -= 360.0f;

  if (!initialized) { heading_smooth = raw; initialized = true; return heading_smooth; }

  float diff = raw - heading_smooth;
  if (diff >  180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;

  const float MAX_STEP = 1.0f;
  if (diff >  MAX_STEP) diff =  MAX_STEP;
  if (diff < -MAX_STEP) diff = -MAX_STEP;

  heading_smooth += diff * 0.05f;

  if (heading_smooth >= 360.0f) heading_smooth -= 360.0f;
  if (heading_smooth <  0.0f)   heading_smooth += 360.0f;

  return heading_smooth;
}

void debug() {
  Serial.print("X:");     Serial.print(xVal);
  Serial.print(" Y:");    Serial.print(yVal);
  Serial.print(" Th:");   Serial.print(theta);
  Serial.print(" Vel:");  Serial.print(velocity);
  Serial.print(" P1:");   Serial.print(pwmMotor1);
  Serial.print(" P2:");   Serial.print(pwmMotor2);
  Serial.print(" SLIP:"); Serial.println(is_slipping);
}

// ── Setup / Loop ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  if (!bno.begin()) Serial.println("No BNO055 detected");

  SPI1.begin();
  radio.begin(&SPI1);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  SPI.begin();
  path_radio.begin(&SPI);
  path_radio.openWritingPipe(address2);
  path_radio.setPALevel(RF24_PA_MIN);
  path_radio.stopListening();
  delay(100);

  myservo.attach(9);
  myservo.write(90);

  pinMode(mainMotor1,    OUTPUT);
  pinMode(mainMotor2,    OUTPUT);
  pinMode(mainMotor1Dir, OUTPUT);
  pinMode(mainMotor2Dir, OUTPUT);
  pinMode(LED_BUILTIN,   OUTPUT);

  pinMode(scPinR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(scPinR), scISRR, RISING);
  pinMode(scPinL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(scPinL), scISRL, RISING);

  lastRFTime     = millis();
  lastSampleTime = millis();
  lastRampTime   = millis();
  delay(1000);
}

void loop() {
  vehicle_heading = getHeading();

  // Read joystick RF — updates xVal/yVal and lastRFTime when data arrives.
  joy_stick_controls();

  // RF failsafe: force neutral when link has been lost for 500 ms.
  if ((millis() - lastRFTime) >= 500) {
    xVal = 512;
    yVal = 512;
  }

  // set_control_params runs AFTER joy_stick_controls so it uses the freshest xVal/yVal.
  set_control_params();
  if(flag);
  drive();
  servo_step_test(100, 68, 2000);

  const unsigned long path_period = 100;
  if (millis() - lastSampleT >= path_period) {
  send_path();
  lastSampleT += path_period;
  }

  // Encoder RPM sampling at 200 ms intervals.
  // update_position and updateImuVelocity run here so they always use fresh avgRpm.
  const unsigned long samplePeriodMs = 200;
  if (millis() - lastSampleTime >= samplePeriodMs) {
    noInterrupts();
    unsigned long count = (pulseCountR + pulseCountL) / 2;
    pulseCountR = 0;
    pulseCountL = 0;
    interrupts();

    float pps = (count * 1000.0f) / samplePeriodMs;
    float rpm = (pps * 60.0f) / pulsesPerRevolution;

    rpmBuffer[rpmIndex] = rpm;
    rpmIndex++;
    if (rpmIndex >= 5) { rpmIndex = 0; bufferFilled = true; }

    int   samples = bufferFilled ? 5 : rpmIndex;
    float sumRpm  = 0;
    for (int i = 0; i < samples; i++) sumRpm += rpmBuffer[i];
    if (samples > 0) avgRpm = sumRpm / samples;

    // Compute derived quantities now that avgRpm is fresh.
    updateImuVelocity();  // updates accel_imu, vel_imu
    update_position();    // updates avgVel, accel_enc, pos_x, pos_y

    // Smooth accel values over the buffer window.
    accelEncBuffer[rpmIndex == 0 ? 4 : rpmIndex - 1] = accel_enc;
    accelImuBuffer[rpmIndex == 0 ? 4 : rpmIndex - 1] = accel_imu;
    float sumAenc = 0, sumAimu = 0;
    for (int i = 0; i < samples; i++) {
      sumAenc += accelEncBuffer[i];
      sumAimu += accelImuBuffer[i];
    }
    if (samples > 0) {
      accel_enc = sumAenc / samples;
      accel_imu = sumAimu / samples;
    }

    is_slipping = (avgVel > SLIP_MIN_SPEED) &&
                  (fabsf(accel_enc - accel_imu) > SLIP_ACCEL_THRESHOLD);

    debug();
    lastSampleTime += samplePeriodMs;
  }
}
