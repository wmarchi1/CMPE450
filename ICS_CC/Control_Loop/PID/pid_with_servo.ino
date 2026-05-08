/*
  Autonomous Vehicle Cascade Controller
  Position PID -> Velocity Command
  Velocity PID -> PWM Command

  Added:
  NRF path_radio telemetry transmitter
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// ===================== NRF PATH RADIO =====================
RF24 path_radio(27, 29);          // CE, CSN
const byte address2[6] = "00100"; // telemetry address

struct DataPacket {
  float desiredPos;
  float currentPos;
  float desiredVel;
  float currentVel;
  float desiredHeading;
  float currentHeading;
  int servoCommand;
  float steeringCorrection;
};

// ===================== Motor Pins =====================
int mainMotor1 = 48;
int mainMotor2 = 49;
int mainMotor1Dir = 36;
int mainMotor2Dir = 37;

// ===================== Encoder Pins =====================
const int scPinR = 6;
const int scPinL = 7;

volatile unsigned long pulseCountR = 0;
volatile unsigned long pulseCountL = 0;

const float pulsesPerRevolution = 187.79855;
const float wheelRadius = 0.0762;   // meters
const float wheelCircumference = 2.0 * PI * wheelRadius;

// ===================== Control Timing =====================
unsigned long lastControlTime = 0;
const unsigned long controlPeriodMs = 50;   // 20 Hz control loop

// ===================== Telemetry Timing =====================
unsigned long lastTXTime = 0;
const unsigned long txPeriodMs = 100;       // 10 Hz NRF transmit

// ===================== Desired Values =====================
float desiredPosition = 20.0;   // meters
float desiredVelocity = 0.0;   // m/s

// ===================== Measured Values =====================
float currentPosition = 0.0;   // meters
float currentVelocity = 0.0;   // m/s

// ===================== Position PID Gains =====================
float Kp_pos = 1.81493632280259;
float Ki_pos = 0.23534906486077;
float Kd_pos = 2.18688527899715;

// ===================== Velocity PID Gains =====================
float Kp_vel = 0.707021748901557;
float Ki_vel = 0.431770610575882;
float Kd_vel = -0.437408677228963;

// ===================== PID Memory =====================
float posIntegral = 0.0;
float prevPosError = 0.0;

float velIntegral = 0.0;
float prevVelError = 0.0;

// ===================== Limits =====================
float maxVelocityCommand = 2.0;   // m/s
int maxPWM = 130;

// ===================== Servo Steering =====================
Servo steeringServo;
int servoCommand = 0;
float steeringCorrection = 0;
const int servoPin = 9;

// ===================== Heading PID =====================
float desiredHeading = 0;     // degrees
float currentHeading = 0.0;     // from IMU

float Kp_heading = 0.5;
float Ki_heading = 0.1;
float Kd_heading = 0.0;

float headingIntegral = 0.0;
float prevHeadingError = 0.0;

float centerServo = 90.0;
float minServo = 22.0;
float maxServo = 158.0;

// ===================== Interrupts =====================
void scISRR() {
  pulseCountR++;
}

void scISRL() {
  pulseCountL++;
}

// ===================== Send Telemetry =====================
void sendTelemetry() {
  DataPacket pkt;

  pkt.desiredPos = desiredPosition;
  pkt.currentPos = currentPosition;
  pkt.desiredVel = desiredVelocity;
  pkt.currentVel = currentVelocity;
  pkt.desiredHeading = desiredHeading;
  pkt.currentHeading = currentHeading;
  pkt.servoCommand = servoCommand;
  pkt.steeringCorrection = steeringCorrection;



  bool success = path_radio.write(&pkt, sizeof(pkt));

  if (!success) {
    Serial.println("TX FAIL");
  } else {
    // Serial.print("TX | dPos: ");
    // Serial.print(pkt.desiredPos);

    // Serial.print(" | cPos: ");
    // Serial.print(pkt.currentPos);

    // Serial.print(" | dVel: ");
    // Serial.print(pkt.desiredVel);

    // Serial.print(" | cVel: ");
    // Serial.println(pkt.currentVel);
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    //while (1);
  }
  delay(1000);

  SPI.begin();
  path_radio.begin(&SPI);
  path_radio.openWritingPipe(address2);
  path_radio.setPALevel(RF24_PA_MIN);
  path_radio.stopListening();
  delay(100);

  pinMode(mainMotor1, OUTPUT);
  pinMode(mainMotor2, OUTPUT);
  pinMode(mainMotor1Dir, OUTPUT);
  pinMode(mainMotor2Dir, OUTPUT);

  pinMode(scPinR, INPUT_PULLUP);
  pinMode(scPinL, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(scPinR), scISRR, RISING);
  attachInterrupt(digitalPinToInterrupt(scPinL), scISRL, RISING);

  lastControlTime = millis();
  lastTXTime = millis();

  Serial.println("Autonomous controller with NRF telemetry ready.");
  steeringServo.attach(servoPin);
  steeringServo.write(90);
}

// ===================== Main Loop =====================
void loop() {
  unsigned long now = millis();

  if (now - lastControlTime >= controlPeriodMs) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    updateVelocity(dt);
    updatePosition(dt);
    autonomousControl(dt);
    currentHeading = getHeading();   // from BNO055
    steeringPID(dt);
    sendTelemetry();
    debugPrint();
  }
}

float getHeading() {
  static float heading_smooth = 0;
  static bool initialized = false;

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float raw = orientationData.orientation.x;  // 0–360

  // Wrap
  if (raw >= 360.0f) raw -= 360.0f;
  if (raw < 0.0f)    raw += 360.0f;

  if (!initialized) {
    heading_smooth = raw;
    initialized = true;
    return heading_smooth;
  }

  // Shortest path difference
  float diff = raw - heading_smooth;
  if (diff > 180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;

  // Rate limit (deg per loop)
  const float MAX_STEP = 5.0f;
  if (diff >  MAX_STEP) diff =  MAX_STEP;
  if (diff < -MAX_STEP) diff = -MAX_STEP;

  // Low-pass filter
  const float ALPHA = 0.2f;
  heading_smooth += diff * ALPHA;

  // Wrap again
  if (heading_smooth >= 360.0f) heading_smooth -= 360.0f;
  if (heading_smooth < 0.0f)    heading_smooth += 360.0f;

  return heading_smooth;
}

// ===================== Update Velocity =====================
void updateVelocity(float dt) {
  noInterrupts();

  unsigned long countR = pulseCountR;
  unsigned long countL = pulseCountL;

  pulseCountR = 0;
  pulseCountL = 0;

  interrupts();

  float avgPulses = (countR + countL) / 2.0;

  float revolutions = avgPulses / pulsesPerRevolution;
  float distance = revolutions * wheelCircumference;

  currentVelocity = distance / dt;
}

// ===================== Update Position =====================
void updatePosition(float dt) {
  currentPosition += currentVelocity * dt;
}

float angleError(float target, float current) {
  float error = target - current;

  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;

  return error;
}

void steeringPID(float dt) {
  float error = angleError(desiredHeading, currentHeading);

  headingIntegral += error * dt;
  float derivative = (error - prevHeadingError) / dt;

  steeringCorrection =
      Kp_heading * error
    + Ki_heading * headingIntegral
    + Kd_heading * derivative;

  prevHeadingError = error;

  servoCommand = centerServo + steeringCorrection;

  servoCommand = constrain(servoCommand, minServo, maxServo);

  steeringServo.write((int)servoCommand);
}

// ===================== Autonomous Cascade Control =====================
void autonomousControl(float dt) {
  float posError = desiredPosition - currentPosition;

  posIntegral += posError * dt;
  float posDerivative = (currentPosition - prevPosError) / dt;

  float velocityCommand =
      Kp_pos * posError
    + Ki_pos * posIntegral
    - Kd_pos * posDerivative;

  prevPosError = posError;

  //desiredVelocity = constrain(
  //  desiredVelocity,
  //  -maxVelocityCommand,
  //  maxVelocityCommand
  //);

  float velError = desiredVelocity + velocityCommand - currentVelocity;

  velIntegral += velError * dt;
  float velDerivative = (velError - prevVelError) / dt;

  float pwmCommand =
      Kp_vel * velError
    + Ki_vel * velIntegral
    + Kd_vel * velDerivative;

  prevVelError = velError;

  //pwmCommand = constrain(pwmCommand, -maxPWM, maxPWM);
  pwmCommand = constrain(pwmCommand, 0, maxPWM);

  drivePWM(pwmCommand);
}

// ===================== Drive Motors =====================
void drivePWM(float pwmCommand) {
  int pwm = abs((int)pwmCommand);

  if (pwmCommand >= 0) {
    digitalWrite(mainMotor1Dir, HIGH);
    digitalWrite(mainMotor2Dir, LOW);
  } else {
    digitalWrite(mainMotor1Dir, LOW);
    digitalWrite(mainMotor2Dir, HIGH);
  }

  analogWrite(mainMotor1, pwm);
  analogWrite(mainMotor2, pwm);
}

// ===================== Debug =====================
void debugPrint() {
  Serial.print("Desired Position: ");
  Serial.print(desiredPosition);

  Serial.print(" | Current Position: ");
  Serial.print(currentPosition);

  Serial.print(" | Desired Velocity: ");
  Serial.print(desiredVelocity);

  Serial.print(" | Current Velocity: ");
  Serial.println(currentVelocity);
}
