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
#include "Path.cpp"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// ===================== NRF PATH RADIO =====================
RF24 path_radio(27, 29);          // CE, CSN
const byte path_address[6] = "00100"; //receives chkpnts from lead

// Transmits follower telemetry
RF24 telemetry_radio(2, 3); //for emergency stop
const byte telemetry_address[6] = "10000"; //addr for francois

// Must match transmitter packet exactly
struct joy_stick_packet {
  float x;
  float y;
  bool e_stop;
};

// RF emergency stop
unsigned long lastRFTime = 0;
const unsigned long rfTimeoutMs = 500;   // stop if no packet for 500 ms
bool remoteEStop = false;

struct DataPacket {
  float   x;
  float   y;
  float   heading;
  float   speed;
  bool e_stop;
};

struct TelemetryPacket {
  float currentX;
  float currentY;
  float desiredPos;
  //float currentPos;
  //float desiredVel;
  float currentVel;
  //float desiredHeading;
  float currentHeading;
  //int servoCommand;
  //float steeringCorrection;
  float velocityCommand;
  //float pwmCommand;
  float filteredPWM;
};

DataPacket path_checkpnt; //received from lead
Path macro_path(200);

// ===================== Motor Pins =====================
int mainMotor1 = 48;
int mainMotor2 = 49;
int mainMotor1Dir = 36;
int mainMotor2Dir = 37;
float pwm_left = 0.0f;
float pwm_right = 0.0f;

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
float desiredPosition = 0.0;   // meters
float desiredVelocity = 0.5;   // m/s
float desiredX = 0;
float desiredY = 0;
float oldDesiredX = 0.0;
float oldDesiredY = 0.0;

float adjustedDesiredVelocity = 0.0;

float destinationTolerance = 0.7;   // meters
float segmentStartPosition = 0.0;

// ===================== Measured Values =====================
float currentPosition = 0.0;   // meters
float currentVelocity = 0.0;   // m/s
float currentX  = -5.0;
float currentY  = 0.0;

// // ===================== Position PID Gains =====================
float Kp_pos = 0.270495557544331;
float Ki_pos = 0.0126024625302669;
float Kd_pos = 0.387212887171821;

// // ===================== Velocity PID Gains =====================
float Kp_vel = 19.6232429592983;
float Ki_vel = 70.9400238823325;
float Kd_vel = 0;

// ===================== PID Memory =====================
float posIntegral = 0.0;
float prevPosError = 0.0;

float velIntegral = 0.0;
float prevVelError = 0.0;

// ===================== Limits =====================
float maxVelocityCommand = 2.0;   // m/s
int maxPWM = 80;

// ===================== Servo Steering =====================
Servo steeringServo;
int servoCommand = 0;
float steeringCorrection = 0;
const int servoPin = 9;

// ===================== Heading PID =====================
float desiredHeading = 0.0;     // degrees
float currentHeading = 0.0;     // from IMU
float desiredCheckPointHeading = 0.0;

float Kp_heading = 0.5;
float Ki_heading = 0;
float Kd_heading = 0;

float headingIntegral = 0.0;
float prevHeadingError = 0.0;

float centerServo = 90.0;
float minServo = 22.0;
float maxServo = 158.0;
float rawServo = 0.0;

// ===================== Output Filtering =====================

// PWM filter
float filteredPWM = 0.0;
const float pwmAlpha = 0.75;   
// smaller = smoother
// typical: 0.05 to 0.3

// Servo filter
float filteredServo = 0.0;
const float servoAlpha = 1.00;

float pwmCommand = 0.0;
float velocityCommand = 0.0;
// ===================== Interrupts =====================
void scISRR() {
  pulseCountR++;
}

void scISRL() {
  pulseCountL++;
}

void emergencyStop() {
  drivePWM(0);

  desiredVelocity = 0.0;
  filteredPWM = 0.0;

  // Prevent PID windup while stopped
  posIntegral = 0.0;
  prevPosError = 0.0;
  velIntegral = 0.0;
  prevVelError = 0.0;
  headingIntegral = 0.0;
  prevHeadingError = 0.0;

  // Center steering during stop
  filteredServo = 90.0;
  servoCommand = 90;
  steeringServo.write(90);

  static unsigned long lastStopPrint = 0;
  if (millis() - lastStopPrint > 500) {;
    lastStopPrint = millis();
  }
}

//===================== Send Telemetry =====================
void sendTelemetry() {
  TelemetryPacket pkt;
  pkt.currentX = currentX;
  pkt.currentY = currentY;
  pkt.desiredPos = desiredPosition;
  //pkt.currentPos = currentPosition;
  //pkt.desiredVel = desiredVelocity;
  pkt.currentVel = currentVelocity;
  //pkt.desiredHeading = desiredHeading;
  pkt.currentHeading = currentHeading;
  //pkt.servoCommand = servoCommand;
  //pkt.steeringCorrection = steeringCorrection;

  pkt.velocityCommand = velocityCommand;
  //-pkt.pwmCommand = pwmCommand;
  pkt.filteredPWM = filteredPWM;


  bool success = telemetry_radio.write(&pkt, sizeof(pkt));

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

// get's a checkpoint from lead vehicle and adds it to micro path
bool receive_path() {
  // Serial.println("Entering receive_path function");

  if (!path_radio.available()) {
    Serial.println("no data in radio");
    return false;
  }

  while (path_radio.available()){
    path_radio.read(&path_checkpnt, sizeof(path_checkpnt));

    if (path_checkpnt.e_stop) {
      remoteEStop = true;
      return true;
    } else {
      remoteEStop = false;
    }

    //checks that we don't have double data
    Coord c;
    c.x = path_checkpnt.x;
    c.y = path_checkpnt.y;
    c.speed = path_checkpnt.speed;
    c.heading = path_checkpnt.heading;

    Coord c2;
    if (macro_path.size() > 0) {
    macro_path.getRear(c2);

    if (macro_path.compare(c, c2)) //duplicate data
      return false;
    }

    Serial.print("-----Received-----");
    Serial.print("X: "); Serial.println(c.x);
    Serial.print("Y: "); Serial.println(c.y);
    // return macro_path.insert(c);
  }

  return true;
}

// ===================== Setup =====================
bool hasTarget = false;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    //while (1);
  }
  delay(1000);

  SPI1.begin(); //for E stop on joystick
  if (!telemetry_radio.begin(&SPI1)) {
    Serial.println("telemetry_radio hardware not responding");
    while(1);
  }
  Serial.print("joy_radio ok, channel: ");
  Serial.println(telemetry_radio.getChannel());
  telemetry_radio.openWritingPipe(telemetry_address);
  telemetry_radio.setPALevel(RF24_PA_MIN);
  telemetry_radio.stopListening();

  SPI.begin(); //checkpoints from leader
  if (!path_radio.begin(&SPI)) {
    Serial.println("path_radio hardware not responding");
    while(1);
  }
  Serial.print("path_radio ok, channel: ");
  Serial.println(path_radio.getChannel());
  path_radio.openReadingPipe(1, path_address);
  path_radio.setPALevel(RF24_PA_MIN);
  path_radio.startListening();
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
  lastRFTime = millis();

  //Serial.println("Autonomous controller with NRF telemetry ready.");
  steeringServo.attach(servoPin);
  // Center steering during stop
  filteredServo = 90.0;
  servoCommand = 90;
  steeringServo.write(90);
  // desiredX = 0;
  // desiredY = 0;
  //segmentStartPosition = currentPosition;

  macro_path.insert({0, 0, 0.1, 0}); //follower goes to origin
  if (macro_path.size() > 0) {
    Coord c;
    macro_path.process(c);
    desiredX = c.x;
    desiredY = c.y;
    desiredCheckPointHeading = c.heading;
    hasTarget = true;
    }
  delay(1000);
}

void getDesiredPolar() {
  float dx = desiredX - currentX;
  float dy = desiredY - currentY;

  desiredPosition = sqrt((dx * dx) + (dy * dy));
  //float safeDesiredX = max(dx, 0.01);
  //float safeDesiredY = max(dy, 0.01);
  float headingDirection =  atan2(dx, dy) * (180/PI);
  //float distanceToTarget = sqrt(dx * dx + dy * dy);
  float checkpointHeading = desiredCheckPointHeading;
  // Since currentPosition is cumulative distance traveled,
  // desiredPosition should also be cumulative.
  //desiredPosition = segmentStartPosition + distanceToTarget;

  //desiredHeading = atan2(dx, dy) * 180.0 / PI;
  if (headingDirection < 0) {
    headingDirection += 360.0;
  }
  headingDirection = -(headingDirection - 90.0);
  if (headingDirection < 0) {
    headingDirection += 360.0;
  }
  headingDirection = headingDirection * PI/180; // convert back to radians
  checkpointHeading = checkpointHeading * PI/180; // convert to radians
  float weightDirection = max((desiredPosition-0.8)*10, 0);
  float weightCheckpoint = 1/(desiredPosition + 0.1);
  float xM = cos(headingDirection)*weightDirection + cos(checkpointHeading)*weightCheckpoint;
  float yM = sin(headingDirection)*weightDirection + sin(checkpointHeading)*weightCheckpoint;
  desiredHeading = atan2(yM, xM) * 180.0 / PI;
  if (desiredHeading < 0) {
    desiredHeading += 360.0;
  }

}

void changePath() {
  if (desiredPosition <= destinationTolerance) {

    if (macro_path.size() == 0) {
      // No new targets → stop or hold
      drivePWM(0);
      desiredVelocity = 0;
      return;
    }

    Coord C;
    if (macro_path.process(C)) {   // <-- IMPORTANT if your API supports bool
      oldDesiredX = desiredX;
      oldDesiredY = desiredY;

      desiredX = C.x;
      desiredY = C.y;
      desiredCheckPointHeading = C.heading;

      // Reset controllers
      posIntegral = 0.0;
      prevPosError = 0.0;
      velIntegral = 0.0;
      prevVelError = 0.0;
      headingIntegral = 0.0;
      prevHeadingError = 0.0;

      desiredPosition = 0.0;
      currentPosition = 0.0;

      Serial.print("New target X: ");
      Serial.print(desiredX);
      Serial.print(" | New target Y: ");
      Serial.println(desiredY);
    }
  }
}

int xVal = 0;
int yVal = 0;
void unpackJoystickData(joy_stick_packet &jdata, int &joyX, int &joyY) {
  joyX = (int)jdata.x;
  joyY = (int)jdata.y;
}

//deprecated
// bool path_emergency() {
//   if (path_radio.available()) {
//     DataPacket jdata;
//     path_radio.read(&jdata, sizeof(jdata));
//     // unpackJoystickData(jdata, xVal, yVal);
//     if (jdata.e_stop)
//       lastRFTime = millis();
//     // send_path();
//     return true;
//   }
//   return false;
// }

// ===================== Main Loop =====================
void loop() {
  unsigned long now = millis();

  receive_path();
  //Emergency stop if transmitter button is pressed
  if (remoteEStop) {
    emergencyStop();
    lastControlTime = millis();   // prevents dt jump when restarting
    return;
  }

  if (!hasTarget && macro_path.size() > 0) {
    Coord c;
    macro_path.process(c);
    desiredX = c.x;
    desiredY = c.y;
    desiredCheckPointHeading = c.heading;
    hasTarget = true;
  }

  if (!hasTarget) {
    steeringServo.write(90);
    return;
  }

  // Normal autonomous control only runs if RF link is alive
  if (hasTarget) {
    if (now - lastControlTime >= controlPeriodMs) {
      float dt = (now - lastControlTime) / 1000.0;
      lastControlTime = now;

      currentHeading = getHeading();

      getDesiredPolar();
      changePath();
      updateVelocity(dt);
      updatePosition(dt);
      autonomousControl(dt);
      steeringPID(dt);
      sendTelemetry();
      //debugPrint();
    }
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
  const float ALPHA = 1.0f;
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

  currentVelocity = (distance / dt) * (1.246);
}

// ===================== Update Position =====================
void updatePosition(float dt) {
  float distanceStep = currentVelocity * dt;

  currentPosition += distanceStep;

  float headingRad = currentHeading * PI / 180.0;

  currentX += distanceStep * cos(headingRad);
  currentY += distanceStep * sin(headingRad);
  Serial.println("");
  Serial.print("CurrentX: ");
  Serial.print(currentX);
  Serial.println("");
  Serial.print("CurrentY: ");
  Serial.print(currentY);
  Serial.println("");
  Serial.print("distanceStep: ");
  Serial.print(distanceStep);
  Serial.println("");
  Serial.print("Position: ");
  Serial.print(currentPosition);
  Serial.println("");
  Serial.print("Heading Rad: ");
  Serial.print(headingRad);
  Serial.println("");
  Serial.print("Heading Deg: ");
  Serial.print(currentHeading);
  Serial.println("");
}

float angleError(float target, float current) {
  if (target - 90 < 0)
    target += 360;
  float error = -target + current;

  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;

  return error;
}

void steeringPID(float dt) {
  float error = angleError(desiredHeading, currentHeading);

  headingIntegral += error * dt;
  //headingIntegral = constrain(headingIntegral, -50, 50);
  float derivative = (error - prevHeadingError) / dt;

  steeringCorrection =
      Kp_heading * error
    + Ki_heading * headingIntegral
    + Kd_heading * derivative;

  prevHeadingError = error;

  // Raw steering command
  // float safeVelocity = max(currentVelocity, 0.05);

  rawServo =  centerServo + atan2(steeringCorrection * 0.515 * PI, 180.0 * currentVelocity) * 180.0 / PI;

// Limit servo range
  float constrainServo = constrain(rawServo, minServo, maxServo);
  filteredServo = servoAlpha * constrainServo  + (1.0 - servoAlpha) * filteredServo;
  servoCommand = (int)filteredServo;
  // if(currentVelocity == 0){
  //   steeringServo.write(90);
  // }
  // else{
  steeringServo.write(servoCommand);
  // }
}

// ===================== Autonomous Cascade Control =====================
void autonomousControl(float dt) {
  float temp_posError = desiredPosition;

  float posError = pwmAlpha * temp_posError + (1.0 - pwmAlpha) * temp_posError;
  posIntegral += posError * dt;
  float posDerivative = (currentPosition - prevPosError) / dt;

  velocityCommand =
      Kp_pos * posError
    + Ki_pos * posIntegral
    - Kd_pos * posDerivative;

  prevPosError = currentPosition;

  //desiredVelocity = constrain(
  //  desiredVelocity,
  //  -maxVelocityCommand,
  //  maxVelocityCommand
  //);

  float temp_velError = desiredVelocity + velocityCommand - currentVelocity;
  float velError = pwmAlpha * temp_velError + (1.0 - pwmAlpha) * temp_velError;

  velIntegral += velError * dt;
  float velDerivative = (velError - prevVelError) / dt;

  pwmCommand =
      Kp_vel * velError
    + Ki_vel * velIntegral
    + Kd_vel * velDerivative;

  prevVelError = velError;

  //pwmCommand = constrain(pwmCommand, -maxPWM, maxPWM);
  pwmCommand = constrain(pwmCommand, 0, maxPWM);

// ================= LOW PASS FILTER =================
  filteredPWM = pwmAlpha * pwmCommand + (1.0 - pwmAlpha) * filteredPWM;

  drivePWM(filteredPWM);
}


// ===================== Drive Motors =====================
void drivePWM(float pwmCommand) {
  //differential drive
  const float wheelBaseFactor = 1.515f;
  const float offsetFactor    = 0.1875f;
  float theta_rad = servoCommand * PI / 180.0f;
  float turnFactor = tan(theta_rad - (PI/2));
  pwm_left  = pwmCommand * (1.0f + ((turnFactor * offsetFactor) / (wheelBaseFactor)));
  pwm_right = pwmCommand * (1.0f - ((turnFactor * offsetFactor) / (wheelBaseFactor)));

  pwm_left = abs((int)pwm_left);
  pwm_right = abs((int)pwm_right);



  if (pwmCommand >= 0) {
    digitalWrite(mainMotor1Dir, LOW);
    digitalWrite(mainMotor2Dir, HIGH);
  } else {
    digitalWrite(mainMotor1Dir, HIGH);
    digitalWrite(mainMotor2Dir, LOW);
  }

  analogWrite(mainMotor1, pwm_left);
  analogWrite(mainMotor2, pwm_right);
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
