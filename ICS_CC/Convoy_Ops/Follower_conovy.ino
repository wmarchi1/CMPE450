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

// RF — joystick receiver
RF24 joy_radio(2, 3); //for emergency stop
const byte joy_address[6] = "00001";

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
float desiredVelocity = 0.0;   // m/s
float desiredX = 0;
float desiredY = 0;
float oldDesiredX = 0.0;
float oldDesiredY = 0.0;

// float pathX[] = {2,0, 4.0, 6.0, 8.0};//, 15.0, 20.0};
// float pathY[] = {2.0, 0.0, 2.0, 0.0};//, -10.0, -10.0};

// const int numPoints = sizeof(pathX) / sizeof(pathX[0]);
int pathCount = 0;

float destinationTolerance = 0.5;   // meters
float segmentStartPosition = 0.0;

// ===================== Measured Values =====================
float currentPosition = 0.0;   // meters
float currentVelocity = 0.0;   // m/s
float currentX  = 0.0;
float currentY  = 0.0;

// // ===================== Position PID Gains =====================
float Kp_pos = 0.47639159476101;
float Ki_pos = 0.00326960229075012;
float Kd_pos = 2;

// // ===================== Velocity PID Gains =====================
float Kp_vel = 0;
float Ki_vel = 65.256384084022102;
float Kd_vel = 0;

// ===================== Position PID Gains =====================
//float Kp_pos = 1.81493632280259;
//float Ki_pos = 0.23534906486077;
//float Kd_pos = 2.18688527899715;

// ===================== Velocity PID Gains =====================
//float Kp_vel = 0.707021748901557;
//float Ki_vel = 0.431770610575882;
//float Kd_vel = -0.437408677228963;

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
float desiredHeading = 0.0;     // degrees
float currentHeading = 0.0;     // from IMU

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
const float pwmAlpha = 0.15;   
// smaller = smoother
// typical: 0.05 to 0.3

// Servo filter
float filteredServo = 0.0;
const float servoAlpha = 0.12;

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
// ===================== Send Telemetry =====================
// void sendTelemetry() {
//   DataPacket pkt;
//   pkt.currentX = currentX;
//   pkt.currentY = currentY;
//   pkt.desiredPos = desiredPosition;
//   pkt.currentPos = currentPosition;
//   //pkt.desiredVel = desiredVelocity;
//   pkt.currentVel = rawServo;
//   pkt.desiredHeading = desiredHeading;
//   pkt.currentHeading = currentHeading;
//   pkt.servoCommand = servoCommand;
//   pkt.steeringCorrection = steeringCorrection;


//   bool success = path_radio.write(&pkt, sizeof(pkt));

//   if (!success) {
//     Serial.println("TX FAIL");
//   } else {
//     // Serial.print("TX | dPos: ");
//     // Serial.print(pkt.desiredPos);

//     // Serial.print(" | cPos: ");
//     // Serial.print(pkt.currentPos);

//     // Serial.print(" | dVel: ");
//     // Serial.print(pkt.desiredVel);

//     // Serial.print(" | cVel: ");
//     // Serial.println(pkt.currentVel);
//   }
// }

// get's a checkpoint from lead vehicle and adds it to micro path
bool receive_path() {
  Serial.println("Entering receive_path function");

  if (!path_radio.available()) {
    Serial.println("no data in radio");
    return false;
  }

  path_radio.read(&path_checkpnt, sizeof(path_checkpnt));

  // if (!path_checkpnt.valid) {
  //   Serial.println("NO data received");
  //   return false;
  // }

  //checks that we don't have double data
  Coord c;
  c.x = path_checkpnt.x;
  c.y = path_checkpnt.y;
  c.speed = path_checkpnt.speed;
  c.heading = path_checkpnt.heading;

  Coord c2;
  macro_path.getRear(c2);

  if (macro_path.compare(c, c2)) //duplicate data
    return false;

  Serial.print("-----Received-----");
  Serial.print("X: "); Serial.println(c.x);
  Serial.print("Y: "); Serial.println(c.y);
  return macro_path.insert(c);
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

  SPI1.begin(); //for E stop on joystick
  if (!joy_radio.begin(&SPI1)) {
    Serial.println("joy_radio hardware not responding");
    while(1);
  }
  Serial.print("joy_radio ok, channel: ");
  Serial.println(joy_radio.getChannel());
  // joy_radio.begin(&SPI1);
  joy_radio.openReadingPipe(1, joy_address);
  joy_radio.setPALevel(RF24_PA_MIN);
  joy_radio.startListening();

  SPI.begin(); //checkpoints from leader
  if (!path_radio.begin(&SPI)) {
    Serial.println("path_radio hardware not responding");
    while(1);
  }
  Serial.print("path_radio ok, channel: ");
  Serial.println(path_radio.getChannel());
  // path_radio.begin(&SPI);
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
  desiredX = 0;
  desiredY = 0;
  delay(1000);
  //segmentStartPosition = currentPosition;
}
void getDesiredPolar() {
  float dx = desiredX - oldDesiredX;
  float dy = desiredY - oldDesiredY;

  desiredPosition = sqrt((dx * dx) + (dy * dy));
  float safeDesiredX = max(dx, 0.01);
  //float safeDesiredY = max(dy, 0.01);
  desiredHeading =  atan2(dy, safeDesiredX) * (180/PI);

  //float distanceToTarget = sqrt(dx * dx + dy * dy);

  // Since currentPosition is cumulative distance traveled,
  // desiredPosition should also be cumulative.
  //desiredPosition = segmentStartPosition + distanceToTarget;

  //desiredHeading = atan2(dx, dy) * 180.0 / PI;
  if (desiredHeading < 0) {
    desiredHeading += 360.0;
  }
}
void changePath() {

  if ((desiredPosition - currentPosition) <= destinationTolerance) {
    pathCount++;

    if (macro_path.size() == 0) {
      drivePWM(0);
      // desiredVelocity = 0;
      Serial.println("Path complete.");
      return;
    }


    Coord C;
    macro_path.process(C);
    oldDesiredX = desiredX;
    oldDesiredY = desiredY;
    desiredX = C.x;
    desiredY = C.y;
    desiredVelocity = C.speed;

    //segmentStartPosition = currentPosition;

    // Reset PID memories when changing target
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
int xVal = 0;
int yVal = 0;
void unpackJoystickData(joy_stick_packet &jdata, int &joyX, int &joyY) {
  joyX = (int)jdata.x;
  joyY = (int)jdata.y;
}

bool joy_stick_controls() {
  if (joy_radio.available()) {
    joy_stick_packet jdata;
    joy_radio.read(&jdata, sizeof(jdata));
    unpackJoystickData(jdata, xVal, yVal);
    lastRFTime = millis();
    //send_path();
    return true;
  }
  return false;
}
// ===================== Main Loop =====================
void loop() {
  unsigned long now = millis();
  joy_stick_controls();
  // Always read RF first.
  // This clears the RX buffer and updates lastRFTime if transmitter is alive.
  //readEmergencyRadio();

  // Emergency stop if transmitter button is pressed
  if ((millis() - lastRFTime) >= 500) {
    emergencyStop();
    lastControlTime = millis();   // prevents dt jump when restarting
    return;
  }

  // Normal autonomous control only runs if RF link is alive
  if (now - lastControlTime >= controlPeriodMs) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    currentHeading = getHeading();

    receive_path();
    getDesiredPolar();
    updateVelocity(dt);
    updatePosition(dt);
    autonomousControl(dt);
    steeringPID(dt);
    // sendTelemetry();
    changePath();

    //debugPrint();
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
  float distanceStep = currentVelocity * dt;

  currentPosition += distanceStep;

  float headingRad = currentHeading * PI / 180.0;

  currentX += distanceStep * cos(headingRad);
  currentY += distanceStep * sin(headingRad);
  // Serial.println("");
  // Serial.print("CurrentX: ");
  // Serial.print(currentX);
  // Serial.println("");
  // Serial.print("CurrentY: ");
  // Serial.print(currentY);
  // Serial.println("");
  // Serial.print("distanceStep: ");
  // Serial.print(distanceStep);
  // Serial.println("");
  // Serial.print("Position: ");
  // Serial.print(currentPosition);
  // Serial.println("");
  // Serial.print("Heading Rad: ");
  // Serial.print(headingRad);
  // Serial.println("");
  // Serial.print("Heading Deg: ");
  // Serial.print(currentHeading);
  // Serial.println("");
}

float angleError(float target, float current) {
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
  float safeVelocity = max(currentVelocity, 0.05);

  rawServo =  centerServo + atan2(steeringCorrection * 0.515 * PI, 180.0 * safeVelocity) * 180.0 / PI;

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

  float velError = max(desiredVelocity, 0.3) + velocityCommand - currentVelocity;

  velIntegral += velError * dt;
  float velDerivative = (velError - prevVelError) / dt;

  float pwmCommand =
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
