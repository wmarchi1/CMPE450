/*
 * Giga R1 NRF24L01 Receiver
 * RF joystick → servo + motors
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


// RF
RF24 radio(2, 3);              // CE, CSN
const byte address[6] = "00001";

RF24 path_radio(27, 29);
const byte address2[6] = "00100"; //leader-follow address used for transmitting path info

// Servo
Servo myservo;

//IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// bool e_stop = false;

//  Joystick (RF) 
int xVal = 512;   // steering
int yVal = 512;   // throttle

float pos_x = 0.0f;   // meters (east)
float pos_y = 0.0f;   // meters (north)

// Control Variables 
int theta = 90;
int theta_old = 90;
float velocity = 0;
float avgRpm = 0;
float vehicle_heading = 0;

int mainMotor1 = 48;
int mainMotor2 = 49;
int mainMotor1Dir = 36;
int mainMotor2Dir = 37;
int mainMotor1Stop = 33;
int mainMotor2Stop = 32;

int pwmMotor1 = 0;
int pwmMotor2 = 0;
float r_RPM_left = 0;
float r_RPM_right = 0;
static float position = 0.0f;
float avgVel;

struct DataPacket {
  float x;
  float y;
  float heading;
  float speed;
};

struct joy_stick_packet{
  float x = 0;
  float y = 0;
  bool e_stop;
};

bool dir1 = false;
bool dir2 = false;

float rampRPM = 0.0;
float rampRate = 6.5;
float maxRPM = 130;
unsigned long lastRampTime = 0;

float maxPWM = 130.0;
float minPWM = 0;
unsigned long period = 10000;
float period2 = 50.0;

volatile unsigned long pulseCountR = 0;
volatile unsigned long pulseCountL = 0;
unsigned long lastSampleTime = 0;
unsigned long lastRFTime = 0;

const float pulsesPerRevolution = 187.79855;
const int scPinR = 6;
const int scPinL = 7;

float rpmBuffer[5] = {0};
int rpmIndex = 0;
bool bufferFilled = false;

void scISRR() {
  pulseCountR++;
}

void scISRL() {
  pulseCountL++;
}
// Unpack 
void unpackJoystickData(joy_stick_packet &jdata, int &joyX, int &joyY) {
  joyX =  jdata.x;      // bits 0–9
  joyY = jdata.y;      // bits 10–19
  
}
void send_path() {
  DataPacket pkt;

  pkt.x = pos_x;                 // meters east
  pkt.y = pos_y;                 // meters north
  pkt.heading = vehicle_heading; // degrees
  pkt.speed = avgVel;            // m/s

  bool success = path_radio.write(&pkt, sizeof(pkt));

  if (!success) {
    Serial.println("TX FAIL");
  } else {
    Serial.print("TX OK | X: ");
    Serial.print(pkt.x, 2);
    Serial.print(" Y: ");
    Serial.print(pkt.y, 2);
    Serial.print(" H: ");
    Serial.print(pkt.heading, 1);
    Serial.print(" V: ");
    Serial.println(pkt.speed, 2);
  }
}

bool joy_stick_controls(){
  if (radio.available()) {
    joy_stick_packet jdata;
    radio.read(&jdata, sizeof(jdata));

    unpackJoystickData(jdata, xVal, yVal);
    lastRFTime = millis();

    send_path();
    return true;
  } else {
    if ((millis() - lastRFTime) >= 500) {
      xVal = 500;
      yVal = 500;
      set_control_params();
      drive();
    }
    return false;
  }
}

void set_control_params(){
  // STEERING (X) 
  if (xVal < 488)
    theta = map(xVal, 0, 488, 158, 90);
  else if (xVal > 535)
    theta = map(xVal, 535, 1023, 90, 22);
  else
    theta = 90;

  // Steering slew-rate limit
  if (theta - theta_old >= 5)  theta = theta_old + 5;
  if (theta - theta_old <= -5) theta = theta_old - 5;

  myservo.write(theta);
  theta_old = theta;

  //  THROTTLE (Y) 
  if (yVal < 488)
    //velocity = map(yVal, 0, 488, -200, 0) / 100.0f;
    velocity = ( (float)(yVal - 0) ) * (0.0 - (-2.0)) / (488.0 - 0.0) + (-2.0);
  else if (yVal > 535)
    //velocity = map(yVal, 535, 1023, 0, 200) / 100.0f;
    velocity = ( (float)(yVal - 535) ) * (2.0 - 0.0) / (1023.0 - 535.0) + 0.0 *1.0;
  else
    velocity = 0;

  //  DIFFERENTIAL DRIVE 
  const float wheelBaseFactor = 1.515f;
  const float offsetFactor    = 0.1875f;

  float theta_rad = theta * PI / 180.0f;

  if (theta == 90) {
    r_RPM_left  = velocity;
    r_RPM_right = velocity;
  } else {
    float turnFactor = tan(theta_rad);
    r_RPM_left  = velocity * (1.0f + offsetFactor / (wheelBaseFactor * turnFactor));
    r_RPM_right = velocity * (1.0f - offsetFactor / (wheelBaseFactor * turnFactor));
  }
}

void drive(){
  if (r_RPM_left < 0) {
    dir1 = false;
    //digitalWrite(mainMotor1Dir, LOW);
    r_RPM_left = abs(r_RPM_left);

  }
  else {
    dir1 = true;
    //digitalWrite(mainMotor1Dir, HIGH);
  }
  if (r_RPM_right < 0) {
    dir2 = true;
    //digitalWrite(mainMotor2Dir, LOW);
    r_RPM_right = abs(r_RPM_right);
  }
  else {
    dir2 = false;
    //digitalWrite(mainMotor2Dir, HIGH);
  }


  // ================= PWM =================
  pwmMotor1 = (int)( (r_RPM_left  / 2.6f) * 130.0f );
  pwmMotor2 = (int)( (r_RPM_right / 2.6f) * 130.0f );
  pwmMotor1 = constrain(pwmMotor1, 0, 130);
  pwmMotor2 = constrain(pwmMotor2, 0, 130);

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
}
bool flag2 = false;
int counter = 0;
void ramp_drive() {
  //float elapsed = (millis() - lastRampTime) / 10.0f;
  if (flag2 == false){
    dir1 = true;
    dir2 = false;
    rampRPM = constrain(rampRate * counter, 0.0f, maxRPM);
  }
  else {
    dir1 = true;
    dir2 = false;
    rampRPM = maxRPM - (constrain(rampRate * counter, 0.0f, maxRPM));
  }

  if(rampRPM >= maxRPM){
    flag2 = true;
    counter = 0;
  }

  pwmMotor1 = constrain((int)rampRPM, 0, 130);
  pwmMotor2 = pwmMotor1;
  if (((millis()- lastRampTime) > 1000)) {
    counter = counter + 1;
    lastRampTime = millis();
  }
  Serial.print(" RPM R:");
  Serial.println(rampRPM);

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
}

void step_drive() {
  unsigned long t = millis() % period;

  float pwmValue = (t < 5000) ? 0 : maxPWM;

  dir1 = true;
  dir2 = false;

  pwmMotor1 = (int)pwmValue;
  pwmMotor2 = (int)pwmValue;

  Serial.print("PWM: ");
  Serial.println(pwmValue);

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
}

float t = 0;

void sin_drive() {
  //float t = millis() / 1000.0f;
  float normalized = (sin(2 * PI * t / period2) + 1.0f) / 2.0f;  // [0, 1]
  t = millis() / 1000.0f;
  pwmMotor1 = (int)(minPWM + normalized * (maxPWM - minPWM));
  pwmMotor1 = constrain(pwmMotor1, 0, 130);

  dir1 = true;
  dir2 = false;

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor1);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);

  Serial.print("PWM: ");
  Serial.println(pwmMotor1);
}

void debug() {
  Serial.print("X:");
  Serial.print(xVal);
  Serial.print(" Y:");
  Serial.print(yVal);
  Serial.print(" Theta:");
  Serial.print(theta);
  Serial.print(" Vel:");
  Serial.print(velocity);
  Serial.print(" PWM1:");
  Serial.print(pwmMotor1);
  Serial.print(" PWM2:");
  Serial.println(pwmMotor2);
  Serial.print(" RPM Right:");
  Serial.print(r_RPM_right);
  Serial.print(" RPM Left:");
  Serial.println(r_RPM_left);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  // while (!Serial) {}
  Wire.begin();
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    //while (1);
  }


  SPI1.begin();

  //joystick input receiver
  radio.begin(&SPI1);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //path_gen transmitter
  SPI.begin();
  path_radio.begin(&SPI);
  path_radio.openWritingPipe(address2);
  path_radio.setPALevel(RF24_PA_MIN);
  path_radio.stopListening();
  delay(100);


  myservo.attach(9);

  pinMode(mainMotor1, OUTPUT);
  pinMode(mainMotor2, OUTPUT);
  pinMode(mainMotor1Dir, OUTPUT);
  pinMode(mainMotor2Dir, OUTPUT);
  // pinMode(mainMotor1Stop, OUTPUT);
  // pinMode(mainMotor2Stop, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(scPinR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(scPinR), scISRR, RISING);

  pinMode(scPinL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(scPinL), scISRL, RISING);
  lastSampleTime = millis();
  lastRFTime = millis();
  lastRampTime = millis();
}

void update_position() {
  static unsigned long lastTime = millis();

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // Convert RPM → velocity (m/s)
  const float radius = 0.0762f;
  const float pi = 3.14159265f;
  float circumference = 2.0f * pi * radius;

  avgVel = (avgRpm * circumference) / 60.0f;

  // Convert heading to radians
  float heading_rad = vehicle_heading * PI / 180.0f;
  // Convert heading to radians

  // Project velocity into world frame
  float vx = avgVel * cos(heading_rad);
  float vy = avgVel * sin(heading_rad);

  // Integrate position
  pos_x += vx * dt;
  pos_y += vy * dt;

  Serial.print("X: ");
  Serial.print(pos_x, 2);
  Serial.print("  Y: ");
  Serial.print(pos_y, 2);
  Serial.print("  Heading: ");
  Serial.print(vehicle_heading);
  Serial.print("  Vel: ");
  Serial.println(avgVel);
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
void loop() {
  set_control_params();

  vehicle_heading = getHeading();

  joy_stick_controls();

  if ((millis() - lastRFTime) < 500) {
    sin_drive();
    update_position();
  } else {
    analogWrite(mainMotor1, 0);
    analogWrite(mainMotor2, 0);
  }

  const unsigned long samplePeriodMs = 200;
  if (millis() - lastSampleTime >= samplePeriodMs) {
    noInterrupts();
    unsigned long count = (pulseCountR + pulseCountL) / 2;
    pulseCountR = 0;
    pulseCountL = 0;
    interrupts();

    float pulsesPerSecond = (count * 1000.0) / samplePeriodMs;
    float rpm = (pulsesPerSecond * 60.0) / pulsesPerRevolution;

    rpmBuffer[rpmIndex] = rpm;
    rpmIndex++;

    if (rpmIndex >= 5) {
      rpmIndex = 0;
      bufferFilled = true;
    }

    int samples = bufferFilled ? 5 : rpmIndex;

    for (int i = 0; i < samples; i++) {
      avgRpm += rpmBuffer[i];
    }

    if (samples > 0) {
      avgRpm /= samples;
    }

    lastSampleTime += samplePeriodMs;
  }
}
