/*
 * Giga R1 NRF24L01 Receiver
 * RF joystick → servo + motors
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// RF
RF24 radio(2, 3);              // CE, CSN
const byte address[6] = "00001";

RF24 path_radio(27, 29);
const byte address2[6] = "00100"; //leader-follow address used for transmitting path info

// Servo
Servo myservo;

//  Joystick (RF) 
int xVal = 512;   // steering
int yVal = 512;   // throttle

// Control Variables 
int theta = 90;
int theta_old = 90;
float velocity = 0;
float avgRpm = 0;

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

bool dir1 = false;
bool dir2 = false;

volatile unsigned long pulseCount = 0;
unsigned long lastSampleTime = 0;

const float pulsesPerRevolution = 189.1768;
const int scPin = 6;

float rpmBuffer[5] = {0};
int rpmIndex = 0;
bool bufferFilled = false;

float rampRPM = 0.0;
float rampRate = 1;       // RPM per second
float maxRPM = 130;         // maximum command
unsigned long lastRampTime = 0;


void scISR() {
  pulseCount++;
}
// Unpack 
void unpackJoystickData(uint32_t packed, int &joyX, int &joyY) {
  joyX =  packed        & 0x03FF;      // bits 0–9
  joyY = (packed >> 10) & 0x03FF;      // bits 10–19
  
}
void send_path() {
  //const char test[] = "Hello world!";
  bool success = path_radio.write(&avgRpm, sizeof(avgRpm)); // Send the data
  float pwmVal = float(pwmMotor1);
  success = path_radio.write(&pwmVal, sizeof(pwmVal)); // Send the data

  if (!success) {
    Serial.println("Path Transmission failed");
  } else {
    Serial.println("Success");
  }
}

bool joy_stick_controls(){
  if (radio.available()) {
    uint32_t data;
    radio.read(&data, sizeof(data));
    unpackJoystickData(data, xVal, yVal);
    // digitalWrite(LED_BUILTIN, HIGH);
    //path_gen
    send_path();
    //digitalWrite(mainMotor1Stop, 1);
    //digitalWrite(mainMotor2Stop, 1);
    return true;
  } else {
    // digitalWrite(LED_BUILTIN, LOW);
       // failsafe
    if ((millis() - lastSampleTime) >= 600) {
      xVal = 500;
      yVal = 500;
      //set_control_params();
      //drive();
    }
    //theta = 90;
    //myservo.write(theta);
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
void ramp_drive() {
  unsigned long now = millis();
  float dt = (now - lastRampTime) / 1000.0f;
  lastRampTime = now;

  rampRPM += rampRate * dt;
  if(rampRPM >= maxRPM){
    rampRPM = maxRPM;
  }

  // Fixed direction: one direction only
  dir1 = true;
  dir2 = false;   // or true, depending on your wiring and desired wheel rotation

  // Convert RPM command to PWM
  pwmMotor1 = rampRPM;
  pwmMotor2 = rampRPM;
  Serial.print(" RPM R:");
  Serial.println(rampRPM);
  pwmMotor1 = constrain(pwmMotor1, 0, 130);
  pwmMotor2 = constrain(pwmMotor2, 0, 130);

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
}

float maxPWM = 130.0;
unsigned long period = 40000;   // total cycle = 40s

void step_drive() {
  unsigned long now = millis();
  unsigned long t = now % period;

  float pwmValue;

  if (t < 20000) {
    pwmValue = 0;          // LOW phase (0–20s)
  } else {
    pwmValue = maxPWM;     // HIGH phase (20–40s)
  }

  // Fixed direction
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
float period2 = 100.0;   // seconds for full sine cycle (adjust as needed)

void sin_drive() {
  float t = millis() / 1000.0;   // time in seconds

  // Sine wave: range [-1, 1]
  float sineVal = sin(2 * PI * t / period2);

  // Shift to [0, 1]
  float normalized = (sineVal + 1.0) / 2.0;

  // Scale to PWM range [0, 130]
  pwmMotor1 = int(normalized * maxPWM);

  // Fixed direction
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
  Serial.begin(9600);
  // while (!Serial) {}
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

  myservo.attach(9);

  pinMode(mainMotor1, OUTPUT);
  pinMode(mainMotor2, OUTPUT);
  pinMode(mainMotor1Dir, OUTPUT);
  pinMode(mainMotor2Dir, OUTPUT);
  // pinMode(mainMotor1Stop, OUTPUT);
  // pinMode(mainMotor2Stop, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(scPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(scPin), scISR, RISING);
  lastSampleTime = millis();
}

void loop() {
  set_control_params();
  if (joy_stick_controls()) {
    sin_drive();
    //debug();
    const unsigned long samplePeriodMs = 200;  // faster update

  if (millis() - lastSampleTime >= samplePeriodMs) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
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

    Serial.print("RPM: ");
    Serial.println(avgRpm);

    lastSampleTime += samplePeriodMs;
  }
  }
}
