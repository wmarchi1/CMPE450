/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 90;    // variable to store the servo position
int xPin = A0;
int yPin = A1;
int buttonPin = 2;
int xVal; // variable for storing joystick x values
int yVal; // variable for storing joystick y values
int buttonState; // variable for storing joystick switch state
int theta = 90;
float velocity = 0;
int theta_old = 90;
bool flag = true;
int mainMotor1 = 11;
int mainMotor2 = 3;
int mainMotor1Dir = 5;
int mainMotor2Dir = 7;
int pwmMotor1 = 0;
int pwmMotor2 = 0;
float r_RPM_left = 0;
float r_RPM_right = 0;
bool dir1 = false;
bool dir2 = false;
void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(mainMotor1,OUTPUT);
  pinMode(mainMotor2,OUTPUT);
  pinMode(mainMotor1Dir,OUTPUT);
  pinMode(mainMotor2Dir,OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600); // initialize the serial monitor
}


void loop() {
  xVal = analogRead(xPin);
  if (xVal < 488) {
    theta = map(xVal,0,488,22,90);
    //flag = true;
  }
  else if (xVal > 535) {
    theta = map(xVal,535,1023,90,158);
    //flag = true;
  }
  else {
    theta = 90;
    //flag = true;

  }
  if (((theta - theta_old) >= 2) && (flag == true))  {
    theta = theta_old + 2;
  }
  if (((theta - theta_old) <= -2) && (flag == true)) {
    theta = theta_old - 2;
  }
  //theta = 45;
  myservo.write(theta);
  theta_old = theta;
  yVal = analogRead(yPin);
  if (yVal < 488) {
    velocity = ( (float)(yVal - 0) ) * (0.0 - (-2.0)) / (488.0 - 0.0) + (-2.0);
    //flag = true;
  }
  else if (yVal > 535) {
    velocity = ( (float)(yVal - 535) ) * (2.0 - 0.0) / (1023.0 - 535.0) + 0.0;
    //flag = true;
  }
  else {
    velocity = 0;
    //flag = true;

  }
// constants
  const float wheelBaseFactor = 1.515f;
  const float offsetFactor    = 0.1875f;

// convert degrees → radians
  float theta_rad = theta * PI / 180.0f;

// avoid division by zero
  if (theta == 90) {
    r_RPM_left  = velocity;
    r_RPM_right = velocity;
  } else {
    float turnFactor = tan(theta_rad);

    r_RPM_left  = velocity * ((1.0f) + (offsetFactor / (wheelBaseFactor * turnFactor)));
    r_RPM_right = velocity * ((1.0f) - (offsetFactor / (wheelBaseFactor * turnFactor)));
  }


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

  pwmMotor1 = (int)( (r_RPM_left  / 2.6f) * 216.0f );
  pwmMotor2 = (int)( (r_RPM_right / 2.6f) * 216.0f );
  pwmMotor1 = constrain(pwmMotor1, 0, 216);
  pwmMotor2 = constrain(pwmMotor2, 0, 216);

  analogWrite(mainMotor1, pwmMotor1);
  analogWrite(mainMotor2, pwmMotor2);
  digitalWrite(mainMotor1Dir, dir1);
  digitalWrite(mainMotor2Dir, dir2);
  buttonState = digitalRead(buttonPin);
 
  // print readings to the serial monitor
  Serial.print("X: ");
  Serial.print(xVal);
  Serial.print(" | Y: ");
  Serial.print(yVal);
  Serial.print(" | Switch: ");
  Serial.print(buttonState);
  Serial.print(" | theta: ");
  Serial.print(theta);
  Serial.print(" | velocity: ");
  Serial.print(velocity);
  //Serial.print(" | pwm motor 1: ");
  //Serial.print(pwmMotor1);
  //Serial.print(" | pwm motor 2: ");
  //Serial.print(pwmMotor2);
  Serial.print(" | rpm motor 1: ");
  Serial.print(r_RPM_left);
  Serial.print(" | rpm motor 2: ");
  Serial.print(r_RPM_right);
  Serial.print(" | direction 1: ");
  Serial.println(dir1);
  Serial.print(" | direction 2: ");
  Serial.println(dir2);
  //Serial.print(" | theta_old: ");
  //Serial.println(theta_old);

  //analogWrite(mainMotor1, 155);
  //analogWrite(mainMotor2, 155);
  //digitalWrite(mainMotor2Dir, 0);

  delay(100);

}

