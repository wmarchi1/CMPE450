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
int theta_old = 90;
bool flag = true;
void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
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
  Serial.print(" | theta_old: ");
  Serial.println(theta_old);
 
  delay(10);

}

