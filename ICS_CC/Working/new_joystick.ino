#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pins for Transciver
#define CE 6
#define CSN 7

struct joy_stick_packet{
  float x = 0;
  float y = 0;
  bool e_stop = false;
};

RF24 radio(CE, CSN);
const byte address[6] = "00001";

// First Joystick Pins
const int xPin1   = A5;
const int yPin1   = A3;
const int butPin1 = 2;

// Variables to hold Joystick Values
int xVal1, yVal1, butState1;

// Timing Control
const unsigned long READ_INTERVAL_MS = 100; 
unsigned long lastReadTime = 0;

// -------- E-STOP State --------
bool e_stop = false;

// For edge detection
bool lastButtonState = HIGH;

// Optional debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
// ------------------------------

bool transmit_cmd(joy_stick_packet &jdata) {

  bool success = radio.write(&jdata, sizeof(jdata));

  if (success)
    return true;

  return false;
}

void setup() {
  Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  pinMode(butPin1, INPUT_PULLUP);
}

void loop() {

  // --- Check button press and toggle e_stop ---
  bool currentButtonState = digitalRead(butPin1);

  // Detect HIGH -> LOW transition (button press)
  if (currentButtonState == LOW &&
      lastButtonState == HIGH &&
      millis() - lastDebounceTime > debounceDelay) {

      e_stop = !e_stop;   // Toggle state
      lastDebounceTime = millis();

      Serial.print("E-STOP toggled: ");
      Serial.println(e_stop ? "ON" : "OFF");
  }

  lastButtonState = currentButtonState;
  // --------------------------------------------

  unsigned long now = millis();
  joy_stick_packet jdata;

  if (now - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = now;

    readJoysticks();

    printJoystickData();

    jdata.x = xVal1;
    jdata.y = yVal1;
    jdata.e_stop = e_stop;   // send current e-stop state

    bool transmission_success = transmit_cmd(jdata);

    if (!transmission_success) {
      Serial.println("Transmission Failed");
    }
    else {
      Serial.println("Transmission Success");
    }
  }
}

// Read Inputs
void readJoysticks() {
  xVal1 = analogRead(xPin1);
  yVal1 = analogRead(yPin1);
  butState1 = digitalRead(butPin1);
}

// Output Data
void printJoystickData() {
  Serial.print("J1 [X:");
  Serial.print(xVal1);

  Serial.print(" Y:");
  Serial.print(yVal1);

  Serial.print(" B:");
  Serial.print(butState1);

  Serial.print(" E_STOP:");
  Serial.print(e_stop);

  Serial.println("]");
}
