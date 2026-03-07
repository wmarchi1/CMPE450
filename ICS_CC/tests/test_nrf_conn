#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(27, 29); //CE, CSN

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  SPI.begin();

  bool ok = radio.begin(&SPI);
  Serial.println(ok ? "Radio detected" : "Radio NOT detected");
}

void loop() {}
