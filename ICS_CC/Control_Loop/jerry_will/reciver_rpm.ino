#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 6);  // CE, CSN
const byte address[6] = "00100";

float avgRpm;
float pwmVal;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Receiver Ready");

  SPI.begin();
  radio.begin(&SPI);

  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  while (radio.available()) {

    // Read first float (avgRpm)
    radio.read(&avgRpm, sizeof(avgRpm));

    // Wait for next packet if needed
    while (!radio.available());

    // Read second float (pwmVal)
    radio.read(&pwmVal, sizeof(pwmVal));

    // Print results
    Serial.print("RPM: ");
    Serial.print(avgRpm);

    Serial.print(" | PWM: ");
    Serial.println(pwmVal);
  }
}
