#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 6);  // CE, CSN
const byte address[6] = "00001";

struct Coord {
    float x;
    float y;
    float speed;
    float heading;
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Receiver Start");

  SPI.begin();
  delay(1000);
  radio.begin(&SPI);
  delay(1000);

  radio.openReadingPipe(1, address);
  delay(1000);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Receiver Ready");
}

void loop() {
  while (radio.available()) {
    // Read first float (avgRpm)
    Coord micro_p;
    radio.read(&micro_p, sizeof(micro_p));

    Serial.print("RX: ");
    Serial.print(micro_p.x);
    Serial.print(", ");
    Serial.print(micro_p.y);
    Serial.print(", ");
    Serial.print(micro_p.speed);
    Serial.print(", ");
    Serial.println(micro_p.heading);
  }

}
