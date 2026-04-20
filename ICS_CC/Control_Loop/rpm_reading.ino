#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 6);  // CE, CSN
const byte address[6] = "00100";

float position;
float avgRpm;
float avgVel;
struct DataPacket {
  float position;           // meters east of origin
  float avgRpm;           // meters north of origin
  float avgVel;     // degrees, 0=north, clockwise
};
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
    DataPacket pkt;
    radio.read(&pkt, sizeof(pkt));
    position = pkt.position;
    avgRpm = pkt.avgRpm;
    avgVel = pkt.avgVel;
    // Print results
    Serial.print("Position (m): ");
    Serial.println(position);
    Serial.print("RPM: ");
    Serial.println(avgRpm);
    Serial.print("velocity: ");
    Serial.println(avgVel);
  }

  //Serial.println("Not working");
}
