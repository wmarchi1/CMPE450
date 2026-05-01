#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 6);  // CE, CSN
const byte address[6] = "00100";

float desiredPos_;
float currentPos_;
float desiredVel_;
float currentVel_;
struct DataPacket {
  //float desiredPos;
  float currentPos;
  //float desiredVel;
  float currentVel;
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
  delay(1000);
  radio.startListening();
  Serial.println("Receiver Ready");
}

void loop() {
  while (radio.available()) {
    // Read first float (avgRpm)
    DataPacket pkt;
    radio.read(&pkt, sizeof(pkt));
    //desiredPos_ = pkt.desiredPos;
    currentPos_ = pkt.currentPos;
    currentVel_ = pkt.currentVel;
    //desiredVel_ = pkt.desiredVel;
    // Print results
    //Serial.print("Desired Position:");
    //Serial.print(desiredPos_);
    Serial.print(" | Current Position: ");
    Serial.print(currentPos_);
    //Serial.print(" | Desired Velocity: ");
    //Serial.println(desiredVel_);
    Serial.print(" | Current Velocity: ");
    Serial.println(currentVel_);
  }

  //Serial.println("Not working");
}
