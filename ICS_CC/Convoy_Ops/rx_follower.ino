#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 6);  // CE, CSN
const byte address[6] = "00100";

float currentX_;
float currentY_;
float desiredPos_;
float currentVel_;
float desiredHeading_;
float currentHeading_;
float velocityCommand_;

struct DataPacket {
  float currentX;
  float currentY;
  float desiredPos;
  //float currentPos;
  //float desiredVel;
  float currentVel;
  float desiredHeading;
  float currentHeading;
  //int servoCommand;
  //float steeringCorrection;
  float velocityCommand;
  //float pwmCommand;
  //float filteredPWM;
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
    DataPacket pkt;
    radio.read(&pkt, sizeof(pkt));

    currentX_ = pkt.currentX;
    currentY_ = pkt.currentY;
    desiredPos_ = pkt.desiredPos;
    currentVel_ = pkt.currentVel;
    desiredHeading_ = pkt.desiredHeading;
    currentHeading_ = pkt.currentHeading;
    velocityCommand_ = pkt.velocityCommand;

    Serial.print(currentVel_);
    Serial.print(",");
    Serial.print(desiredPos_);
    Serial.print(",");
    Serial.print(currentX_);
    Serial.print(",");
    Serial.print(currentY_);
    Serial.print(",");
    Serial.print(desiredHeading_);
    Serial.print(",");
    Serial.print(currentHeading_);
    Serial.print(",");
    Serial.println(velocityCommand_);
  }
}
