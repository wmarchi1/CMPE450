#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 6);  // CE, CSN
const byte address[6] = "00100";

float desiredPos_;
float currentPos_;
//float desiredVel_;
float currentVel_;
float desiredHead_;
float currentHead_;
//int servoCommand_;
//float steeringCorrection_;
float currentX_;
float currentY_;
float velocityCommand_;
float pwmCommand_;
float filteredPWM_;
struct DataPacket {
  float currentX;
  float currentY;
  float desiredPos;
  //float currentPos;
  //float desiredVel;
  float currentVel;
  //float desiredHeading;
  //float currentHeading;
  //int servoCommand;
  //float steeringCorrection;
  float velocityCommand;
  float pwmCommand;
  float filteredPWM;
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
    desiredPos_ = pkt.desiredPos;
    //currentPos_ = pkt.currentPos;
    currentVel_ = pkt.currentVel;
    //desiredVel_ = pkt.desiredVel;
  //  desiredHead_ = pkt.desiredHeading;
  //  currentHead_ = pkt.currentHeading;
  //  servoCommand_ = pkt.servoCommand;
  //  steeringCorrection_ = pkt.steeringCorrection;
    currentX_ = pkt.currentX;
    currentY_ = pkt.currentY;
    velocityCommand_ = pkt.velocityCommand;
    pwmCommand_ = pkt.pwmCommand;
    filteredPWM_ = pkt.filteredPWM;

    // Print results
    // Serial.print("Desired Position:");
    // Serial.print(desiredPos_);
    // Serial.print(" | Current Position: ");
    // Serial.print(currentPos_);
    // Serial.print(" | Desired Velocity: ");
    // Serial.println(desiredVel_);
    // Serial.print(" | Current Velocity: ");
    // Serial.println(currentVel_);
    // Serial.print(" | Desired Heading: ");
    // Serial.println(desiredHead_);
    // Serial.print(" | Current Heading: ");
    // Serial.println(currentHead_);
    // Serial.print(" | Servo Command: ");
    // Serial.println(servoCommand_);
    // Serial.print(" | Steering Correction: ");
    // Serial.println(steeringCorrection_);

    Serial.print(currentVel_);
    Serial.print(",");
    Serial.print(desiredPos_);
    Serial.print(",");
    Serial.print(currentX_);
    Serial.print(",");
    Serial.print(currentY_);
    Serial.print(",");
    Serial.print(velocityCommand_);
    Serial.print(",");
    Serial.print(pwmCommand_);
    Serial.print(",");
    Serial.println(filteredPWM_);

  }

  //Serial.println("Not working");
}
