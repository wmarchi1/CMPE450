#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 6);  // CE, CSN
const byte address[6] = "00100";

struct DataPacket {
  float x;
  float y;
  float heading;
  float speed;
  float vel_imu;
  float   accel_enc;
  float   accel_imu;
  uint8_t slipping;
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

    Serial.print("X: ");       Serial.print(pkt.x, 2);
    Serial.print("  Y: ");     Serial.print(pkt.y, 2);
    Serial.print("  H: ");     Serial.print(pkt.heading, 1);
    Serial.print("  V_enc: "); Serial.print(pkt.speed, 2);
    Serial.print("  V_imu: "); Serial.print(pkt.vel_imu, 2);
    Serial.print("  A_enc: "); Serial.print(pkt.accel_enc, 2);
    Serial.print("  A_imu: "); Serial.print(pkt.accel_imu, 2);
    Serial.print("  SLIP: ");  Serial.println(pkt.slipping);
  }
}
