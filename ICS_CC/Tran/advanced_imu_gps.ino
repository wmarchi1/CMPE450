
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ---- RF24 pins (must match transmitter) ----
#define RF_CE_PIN   6
#define RF_CSN_PIN  7

// ---- Receive timeout ----
const uint32_t PACKET_TIMEOUT_MS = 2000;   // warn if no packet for 2 s

//  DATA PACKET — must be byte-for-byte identical to transmitter
struct DataPacket {
  float   x;          // meters east of origin
  float   y;          // meters north of origin
  float   heading;    // degrees, 0=north, clockwise
  float   speed;      // m/s
  float   pitch;      // degrees
  float   roll;       // degrees
  uint8_t gps_fix;    // 1 = GPS fix active
  uint8_t sats;       // satellite count
};

RF24 radio(RF_CE_PIN, RF_CSN_PIN);
const byte RF_ADDRESS[6] = "00100";   // must match transmitter

DataPacket pkt;
uint32_t   last_rx_ms   = 0;
uint32_t   packet_count = 0;
bool       link_ok      = false;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  delay(300);

  Serial.println("\n=== Vehicle Localizer — RECEIVER ===");

  SPI.begin();

  if (!radio.begin()) {
    Serial.println("ERROR: RF24 not found — check wiring");
    while (1);
  }

  // ---- Radio config — must exactly mirror transmitter ----
  radio.openReadingPipe(1, RF_ADDRESS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.startListening();

  Serial.println("RF24 OK — listening on pipe 1");
  Serial.println("CSV header: x,y,heading,speed,pitch,roll,gps_fix,sats");
  Serial.println();
}

// ============================================================
//  LOOP
// ============================================================

void loop() {

  // ---- Check for incoming packet ----
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    last_rx_ms   = millis();
    packet_count++;
    link_ok = true;

    // ---- CSV output (for Serial Plotter / data logger) ----
    Serial.print(pkt.x,       2);  Serial.print(",");
    Serial.print(pkt.y,       2);  Serial.print(",");
    Serial.print(pkt.heading, 1);  Serial.print(",");
    Serial.print(pkt.speed,   2);  Serial.print(",");
    Serial.print(pkt.pitch,   1);  Serial.print(",");
    Serial.print(pkt.roll,    1);  Serial.print(",");
    Serial.print(pkt.gps_fix);     Serial.print(",");
    Serial.println(pkt.sats);

    // ---- Human-readable debug 
    /*
    Serial.print("[#");        Serial.print(packet_count);
    Serial.print("] X:");      Serial.print(pkt.x,       2);
    Serial.print(" Y:");       Serial.print(pkt.y,       2);
    Serial.print(" HDG:");     Serial.print(pkt.heading, 1);
    Serial.print("° SPD:");    Serial.print(pkt.speed,   2);
    Serial.print("m/s P:");    Serial.print(pkt.pitch,   1);
    Serial.print("° R:");      Serial.print(pkt.roll,    1);
    Serial.print("° GPS:");    Serial.print(pkt.gps_fix ? "FIX" : "NOFIX");
    Serial.print(" SAT:");     Serial.println(pkt.sats);
    */
  }

  // ---- Link-loss warning ----
  if (link_ok && (millis() - last_rx_ms > PACKET_TIMEOUT_MS)) {
    Serial.print("WARN: no packet for ");
    Serial.print((millis() - last_rx_ms) / 1000);
    Serial.println("s — check transmitter");
    link_ok = false;   // suppress repeated warnings until next good packet
  }
}
