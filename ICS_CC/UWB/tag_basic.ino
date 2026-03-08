#include <RYUW122_UWB.h>

// --- Pin Definitions ---
#define RYUW122_RESET_PIN 3

// --- UWB Module Object ---
RYUW122_UWB uwb(Serial1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(115200);

  Serial.println("RYUW122 Tag Example");

  // Initialize module
  if (!uwb.begin(RYUW122_RESET_PIN)) {
    while (1) {
      Serial.println("Module offline");
      delay(500);
    }
  }

  // Hardware reset for clean startup
  uwb.reset();
  delay(500);

  // Set module mode to TAG
  uwb.setMode(MODE_TAG);
  delay(50);

  // Set network ID (max 8 chars) and tag address (max 8 chars)
  uwb.setNetworkID("ICSTEST");
  delay(50);
  uwb.setAddress("TAG001");
  delay(50);

  // Set response message
  uwb.setTagResponseMessage("HELLO", 5, true);

  // Increase distance timeout for reliability
  uwb.setDistanceResponseTimeout(500);

  Serial.println("Tag ready!");
}

void loop() {
  RYUW122_MessageInfo info;

  // Listen for messages from anchors
  if (uwb.receiveMessage(info)) {
    Serial.println("----- MESSAGE RECEIVED -----");
    Serial.print("Anchor address: "); Serial.println(info.address);
    Serial.print("Payload length: "); Serial.println(info.payloadLength);
    Serial.print("Payload: "); Serial.println(info.payload);
    Serial.print("Estimated distance: "); Serial.print(info.distance); Serial.println(" cm");
    Serial.println();
  }

  delay(100);
}
