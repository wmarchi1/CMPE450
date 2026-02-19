#include <RYUW122_UWB.h>

// --- Pin Definitions ---
#define RYUW122_RESET_PIN 6

// --- UWB Module Object ---
RYUW122_UWB uwb(Serial1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(115200);

  Serial.println("RYUW122 example: Verified Tag");

  // Initialize module
  if (!uwb.begin(RYUW122_RESET_PIN)) {
    while (1) {
      Serial.println("Module offline");
      delay(500);
    }
  }
  Serial.println("Module online!");

  // Set network and address
  uwb.setNetworkID("ICSTEST");
  delay(50);
  uwb.setAddress("TAG1");
  delay(50);

  // Verify Network ID
  char netID[32];
  if (uwb.getNetworkID(netID, sizeof(netID))) {
    Serial.print("Network ID: "); Serial.println(netID);
  } else {
    Serial.println("Failed to read Network ID");
  }

  // Verify Address
  char addr[32];
  if (uwb.getAddress(addr, sizeof(addr))) {
    Serial.print("Tag Address: "); Serial.println(addr);
  } else {
    Serial.println("Failed to read Address");
  }

  // Set response message
  uwb.setTagResponseMessage("HELLO");
}

void loop() {
  RYUW122_MessageInfo info;

  // Check for messages from anchors
  if (uwb.receiveMessage(info)) {
    Serial.println("----- MESSAGE RECEIVED -----");
    Serial.print("Anchor address: "); Serial.println(info.address);
    Serial.print("Payload length: "); Serial.println(info.payloadLength);
    Serial.print("Payload: "); Serial.println(info.payload);
    Serial.print("Estimated distance: "); Serial.print(info.distance); Serial.println(" cm");
    Serial.println();
  }
}
