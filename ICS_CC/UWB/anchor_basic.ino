#include <RYUW122_UWB.h>

// --- Pin Definitions ---
#define RYUW122_RESET_PIN 6

// --- UWB Module Object ---
RYUW122_UWB uwb(Serial1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(115200);  // GIGA only needs baud rate

  Serial.println("RYUW122 example: Verified Anchor");

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
  uwb.setAddress("ANCOR1");
  delay(50);

  // Verify Network ID
  char netID[32];
  if (uwb.getNetworkID(netID, sizeof(netID))) {
    Serial.print("Network ID: "); 
    Serial.println(netID);
  } else {
    Serial.println("Failed to read Network ID");
  }

  // Verify Address
  char addr[32];
  if (uwb.getAddress(addr, sizeof(addr))) {
    Serial.print("Anchor Address: "); 
    Serial.println(addr);
  } else {
    Serial.println("Failed to read Address");
  }
}

void loop() {
  RYUW122_MessageInfo info;

  // Send message to tag
  uwb.sendMessage("TAG1", "TEST_MSG");

  // Check for responses
  if (uwb.receiveMessage(info)) {
    Serial.println("----- MESSAGE RECEIVED -----");
    Serial.print("Tag address: "); Serial.println(info.address);
    Serial.print("Payload length: "); Serial.println(info.payloadLength);
    Serial.print("Payload: "); Serial.println(info.payload);
    Serial.print("Distance: "); Serial.print(info.distance); Serial.println(" cm");
    Serial.println();
  } else {
    Serial.println("Anchor did not receive a response from tag");
  }

  delay(1000); // Wait 1 second before next message
}
