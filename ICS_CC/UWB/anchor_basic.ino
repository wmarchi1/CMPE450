#include <RYUW122_UWB.h>

// --- Pin Definitions ---
#define RYUW122_RESET_PIN 6

// --- UWB Module Object ---
RYUW122_UWB uwb(Serial1);

// Optional: Kalman Filter for smoothing distance
float kalmanX = 10;
float kalmanP = 100;
float kalmanQ = 10;
float kalmanR = 50;

float applyKalman(float measurement) {
  kalmanP = kalmanP + kalmanQ;
  float K = kalmanP / (kalmanP + kalmanR);
  kalmanX = kalmanX + K * (measurement - kalmanX);
  kalmanP = (1 - K) * kalmanP;
  return kalmanX;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(115200);

  Serial.println("RYUW122 Anchor Example");

  // Initialize module
  if (!uwb.begin(RYUW122_RESET_PIN)) {
    while (1) {
      Serial.println("Module offline");
      delay(500);
    }
  }

  // Hardware reset
  uwb.reset();
  delay(500);

  // Set module mode to ANCHOR
  uwb.setMode(MODE_ANCHOR);
  delay(50);

  // Set network ID and address
  uwb.setNetworkID("ICSTEST");
  delay(50);
  uwb.setAddress("ANCH01");
  delay(50);

  // Increase distance timeout
  uwb.setDistanceResponseTimeout(500);

  Serial.println("Anchor ready!");
}

void loop() {
  RYUW122_MessageInfo info;

  // Send a message to tag
  uwb.sendMessage("TAG001", "PING");

  // Listen for response
  if (uwb.receiveMessage(info)) {
    float filteredDistance = applyKalman(info.distance);

    Serial.println("----- RESPONSE RECEIVED -----");
    Serial.print("Tag address: "); Serial.println(info.address);
    Serial.print("Payload length: "); Serial.println(info.payloadLength);
    Serial.print("Payload: "); Serial.println(info.payload);
    Serial.print("Raw distance: "); Serial.print(info.distance); Serial.println(" cm");
    Serial.print("Filtered distance: "); Serial.print(filteredDistance); Serial.println(" cm");
    Serial.println();
  } else {
    Serial.println("No response from tag");
  }

  delay(1000);
}
