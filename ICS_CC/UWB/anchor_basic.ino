#include <RYUW122_UWB.h>

// --- Pin Definitions ---
#define RYUW122_RESET_PIN 6

// --- UWB Module Object ---
RYUW122_UWB uwb(Serial1);

// --- Kalman Filter Variables ---
float kalmanX = 10;      // Filtered distance 0
float kalmanP = 100;      // Initial estimation error 1
/*
P
This represents how confident the filter is in kalmanX.
Higher values → filter is less confident, relies more on measurements.
Lower values → filter trusts its estimate more.
*/
float kalmanQ = 10;   // Process noise 0.01
/*
Q
This represents the uncertainty in the system — how much you expect the distance to change between measurements.
Small Q → assume the distance changes slowly → smoother output.
Large Q → assume distance can change quickly → filter reacts faster to sudden changes.
*/
float kalmanR = 50;    // Measurement noise 0.1
/*
R
This represents the uncertainty in the measurements themselves
Large R → measurements are noisy, filter trusts its estimate more.
Small R → measurements are very reliable, filter trusts new readings more.
*/

float applyKalman(float measurement) {
    // Prediction update
    kalmanP = kalmanP + kalmanQ;

    // Kalman gain
    float K = kalmanP / (kalmanP + kalmanR);

    // Measurement update
    kalmanX = kalmanX + K * (measurement - kalmanX);
    kalmanP = (1 - K) * kalmanP;

    return kalmanX;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial1.begin(115200);

    Serial.println("RYUW122 example: Anchor with Kalman Filter");

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

    if (uwb.receiveMessage(info)) {
        float filteredDistance = applyKalman(info.distance);

        Serial.println("----- MESSAGE RECEIVED -----");
        Serial.print("Tag address: ");
        Serial.println(info.address);
        Serial.print("Payload length: ");
        Serial.println(info.payloadLength);
        Serial.print("Payload: ");
        Serial.println(info.payload);
        Serial.print("Raw distance: ");
        Serial.print(info.distance);
        Serial.println(" cm");
        Serial.print("Filtered distance: ");
        Serial.print(filteredDistance);
        Serial.println(" cm");
        Serial.println();
    } else {
        Serial.println("Anchor did not receive a response from tag");
    }

    delay(1000);
}
