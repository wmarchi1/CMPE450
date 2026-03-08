#include <RYUW122_UWB.h>

// --- Pin Definitions ---
#define RYUW122_RESET_PIN 6

// --- UWB Module Object ---
RYUW122_UWB uwb(Serial1);

// --- Kalman Filter Class ---
class KalmanFilter {
public:
    KalmanFilter(float initialX = 0, float initialP = 100, float Q = 10, float R = 50)
        : x(initialX), P(initialP), Q(Q), R(R) {}

    float update(float measurement) {
        // Prediction update
        P = P + Q;
        // Kalman gain
        float K = P / (P + R);
        // Measurement update
        x = x + K * (measurement - x);
        P = (1 - K) * P;
        return x;
    }

private:
    float x; // filtered estimate
    float P; // estimation error
    float Q; // process noise
    float R; // measurement noise
};

// --- Create a filter for each tag ---
KalmanFilter filterTAG1(0);
KalmanFilter filterTAG2(0);

float prev_distance_tag1 = 0;
float prev_distance_tag2 = 0;

int printPosUpdate(float tag1, float tag2){
  int flag = 0;
  if(abs(tag1 - prev_distance_tag1) > 5){
    flag++;
  }
  if(abs(tag2 - prev_distance_tag2) > 5){
    flag = flag + 2;
  }
  prev_distance_tag1 = tag1;
  prev_distance_tag2 = tag2;
  return flag;
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial1.begin(115200);

    Serial.println("RYUW122 Anchor with Kalman Filters");

    if (!uwb.begin(RYUW122_RESET_PIN)) {
        while (1) {
            Serial.println("Module offline");
            delay(500);
        }
    }

    Serial.println("Module online!");
    uwb.setNetworkID("ICSTEST");
    delay(50);
    uwb.setAddress("ANCOR1");
    delay(50);

    char netID[32];
    if (uwb.getNetworkID(netID, sizeof(netID))) Serial.println(netID);

    char addr[32];
    if (uwb.getAddress(addr, sizeof(addr))) Serial.println(addr);
}

float filteredDistance1, filteredDistance2 = 0;

void loop() {
    RYUW122_MessageInfo info;
    // --- Query TAG1 ---
    uwb.sendMessage("TAG001", "TEST_MSG");
    if (uwb.receiveMessage(info)) {
        filteredDistance1 = filterTAG1.update(info.distance);
        int print = printPosUpdate(filteredDistance1, filteredDistance2);
        if(print == 1 || print == 3){
          Serial.println("----- MESSAGE FROM TAG1 -----");
          Serial.print("Raw distance: "); Serial.println(info.distance);
          Serial.print("Filtered distance: "); Serial.println(filteredDistance1);
          Serial.println();
        }
    } else {
        Serial.println("No response from TAG1");
    }

    delay(50);

    // --- Query TAG2 ---
    uwb.sendMessage("TAG002", "TEST_MSG");
    if (uwb.receiveMessage(info)) {
        filteredDistance2 = filterTAG2.update(info.distance);

        int print = printPosUpdate(filteredDistance1,filteredDistance2);
        if(print == 2 || print == 3){
          Serial.println("----- MESSAGE FROM TAG2 -----");
          Serial.print("Raw distance: "); Serial.println(info.distance);
          Serial.print("Filtered distance: "); Serial.println(filteredDistance2);
          Serial.println();
        }
    } else {
        Serial.println("No response from TAG2");
    }

    delay(1000);
}
