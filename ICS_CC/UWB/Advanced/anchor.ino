#include <RYUW122_UWB.h>

// ---------------- Pin Definitions ----------------
#define RYUW122_RESET_PIN 6

// ---------------- UWB Module ----------------
RYUW122_UWB uwb(Serial1);

// ---------------- Kalman Filter (TAG001) ----------------
float kalmanX = 10;
float kalmanP = 100;
float kalmanQ = 10;
float kalmanR = 50;

float applyKalman0(float measurement) {
  kalmanP = kalmanP + kalmanQ;
  float K = kalmanP / (kalmanP + kalmanR);
  kalmanX = kalmanX + K * (measurement - kalmanX);
  kalmanP = (1 - K) * kalmanP;
  return kalmanX;
}

// ---------------- Kalman Filter (TAG002) ----------------
float kalman1X = 10;
float kalman1P = 100;
float kalman1Q = 10;
float kalman1R = 50;

float applyKalman1(float measurement) {
  kalman1P = kalman1P + kalman1Q;
  float K = kalman1P / (kalman1P + kalman1R);
  kalman1X = kalman1X + K * (measurement - kalman1X);
  kalman1P = (1 - K) * kalman1P;
  return kalman1X;
}

// ---------------- Averaging ----------------
const int array_size = 5;

float avg_12[array_size];
float avg_13[array_size];

int index1 = 0;

float average(float *arr) {
  float sum = 0;

  for (int i = 0; i < array_size; i++) {
    sum += arr[i];
  }

  return sum / array_size;
}

// ---------------- Setup ----------------
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

  // Set anchor mode
  uwb.setMode(MODE_ANCHOR);
  delay(50);

  // Network settings
  uwb.setNetworkID("ICSTEST");
  delay(50);

  uwb.setAddress("ANCH01");
  delay(50);

  // Increase timeout
  uwb.setDistanceResponseTimeout(500);

  Serial.println("Anchor ready!");
}

// ---------------- Main Loop ----------------
void loop() {

  RYUW122_MessageInfo info;

  // -------- Ping TAG001 --------
  uwb.sendMessage("TAG001", "PING");

  if (uwb.receiveMessage(info)) {

    float filteredDistance = applyKalman0(info.distance);
    avg_12[index1] = filteredDistance;

  } else {

    Serial.println("No response from TAG001");

  }

  delay(100);


  // -------- Ping TAG002 --------
  uwb.sendMessage("TAG002", "PING");

  if (uwb.receiveMessage(info)) {

    float filteredDistance = applyKalman1(info.distance);
    avg_13[index1] = filteredDistance;

  } else {

    Serial.println("No response from TAG002");

  }

  delay(100);

  // Move index
  index1++;

  // Print averaged values every 5 samples
  if (index1 == array_size) {

    Serial.print("Distance between Vehicle 1 and 2: ");
    Serial.println(average(avg_12));

    Serial.print("Distance between Vehicle 1 and 3: ");
    Serial.println(average(avg_13));

    Serial.println("------------------------------");

    index1 = 0;
  }

  delay(1000);
}
