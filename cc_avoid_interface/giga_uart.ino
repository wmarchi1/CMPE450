#include <string>

int PATH_LEN = 5;
String path[5] = {"(0, 0, 40)", "(1, 0, 70)", "(1, 3, 20)", "(3, 3, 0)", "(5, 3, 0)"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  if (Serial.readStringUntil('\n') == "Start") {
    // Serial.println("START");

    for (int i = 0; i < PATH_LEN; i++) {
        Serial.println(path[i]);
    }

    Serial.println("END");
  }
}
