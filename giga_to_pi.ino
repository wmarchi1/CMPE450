#include <string>

std::string path[5] = {"(0, 0)", "(1, 0)", "(3, 0)", "(3, 3)", "(5, 3)"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
    // if (Serial.available() > 0) { // checks if raspberry pi has sent a message
    Serial.write((uint8_t*)path, sizeof(path));
    delay(7000);

    // }

}
