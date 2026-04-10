#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <string>

static unsigned long lastPrint = 0;
// Create RF24 radio object with CE and CSN pins 9 and 10
RF24 radio(7, 8); // CE, CSN
bool flag = false;
const byte address[6] = "00100"; // Address for communication, must be the same on both boards
int i = 0;

void setup() {
  Serial.begin(9600); // Start serial communication
  SPI1.begin();

  radio.begin(&SPI1); // Initialize the radio
  radio.openReadingPipe(1, address); // Open a writing pipe to the specific address
  radio.setPALevel(RF24_PA_MIN); // Set the Power Amplifier level (MIN, LOW, HIGH, MAX)
  radio.startListening(); // Set the module as a receiver
}

void loop() {
  if (radio.available()) {
    float test;
    radio.read(&test, sizeof(test)); // Send the data
    if(test == 999999) {
      Serial.println("");
      flag = true;
    }
    else{
    Serial.print(test);
    if(flag == true){
      Serial.print(",");
    }
    flag = false;
    } 
  } else {
    if (millis() - lastPrint > 5000) {
      Serial.println("NO data received");
      lastPrint = millis();
    }
  }
  //delay(500);
}
