/*
 * Giga R1 NRF24L01 Receiver Code
 */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Create RF24 radio object with CE and CSN pins 9 and 10
RF24 radio(2, 3); // CE, CSN

const byte address[6] = "00001"; // Address for communication, must be the same on both boards
unsigned long latency = 0;
unsigned long old_time = 0;

void setup() {
  SPI1.begin();
  Serial.begin(9600); // Start serial communication
  radio.begin(&SPI1); // Initialize the radio
  radio.openReadingPipe(1, address); // Open a reading pipe with ID 1 to the specific address
  radio.setPALevel(RF24_PA_MIN); // Set the Power Amplifier level
  radio.startListening(); // Set the module as a receiver
}

void loop() {
  if (radio.available()) { // Check if data is available to be received
    uint32_t data; // Create a buffer to store the received data (max 32 bytes)
    radio.read(&data, sizeof(data)); // Read the data into the buffer
    // Serial.println("Data Received");
    
    Serial.print(" Packed (HEX): 0x");
    Serial.println(data, HEX);

    Serial.print(" Packed (BIN): ");
    Serial.println(data, BIN);
    latency = millis() - old_time;
    old_time = millis();
    Serial.print("latency: ");
    Serial.println(latency);
  }
}
