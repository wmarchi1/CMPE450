#include <Adafruit_GPS.h>

#define GPSSerial Serial1

Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();
int count = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("GPS Data Reader");

  // Start GPS
  GPS.begin(9600);

  // Output RMC + GGA (recommended)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Update rate 1Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(1000);
}

void loop() {

  // Read GPS characters
  char c = GPS.read();

  // Optional: print raw GPS data
  // Serial.write(c);

  // Check if a new sentence is received
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // Print data every second
  if (millis() - timer > 1000) {
    timer = millis();

    if (GPS.fix) {
      Serial.println("------ GPS DATA ------");

      Serial.print("Latitude: ");
      Serial.println(GPS.latitudeDegrees, 6);

      Serial.print("Longitude: ");
      Serial.println(GPS.longitudeDegrees, 6);

      Serial.print("Speed (knots): ");
      Serial.println(GPS.speed);

      Serial.print("Altitude (m): ");
      Serial.println(GPS.altitude);

      Serial.print("Satellites: ");
      Serial.println((int)GPS.satellites);

      Serial.print("Time: ");
      Serial.print(GPS.hour); Serial.print(":");
      Serial.print(GPS.minute); Serial.print(":");
      Serial.println(GPS.seconds);

      Serial.println();
    }
    else {
      count ++;
      Serial.print("Waiting for GPS fix...");
      Serial.println(count);
      delay(10000);
    }
  }
}
