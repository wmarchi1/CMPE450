#include <Adafruit_GPS.h>

#define GPSSerial Serial1

Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("GPS Parsing Test");

  GPS.begin(9600);

  // Output RMC + GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(1000);
}

void loop()
{
  char c = GPS.read();

  if (GPSECHO)
    if (c) Serial.print(c);

  // if a sentence is received
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // print data every second
  static uint32_t timer = millis();
  if (millis() - timer > 1000) {
    timer = millis();

    Serial.print("Fix: ");
    Serial.println((int)GPS.fix);

    if (GPS.fix) {
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

      Serial.println();
    }
  }
}
