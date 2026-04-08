#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

// ---------------- GPS ----------------
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

// ---------------- IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ---------------- STATE ----------------
double x = 0, y = 0;
double vx = 0, vy = 0;

double ax_bias = 0, ay_bias = 0;

// GPS origin
double lat0 = 0, lon0 = 0;
bool gps_initialized = false;

// timing
unsigned long lastTime = 0;

// tuning
const double GPS_BLEND = 0.6;
const double ACCEL_DEADBAND = 0.05;

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("IMU + GPS Tracking System");

  // ---- GPS INIT (your exact style) ----
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(1000);

  // ---- IMU INIT ----
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("BNO055 error");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // ---- WAIT FOR GPS LOCK ----
  waitForGPSLock();

  // ---- CALIBRATE IMU ----
  calibrateBias();

  lastTime = micros();

  Serial.println("System Ready");
}

// ---------------- WAIT FOR GPS LOCK ----------------
void waitForGPSLock() {
  Serial.println("Waiting for GPS lock...");

  while (true) {
    char c = GPS.read();

    if (GPSECHO && c) Serial.print(c);

    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))
        continue;
    }

    if (GPS.fix) {
      //delay(10000);
      lat0 = GPS.latitudeDegrees;
      lon0 = GPS.longitudeDegrees;
      gps_initialized = true;

      Serial.println("GPS LOCK ACQUIRED");
      Serial.print("Satellites: ");
      Serial.println((int)GPS.satellites);

      Serial.print("Origin set: ");
      Serial.print(lat0, 6); Serial.print(", ");
      Serial.println(lon0, 6);

      delay(2000); // stabilize
      break;
    }
  }
}

// ---------------- CALIBRATE IMU ----------------
void calibrateBias() {
  Serial.println("Calibrating IMU... keep still");

  for (int i = 0; i < 200; i++) {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    ax_bias += accel.x();
    ay_bias += accel.y();
    delay(5);
  }

  ax_bias /= 200;
  ay_bias /= 200;

  Serial.print("Bias: ");
  Serial.print(ax_bias); Serial.print(", ");
  Serial.println(ay_bias);
}

// ---------------- LAT/LON → XY ----------------
void latLonToXY(double lat, double lon, double &x_out, double &y_out) {
  const double R = 6378137.0;

  double dLat = (lat - lat0) * PI / 180.0;
  double dLon = (lon - lon0) * PI / 180.0;

  double lat_avg = (lat + lat0) * 0.5 * PI / 180.0;

  x_out = R * dLon * cos(lat_avg);
  y_out = R * dLat;
}

// ---------------- LOOP ----------------
void loop() {

  // -------- GPS READ (your exact pattern) --------
  char c = GPS.read();
  if (GPSECHO && c) Serial.print(c);

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // -------- TIME --------
  unsigned long now = micros();
  double dt = (now - lastTime) / 1e6;
  lastTime = now;

  if (dt <= 0 || dt > 0.1) return;

  // -------- IMU --------
  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

  double heading = atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy*qy + qz*qz));

  double ax = accel.x() - ax_bias;
  double ay = accel.y() - ay_bias;

  if (fabs(ax) < ACCEL_DEADBAND) ax = 0;
  if (fabs(ay) < ACCEL_DEADBAND) ay = 0;

  // rotate to world
  double ax_w = ax * cos(heading) - ay * sin(heading);
  double ay_w = ax * sin(heading) + ay * cos(heading);

  // integrate
  vx += ax_w * dt;
  vy += ay_w * dt;

  x += vx * dt;
  y += vy * dt;

  if (GPS.fix && gps_initialized) {

  double gps_x, gps_y;
  latLonToXY(GPS.latitudeDegrees, GPS.longitudeDegrees, gps_x, gps_y);

  // ---- smooth GPS ----
  static double gps_x_filt = 0, gps_y_filt = 0;
  gps_x_filt = 0.8 * gps_x_filt + 0.2 * gps_x;
  gps_y_filt = 0.8 * gps_y_filt + 0.2 * gps_y;

  // ---- reject bad jumps ----
  double dx = gps_x_filt - x;
  double dy = gps_y_filt - y;
  double dist = sqrt(dx*dx + dy*dy);

  if (dist > 10.0) return;

  // ---- dynamic weighting ----
  double weight = GPS_BLEND;

  bool stationary = (fabs(vx) < 0.1 && fabs(vy) < 0.1);
  if (stationary) weight *= 2.0;

  // ---- apply correction ----
  x = x * (1.0 - weight) + gps_x_filt * weight;
  y = y * (1.0 - weight) + gps_y_filt * weight;

  vx *= (1.0 - weight);
  vy *= (1.0 - weight);
}

  // -------- PRINT --------
  static uint32_t timer = millis();
  if (millis() - timer > 200) {
    timer = millis();

    /*Serial.println("------");

    Serial.print("Heading (deg): ");
    Serial.println(heading * 180.0 / PI);

    Serial.print("Position (m): ");
    Serial.print(x); Serial.print(", ");
    Serial.println(y);

    Serial.print("Satellites: ");
    Serial.println((int)GPS.satellites);
    */
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
    //Serial.print(",");
    //Serial.println(heading * 180.0 / PI);

  }
}
