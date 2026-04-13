// ============================================================
//  Vehicle Localizer — IMU + GPS + RF24
//  BNO055 (NDOF, tilt-compensated) + Adafruit GPS + nRF24L01
//
//  Features:
//    - Tilt-compensated heading via BNO055 Euler angles
//    - Slope-aware IMU trust scaling
//    - Smooth turn limiting (yaw rate cap + low-pass)
//    - Speed hard-capped at 3 m/s
//    - Velocity decay to prevent IMU runaway
//    - Gentle GPS blending (no position jumps)
//    - Single atomic RF24 packet
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

// ============================================================
//  CONFIGURATION
// ============================================================

// RF24 pins
#define RF_CE_PIN       6
#define RF_CSN_PIN      7

// GPS serial port
#define GPS_SERIAL      Serial1
#define GPS_BAUD        9600

// BNO055
#define BNO_ID          55
#define BNO_ADDR        0x28

// Physics limits
const float MAX_SPEED_MS      = 3.0f;    // hard cap m/s
const float MAX_YAW_RATE_DEG  = 35.0f;  // max turn rate deg/s
const float HEADING_ALPHA     = 0.15f;  // heading low-pass (0=frozen, 1=raw)

// Sensor tuning
const float ACCEL_DEADBAND    = 0.05f;  // m/s^2 — zero small accel noise
const float VEL_DECAY         = 0.994f; // per-cycle velocity drag
const float BIAS_SAMPLES      = 300;

// GPS fusion
const float GPS_BLEND_MOVING  = 0.12f;  // position correction weight while moving
const float GPS_BLEND_STOPPED = 0.45f;  // stronger correction when stationary
const float GPS_GLITCH_LIMIT  = 8.0f;   // m — ignore GPS jumps larger than this
const float GPS_SMOOTH_ALPHA  = 0.15f;  // raw GPS low-pass before fusion
const int   MIN_SATELLITES    = 4;

// Slope thresholds (degrees of combined pitch+roll)
const float SLOPE_MILD        = 5.0f;   // reduce accel trust slightly
const float SLOPE_MODERATE    = 15.0f;  // reduce accel trust significantly
const float SLOPE_STEEP       = 25.0f;  // near-full GPS reliance

// Transmit interval
const uint32_t TX_INTERVAL_MS = 200;

// ============================================================
//  RF24 PACKET
// ============================================================

struct DataPacket {
  float x;           // meters east of origin
  float y;           // meters north of origin
  float heading;     // degrees, 0=north, clockwise
  float speed;       // m/s
  float pitch;       // degrees
  float roll;        // degrees
  uint8_t gps_fix;   // 1 = GPS fix active
  uint8_t sats;      // satellite count
};

// ============================================================
//  GLOBALS
// ============================================================

RF24 radio(RF_CE_PIN, RF_CSN_PIN);
const byte RF_ADDRESS[6] = "00100";

Adafruit_GPS GPS(&GPS_SERIAL);
Adafruit_BNO055 bno(BNO_ID, BNO_ADDR);

// State
double pos_x = 0, pos_y = 0;   // world-frame position (m)
double vel_x = 0, vel_y = 0;   // world-frame velocity (m/s)

// IMU calibration offsets
double ax_bias = 0, ay_bias = 0;

// Heading smoothing
double heading_smooth = 0;
bool   heading_ready  = false;

// GPS origin (set on first fix)
double lat0 = 0, lon0 = 0;
bool   gps_origin_set = false;

// GPS filtered position
double gps_x_filt = 0, gps_y_filt = 0;
bool   gps_filt_ready = false;

unsigned long last_micros = 0;

// ============================================================
//  UTILITY — angle wraparound
// ============================================================

double wrapDeg(double deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg <    0.0) deg += 360.0;
  return deg;
}

double angleDiffDeg(double target, double current) {
  double d = target - current;
  if (d >  180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

// ============================================================
//  HEADING SMOOTHER
//  Rate-limits and low-pass filters heading.
//  Handles 0/360 wraparound correctly.
// ============================================================

double smoothHeading(double raw_deg, double dt) {
  raw_deg = wrapDeg(raw_deg);

  if (!heading_ready) {
    heading_smooth = raw_deg;
    heading_ready  = true;
    return heading_smooth;
  }

  double diff = angleDiffDeg(raw_deg, heading_smooth);

  // Clamp rate of change
  double max_delta = MAX_YAW_RATE_DEG * dt;
  if (diff >  max_delta) diff =  max_delta;
  if (diff < -max_delta) diff = -max_delta;

  // Low-pass toward rate-limited target
  heading_smooth += diff * HEADING_ALPHA;
  heading_smooth  = wrapDeg(heading_smooth);

  return heading_smooth;
}

// ============================================================
//  LAT/LON → LOCAL XY  (equirectangular, accurate <10 km)
// ============================================================

void latLonToXY(double lat, double lon, double &xo, double &yo) {
  const double R = 6378137.0;
  double dLat    = (lat - lat0) * PI / 180.0;
  double dLon    = (lon - lon0) * PI / 180.0;
  double lat_mid = (lat + lat0) * 0.5 * PI / 180.0;
  xo = R * dLon * cos(lat_mid);
  yo = R * dLat;
}

// ============================================================
//  CALIBRATE IMU BIAS  (vehicle must be stationary)
// ============================================================

void calibrateBias() {
  Serial.println("Calibrating IMU bias — keep vehicle still...");
  ax_bias = 0;
  ay_bias = 0;

  for (int i = 0; i < (int)BIAS_SAMPLES; i++) {
    imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    ax_bias += a.x();
    ay_bias += a.y();
    delay(5);
  }

  ax_bias /= BIAS_SAMPLES;
  ay_bias /= BIAS_SAMPLES;

  Serial.print("Bias: ax="); Serial.print(ax_bias, 4);
  Serial.print("  ay="); Serial.println(ay_bias, 4);
}

// ============================================================
//  WAIT FOR GPS LOCK
// ============================================================

void waitForGPSLock() {
  Serial.println("Waiting for GPS lock...");

  while (true) {
    GPS.read();

    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) {
      Serial.print(".");

      if (GPS.fix && (int)GPS.satellites >= MIN_SATELLITES) {
        lat0 = GPS.latitudeDegrees;
        lon0 = GPS.longitudeDegrees;
        gps_origin_set = true;

        Serial.println();
        Serial.print("GPS lock — satellites: ");
        Serial.print((int)GPS.satellites);
        Serial.print("  HDOP: ");
        Serial.println(GPS.HDOP, 2);

        delay(1000);
        return;
      }
    }
  }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // wait for USB serial on Leonardo/32u4
  delay(500);

  Serial.println("\n=== Vehicle Localizer ===");

  // ---- RF24 ----
  /*
  SPI.begin();
  if (!radio.begin()) {
    Serial.println("ERROR: RF24 not found");
    while (1);
  }
  radio.openWritingPipe(RF_ADDRESS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);   // best range/reliability
  radio.setRetries(3, 5);
  radio.stopListening();
  */
  Serial.println("RF24 OK");

  // ---- GPS ----
  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  delay(500);
  Serial.println("GPS OK");

  // ---- BNO055 ----
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("ERROR: BNO055 not found — check wiring and I2C address");
    while (1);
  }
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 OK");

  // ---- Wait for sensor calibration quality ----
  Serial.println("Waiting for BNO055 calibration (move vehicle in figure-8)...");
  while (true) {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("SYS:"); Serial.print(sys);
    Serial.print(" GYR:"); Serial.print(gyro);
    Serial.print(" ACC:"); Serial.print(accel);
    Serial.print(" MAG:"); Serial.println(mag);
    if (sys >= 1 && mag >= 1) break;
    delay(500);
  }
  Serial.println("BNO055 calibrated");

  // ---- GPS lock ----
  waitForGPSLock();

  // ---- IMU bias ----
  calibrateBias();

  last_micros = micros();
  Serial.println("System ready\n");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {

  // ---- GPS READ ----
  GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());

  // ---- DELTA TIME ----
  unsigned long now = micros();
  double dt = (now - last_micros) / 1.0e6;
  last_micros = now;

  // Guard: skip bad dt (startup spike or micros() rollover)
  if (dt <= 0.0 || dt > 0.1) return;

  // ---- READ BNO055 ----
  // Euler angles: tilt-compensated by BNO055 fusion — works on slopes
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  double heading_raw = euler.x();   // 0–360, clockwise from north
  double pitch       = euler.y();   // positive = nose up
  double roll        = euler.z();   // positive = right side down

  // ---- SMOOTH HEADING ----
  double heading_deg = heading_raw;
  //ouble heading_deg = smoothHeading(heading_raw, dt);
  double heading_rad = heading_deg * PI / 180.0;

  // ---- TILT ASSESSMENT ----
  double tilt = sqrt(pitch * pitch + roll * roll);  // combined slope angle

  // Trust scale: reduce IMU accel contribution on slopes
  double accel_trust;
  if      (tilt < SLOPE_MILD)     accel_trust = 1.0;
  else if (tilt < SLOPE_MODERATE) accel_trust = 0.6;
  else if (tilt < SLOPE_STEEP)    accel_trust = 0.2;
  else                             accel_trust = 0.05;

  // ---- PROCESS ACCELERATION ----
  double ax = (accel.x() - ax_bias) * accel_trust;
  double ay = (accel.y() - ay_bias) * accel_trust;

  if (fabs(ax) < ACCEL_DEADBAND) ax = 0;
  if (fabs(ay) < ACCEL_DEADBAND) ay = 0;

  // Rotate body-frame accel into world frame using smoothed heading
  double ax_w = ax * cos(heading_rad) - ay * sin(heading_rad);
  double ay_w = ax * sin(heading_rad) + ay * cos(heading_rad);

  // ---- INTEGRATE VELOCITY ----
  vel_x = vel_x * VEL_DECAY + ax_w * dt;
  vel_y = vel_y * VEL_DECAY + ay_w * dt;

  // Hard speed cap — scale velocity vector, preserve direction
  double speed = sqrt(vel_x * vel_x + vel_y * vel_y);
  if (speed > MAX_SPEED_MS) {
    double scale = MAX_SPEED_MS / speed;
    vel_x *= scale;
    vel_y *= scale;
    speed  = MAX_SPEED_MS;
  }

  // ---- INTEGRATE POSITION ----
  pos_x += vel_x * dt;
  pos_y += vel_y * dt;

  // ---- GPS CORRECTION ----
  if (GPS.fix && gps_origin_set) {

    double gps_x_raw, gps_y_raw;
    latLonToXY(GPS.latitudeDegrees, GPS.longitudeDegrees, gps_x_raw, gps_y_raw);

    // Initialise GPS filter on first valid fix
    if (!gps_filt_ready) {
      gps_x_filt  = gps_x_raw;
      gps_y_filt  = gps_y_raw;
      pos_x       = gps_x_raw;
      pos_y       = gps_y_raw;
      gps_filt_ready = true;
    }

    // Low-pass the raw GPS to suppress single-fix noise
    gps_x_filt = (1.0 - GPS_SMOOTH_ALPHA) * gps_x_filt + GPS_SMOOTH_ALPHA * gps_x_raw;
    gps_y_filt = (1.0 - GPS_SMOOTH_ALPHA) * gps_y_filt + GPS_SMOOTH_ALPHA * gps_y_raw;

    double err_x = gps_x_filt - pos_x;
    double err_y = gps_y_filt - pos_y;
    double err   = sqrt(err_x * err_x + err_y * err_y);

    // Reject implausible GPS jumps (multipath, ionospheric spike)
    if (err < GPS_GLITCH_LIMIT) {

      // Determine blend weight — trust GPS more when stopped or on slopes
      double w = GPS_BLEND_MOVING;
      if (speed < 0.2)          w = GPS_BLEND_STOPPED;
      if (tilt > SLOPE_MODERATE) w = max(w, 0.40);  // slope: lean on GPS
      if (tilt > SLOPE_STEEP)    w = max(w, 0.70);  // steep: near-full GPS

      // Position correction
      pos_x += err_x * w;
      pos_y += err_y * w;

      // Gentle velocity correction — damp drift component, keep direction
      vel_x *= (1.0 - w * 0.25);
      vel_y *= (1.0 - w * 0.25);
    }
  }

  // ---- TRANSMIT ----
  static uint32_t tx_timer = millis();
  if (millis() - tx_timer >= TX_INTERVAL_MS) {
    tx_timer = millis();

    DataPacket pkt;
    pkt.x        = (float)pos_x;
    pkt.y        = (float)pos_y;
    pkt.heading  = (float)heading_deg;
    pkt.speed    = (float)speed;
    pkt.pitch    = (float)pitch;
    pkt.roll     = (float)roll;
    pkt.gps_fix  = GPS.fix ? 1 : 0;
    pkt.sats     = (uint8_t)GPS.satellites;

    bool ok = radio.write(&pkt, sizeof(pkt));

    
    Serial.print(pos_x,      2);
    Serial.print(",");
    Serial.println(pos_y,      2);
    
    
    /*
    // Debug output
    Serial.print("X:");      Serial.print(pos_x,      2);
    Serial.print(" Y:");     Serial.print(pos_y,      2);
    Serial.print(" HDG:");   Serial.print(heading_deg, 1);
    Serial.print(" SPD:");   Serial.print(speed,       2);
    Serial.print(" PITCH:"); Serial.print(pitch,       1);
    Serial.print(" ROLL:");  Serial.print(roll,        1);
    Serial.print(" TILT:");  Serial.print(tilt,        1);
    Serial.print(" SAT:");   Serial.print((int)GPS.satellites);
    Serial.print(GPS.fix ? " FIX" : " NOFIX");
    Serial.println(ok       ? " TX:OK" : " TX:FAIL");
    */
  }
}
