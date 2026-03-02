#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Timing
unsigned long lastTime = 0;

// Velo
float vx = 0, vy = 0, vz = 0;

// Position
float dx = 0, dy = 0, dz = 0;
float prev_vx = 0, prev_vy = 0, prev_vz = 0;
float prev_ax = 0, prev_ay = 0, prev_az = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\nMPU6050 ");
  Serial.println("-----------------------------------");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastTime = millis();
}

void loop() {

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // seconds
  lastTime = currentTime;

  // Acceleration in m/s^2
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Integrate acceleration 
  vx = (ax - prev_ax) / 2 * dt;
  vy = (ay - prev_ay) / 2 * dt;
  vz = (az - prev_az) / 2 * dt;

  prev_ax = ax;
  prev_ay = ay;
  prev_az = az;

  // Integrate velocity
  dx += (vx - prev_vx) / 2 * dt;
  dy += (vy - prev_vy) / 2 * dt;
  dz += (vz - prev_vz) / 2 * dt;

  prev_vx = vx;
  prev_vy = vy;
  prev_vz = vz;


  Serial.println("----- Motion Tracking -----");

  Serial.print("Velocity (m/s) | ");
  Serial.print("X: "); Serial.print(vx, 3);
  Serial.print(" Y: "); Serial.print(vy, 3);
  //Serial.print(" Z: "); Serial.println(vz, 3);

  Serial.print("Distance Change (m) | ");
  Serial.print("X: "); Serial.print(dx, 3);
  Serial.print(" Y: "); Serial.print(dy, 3);
  //Serial.print(" Z: "); Serial.println(dz, 3);

  Serial.println();

  delay(50);  // small control delay
}
