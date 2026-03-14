#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

MPU6050 mpu;

// Filtered angles
float thetaX = 0.0;   // roll  = rotation about X-axis
float thetaY = 0.0;   // pitch = rotation about Y-axis
float thetaZ = 0.0;   // yaw   = rotation about Z-axis

unsigned long lastTime = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("MPU6050 connection successful");

  lastTime = micros();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Convert gyro raw readings to deg/s
  // Assumes default full scale = +/-250 deg/s
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Accelerometer angles in degrees
  float accelThetaX = atan2((float)ay, sqrt((float)ax * ax + (float)az * az)) * 180.0 / PI;
  float accelThetaY = atan2((float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;

  // Complementary filter constant
  float alpha = 0.98;

  // Filtered angles
  thetaX = alpha * (thetaX + gyroX * dt) + (1.0 - alpha) * accelThetaX;
  thetaY = alpha * (thetaY + gyroY * dt) + (1.0 - alpha) * accelThetaY;

  // Yaw from gyro only
  thetaZ = thetaZ + gyroZ * dt;

  Serial.print("ThetaX: ");
  Serial.print(thetaX);

  Serial.print("\tThetaY: ");
  Serial.print(thetaY);

  Serial.print("\tThetaZ: ");
  Serial.println(thetaZ);

  delay(10);
}
