#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Control print rate (milliseconds)
const unsigned long PRINT_INTERVAL = 100;  // 10 Hz
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\nAdafruit MPU6050 Motion Monitor");
  Serial.println("-----------------------------------");
  Serial.println("Axis Definitions:");
  Serial.println("X Axis = Roll (Left/Right tilt)");
  Serial.println("Y Axis = Pitch (Forward/Back tilt)");
  Serial.println("Z Axis = Vertical (Up/Down)");
  Serial.println();

  // Initialize sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  Serial.println("MPU6050 Found!");

  // Configure sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Sensor configured.\n");
}

void loop() {

  // Control output speed
  if (millis() - lastPrintTime < PRINT_INTERVAL) return;
  lastPrintTime = millis();

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Acceleration (m/s^2)
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Gyroscope (convert rad/s → deg/s)
  float gx = gyro.gyro.x * 180.0 / PI;
  float gy = gyro.gyro.y * 180.0 / PI;
  float gz = gyro.gyro.z * 180.0 / PI;

  Serial.println("----- Motion Data -----");

  Serial.print("Acceleration (m/s^2) | ");
  Serial.print("X: "); Serial.print(ax, 2);
  Serial.print("  Y: "); Serial.print(ay, 2);
  Serial.print("  Z: "); Serial.println(az, 2);

  Serial.print("Rotation (deg/s)     | ");
  Serial.print("X: "); Serial.print(gx, 1);
  Serial.print("  Y: "); Serial.print(gy, 1);
  Serial.print("  Z: "); Serial.println(gz, 1);

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.println();
}
