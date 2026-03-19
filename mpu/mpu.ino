#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "../elegoo/elegoo.hpp"

Adafruit_MPU6050 mpu;
YawTracker yawTracker;

float calibrateGyroZBias()
{
    const int sampleCount = 200;
    float gyroZSum = 0.0f;

    Serial.println("Keep MPU6050 still for gyro calibration...");

    for (int i = 0; i < sampleCount; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gyroZSum += g.gyro.z;
      delay(5);
    }

    return gyroZSum / sampleCount;
}

void setup(void)
{
    Serial.begin(115200);

    // Try to initialize!
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    Serial.println("MPU6050 Found!");

    // set accelerometer range to +-2G
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

    // set gyro range to +-250 deg/s
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    // set filter bandwidth to 21 Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(100);
    float gyroZBias = calibrateGyroZBias();
    yawTracker.begin(gyroZBias);

    Serial.print("Gyro Z bias: ");
    Serial.println(gyroZBias, 6);
}

void loop()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    yawTracker.update(g.gyro.z);

    Serial.print("Yaw: ");
    Serial.print(yawTracker.getYawDegrees());
    Serial.println(" deg");

    delay(20);
}
