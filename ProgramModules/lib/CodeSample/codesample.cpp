#include "codesample.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

IMU::IMU() {}

bool IMU::begin() {
  Wire.begin(21, 22); // SDA: GPIO 21, SCL: GPIO 22
  if (!mpu.begin()) {
    Serial.println("No se encontró MPU6050");
    return false;
  }
  Serial.println("MPU6050 conectado");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

void IMU::readSensors(float &pitch, float &roll) {
  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);

  // Calcular ángulos de inclinación (en grados)
  pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
}