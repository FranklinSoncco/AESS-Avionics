#ifndef IMU_H #define IMU_H

#include <Adafruit_MPU6050.h> #include <Adafruit_Sensor.h>

class IMU { public: IMU(); bool begin(); void readSensors(float &pitch, float &roll);

private: Adafruit_MPU6050 mpu; };

#endif