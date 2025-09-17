#ifndef MPU9250_H
#define MPU9250_H

#include <Wire.h>
#include <math.h>

#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C

class MPU9250 {
public:
    // Constructor
    MPU9250();
    
    // Inicialización
    bool begin();
    void calibrateGyro(int samples = 2000);
    void calibrateMag(int samples = 1000);
    
    // Lectura de datos brutos
    void readRawData();
    
    // Cálculo de variables derivadas
    void calculateOrientation();
    void calculateQuaternion();
    void calculateLinearAcceleration();
    void calculateWorldAcceleration();
    
    // Filtro de Kalman
    void kalmanUpdate();
    
    // Getter functions
    float getRoll() { return kalmanAngleRoll; }
    float getPitch() { return kalmanAnglePitch; }
    float getYaw() { return yaw; }
    float getQuaternionX() { return q[0]; }
    float getQuaternionY() { return q[1]; }
    float getQuaternionZ() { return q[2]; }
    float getQuaternionW() { return q[3]; }
    float getLinearAccelX() { return linearAccelX; }
    float getLinearAccelY() { return linearAccelY; }
    float getLinearAccelZ() { return linearAccelZ; }
    float getWorldAccelX() { return worldAccelX; }
    float getWorldAccelY() { return worldAccelY; }
    float getWorldAccelZ() { return worldAccelZ; }
    
    // Variables brutas (accesibles para debug)
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float temperature;

private:
    // Kalman filter functions
    void kalman1D(float &angle, float &uncertainty, float rate, float measurement);
    
    // Helper functions
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *data);
    
    // Calibration offsets
    float gyroCalX, gyroCalY, gyroCalZ;
    float magCalX, magCalY, magCalZ;
    float magScaleX, magScaleY, magScaleZ;
    
    // Variables de Kalman
    float kalmanAngleRoll, kalmanUncertaintyRoll;
    float kalmanAnglePitch, kalmanUncertaintyPitch;
    
    // Quaternion para orientación
    float q[4]; // [w, x, y, z]
    
    // Variables derivadas
    float roll, pitch, yaw;
    float linearAccelX, linearAccelY, linearAccelZ;
    float worldAccelX, worldAccelY, worldAccelZ;
    
    // Tiempo para integración
    unsigned long lastUpdate;
    float dt;
};

#endif