#include "MPU9250.h"

MPU9250::MPU9250() {
    // Inicializar variables
    gyroCalX = 0; gyroCalY = 0; gyroCalZ = 0;
    magCalX = 0; magCalY = 0; magCalZ = 0;
    magScaleX = 1; magScaleY = 1; magScaleZ = 1;
    
    kalmanAngleRoll = 0; kalmanUncertaintyRoll = 2*2;
    kalmanAnglePitch = 0; kalmanUncertaintyPitch = 2*2;
    
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    
    lastUpdate = micros();
}

bool MPU9250::begin() {
    Wire.begin();
    
    // Reset MPU9250
    writeByte(MPU9250_ADDRESS, 0x6B, 0x80);
    delay(100);
    
    // Wake up MPU9250
    writeByte(MPU9250_ADDRESS, 0x6B, 0x00);
    delay(100);
    
    // Configure gyro and accelerometer
    writeByte(MPU9250_ADDRESS, 0x1A, 0x05); // DLPF = 5 (94Hz for accel, 98Hz for gyro)
    writeByte(MPU9250_ADDRESS, 0x1B, 0x08); // Gyro ±500dps
    writeByte(MPU9250_ADDRESS, 0x1C, 0x10); // Accel ±8g
    
    // Configure magnetometer
    writeByte(MPU9250_ADDRESS, 0x37, 0x02); // Enable bypass mode for magnetometer
    writeByte(AK8963_ADDRESS, 0x0A, 0x16);  // 100Hz continuous measurement
    
    return true;
}

void MPU9250::calibrateGyro(int samples) {
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for(int i = 0; i < samples; i++) {
        readRawData();
        sumX += gyroX;
        sumY += gyroY;
        sumZ += gyroZ;
        delay(1);
    }
    
    gyroCalX = sumX / samples;
    gyroCalY = sumY / samples;
    gyroCalZ = sumZ / samples;
}

void MPU9250::readRawData() {
    // Leer acelerómetro y giroscopio
    uint8_t rawData[14];
    readBytes(MPU9250_ADDRESS, 0x3B, 14, rawData);
    
    accelX = (float)((int16_t)(rawData[0] << 8 | rawData[1])) / 4096.0;
    accelY = (float)((int16_t)(rawData[2] << 8 | rawData[3])) / 4096.0;
    accelZ = (float)((int16_t)(rawData[4] << 8 | rawData[5])) / 4096.0;
    
    temperature = (float)((int16_t)(rawData[6] << 8 | rawData[7])) / 333.87 + 21.0;
    
    gyroX = (float)((int16_t)(rawData[8] << 8 | rawData[9])) / 65.5;
    gyroY = (float)((int16_t)(rawData[10] << 8 | rawData[11])) / 65.5;
    gyroZ = (float)((int16_t)(rawData[12] << 8 | rawData[13])) / 65.5;
    
    // Aplicar calibración
    gyroX -= gyroCalX;
    gyroY -= gyroCalY;
    gyroZ -= gyroCalZ;
    
    // Leer magnetómetro
    uint8_t magData[7];
    readBytes(AK8963_ADDRESS, 0x03, 7, magData);
    
    magX = (float)((int16_t)(magData[1] << 8 | magData[0])) * 0.6;
    magY = (float)((int16_t)(magData[3] << 8 | magData[2])) * 0.6;
    magZ = (float)((int16_t)(magData[5] << 8 | magData[4])) * 0.6;
    
    // Aplicar calibración magnetómetro
    magX = (magX - magCalX) * magScaleX;
    magY = (magY - magCalY) * magScaleY;
    magZ = (magZ - magCalZ) * magScaleZ;
}

void MPU9250::calculateOrientation() {
    // Calcular tiempo transcurrido
    unsigned long now = micros();
    dt = (now - lastUpdate) / 1000000.0;
    lastUpdate = now;
    
    // Calcular ángulos a partir del acelerómetro
    float accelRoll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / M_PI;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
    
    // Aplicar filtro de Kalman
    kalman1D(kalmanAngleRoll, kalmanUncertaintyRoll, gyroX, accelRoll);
    kalman1D(kalmanAnglePitch, kalmanUncertaintyPitch, gyroY, accelPitch);
    
    roll = kalmanAngleRoll;
    pitch = kalmanAnglePitch;
    
    // Calcular yaw usando magnetómetro (fusión sensor)
    float magXhor = magX * cos(pitch * M_PI/180) + magZ * sin(pitch * M_PI/180);
    float magYhor = magX * sin(roll * M_PI/180) * sin(pitch * M_PI/180) + 
                   magY * cos(roll * M_PI/180) - 
                   magZ * sin(roll * M_PI/180) * cos(pitch * M_PI/180);
    
    yaw = atan2(-magYhor, magXhor) * 180.0 / M_PI;
}

void MPU9250::kalman1D(float &angle, float &uncertainty, float rate, float measurement) {
    // Predicción
    angle += dt * rate;
    uncertainty += dt * dt * 4 * 4; // 4°/s de incertidumbre en la tasa
    
    // Actualización
    float gain = uncertainty / (uncertainty + 3 * 3); // 3° de incertidumbre en la medición
    angle += gain * (measurement - angle);
    uncertainty = (1 - gain) * uncertainty;
}

void MPU9250::calculateQuaternion() {
    // Implementar fusión de sensores Madgwick o Mahony aquí
    // (Código simplificado para ejemplo)
    float ax = accelX, ay = accelY, az = accelZ;
    float gx = gyroX * M_PI/180, gy = gyroY * M_PI/180, gz = gyroZ * M_PI/180;
    
    // Normalizar acelerómetro
    float norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm == 0) return;
    ax /= norm; ay /= norm; az /= norm;
    
    // Gradiente descendente
    float vx = 2*(q[1]*q[3] - q[0]*q[2]);
    float vy = 2*(q[0]*q[1] + q[2]*q[3]);
    float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);
    
    // Integral de error
    exInt += ex * Ki * dt;
    eyInt += ey * Ki * dt;
    ezInt += ez * Ki * dt;
    
    // Ajustar tasa gyro
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;
    
    // Integrar cuaternión
    q[0] += (-q[1]*gx - q[2]*gy - q[3]*gz) * dt/2;
    q[1] += (q[0]*gx + q[2]*gz - q[3]*gy) * dt/2;
    q[2] += (q[0]*gy - q[1]*gz + q[3]*gx) * dt/2;
    q[3] += (q[0]*gz + q[1]*gy - q[2]*gx) * dt/2;
    
    // Normalizar cuaternión
    norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}

// Resto de implementaciones de helper functions...