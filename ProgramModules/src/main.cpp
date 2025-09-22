#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <math.h>

// ---------- Objetos globales ----------
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;   // 0x76/0x77
TinyGPSPlus gps;

// UART1 pins para ESP32 (GPS)
static const int GPS_RX_PIN = 16; // ESP32 RX1  <=  GPS TX
static const int GPS_TX_PIN = 17; // ESP32 TX1  =>  GPS RX
static const uint32_t GPS_BAUD  = 9600;

// I2C pins ESP32
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// ---------- Estado para tiempos ----------
static unsigned long lastPrintMs = 0;     // impresión legible (1 Hz)
static unsigned long lastCsvMs   = 0;     // impresión CSV (60 s)
static const unsigned long PRINT_PERIOD_MS = 1000;
static const unsigned long CSV_PERIOD_MS   = 60000;

// ---------- Últimos datos GPS válidos ----------
double lastLat = NAN, lastLng = NAN;
uint32_t lastSats = 0;
int lastHour = -1, lastMin = -1, lastSec = -1;
bool haveFix = false;

// ---------- Utilidades ----------
static inline float rad2deg(float r){ return r * 180.0f / PI; }

static void printTwoDigits(int v) {
  if (v < 10) Serial.print('0');
  Serial.print(v);
}

// Lee NMEA sin bloquear
static void serviceGPS() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    gps.encode(c);
  }

  // Actualiza caché de últimos valores válidos
  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
  }
  if (gps.satellites.isValid()) {
    lastSats = gps.satellites.value();
  }
  if (gps.time.isValid()) {
    lastHour = gps.time.hour();
    lastMin  = gps.time.minute();
    lastSec  = gps.time.second();
  }
  // Consideramos "fix razonable" con >= 3 satélites y location válida
  haveFix = gps.location.isValid() && gps.satellites.isValid() && (gps.satellites.value() >= 3);
}

void setup() {
  Serial.begin(115200);
  // Espera breve al monitor
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) { /* wait */ }

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // ---------- Inicializar MPU6050 ----------
  if (!mpu.begin()) {
    Serial.println("ERROR: No se encontró MPU6050.");
    while (true) delay(10);
  }
  Serial.println("MPU6050 conectado.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ---------- Inicializar BMP280 ----------
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 no en 0x76, probando 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("ERROR: No se encontró BMP280.");
      while (true) delay(10);
    }
  }
  Serial.println("BMP280 conectado.");
  // Config opcional: oversampling/standby si quieres ajustar consumo/resolución

  // ---------- GPS UART1 ----------
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(200);
  Serial.println("====== Monitor Integrado IMU+BARO+GPS ======");
  Serial.println("Esperando datos válidos de sensores/GPS...\n");

  // ---------- Encabezado CSV ----------
  Serial.println(
    "Timestamp_ms,"
    "AccelX,AccelY,AccelZ,"
    "GyroX,GyroY,GyroZ,"
    "TempMPU,Pitch_deg,Roll_deg,"
    "TempBMP_C,Pressure_hPa,"
    "GPS_Lat,GPS_Lon,GPS_Sats,UTC_HH,UTC_MM,UTC_SS"
  );
}

void loop() {
  // 1) Leer continuamente GPS (no bloquear)
  serviceGPS();

  // 2) Leer IMU
  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);

  // 3) Leer BMP280
  float temp_bmp = bmp.readTemperature();         // °C
  float pressure = bmp.readPressure() / 100.0f;   // hPa

  // 4) Calcular inclinaciones (Pitch/Roll) a partir de acelerómetro
  //    Pitch: rotación sobre eje Y; Roll: rotación sobre eje X (convención simple)
  float pitch = rad2deg(atan2f(-a.acceleration.x,
                    sqrtf(a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z)));
  float roll  = rad2deg(atan2f(a.acceleration.y, a.acceleration.z));

  unsigned long now = millis();

  // ----------- Impresión legible cada 1 s -----------
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    Serial.println("------ LECTURA 1 Hz ------");

    Serial.print("IMU Accel (m/s^2): X=");
    Serial.print(a.acceleration.x, 3);
    Serial.print(" Y=");
    Serial.print(a.acceleration.y, 3);
    Serial.print(" Z=");
    Serial.println(a.acceleration.z, 3);

    Serial.print("IMU Gyro (deg/s): X=");
    Serial.print(g.gyro.x, 3);
    Serial.print(" Y=");
    Serial.print(g.gyro.y, 3);
    Serial.print(" Z=");
    Serial.println(g.gyro.z, 3);

    Serial.print("IMU Temp: ");
    Serial.print(temp_mpu.temperature, 2);
    Serial.println(" °C");

    Serial.print("Inclinación: Pitch=");
    Serial.print(pitch, 2);
    Serial.print("°, Roll=");
    Serial.print(roll, 2);
    Serial.println("°");

    Serial.print("BMP280: Temp=");
    Serial.print(temp_bmp, 2);
    Serial.print(" °C, Presión=");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

    Serial.print("GPS: ");
    if (haveFix) {
      Serial.print("Lat=");
      Serial.print(lastLat, 6);
      Serial.print(" Lon=");
      Serial.print(lastLng, 6);
      Serial.print(" Sats=");
      Serial.print(lastSats);
      Serial.print(" UTC=");
      if (lastHour >= 0) {
        printTwoDigits(lastHour); Serial.print(":");
        printTwoDigits(lastMin);  Serial.print(":");
        printTwoDigits(lastSec);
      } else {
        Serial.print("--:--:--");
      }
    } else {
      Serial.print("Sin fix (buscando...)");
      if (lastSats > 0) {
        Serial.print(" | Sats vistos=");
        Serial.print(lastSats);
      }
    }
    Serial.println();

    lastPrintMs = now;
  }

  // ----------- Línea CSV cada 60 s -----------
  if (now - lastCsvMs >= CSV_PERIOD_MS) {
    Serial.print(now); Serial.print(",");

    Serial.print(a.acceleration.x, 6); Serial.print(",");
    Serial.print(a.acceleration.y, 6); Serial.print(",");
    Serial.print(a.acceleration.z, 6); Serial.print(",");

    Serial.print(g.gyro.x, 6); Serial.print(",");
    Serial.print(g.gyro.y, 6); Serial.print(",");
    Serial.print(g.gyro.z, 6); Serial.print(",");

    Serial.print(temp_mpu.temperature, 2); Serial.print(",");
    Serial.print(pitch, 2); Serial.print(",");
    Serial.print(roll, 2);  Serial.print(",");

    Serial.print(temp_bmp, 2);  Serial.print(",");
    Serial.print(pressure, 2);  Serial.print(",");

    // GPS (si no hay fix, manda NaN)
    if (haveFix) {
      Serial.print(lastLat, 6); Serial.print(",");
      Serial.print(lastLng, 6); Serial.print(",");
      Serial.print(lastSats);    Serial.print(",");
      Serial.print((lastHour >= 0) ? lastHour : -1); Serial.print(",");
      Serial.print((lastMin  >= 0) ? lastMin  : -1); Serial.print(",");
      Serial.print((lastSec  >= 0) ? lastSec  : -1);
    } else {
      Serial.print("nan,nan,");
      Serial.print(lastSats); Serial.print(",");
      Serial.print("-1,-1,-1");
    }
    Serial.println();

    lastCsvMs = now;
  }

  // Pequeño delay para no saturar la CPU (GPS sigue atendido arriba)
}