#include <Arduino.h>

#include <HardwareSerial.h>

// Creamos un puerto UART extra
HardwareSerial LoraSerial(1);

// Definición de pines según tu conexión
#define PIN_M0   2
#define PIN_M1   4
#define PIN_RXD  5   // RX del ESP32 (entra lo que transmite el módulo)
#define PIN_TXD 21   // TX del ESP32 (sale hacia RXD del módulo)
#define PIN_AUX 25

void setup() {
  // Configuración de pines de control
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_AUX, INPUT);

  // Ponemos el módulo en modo NORMAL (M0=0, M1=0)
  digitalWrite(PIN_M0, LOW);
  digitalWrite(PIN_M1, LOW);

  // Serial para depuración
  Serial.begin(115200);
  delay(1000);
  Serial.println("===== Prueba E220 - Receptor =====");

  // Inicializamos UART1 para el módulo
  LoraSerial.begin(9600, SERIAL_8N1, PIN_RXD, PIN_TXD);
}

void loop() {
  // Si hay datos disponibles en el módulo, los leemos
  if (LoraSerial.available()) {
    String mensaje = LoraSerial.readStringUntil('\n');  // hasta salto de línea
    Serial.print("Recibido: ");
    Serial.println(mensaje);
  }
}