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

  // Inicializamos el puerto serie para debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("===== Prueba E220 - Emisor =====");

  // Inicializamos el puerto serie para el módulo Lora
  // Velocidad típica: 9600 (ajústalo según config del módulo)
  LoraSerial.begin(9600, SERIAL_8N1, PIN_RXD, PIN_TXD);
}

void loop() {
  // Enviamos un mensaje de prueba cada 2 segundos
  String mensaje = "Hola desde ESP32!";
  LoraSerial.println(mensaje);
  Serial.println("Enviado: " + mensaje);

  // Esperamos a que AUX esté libre antes de siguiente envío (opcional)
  while (digitalRead(PIN_AUX) == LOW) {
    // ocupado
  }

  delay(2000);
}