/**
 * TEAM PAMBACODE
 *
 * Este programa recopila lecturas de un arreglo de 6 sensores analógicos (pins A0–A5)
 * controlando emisores (LEDs IR) mediante un pin digital que actúa como disparador
 * de la lectura (ledEnable). Permite pausar y reanudar las mediciones vía consola
 * serie ('p' para parar, 'r' para reanudar), e imprime timestamp y valores de cada
 * sensor en formato tabulado para su análisis (por ejemplo, desde Python).
 *
 * Fecha:     2025/05/28
 * Versión:   3.1.0
 *
 * Autores:
 * - César Arturo       / CesarDAlvin
 * - Sara Crystel       / Sara130401
 * - Ceron Dauzon       / Juryelcd
 */

// ------ Configuración de pines ------
// Pin digital que habilita los emisores (LEDs IR) y actúa como disparador
const uint8_t ledEnable  = 2;
// Número de sensores analógicos a leer (A0–A5)
const uint8_t numSensors = 6;
// Array con los pines analógicos de los sensores
const uint8_t sensorPins[numSensors] = { A0, A1, A2, A3, A4, A5 };

// Estado de medición: true = midiendo, false = detenido
bool running = true;

void setup() {
  // Inicializa comunicación serie a 9600 baudios
  Serial.begin(9600);
  while (!Serial) { ; }  // Espera a que el puerto serie se abra

  // Configura ledEnable como salida y lo apaga al inicio
  pinMode(ledEnable, OUTPUT);
  digitalWrite(ledEnable, LOW);

  // Cabecera con nombre de columnas (sin columna IR)
  Serial.println("Tiempo\tS0\tS1\tS2\tS3\tS4\tS5");
  Serial.println("Envía 'p' para PARAR, 'r' para REANUDAR.");
}

void loop() {
  // --- 0) Leer comandos de paro/reanudar por serie ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'p' || cmd == 'P') {
      running = false;
      Serial.println("=== Mediciones PARADAS ===");
    }
    else if (cmd == 'r' || cmd == 'R') {
      running = true;
      Serial.println("=== Mediciones REANUDADAS ===");
    }
  }

  // Si está detenido, esperamos un poco y volvemos a checar comandos
  if (!running) {
    delay(100);
    return;
  }

  // --- 1) Obtener y formatear tiempo en HH:MM:SS.mmm ---
  unsigned long t        = millis();
  unsigned long ms       = t % 1000;
  unsigned long totalSec = t / 1000;
  unsigned int sec       = totalSec % 60;
  unsigned int min       = (totalSec / 60) % 60;
  unsigned int hr        = totalSec / 3600;

  if (hr  < 10) Serial.print('0');
  Serial.print(hr);
  Serial.print(':');
  if (min < 10) Serial.print('0');
  Serial.print(min);
  Serial.print(':');
  if (sec < 10) Serial.print('0');
  Serial.print(sec);
  Serial.print('.');
  if (ms < 100) Serial.print('0');
  if (ms <  10) Serial.print('0');
  Serial.print(ms);

  // --- 2) Activar emisores para iluminar los sensores ---
  digitalWrite(ledEnable, HIGH);
  delayMicroseconds(100);  // Retardo breve para estabilizar emisores

  // --- 3) Leer sensores analógicos A0–A5 (sin disparador IR) ---
  int readings[numSensors];
  for (uint8_t i = 0; i < numSensors; i++) {
    readings[i] = analogRead(sensorPins[i]);  // Lectura 0–1023
  }

  // --- 4) Imprimir lecturas en consola serie ---
  Serial.print('\t');
  for (uint8_t i = 0; i < numSensors; i++) {
    Serial.print(readings[i]);
    if (i < numSensors - 1) Serial.print('\t');
  }
  Serial.println();

  // --- 5) Apagar emisores y esperar antes de siguiente muestra ---
  digitalWrite(ledEnable, LOW);
  delay(50);  // Aproximadamente 20 lecturas por segundo
}

