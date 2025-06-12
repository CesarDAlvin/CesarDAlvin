/**
 * TEAM PAMBACODE
 *
 * Programa de registro de lecturas de un sensor LDR con control de emisores LED.
 * Permite pausar y reanudar las mediciones vía comandos serie ('p' para parar,
 * 'r' para reanudar'), e imprime cada muestra con timestamp en formato
 * HH:MM:SS.mmm seguido del valor ADC de luminosidad.
 *
 * Fecha:     2025/06/11
 * Versión:   1.0.0
 *
 * Autores:
 * - César Arturo       / CesarDAlvin
 * - Sara Crystel       / Sara130401
 * - Ceron Dauzon       / Juryelcd
 */

// Pin analógico donde está conectado el LDR (mediante divisor de tensión)
const int sensorPin   = A0;

// Pin digital que habilita los emisores (LEDs IR u otro)
const int ledEnable   = 2;

// Estado de medición: true = midiendo, false = detenido
bool running = true;

void setup() {
  // Configura el pin de emisores como salida y lo apaga inicialmente
  pinMode(ledEnable, OUTPUT);
  digitalWrite(ledEnable, LOW);

  // Inicia comunicación serie a 9600 baudios
  Serial.begin(9600);

  // Breve retardo para dar tiempo a que un programa Python u otro cliente
  // abra el puerto serie antes de empezar a enviar datos
  delay(2000);

  // Mensaje inicial de instrucciones para el usuario
  Serial.println("Envía 'p' para PARAR, 'r' para REANUDAR.");
}

void loop() {
  // --- 0) Comprobar si hay comando serie para pausar/reanudar ---
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

  // Si está detenido, esperamos un breve intervalo y volvemos a checar comandos
  if (!running) {
    delay(100);
    return;
  }

  // --- 1) Obtener y formatear tiempo transcurrido desde inicio ---
  unsigned long t        = millis();         // Tiempo en ms desde arranque
  unsigned long ms       = t % 1000;         // Milisegundos restantes
  unsigned long totalSec = t / 1000;         // Total de segundos
  unsigned int sec       = totalSec % 60;    // Segundos (0–59)
  unsigned int min       = (totalSec / 60) % 60; // Minutos (0–59)
  unsigned int hr        = totalSec / 3600;  // Horas

  // Imprime timestamp en formato HH:MM:SS.mmm
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
  Serial.print('\t');  // Tabulador separador antes del valor ADC

  // --- 2) Activar emisores para iluminar el LDR antes de la medición ---
  digitalWrite(ledEnable, HIGH);
  delayMicroseconds(100);  // Retardo breve para estabilizar la iluminación

  // --- 3) Leer valor ADC del LDR y enviarlo ---
  int valorADC = analogRead(sensorPin);
  Serial.println(valorADC);

  // --- 4) Apagar emisores tras la lectura ---
  digitalWrite(ledEnable, LOW);

  // Espera aproximada para lograr ~20 lecturas por segundo
  delay(50);
}
