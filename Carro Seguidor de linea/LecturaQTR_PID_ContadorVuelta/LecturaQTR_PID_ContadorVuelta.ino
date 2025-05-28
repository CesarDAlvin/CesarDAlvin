// ====== CONFIGURACIÓN DE PINS ======
// Definición de pines analógicos para los 6 sensores QTR-8A
const int sensorPins[6] = {A0, A1, A2, A3, A4, A5};
// Pin digital que controla la activación de los emisores IR (LEDON)
const int ledEnable    = 2;
// Pin PWM para el motor izquierdo (ajústalo según tu driver)
const int motorIzqPin  = 5;
// Pin PWM para el motor derecho (ajústalo según tu driver)
const int motorDerPin  = 6;

// ====== PARÁMETROS DE HISTERESIS ======
// Umbral alto; si lectura > THRESH_HIGH se considera “negro”
const int THRESH_HIGH = 700;  
// Umbral bajo; si lectura < THRESH_LOW se considera “blanco”
const int THRESH_LOW  = 550;  

// ====== POSICIÓN DE CADA SENSOR (mm desde el centro) ======
// Vector con posición de cada sensor respecto al centro del robot
const float posMM[6] = {-20, -12, -4, +4, +12, +20};

// ====== PARÁMETROS PID ======
// Ganancia proporcional
const float Kp = 0.5;
// Ganancia integral
const float Ki = 0.1;
// Ganancia derivativa
const float Kd = 0.2;

// Velocidad base de los motores (0–255)
const int baseSpeed = 150;

// ====== VARIABLES DE CONTROL ======
// Flag general para parar o reanudar el robot
bool   running    = true;      
// Estados de detección para cada sensor (0=blanco, 1=negro)
bool   detected[6] = {0};      
// Último error válido (mm)
float  lastError  = 0;         
// Error previo (para cálculo derivativo)
float  prevError  = 0;         
// Integral acumulada (para Ki)
float  integral   = 0;         
// Timestamp de la última iteración (ms)
unsigned long lastTime = 0;    

// ====== VARIABLES PARA RECORRIDO DE VUELTAS ======
// Contador total de vueltas detectadas
int vueltas = 0;
// Bandera para indicar que se ha cruzado la línea completa
bool banderaCruce = false;

void setup() {
  // Configura ledEnable como salida
  pinMode(ledEnable,   OUTPUT);
  // Configura motorIzqPin como salida PWM
  pinMode(motorIzqPin, OUTPUT);
  // Configura motorDerPin como salida PWM
  pinMode(motorDerPin, OUTPUT);

  // Inicialmente desactiva los emisores IR
  digitalWrite(ledEnable, LOW);  
  // Inicia comunicación serie a 9600 baudios
  Serial.begin(9600);
  // Pequeña pausa para estabilizar
  delay(2000);
  // Mensaje inicial de control remoto
  Serial.println("Envía 'p' para PARAR, 'r' para REANUDAR.");
  // Establece el instante inicial para temporización
  lastTime = millis();
}

void loop() {
  // 0) Paro / Reanudo por serie
  if (Serial.available()) {
    // Lee el comando entrante
    char c = Serial.read();
    if (c=='p'||c=='P') {
      // Parar la medición
      running = false; 
      Serial.println("=== PARADO ===");
    }
    else if (c=='r'||c=='R') {
      // Reanudar la medición
      running = true;  
      Serial.println("=== REANUDADO ===");
    }
  }
  // Si está parado, espera 100 ms y vuelve al inicio
  if (!running) {
    delay(100);
    return;
  }

  // 1) Mantener ~20 Hz de muestreo
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  // Si no han pasado los 50 ms, salta el ciclo
  if (dt < 0.05) return;
  lastTime = now;

  // 2) Encender emisores IR y esperar estabilización
  digitalWrite(ledEnable, HIGH);
  // Espera microsegundos para estabilizar lectura
  delayMicroseconds(100);

  // 3) Leer sensores con histéresis
  for (int i = 0; i < 6; i++) {
    // Lectura analógica del sensor i
    int L = analogRead(sensorPins[i]);
    if      (L > THRESH_HIGH) detected[i] = 1;  // pasa a negro
    else if (L < THRESH_LOW)  detected[i] = 0;  // pasa a blanco
    // Si L está entre umbrales, mantiene detected[i]
  }

  // 4) Apagar emisores IR
  digitalWrite(ledEnable, LOW);

  // 5) Cálculo de error ponderado
  float num = 0, den = 0;
  for (int i = 0; i < 6; i++) {
    num += detected[i] * posMM[i];
    den += detected[i];
  }
  // Si no detecta línea, conserva el último error
  float error = (den == 0) ? lastError : (num / den);
  lastError = error;

  // 6) Componente derivativa
  float derivative = (error - prevError) / dt;
  prevError = error;

  // 7) Componente integral
  integral += error * dt;

  // 8) PID: calcular corrección total
  float correction = Kp*error + Ki*integral + Kd*derivative;

  // 9) Ajustar velocidades con límites (0–255)
  int speedL = constrain(baseSpeed + correction, 0, 255);
  int speedR = constrain(baseSpeed - correction, 0, 255);
  analogWrite(motorIzqPin, speedL);
  analogWrite(motorDerPin, speedR);

  // 10) Detectar cruce completo para contar vuelta
  bool extremoIzqNegro = detected[0];
  bool extremoDerNegro = detected[5];
  // Si ambos extremos están en negro y aún no habíamos marcado cruce
  if (extremoIzqNegro && extremoDerNegro && !banderaCruce) {
    banderaCruce = true;
  }
  // Si ambos extremos ya no ven negro y la bandera estaba activa
  if (!extremoIzqNegro && !extremoDerNegro && banderaCruce) {
    // Contamos una vuelta y resetamos la bandera
    vueltas++;
    banderaCruce = false;
    Serial.print(" → Vuelta detectada. Total: ");
    Serial.println(vueltas);
  }

  // 11) Debug opcional: imprime error, corrección y velocidades
  Serial.print("Err(mm)="); Serial.print(error,2);
  Serial.print("  Corr=");  Serial.print(correction,2);
  Serial.print("  V=");     Serial.print(speedL);
  Serial.print("/");         Serial.println(speedR);
}
