/**
 * TEAM PAMBACODE
 *
 * Programa multifunción: seguimiento de línea con sensores QTR y pruebas de motor
 * usando driver TB6612FNG.  
 * Modos:
 *   - Seguidor de línea (calibración por LDR, PID, detección de curvas y cinemática).
 *   - Pruebas de motores A/B (control manual, display de datos, rampas).
 * Selección de modo y configuración vía comandos serie.  
 *
 * Fecha:     2025/06/11
 * Versión:   1.0.0
 *
 * Autores:
 * - César Arturo       / CesarDAlvin
 * - Sara Crystel       / Sara130401
 * - Ceron Dauzon       / Juryelcd
 */

#include <Arduino.h>

// ====== CONFIGURACIÓN DE PINS ======
// Pines analógicos de sensores QTR para línea
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
// Pin que habilita LEDs IR
const int ledEnable     = 2;
// LDR digital para disparar calibración de línea
const int LDRPin        = 10;

// Pines de control del driver TB6612FNG para motores A y B
const int PIN_AIN1 = 5, PIN_AIN2 = 4, PIN_PWMA = 3;
const int PIN_BIN1 = 7, PIN_BIN2 = 8, PIN_PWMB = 9;
const int PIN_STBY = 6;

// ====== PARÁMETROS DE HISTERESIS ======
// Umbrales para filtro de cambio de estado en sensores analógicos
const int THRESH_HIGH = 850;
const int THRESH_LOW  = 700;

// ====== POSICIÓN DE CADA SENSOR (mm) ======
// Coordenadas laterales de cada sensor QTR respecto al centro del robot
const float posMM[6] = { +20, +12, +4, -4, -12, -20 };

// ====== PARÁMETROS PID ======
// Ganancias proporcional, integral y derivativa [PWM/mm], [PWM·s/mm], [PWM·s/mm]
const float Kp = 10.0;
const float Ki = 1.0;
const float Kd = 0.30;

// ====== PARÁMETROS FÍSICOS ======
const float Tmax          = 0.009807;  // Torque motor máximo (Nm)
const int   MOTOR_MAX_RPM = 2500;      // RPM sin carga
const float GEAR_RATIO    = 10.0;      // Relación de reducción
const float MOTOR_RADIUS  = 0.02;      // Radio de rueda (m)
const float L             = 0.15;      // Distancia entre ejes (m)

// ====== LÍMITES PWM ======
const int PWM_MIN  = 200;   // Mínimo PWM efectivo
const int PWM_BASE = 400;   // PWM de crucero
const int PWM_MAX  = 1010;  // Máximo PWM permitido

// ====== MENÚ PRINCIPAL ======
// Modos generales: línea o motor
enum MainMode { MODE_UNSELECTED, MODE_LINE, MODE_MOTOR };
MainMode mainMode = MODE_UNSELECTED;

// ====== ESTADOS DE INTERFAZ ======
enum UIState {
  U_SELECT_MAIN,  // Selección inicial
  // Línea
  U_MENU_LINE,
  // Motores
  U_MENU_MOTOR,
  U_MANUAL,
  U_DISPLAY,
  U_RAMP,
  U_SELECT_MOTOR
};
UIState uiState = U_SELECT_MAIN;

// ====== VARIABLES SEGUIDOR DE LÍNEA ======
bool   calibrating      = true;       // Flag de calibración LDR
bool   runningLF        = false;      // Ejecución de seguimiento
bool   detected[6]      = {0};        // Estados digitales filtrados
float  lastRawPosition  = 0;          // Última posición bruta
float  prevError        = 0;          // Error anterior
float  integral         = 0;          // Integral acumulada
float  setpoint         = 0;          // Posición objetivo
unsigned long lastTime  = 0;          // Timestamp última muestra
bool   inCurve          = false;      // Flag curva
unsigned long curveStart= 0;          // Inicio de curva
float  curveDist        = 0, curveAng = 0; // Distancia y ángulo de curva
float  prev_vL = 0, prev_vR = 0, prev_vCar = 0; // Velocidades previas
float  distL = 0, distR = 0, vCar = 0, aCar = 0; // Cinemática
unsigned int samplingInterval = 1000; // Intervalo muestreo (ms)

// ====== VARIABLES MODO MOTOR ======
enum OpState { OP_STOP, OP_RAMP };
OpState opA = OP_STOP, opB = OP_STOP; // Estado rampa de A y B
enum MotorSel { MS_A, MS_B, MS_BOTH };
MotorSel motorSel = MS_A;             // Motor(s) seleccionados
// Parámetros rampa A
int pwmA = 0, r0A = 0, r1A = 0, secsA = 0;
unsigned long rampStartA = 0;
bool holdPhaseA = false;
unsigned long lastDispA = 0;
float lastVelA = 0, distA_m = 0;
// Parámetros rampa B
int pwmB = 0, r0B = 0, r1B = 0, secsB = 0;
unsigned long rampStartB = 0;
bool holdPhaseB = false;
unsigned long lastDispB = 0;
float lastVelB = 0, distB_m = 0;

// ====== PROTOTIPOS DE FUNCIONES ======
void showSelectMain();
void showMenuLine();
float calibrateSetpoint();
String readLine();
int readInt(const char* prompt, int minV, int maxV);
void printMenuMotor();
void applyPWM_A(int p);
void applyPWM_B(int p);
void updateRampA();
void updateRampB();
void doDisplayA();
void doDisplayB();

void setup() {
  Serial.begin(9600);
  delay(2000);  // Tiempo para iniciar monitor serie

  // Configuración de pines QTR y LDR
  pinMode(ledEnable, OUTPUT);
  pinMode(LDRPin, INPUT);
  digitalWrite(ledEnable, LOW);

  // Configuración de pines del driver de motor
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);  // Sacar de standby

  // Mostrar menú de selección principal
  showSelectMain();
}

void loop() {
  // — Selección de modo principal (línea o motor) —
  if (uiState == U_SELECT_MAIN) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '1') {
        // Iniciar modo seguidor de línea
        mainMode = MODE_LINE;
        uiState   = U_MENU_LINE;
        setpoint = calibrateSetpoint();
        Serial.print("Setpoint inicial: "); Serial.print(setpoint, 2); Serial.println(" mm");
        Serial.println("Esperando señal LDR para iniciar seguimiento...");
        showMenuLine();
        lastTime = millis();
      }
      else if (c == '2') {
        // Iniciar modo pruebas de motor
        mainMode = MODE_MOTOR;
        uiState   = U_MENU_MOTOR;
        printMenuMotor();
      }
    }
    return;
  }

  // — MODO SEGUIDOR DE LÍNEA —
  if (mainMode == MODE_LINE) {
    // *** 1) Calibración vía LDR digital ***
    if (calibrating) {
      int ldrState = digitalRead(LDRPin);
      Serial.print("LDR estado = ");
      Serial.println(ldrState == HIGH ? "HIGH (luz)" : "LOW (oscuro)");
      if (ldrState == HIGH) {
        calibrating = false;
        Serial.println("Señal LDR detectada: iniciando seguimiento");
        Serial.println("GOOOO!");
      } else {
        setpoint = calibrateSetpoint();
        Serial.print("Recalibrando setpoint: "); Serial.print(setpoint, 2); Serial.println(" mm");
      }
      delay(100);
      return;
    }

    // *** 2) Menú de línea por serie ***
    if (Serial.available() && uiState == U_MENU_LINE) {
      char c = Serial.read();
      if      (c == '1') { runningLF = true;  Serial.println("=== LECTURA INICIADA ==="); }
      else if (c == '2') { runningLF = false; Serial.println("=== LECTURA DETENIDA ==="); }
      else if (c == '3') {
        // Cambiar intervalo de muestreo
        Serial.println("Ingrese intervalo muestreo (0–3000 ms) y ENTER, o 'm' para volver:");
        while (1) {
          if (!Serial.available()) continue;
          String line = Serial.readStringUntil('\n');
          line.trim();
          if (line.equalsIgnoreCase("m")) {
            Serial.println("Cancelado.");
            showMenuLine();
            break;
          }
          bool num = line.length() > 0;
          for (char ch : line) if (!isDigit(ch)) { num = false; break; }
          if (num) {
            unsigned long v = line.toInt();
            if (v <= 3000) {
              samplingInterval = v;
              Serial.print("Intervalo actualizado: "); Serial.print(v); Serial.println(" ms");
              break;
            }
          }
          Serial.println("Inválido. 0–3000 ms o 'm':");
        }
      }
      else if (c == 'm' || c == 'M') showMenuLine();
    }
    if (!runningLF) { delay(100); return; }

    // *** 3) Mantener frecuencia de muestreo fija ***
    unsigned long now = millis();
    if (now - lastTime < samplingInterval) return;
    float dt = (now - lastTime) / 1000.0;  // Delta tiempo en s
    lastTime = now;

    // *** 4) Leer sensores QTR con histéresis y calcular posición ***
    digitalWrite(ledEnable, HIGH);
    delayMicroseconds(100);
    float num = 0, den = 0;
    int rawQTR[6];
    for (int i = 0; i < 6; i++) {
      rawQTR[i] = analogRead(sensorPins[i]);
      detected[i] = rawQTR[i] > THRESH_HIGH
                  ? true
                  : rawQTR[i] < THRESH_LOW
                  ? false
                  : detected[i];
      num += detected[i] * posMM[i];
      den += detected[i];
    }
    digitalWrite(ledEnable, LOW);

    // *** 5) Conteo de vueltas completas ***
    bool extI = detected[0], extD = detected[5];
    static bool flagCruce = false;
    if (extI && extD && !flagCruce) flagCruce = true;
    if (!extI && !extD && flagCruce) {
      flagCruce = false;
      Serial.print("→ Vuelta detectada. Total: ");
      Serial.println(++setpoint);
    }

    // *** 6) Cálculo del PID ***
    float position   = den == 0 ? lastRawPosition : (num / den);
    float error      = position - setpoint;
    lastRawPosition  = position;
    float derivative = (error - prevError) / dt;
    float P = Kp * error;
    float D = Kd * derivative;
    float predictedI = integral + error * dt;
    float rawCorr = P + Ki * predictedI + D;
    if (!(rawCorr > PWM_MAX - PWM_BASE || rawCorr < PWM_MIN - PWM_BASE)) {
      integral = predictedI;
    }
    float correction = P + Ki * integral + D;
    prevError = error;

    // *** 7) Cálculo de señales PWM para ruedas ***
    int pwmL = constrain(PWM_BASE + correction, PWM_MIN, PWM_MAX);
    int pwmR = constrain(PWM_BASE - correction, PWM_MIN, PWM_MAX);
    if (fabs(error) <= 2.0) pwmL = pwmR = PWM_BASE;  // Zona muerta

    // *** 8) Detección de curva ***
    bool turning = (abs(pwmL - PWM_BASE) > 50 || abs(pwmR - PWM_BASE) > 50);
    if (turning && !inCurve) {
      inCurve = true;
      curveStart = now;
      curveDist = curveAng = 0;
    }

    // *** 9) Salida a motores A y B ***
    applyPWM_A(pwmL);
    applyPWM_B(pwmR);

    // *** 10) Modelo cinemático: velocidad, aceleración y distancia ***
    float RPMmL = (pwmL / 1023.0) * MOTOR_MAX_RPM;
    float RPMrL = RPMmL / GEAR_RATIO;
    float wL    = RPMrL * 2 * PI / 60.0;
    float vL    = wL * MOTOR_RADIUS;
    float aL    = (vL - prev_vL) / dt; prev_vL = vL; distL += vL * dt;

    float RPMmR = (pwmR / 1023.0) * MOTOR_MAX_RPM;
    float RPMrR = RPMmR / GEAR_RATIO;
    float wR    = RPMrR * 2 * PI / 60.0;
    float vR    = wR * MOTOR_RADIUS;
    float aR    = (vR - prev_vR) / dt; prev_vR = vR; distR += vR * dt;

    vCar    = (vL + vR) / 2; 
    aCar    = (vCar - prev_vCar) / dt; 
    prev_vCar = vCar;

    // *** 11) Acumular datos durante curva ***
    if (inCurve) {
      curveDist += vCar * dt;
      curveAng  += ((vR - vL) / L) * dt;
    }

    // *** 12) Finalización de curva y reporte ***
    if (!turning && inCurve) {
      unsigned long dtC = now - curveStart;
      Serial.println("--- CURVA TERMINADA ---");
      Serial.print("Tiempo (ms): ");   Serial.println(dtC);
      Serial.print("Dist L (m): ");    Serial.println(distL, 3);
      Serial.print("Dist R (m): ");    Serial.println(distR, 3);
      Serial.print("Dist carro (m): ");Serial.println(curveDist, 3);
      Serial.print("Ángulo (°): ");    Serial.println(curveAng * 180.0 / PI, 2);
      inCurve = false;
    }

    // *** 13) Impresión de debug por serie ***
    Serial.print("Raw QTR: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(rawQTR[i]);
      Serial.print(i < 5 ? "," : "\n");
    }
    Serial.print("Err(mm)="); Serial.print(error, 2);
    Serial.print(" P=");      Serial.print(P, 2);
    Serial.print(" I=");      Serial.print(Ki * integral, 2);
    Serial.print(" D=");      Serial.print(D, 2);
    Serial.print(" Corr=");   Serial.print(correction, 2);
    Serial.print(" PWM L/R=");Serial.print(pwmL); Serial.print("/");
    Serial.println(pwmR);
    Serial.println("--------------------------------------------------");
  }

  // — MODO PRUEBAS DE MOTORES —
  if (mainMode == MODE_MOTOR) {
    // Actualiza rampas de A y B
    updateRampA();
    updateRampB();

    // 2) Selección del motor a controlar
    if (uiState == U_SELECT_MOTOR && Serial.available()) {
      String s = readLine();
      if (s.equalsIgnoreCase("m")) {
        uiState = U_MENU_MOTOR;
        printMenuMotor();
        return;
      }
      if      (s.equalsIgnoreCase("A"))       motorSel = MS_A;
      else if (s.equalsIgnoreCase("B"))       motorSel = MS_B;
      else if (s.equalsIgnoreCase("A+B") ||
               s.equalsIgnoreCase("BOTH"))    motorSel = MS_BOTH;
      else {
        Serial.print("Inválido, ingresa A, B, A+B o 'm': ");
        return;
      }
      Serial.print(" >> Ahora: ");
      Serial.println(motorSel == MS_A  ? "A" :
                     motorSel == MS_B  ? "B" : "A+B");
      uiState = U_MENU_MOTOR;
      printMenuMotor();
      return;
    }

    // 3) Menú principal de motores
    if (uiState == U_MENU_MOTOR && Serial.available()) {
      char c = readLine().charAt(0);
      switch (c) {
        case '1':
          uiState = U_MANUAL;
          Serial.print("\n→ MODO MANUAL (m para menú)\n  Ingresa PWM (200–1010): ");
          break;
        case '2':
          uiState = U_DISPLAY;
          Serial.println("\n--- VISUALIZACIÓN (q o m sale) ---");
          lastDispA = lastDispB = millis();
          lastVelA = lastVelB = 0;
          distA_m = distB_m = 0;
          break;
        case '3':
          uiState = U_RAMP;
          Serial.println("\n→ Configurar Rampa (m para menú en cualquier paso)");
          // Configuración de rampa para A y B...
          // (llamada a readInt y setup de rampa)
          Serial.println("\n--- Rampas iniciadas (x detiene) ---");
          break;
        case '4':
          uiState = U_SELECT_MOTOR;
          Serial.print("\nSelecciona motor (A/B/A+B, m para menú): ");
          break;
        case '0':
          opA = opB = OP_STOP;
          applyPWM_A(0); applyPWM_B(0);
          Serial.println("→ Todos los motores detenidos");
          printMenuMotor();
          break;
        case 'm':
          printMenuMotor();
          break;
        default:
          Serial.println("Opción inválida");
          printMenuMotor();
      }
    }

    // 4) MODO MANUAL: lectura de PWM directo
    if (uiState == U_MANUAL && Serial.available()) {
      String s = readLine();
      if (s.equalsIgnoreCase("m")) {
        uiState = U_MENU_MOTOR;
        printMenuMotor();
        return;
      }
      int p = s.toInt();
      if (p < PWM_MIN || p > PWM_MAX) {
        Serial.print("  → rango[200–1010], reingresa o 'm': ");
        return;
      }
      if (motorSel == MS_A || motorSel == MS_BOTH) {
        applyPWM_A(p); opA = OP_STOP;
      }
      if (motorSel == MS_B || motorSel == MS_BOTH) {
        applyPWM_B(p); opB = OP_STOP;
      }
      uiState = U_MENU_MOTOR;
      printMenuMotor();
    }

    // 5) MODO DISPLAY: mostrar datos periódicos de A/B
    if (uiState == U_DISPLAY) {
      if (Serial.available()) {
        char c = readLine().charAt(0);
        if (c == 'q' || c == 'm') {
          uiState = U_MENU_MOTOR;
          printMenuMotor();
          return;
        }
      }
      if (motorSel == MS_A || motorSel == MS_BOTH) doDisplayA();
      if (motorSel == MS_B || motorSel == MS_BOTH) doDisplayB();
    }

    // 6) DETENER RAMPAS con 'x'
    if (Serial.available()) {
      char c = readLine().charAt(0);
      if (c == 'x') {
        opA = opB = OP_STOP;
        applyPWM_A(0); applyPWM_B(0);
        Serial.println("\n--- Rampas detenidas por 'x' ---");
        uiState = U_MENU_MOTOR;
        printMenuMotor();
      }
    }
  }
}

// ====== DEFINICIONES DE FUNCIONES ======

void showSelectMain() {
  // Imprime el menú principal de selección de modo
  Serial.println(F("\n=== MENÚ PRINCIPAL ==="));
  Serial.println(F("1: Seguidor de línea"));
  Serial.println(F("2: Pruebas de motores"));
  Serial.print  (F("Seleccione: "));
}

void showMenuLine() {
  // Imprime el menú de opciones para seguidor de línea
  Serial.println(F("\n===== MENÚ LÍNEA ====="));
  Serial.println(F("1: Iniciar lectura"));
  Serial.println(F("2: Parar lectura"));
  Serial.println(F("3: Cambiar intervalo muestreo"));
  Serial.println(F("m: Mostrar menú"));
  Serial.println(F("======================"));
}

float calibrateSetpoint() {
  // Realiza Ncal lecturas para calibrar la posición objetivo (setpoint)
  const int Ncal = 50;
  float sumPos = 0, lastPos = lastRawPosition;
  for (int k = 0; k < Ncal; k++) {
    digitalWrite(ledEnable, HIGH);
    delayMicroseconds(100);
    float num = 0, den = 0;
    for (int i = 0; i < 6; i++) {
      int Lval = analogRead(sensorPins[i]);
      detected[i] = (Lval > THRESH_HIGH) ? true
                    : (Lval < THRESH_LOW)  ? false
                                           : detected[i];
      num += detected[i] * posMM[i];
      den += detected[i];
    }
    digitalWrite(ledEnable, LOW);
    float pos = (den == 0) ? lastPos : num / den;
    sumPos += pos;
    lastPos = pos;
    lastRawPosition = pos;
    delay(50);
  }
  return sumPos / Ncal;
}

String readLine() {
  // Lee y recorta una línea desde Serial
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

int readInt(const char* prompt, int minV, int maxV) {
  // Lee y valida un entero entre minV y maxV o permite salir con 'm'
  int v = minV - 1;
  Serial.print(prompt);
  while (v < minV || v > maxV) {
    while (!Serial.available());
    String s = readLine();
    if (s.equalsIgnoreCase("m")) {
      uiState = U_MENU_MOTOR;
      printMenuMotor();
      return -1;
    }
    if (s.length() == 0) {
      Serial.print("  → vacío, reingresa o 'm': ");
      continue;
    }
    v = s.toInt();
    if (v < minV || v > maxV) {
      Serial.print("  → fuera de rango [");
      Serial.print(minV); Serial.print("-"); Serial.print(maxV);
      Serial.print("], reingresa o 'm': ");
    }
  }
  Serial.print("  >> Aceptado: ");
  Serial.println(v);
  return v;
}

void printMenuMotor() {
  // Imprime el menú de opciones para pruebas de motor
  Serial.println(F("\n=== MENÚ MOTORES ==="));
  Serial.println(F("1: PWM manual"));
  Serial.println(F("2: Visualizar datos"));
  Serial.println(F("3: Rampa progresiva"));
  Serial.println(F("4: Seleccionar motor (A/B/A+B)"));
  Serial.println(F("0: Detener todos"));
  Serial.println(F("m: Mostrar menú"));
  Serial.print  (F("Opción: "));
}

void applyPWM_A(int p) {
  // Aplica PWM al motor A forzando valor mínimo
  if (p > 0 && p < PWM_MIN) p = PWM_MIN;
  pwmA = p;
  digitalWrite(PIN_AIN1, p > 0 ? HIGH : LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, map(p, 0, PWM_MAX, 0, 255));
}

void applyPWM_B(int p) {
  // Aplica PWM al motor B forzando valor mínimo
  if (p > 0 && p < PWM_MIN) p = PWM_MIN;
  pwmB = p;
  digitalWrite(PIN_BIN1, p > 0 ? HIGH : LOW);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, map(p, 0, PWM_MAX, 0, 255));
}

void updateRampA() {
  // Actualiza el estado de rampa progresiva para motor A
  if (opA != OP_RAMP) return;
  unsigned long now = millis();
  if (!holdPhaseA) {
    if (now < rampStartA + secsA * 1000UL) {
      float frac = float(now - rampStartA) / (secsA * 1000.0);
      int tr = r0A + (r1A - r0A) * frac;
      applyPWM_A(round(tr * (float)PWM_MAX / MOTOR_MAX_RPM));
    } else {
      holdPhaseA = true;
      rampStartA = now;
    }
  } else {
    if (now < rampStartA + 2000UL) {
      applyPWM_A(round(r1A * (float)PWM_MAX / MOTOR_MAX_RPM));
    } else {
      holdPhaseA = false;
      rampStartA = now;
    }
  }
}

void updateRampB() {
  // Actualiza el estado de rampa progresiva para motor B
  if (opB != OP_RAMP) return;
  unsigned long now = millis();
  if (!holdPhaseB) {
    if (now < rampStartB + secsB * 1000UL) {
      float frac = float(now - rampStartB) / (secsB * 1000.0);
      int tr = r0B + (r1B - r0B) * frac;
      applyPWM_B(round(tr * (float)PWM_MAX / MOTOR_MAX_RPM));
    } else {
      holdPhaseB = true;
      rampStartB = now;
    }
  } else {
    if (now < rampStartB + 2000UL) {
      applyPWM_B(round(r1B * (float)PWM_MAX / MOTOR_MAX_RPM));
    } else {
      holdPhaseB = false;
      rampStartB = now;
    }
  }
}

void doDisplayA() {
  // Muestra datos periódicos de motor A: PWM, RPM, v, a, distancia
  unsigned long now = millis();
  if (now - lastDispA < 500) return;
  float dt = (now - lastDispA) / 1000.0;
  lastDispA = now;
  float rpmv = (float)pwmA / PWM_MAX * MOTOR_MAX_RPM / GEAR_RATIO;
  float rad  = rpmv * 2 * PI / 60.0;
  float v    = rad * MOTOR_RADIUS;
  float a    = (v - lastVelA) / dt;
  distA_m   += v * dt;
  lastVelA  = v;
  Serial.print("A→ PWM="); Serial.print(pwmA);
  Serial.print(" RPM=");    Serial.print(rpmv, 1);
  Serial.print(" v=");      Serial.print(v, 4);
  Serial.print(" a=");      Serial.print(a, 4);
  Serial.print(" d=");      Serial.println(distA_m, 4);
}

void doDisplayB() {
  // Muestra datos periódicos de motor B: PWM, RPM, v, a, distancia
  unsigned long now = millis();
  if (now - lastDispB < 500) return;
  float dt = (now - lastDispB) / 1000.0;
  lastDispB = now;
  float rpmv = (float)pwmB / PWM_MAX * MOTOR_MAX_RPM / GEAR_RATIO;
  float rad  = rpmv * 2 * PI / 60.0;
  float v    = rad * MOTOR_RADIUS;
  float a    = (v - lastVelB) / dt;
  distB_m   += v * dt;
  lastVelB  = v;
  Serial.print("B→ PWM="); Serial.print(pwmB);
  Serial.print(" RPM=");    Serial.print(rpmv, 1);
  Serial.print(" v=");      Serial.print(v, 4);
  Serial.print(" a=");      Serial.print(a, 4);
  Serial.print(" d=");      Serial.println(distB_m, 4);
}

