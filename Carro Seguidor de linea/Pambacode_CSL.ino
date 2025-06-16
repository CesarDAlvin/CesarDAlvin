/**
 * TEAM PAMBACODE
 *
 * Seguimiento de línea con sensores QTR, PID, detección de curvas
 * y modelo cinemático. Calibración de sensores 5 s, posición 10 s,
 * PWM base = 400, muestreo 20 Hz.
 *
 * Versión: 2.0.4 – Calibración automática de sensores
 */
#include <Arduino.h>

// ===== PINS =====
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };
const int ledEnable     = 2;
const int LDRPin        = 10;
const int PIN_AIN1      = 5, PIN_AIN2 = 4, PIN_PWMA = 3;
const int PIN_BIN1      = 7, PIN_BIN2 = 8, PIN_PWMB = 9;
const int PIN_STBY      = 6;

// ===== PARÁMETROS =====
// Umbrales de histéresis en escala 0–1000
const int   TH_HIGH    = 820;
const int   TH_LOW     = 799;
// Posiciones laterales de cada sensor (mm)
const float posMM[6]   = { 20, 12, 4, -4, -12, -20 };

// PID (se calibran en tiempo de ejecución)
float Kp, Ki, Kd;

// Cinemática y físico
const int   MOTOR_RPM  = 2500;
const float GEAR       = 10.0;
const float WHEEL_R    = 0.02;
const float AXLE_D     = 0.15;

// PWM y muestreo
const int   PWM_MIN    = 200;
const int   PWM_BASE   = 370;
const int   PWM_MAX    = 1010;
const unsigned int SAMPLE_MS = 50;  // 20 Hz

// ===== ESTADO =====
int    minVal[6], maxVal[6];  // rangos para cada sensor
bool   detected[6]    = { false };
float  lastRawPos     = 0;
float  prevError      = 0;
float  integral       = 0;
float  setpoint       = 0;
unsigned long lastSample = 0;

// Cinemática dinámica
float prev_vL = 0, prev_vR = 0, prev_vCar = 0;
float distL   = 0, distR   = 0;
float vCar    = 0, aCar    = 0;

// Distancias totales y curvas
float straightDist   = 0;
float curveTotalDist = 0;
float totalDist      = 0;
bool  inCurve        = false;
unsigned long curveStart = 0;
float curveDist = 0, curveAng = 0;

enum VehicleState { VS_STRAIGHT, VS_CURVE };
VehicleState vsState = VS_STRAIGHT;

// ===== PROTOTIPOS =====
void calibrateSensors(unsigned long calTimeMs);
void calibrate10s();
void calibratePID();
float readPos();
void applyPWM(int p, int in1, int in2, int pw);
void iteration();

void setup() {
  Serial.begin(9600);
  delay(2000);

  // --- Pines ---
  pinMode(ledEnable, OUTPUT);
  pinMode(LDRPin,   INPUT);
  digitalWrite(ledEnable, LOW);
  const int motorPins[] = {
    PIN_AIN1, PIN_AIN2, PIN_PWMA,
    PIN_BIN1, PIN_BIN2, PIN_PWMB,
    PIN_STBY
  };
  for (size_t i = 0; i < sizeof(motorPins)/sizeof(motorPins[0]); ++i) {
    pinMode(motorPins[i], OUTPUT);
  }
  digitalWrite(PIN_STBY, HIGH);

  // 1) Calibración sensores (5 s)
  Serial.println("Calibrando sensores (5 s), barre sobre blanco y negro...");
  calibrateSensors(5000);

  // 2) Calibrar posición (10 s)
  Serial.println("Calibrando posición (10 s)...");
  calibrate10s();
  Serial.print("Setpoint = "); Serial.print(setpoint,2); Serial.println(" mm");

  // 3) Calibrar PID
  calibratePID();

  // Arrancar bucle
  lastSample = millis();
}

void loop() {
  if (millis() - lastSample < SAMPLE_MS) return;
  lastSample = millis();
  iteration();
}

// ————— FUNCIONES —————

/**
 * @brief Detecta min/max de cada QTR en calTimeMs ms.
 */
void calibrateSensors(unsigned long calTimeMs) {
  // Inicializar rangos
  for (int i = 0; i < 6; i++) {
    minVal[i] = 1023;
    maxVal[i] =    0;
  }
  unsigned long start = millis();
  while (millis() - start < calTimeMs) {
    for (int i = 0; i < 6; i++) {
      int v = analogRead(sensorPins[i]);
      if (v < minVal[i]) minVal[i] = v;
      if (v > maxVal[i]) maxVal[i] = v;
    }
    delay(20);
  }
  Serial.println("Sensores calibrados:");
  for (int i = 0; i < 6; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" min="); Serial.print(minVal[i]);
    Serial.print(" max="); Serial.println(maxVal[i]);
  }
}

/**
 * @brief Promedia readPos() durante 10 s para fijar setpoint.
 */
void calibrate10s() {
  unsigned long start    = millis();
  unsigned long lastTime = start;
  int cnt = 0;
  float sum = 0;
  while (millis() - start < 10000) {
    if (millis() - lastTime < SAMPLE_MS) continue;
    lastTime = millis();
    sum += readPos();
    cnt++;
  }
  setpoint = cnt ? sum / cnt : 0;
}

/**
 * @brief Calcula Kp/Ki/Kd según maxError en 0–5000.
 */
void calibratePID() {
  float errPos = fabs(setpoint -   0.0f);
  float errNeg = fabs(setpoint - 2500.0f);
  float maxError = max(errPos, errNeg);
  Kp = 7.0;
  Ki = Kp * 0.f;
  Kd = Kp * 0.25f;
  Serial.print("PID → Kp="); Serial.print(Kp,4);
  Serial.print(" Ki=");      Serial.print(Ki,6);
  Serial.print(" Kd=");      Serial.println(Kd,6);
}

/**
 * @brief Lee posición normalizando con minVal/maxVal y umbral.
 */
float readPos() {
  digitalWrite(ledEnable, HIGH);
  delayMicroseconds(100);
  float num = 0, den = 0;
  for (int i = 0; i < 6; i++) {
    int raw = analogRead(sensorPins[i]);
    raw = constrain(raw, minVal[i], maxVal[i]);
    int v = map(raw, minVal[i], maxVal[i], 0, 1000);
    detected[i] = v > TH_HIGH ? true
                 : v < TH_LOW  ? false
                               : detected[i];
    num += detected[i] * posMM[i];
    den += detected[i];
  }
  digitalWrite(ledEnable, LOW);
  float p = den ? num/den : lastRawPos;
  lastRawPos = p;
  return p;
}

/**
 * @brief Aplica PWM y dirección al TB6612FNG.
 */
void applyPWM(int p, int in1, int in2, int pw) {
  if (p > 0 && p < PWM_MIN) p = PWM_MIN;
  p = constrain(p, 0, PWM_MAX);
  digitalWrite(in1, p > 0 ? HIGH : LOW);
  digitalWrite(in2, LOW);
  analogWrite(pw, map(p, 0, PWM_MAX, 0, 255));
}

/**
 * @brief Ejecuta un ciclo de PID, motores, cinemática y curvas.
 */
void iteration() {
  unsigned long t0 = millis();

  // --- PID ---
  float pos = readPos();
  float err = setpoint - pos;  // ERROR invertido: positivo → gira izq, negativo → gira der
  float dt  = SAMPLE_MS / 1000.0f;
  float P   = Kp * err;
  float D   = Kd * (err - prevError) / dt;
  float Itmp= integral + err * dt;
  float raw = P + Ki * Itmp + D;
  if (raw >= PWM_MIN - PWM_BASE && raw <= PWM_MAX - PWM_BASE) {
    integral = Itmp;
  }
  float corr = P + Ki * integral + D;
  prevError = err;

  int pwmL = constrain(PWM_BASE + corr, PWM_MIN, PWM_MAX);
  int pwmR = constrain(PWM_BASE - corr, PWM_MIN, PWM_MAX);
  if (fabs(err) <= 3.0f) pwmL = pwmR = PWM_BASE;

  // --- Motores ---
  applyPWM(pwmL, PIN_AIN1, PIN_AIN2, PIN_PWMA);
  applyPWM(pwmR, PIN_BIN1, PIN_BIN2, PIN_PWMB);

  // --- Cinemática ruedas ---
  float RPMmL = (pwmL / 1023.0f) * MOTOR_RPM;
  float RPMrL = RPMmL / GEAR;
  float wL    = RPMrL * 2 * PI / 60.0f;
  float vL    = wL * WHEEL_R;
  float aL    = (vL - prev_vL) / dt; prev_vL = vL; distL += vL * dt;

  float RPMmR = (pwmR / 1023.0f) * MOTOR_RPM;
  float RPMrR = RPMmR / GEAR;
  float wR    = RPMrR * 2 * PI / 60.0f;
  float vR    = wR * WHEEL_R;
  float aR    = (vR - prev_vR) / dt; prev_vR = vR; distR += vR * dt;

  vCar    = (vL + vR) / 2.0f;
  aCar    = (vCar - prev_vCar) / dt; prev_vCar = vCar;

  // --- Curvas vs recta ---
  bool turning = fabs(pwmL - PWM_BASE) > 50 || fabs(pwmR - PWM_BASE) > 50;
  if (turning) {
    if (!inCurve) {
      inCurve    = true;
      curveStart = millis();
      curveDist = curveAng = 0;
    }
    curveDist += vCar * dt;
    curveAng  += ((vR - vL) / AXLE_D) * dt;
    vsState   = VS_CURVE;
  } else {
    if (inCurve) {
      unsigned long dT = millis() - curveStart;
      Serial.println("--- CURVA TERMINADA ---");
      Serial.print("Tiempo (ms): ");   Serial.println(dT);
      Serial.print("Dist curva (m): "); Serial.println(curveDist, 3);
      Serial.print("Ángulo (°): ");     Serial.println(curveAng * 180.0f/PI, 2);
      curveTotalDist += curveDist;
      inCurve = false;
    }
    straightDist += vCar * dt;
    vsState       = VS_STRAIGHT;
  }
  totalDist = straightDist + curveTotalDist;

  // --- Imprimir datos ---
  Serial.print("A→ PWM="); Serial.print(pwmL);
  Serial.print(" RPM=");    Serial.print(RPMmL,1);
  Serial.print(" v=");      Serial.print(vL,4);
  Serial.print(" a=");      Serial.print(aL,4);
  Serial.print(" d=");      Serial.println(distL,4);

  Serial.print("B→ PWM="); Serial.print(pwmR);
  Serial.print(" RPM=");    Serial.print(RPMmR,1);
  Serial.print(" v=");      Serial.print(vR,4);
  Serial.print(" a=");      Serial.print(aR,4);
  Serial.print(" d=");      Serial.println(distR,4);

  Serial.print("Error="); Serial.print(err,2);
  Serial.print(" P=");     Serial.print(P,2);
  Serial.print(" I=");     Serial.print(integral * Ki,2);
  Serial.print(" D=");     Serial.print(D,2);
  Serial.print(" Corr=");  Serial.println(corr,2);

  Serial.print("Vehículo v="); Serial.print(vCar,4);
  Serial.print(" m/s  a=");     Serial.print(aCar,4);
  Serial.println(" m/s²");

  Serial.print("Dist tot=");   Serial.print(totalDist,3);
  Serial.print(" m  Recta=");  Serial.print(straightDist,3);
  Serial.print(" m  Curvas="); Serial.print(curveTotalDist,3);
  Serial.print(" m  Estado=");
  Serial.println(vsState==VS_STRAIGHT?"Recta":"Curva");

  Serial.print("IterTime(ms)=");
  Serial.println(millis() - t0);
  Serial.println();
}
