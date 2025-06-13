/**
 * TEAM PAMBACODE
 *
 * Programa multifunción: 
 *  - Seguidor de línea con sensores QTR y control PID, detección de curvas y modelo cinemático.
 *  - Pruebas de motores A/B con driver TB6612FNG: control manual, display de datos y rampas.
 *
 * Comunicación y configuración por comandos serie.
 *
 * Fecha:     2025/06/13
 * Versión:   1.1.3 – Código optimizado y mejor documentado
 *
 * Autores:
 *  - César Arturo       / CesarDAlvin
 *  - Sara Crystel       / Sara130401
 *  - Ceron Dauzon       / Juryelcd
 */

#include <Arduino.h>

// ======= PINES =======
static constexpr int SENSOR_PINS[6] = { A0, A1, A2, A3, A4, A5 };
static constexpr int LED_ENABLE     = 2;
static constexpr int LDR_PIN        = 10;

static constexpr int PIN_AIN1 = 5, PIN_AIN2 = 4, PIN_PWMA = 3;
static constexpr int PIN_BIN1 = 7, PIN_BIN2 = 8, PIN_PWMB = 9;
static constexpr int PIN_STBY = 6;

// ======= PARÁMETROS =======
static constexpr int   THRESH_HIGH   = 850;
static constexpr int   THRESH_LOW    = 700;
static constexpr float POS_MM[6]     = { +20, +12, +4, -4, -12, -20 };
static constexpr float K_P           = 10.0f;
static constexpr float K_I           = 1.0f;
static constexpr float K_D           = 0.30f;
static constexpr float MOTOR_TORQUE  = 0.0817f;
static constexpr int   MOTOR_MAX_RPM = 2500;
static constexpr float GEAR_RATIO    = 10.0f;
static constexpr float WHEEL_RADIUS  = 0.02f;
static constexpr float AXLE_DISTANCE = 0.15f;
static constexpr int   PWM_MIN       = 200;
static constexpr int   PWM_BASE      = 400;
static constexpr int   PWM_MAX       = 1010;

// ======= ENUMERACIONES =======
enum class MainMode   { Unselected, LineFollower, MotorTest };
enum class UIState    { SelectMain, MenuLine, MenuMotor, Manual, Display, Ramp, SelectMotor };
enum class VehicleSt  { Straight, Curve };
enum class MotorSel   { A, B, Both };
enum class RampState  { Stop, Ramp };

// ======= VARIABLES GLOBALES =======
// Modo y estado
MainMode  mainMode    = MainMode::Unselected;
UIState   uiState     = UIState::SelectMain;

// Line follower
bool        calibrating       = true;
bool        runningLineFollow = false;
bool        detected[6]       = { false };
float       lastRawPosition   = 0.0f;
float       prevError         = 0.0f;
float       integralSum       = 0.0f;
float       setpoint          = 0.0f;
unsigned long lastSampleTime  = 0;

// Cinemática y curvas
bool        inCurve           = false;
unsigned long curveStartTime  = 0;
float       curveDistance     = 0.0f;
float       curveAngle        = 0.0f;
float       straightDist      = 0.0f;
float       curveTotalDist    = 0.0f;
float       totalDist         = 0.0f;
VehicleSt   vehicleState      = VehicleSt::Straight;

// Cinemática instantánea y previas
float prev_vL    = 0.0f, prev_vR    = 0.0f, prev_vCar = 0.0f;
float distL      = 0.0f, distR      = 0.0f, vCar      = 0.0f, aCar = 0.0f;

// Parámetros de muestreo
unsigned int samplingInterval = 1000;

// Últimos valores PID
float lastErr  = 0.0f;
float lastP    = 0.0f;
float lastI    = 0.0f;
float lastD    = 0.0f;
float lastCorr = 0.0f;

// Pruebas de motor
RampState opA = RampState::Stop, opB = RampState::Stop;
MotorSel  motorSel = MotorSel::A;

// Rampa A
int         pwmA       = 0, r0A = 0, r1A = 0, secsA = 0;
bool        holdPhaseA = false;
unsigned long rampStartA = 0;

// Rampa B
int         pwmB       = 0, r0B = 0, r1B = 0, secsB = 0;
bool        holdPhaseB = false;
unsigned long rampStartB = 0;

// Display motores
unsigned long lastDispA = 0, lastDispB = 0;
float          lastVelA = 0.0f, lastVelB = 0.0f;
float          distA_m  = 0.0f, distB_m  = 0.0f;

// ======= PROTOTIPOS =======
void showSelectMain();
void showMenuLine();
float calibrateSetpoint();
String readLine();
void printMenuMotor();
void applyPWM(int pwm, int pinIn1, int pinIn2, int pinPw);
void updateRamp(RampState &state, unsigned long &startTime,
                bool &hold, int r0, int r1, int secs,
                int pinIn1, int pinIn2, int pinPw);
void doDisplayA();
void doDisplayB();
void displayKinematics();

// ======= SETUP =======
void setup() {
  Serial.begin(9600);
  delay(2000);

  // Sensores QTR
  pinMode(LED_ENABLE, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  digitalWrite(LED_ENABLE, LOW);

  // Driver TB6612FNG: motores A, B y STBY
  const int motorPins[] = {
    PIN_AIN1, PIN_AIN2, PIN_PWMA,
    PIN_BIN1, PIN_BIN2, PIN_PWMB,
    PIN_STBY
  };
  for (size_t i = 0; i < sizeof(motorPins)/sizeof(motorPins[0]); ++i) {
    pinMode(motorPins[i], OUTPUT);
  }

  showSelectMain();
}

// ======= LOOP =======
void loop() {
  // —— Selección de modo principal ——
  if (uiState == UIState::SelectMain) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '1') {
        mainMode = MainMode::LineFollower;
        uiState   = UIState::MenuLine;
        setpoint  = calibrateSetpoint();
        Serial.print("Setpoint inicial: "); Serial.print(setpoint,2); Serial.println(" mm");
        Serial.println("Esperando señal LDR para iniciar seguimiento...");
        showMenuLine();
        lastSampleTime = millis();
      }
      else if (c == '2') {
        mainMode = MainMode::MotorTest;
        uiState   = UIState::MenuMotor;
        printMenuMotor();
      }
    }
    return;
  }

  // ===== SEGUIDOR DE LÍNEA =====
  if (mainMode == MainMode::LineFollower) {
    // 1) Calibración LDR
    if (calibrating) {
      int ldr = digitalRead(LDR_PIN);
      Serial.print("LDR estado = "); Serial.println(ldr==HIGH?"HIGH (luz)":"LOW (oscuro)");
      if (ldr == HIGH) {
        calibrating = false;
        Serial.println("Señal LDR detectada: iniciando seguimiento");
        Serial.println("GOOOO!");
      } else {
        setpoint = calibrateSetpoint();
        Serial.print("Recalibrando setpoint: "); Serial.print(setpoint,2); Serial.println(" mm");
      }
      delay(100);
      return;
    }

    // 2) Menú Línea
    if (Serial.available() && uiState == UIState::MenuLine) {
      char c = Serial.read();
      if (c=='1') {
        runningLineFollow = true; Serial.println("=== LECTURA INICIADA ===");
      }
      else if (c=='2') {
        runningLineFollow = false;
        applyPWM(0, PIN_AIN1, PIN_AIN2, PIN_PWMA);
        applyPWM(0, PIN_BIN1, PIN_BIN2, PIN_PWMB);
        Serial.println("=== LECTURA DETENIDA – MOTORES APAGADOS ===");
      }
      else if (c=='3') {
        Serial.println("Ingrese intervalo muestreo (0–3000 ms) o 'm' para cancelar:");
        while (true) {
          if (!Serial.available()) continue;
          String line = readLine();
          if (line.length() == 0) {
            Serial.println("No ingresaste nada. Ingresa un número o 'm':");
            continue;
          }
          if (line.equalsIgnoreCase("m")) {
            Serial.println("Cambio de intervalo cancelado.");
            break;
          }
          int v = line.toInt();
          if (v>=0 && v<=3000) {
            samplingInterval = v;
            Serial.print("Intervalo actualizado: "); Serial.print(v); Serial.println(" ms");
            break;
          }
          Serial.println("Valor inválido. Debe ser 0–3000 o 'm':");
        }
        showMenuLine();
      }
      else if (c=='m' || c=='M') {
        showMenuLine();
      }
    }
    if (!runningLineFollow) {
      delay(100);
      return;
    }

    // 3) Muestreo fijo
    unsigned long now = millis();
    if (now - lastSampleTime < samplingInterval) return;
    float dt = (now - lastSampleTime)/1000.0f;
    lastSampleTime = now;

    // 4) Lectura QTR
    digitalWrite(LED_ENABLE, HIGH);
    delayMicroseconds(100);
    float num=0, den=0;
    int raw[6];
    for (int i=0;i<6;i++){
      raw[i] = analogRead(SENSOR_PINS[i]);
      detected[i] = raw[i]>THRESH_HIGH ? true : raw[i]<THRESH_LOW ? false : detected[i];
      num += detected[i]*POS_MM[i];
      den += detected[i];
    }
    digitalWrite(LED_ENABLE, LOW);

    // 5) Conteo de vueltas
    bool extL = detected[0], extR = detected[5];
    static bool crossFlag = false;
    if (extL && extR && !crossFlag) crossFlag = true;
    if (!extL && !extR && crossFlag) {
      crossFlag = false;
      Serial.print("→ Vuelta detectada. Total: "); Serial.println(++setpoint);
    }

    // 6) PID
    float position = (den==0? lastRawPosition : num/den);
    float error    = position - setpoint;
    lastRawPosition = position;

    float P = K_P*error;
    float D = K_D*((error-prevError)/dt);
    float I_temp = integralSum + error*dt;
    float rawCorr = P + K_I*I_temp + D;
    if (rawCorr <= PWM_MAX-PWM_BASE && rawCorr >= PWM_MIN-PWM_BASE) {
      integralSum = I_temp;
    }
    float correction = P + K_I*integralSum + D;

    prevError = error;
    lastErr   = error;
    lastP     = P;
    lastI     = K_I*integralSum;
    lastD     = D;
    lastCorr  = correction;

    // 7) PWM ruedas
    int pwmL = constrain(PWM_BASE+correction, PWM_MIN, PWM_MAX);
    int pwmR = constrain(PWM_BASE-correction, PWM_MIN, PWM_MAX);
    if (fabs(error)<=2.0f) pwmL = pwmR = PWM_BASE;

    bool turning = (abs(pwmL-PWM_BASE)>50 || abs(pwmR-PWM_BASE)>50);

    // 8) Aplicar PWM
    applyPWM(pwmL, PIN_AIN1, PIN_AIN2, PIN_PWMA);
    applyPWM(pwmR, PIN_BIN1, PIN_BIN2, PIN_PWMB);

    // 9) Cinemática
    float RPMmL = (pwmL/1023.0f)*MOTOR_MAX_RPM;
    float RPMrL = RPMmL/GEAR_RATIO;
    float wL    = RPMrL*2*PI/60.0f;
    float vL    = wL*WHEEL_RADIUS;
    float aL    = (vL-prev_vL)/dt; prev_vL=vL; distL+=vL*dt;

    float RPMmR = (pwmR/1023.0f)*MOTOR_MAX_RPM;
    float RPMrR = RPMmR/GEAR_RATIO;
    float wR    = RPMrR*2*PI/60.0f;
    float vR    = wR*WHEEL_RADIUS;
    float aR    = (vR-prev_vR)/dt; prev_vR=vR; distR+=vR*dt;

    vCar    = (vL+vR)/2.0f;
    aCar    = (vCar-prev_vCar)/dt; prev_vCar=vCar;

    // 10) Distancias y estado
    if (turning) {
      if (!inCurve) {
        inCurve = true;
        curveStartTime = now;
        curveDistance = curveAngle = 0.0f;
      }
      curveDistance += vCar*dt;
      curveAngle    += ((vR-vL)/AXLE_DISTANCE)*dt;
      vehicleState = VehicleSt::Curve;
    } else {
      if (inCurve) {
        unsigned long dtC = now - curveStartTime;
        Serial.println("--- CURVA TERMINADA ---");
        Serial.print("Tiempo (ms): "); Serial.println(dtC);
        Serial.print("Dist curva (m): "); Serial.println(curveDistance,3);
        Serial.print("Ángulo (°): ");   Serial.println(curveAngle*180.0f/PI,2);
        curveTotalDist += curveDistance;
        inCurve = false;
      }
      straightDist += vCar*dt;
      vehicleState = VehicleSt::Straight;
    }
    totalDist = straightDist + curveTotalDist;

    // 11) Impresión unificada
    Serial.println("------------------------------");
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

    Serial.print("Error=");  Serial.print(lastErr,2);
    Serial.print("  P=");    Serial.print(lastP,2);
    Serial.print("  I=");    Serial.print(lastI,2);
    Serial.print("  D=");    Serial.print(lastD,2);
    Serial.print("  Corr="); Serial.println(lastCorr,2);

    Serial.print("Vehículo v="); Serial.print(vCar,4);
    Serial.print(" m/s  a=");    Serial.print(aCar,4);
    Serial.println(" m/s²");
    Serial.print("Dist tot=");    Serial.print(totalDist,3);
    Serial.print(" m  Recta=");   Serial.print(straightDist,3);
    Serial.print(" m  Curvas=");  Serial.print(curveTotalDist,3);
    Serial.print(" m  Estado=");  Serial.println(vehicleState==VehicleSt::Straight?"Recta":"Curva");
  }

  // ===== PRUEBAS DE MOTORES =====
  if (mainMode == MainMode::MotorTest) {
    // Actualizar rampas
    updateRamp(opA, rampStartA, holdPhaseA, r0A, r1A, secsA, PIN_AIN1, PIN_AIN2, PIN_PWMA);
    updateRamp(opB, rampStartB, holdPhaseB, r0B, r1B, secsB, PIN_BIN1, PIN_BIN2, PIN_PWMB);

    // 1) Selección de motor...
    if (uiState == UIState::SelectMotor && Serial.available()) {
      String s = readLine();
      if (s.equalsIgnoreCase("m")) {
        uiState = UIState::MenuMotor;
        printMenuMotor(); return;
      }
      if      (s.equalsIgnoreCase("A"))   motorSel = MotorSel::A;
      else if (s.equalsIgnoreCase("B"))   motorSel = MotorSel::B;
      else if (s.equalsIgnoreCase("A+B")) motorSel = MotorSel::Both;
      else {
        Serial.print("Inválido, ingresa A, B, A+B o 'm': ");
        return;
      }
      Serial.print(" >> Ahora: ");
      Serial.println(motorSel==MotorSel::A?"A":motorSel==MotorSel::B?"B":"A+B");
      uiState = UIState::MenuMotor;
      printMenuMotor(); return;
    }

    // 2) Menú principal de motores
    if (uiState == UIState::MenuMotor && Serial.available()) {
      char c = readLine().charAt(0);
      switch (c) {
        case '1':
          uiState = UIState::Manual;
          Serial.print("\n→ MODO MANUAL (m para menú)\n  Ingresa PWM (200–1010): ");
          break;
        case '2':
          uiState = UIState::Display;
          Serial.println("\n--- VISUALIZACIÓN (q o m sale) ---");
          lastDispA = lastDispB = millis();
          lastVelA = lastVelB = 0.0f;
          distA_m = distB_m = 0.0f;
          break;
        case '3':
          uiState = UIState::Ramp;
          Serial.println("\n→ Configurar Rampa (m para menú)");
          // Pedir r0A,r1A,secsA...
          Serial.println("\n--- Rampas iniciadas (x detiene) ---");
          break;
        case '4':
          uiState = UIState::SelectMotor;
          Serial.print("\nSelecciona motor (A/B/A+B, m para menú): ");
          break;
        case '0':
          opA = opB = RampState::Stop;
          applyPWM(0, PIN_AIN1, PIN_AIN2, PIN_PWMA);
          applyPWM(0, PIN_BIN1, PIN_BIN2, PIN_PWMB);
          Serial.println("→ Todos los motores detenidos");
          printMenuMotor();
          break;
        case 'm':
          printMenuMotor();
          break;
      }
    }

    // 3) MODO MANUAL
    if (uiState == UIState::Manual && Serial.available()) {
      String s = readLine();
      if (s.equalsIgnoreCase("m")) {
        uiState = UIState::MenuMotor;
        printMenuMotor(); return;
      }
      int p = s.toInt();
      if (p < PWM_MIN || p > PWM_MAX) {
        Serial.print(" → rango[200–1010], reingresa o 'm': ");
        return;
      }
      if (motorSel==MotorSel::A || motorSel==MotorSel::Both) {
        applyPWM(p, PIN_AIN1, PIN_AIN2, PIN_PWMA);
        opA = RampState::Stop;
      }
      if (motorSel==MotorSel::B || motorSel==MotorSel::Both) {
        applyPWM(p, PIN_BIN1, PIN_BIN2, PIN_PWMB);
        opB = RampState::Stop;
      }
      uiState = UIState::MenuMotor;
      printMenuMotor();
    }

    // 4) MODO DISPLAY
    if (uiState == UIState::Display) {
      if (Serial.available()) {
        char c = readLine().charAt(0);
        if (c=='q'||c=='m') {
          applyPWM(0, PIN_AIN1, PIN_AIN2, PIN_PWMA);
          applyPWM(0, PIN_BIN1, PIN_BIN2, PIN_PWMB);
          Serial.println("=== Test finalizado ===");
          uiState = UIState::MenuMotor;
          printMenuMotor(); return;
        }
      }
      if (motorSel==MotorSel::A || motorSel==MotorSel::Both) doDisplayA();
      if (motorSel==MotorSel::B || motorSel==MotorSel::Both) doDisplayB();
      displayKinematics();
      Serial.print("Error=");  Serial.print(lastErr,2);
      Serial.print("  P=");    Serial.print(lastP,2);
      Serial.print("  I=");    Serial.print(lastI,2);
      Serial.print("  D=");    Serial.print(lastD,2);
      Serial.print("  Corr="); Serial.println(lastCorr,2);
      Serial.println("------------------------------");
    }

    // 5) Detener rampas con 'x'
    if (Serial.available()) {
      char c = readLine().charAt(0);
      if (c=='x') {
        opA = opB = RampState::Stop;
        applyPWM(0, PIN_AIN1, PIN_AIN2, PIN_PWMA);
        applyPWM(0, PIN_BIN1, PIN_BIN2, PIN_PWMB);
        Serial.println("\n--- Rampas detenidas por 'x' ---");
        uiState = UIState::MenuMotor;
        printMenuMotor();
      }
    }
  } // fin MotorTest
}

// ======= FUNCIONES AUXILIARES =======

/**
 * @brief Mostrar menú principal
 */
void showSelectMain() {
  Serial.println(F("\n=== MENÚ PRINCIPAL ==="));
  Serial.println(F("1: Seguidor de línea"));
  Serial.println(F("2: Pruebas de motores"));
  Serial.print  (F("Seleccione: "));
}

/**
 * @brief Mostrar menú del seguidor de línea
 */
void showMenuLine() {
  Serial.println(F("\n===== MENÚ LÍNEA ====="));
  Serial.println(F("1: Iniciar lectura"));
  Serial.println(F("2: Parar lectura"));
  Serial.println(F("3: Cambiar intervalo muestreo"));
  Serial.println(F("m: Mostrar menú"));
  Serial.println(F("======================"));
}

/**
 * @brief Calibración inicial de setpoint mediante LDR
 * @return Posición promedio (mm)
 */
float calibrateSetpoint() {
  const int Ncal = 50;
  float sumPos = 0, lastPos = lastRawPosition;
  for (int i = 0; i < Ncal; i++) {
    digitalWrite(LED_ENABLE, HIGH);
    delayMicroseconds(100);
    float num=0, den=0;
    for (int j=0; j<6; j++){
      int val = analogRead(SENSOR_PINS[j]);
      detected[j] = val>THRESH_HIGH ? true
                     : val<THRESH_LOW ? false
                                     : detected[j];
      num += detected[j]*POS_MM[j];
      den += detected[j];
    }
    digitalWrite(LED_ENABLE, LOW);
    float pos = (den==0)? lastPos : num/den;
    sumPos += pos;
    lastPos = pos;
    lastRawPosition = pos;
    delay(50);
  }
  return sumPos / Ncal;
}

/**
 * @brief Leer línea completa desde Serial
 */
String readLine() {
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

/**
 * @brief Mostrar menú de pruebas de motores
 */
void printMenuMotor() {
  Serial.println(F("\n=== MENÚ MOTORES ==="));
  Serial.println(F("1: PWM manual"));
  Serial.println(F("2: Visualizar datos"));
  Serial.println(F("3: Rampa progresiva"));
  Serial.println(F("4: Seleccionar motor (A/B/A+B)"));
  Serial.println(F("0: Detener todos"));
  Serial.println(F("m: Mostrar menú"));
  Serial.print  (F("Opción: "));
}

/**
 * @brief Aplicar PWM a un motor con TB6612FNG
 */
void applyPWM(int pwm, int pinIn1, int pinIn2, int pinPw) {
  if (pwm>0 && pwm<PWM_MIN) pwm = PWM_MIN;
  analogWrite(pinPw, map(pwm,0,PWM_MAX,0,255));
  digitalWrite(pinIn1, pwm>0?HIGH:LOW);
  digitalWrite(pinIn2, LOW);
}

/**
 * @brief Actualizar rampa progresiva de un motor
 */
void updateRamp(RampState &state, unsigned long &startTime,
                bool &hold, int r0, int r1, int secs,
                int pinIn1, int pinIn2, int pinPw)
{
  if (state != RampState::Ramp) return;
  unsigned long now = millis();
  if (!hold) {
    if (now < startTime + secs*1000UL) {
      float frac = (now - startTime) / float(secs*1000UL);
      int tr = r0 + (r1 - r0)*frac;
      applyPWM(round(tr*(float)PWM_MAX/MOTOR_MAX_RPM), pinIn1, pinIn2, pinPw);
    } else {
      hold = true;
      startTime = now;
    }
  } else {
    if (now < startTime + 2000UL) {
      applyPWM(round(r1*(float)PWM_MAX/MOTOR_MAX_RPM), pinIn1, pinIn2, pinPw);
    } else {
      hold = false;
      startTime = now;
    }
  }
}

/**
 * @brief Mostrar datos de motor A (formato fijo)
 */
void doDisplayA() {
  unsigned long now = millis();
  if (now - lastDispA < 1000) return;
  float dt = (now - lastDispA) / 1000.0f;
  lastDispA = now;

  float rpmv = float(pwmA)/PWM_MAX * MOTOR_MAX_RPM / GEAR_RATIO;
  float rad  = rpmv * 2*PI / 60.0f;
  float v    = rad * WHEEL_RADIUS;
  float a    = (v - lastVelA)/dt;
  distA_m   += v*dt;
  lastVelA   = v;

  Serial.print("A→ PWM="); Serial.print(pwmA);
  Serial.print(" RPM=");     Serial.print(rpmv,1);
  Serial.print(" v=");       Serial.print(v,4);
  Serial.print(" a=");       Serial.print(a,4);
  Serial.print(" d=");       Serial.println(distA_m,4);
}

/**
 * @brief Mostrar datos de motor B (formato fijo)
 */
void doDisplayB() {
  unsigned long now = millis();
  if (now - lastDispB < 1000) return;
  float dt = (now - lastDispB) / 1000.0f;
  lastDispB = now;

  float rpmv = float(pwmB)/PWM_MAX * MOTOR_MAX_RPM / GEAR_RATIO;
  float rad  = rpmv * 2*PI / 60.0f;
  float v    = rad * WHEEL_RADIUS;
  float a    = (v - lastVelB)/dt;
  distB_m   += v*dt;
  lastVelB   = v;

  Serial.print("B→ PWM="); Serial.print(pwmB);
  Serial.print(" RPM=");     Serial.print(rpmv,1);
  Serial.print(" v=");       Serial.print(v,4);
  Serial.print(" a=");       Serial.print(a,4);
  Serial.print(" d=");       Serial.println(distB_m,4);
}

/**
 * @brief Mostrar estado cinemático del vehículo
 */
void displayKinematics() {
  Serial.print("Vehículo v="); Serial.print(vCar,4);
  Serial.print(" m/s  a=");    Serial.print(aCar,4);
  Serial.println(" m/s²");

  Serial.print("Dist total (m): ");   Serial.println(totalDist,3);
  Serial.print("Dist recta (m): ");   Serial.println(straightDist,3);
  Serial.print("Dist curvas (m): ");  Serial.println(curveTotalDist,3);

  Serial.print("Estado: ");
  Serial.println(vehicleState==VehicleSt::Straight ? "Linea recta" : "Curva");
}
