/**
 * TEAM PAMBACODE
 *
 * Programa de seguimiento de línea y control cinemático de un carro motorizado.
 * Integra:
 *   - Lectura de un arreglo de sensores QTR para detección de línea.
 *   - Filtro de histéresis para estabilizar lecturas digitales.
 *   - Control PID para mantener el carro sobre la línea (setpoint de posición).
 *   - Modelo cinemático: cálculo de torque, RPM, velocidad lineal y aceleración.
 *   - Parametrización de curvas: detección de giro, acumulación de distancia y ángulo.
 *   - Simulación de envío de datos a los motores mediante analogWrite (comentado).
 *
 * Fecha:     2025/06/01
 * Versión:   3.0.0
 *
 * Autores:
 * - César Arturo       / CesarDAlvin
 * - Sara Crystel       / Sara130401
 * - Ceron Dauzon       / Juryelcd
 */

// ====== CONFIGURACIÓN DE PINS ======
const int sensorPins[6] = { A0, A1, A2, A3, A4, A5 };  // Pines analógicos de sensores QTR
const int ledEnable     = 2;                           // Pin digital para habilitar IR LEDs

// ====== PARÁMETROS DE HISTERESIS ======
// Umbrales alto/bajo para estabilizar cambios de estado de los sensores
const int THRESH_HIGH = 700;
const int THRESH_LOW  = 550;

// ====== POSICIÓN DE CADA SENSOR (mm) ======
// Coordenadas laterales de cada sensor respecto al centro del carro
const float posMM[6] = { -20, -12, -4, +4, +12, +20 };

// ====== PARÁMETROS PID ======
// Kp: ganancia proporcional (PWM/mm), Ki: integral (PWM·s/mm), Kd: derivativo (PWM·s/mm)
const float Kp =  5.0;    // Reacción proporcional al error de posición
const float Ki =  2.0;    // Acumulación de error a lo largo del tiempo (para eliminar offset)
const float Kd =  1.0;    // Amortiguación de cambios bruscos en el error

// ====== PARÁMETROS FÍSICOS ======
// Torque máximo del motor (N·m), RPM sin carga, relación de engranajes, radio de rueda y distancia entre ejes
const float Tmax      = 0.009807;   // Torque motor máximo (N·m)
const float motorRPM0 = 3000.0;     // RPM sin carga
const float gearRatio = 10.0;       // Reducción mecánica (motor→rueda)
const float wheelR    = 0.02;       // Radio de la rueda (m)
const float L         = 0.20;       // Distancia entre ruedas (m)

// ====== LÍMITES PWM ======
const int PWM_MIN   = 200;          // PWM mínimo para vencer fricción y arrancar
const int PWM_BASE  = 400;          // PWM de crucero (sin corrección)
const int PWM_MAX   = 1010;         // PWM máximo permitido

// ====== VARIABLES DE CONTROL ======
bool   running         = false;     // Flag de ejecución principal
bool   detected[6]     = {0};       // Estados filtrados de cada sensor
float  lastRawPosition = 0;         // Última posición calculada de línea (mm)
float  prevError       = 0;         // Error de la iteración previa (para derivativo)
float  integral        = 0;         // Integral acumulada del error
unsigned long lastTime = 0;         // Timestamp de la última muestra

// ====== SETPOINT ======
float setpoint = 0;                 // Posición objetivo (calibrada al inicio)

// ====== CURVAS ======
bool   inCurve      = false;        // Indica si el carro está girando
unsigned long curveStart = 0;       // Timestamp de inicio de curva
float  curveDist    = 0;            // Distancia recorrida durante la curva (m)
float  curveAng     = 0;            // Ángulo total girado en la curva (rad)

// ====== CINEMÁTICA ======
// Variables para integrar velocidad y distancia de cada rueda y del carro
float  prev_vL   = 0, prev_vR   = 0;
float  prev_vCar = 0;
float  distL     = 0, distR     = 0;
float  vCar      = 0, aCar      = 0;

// ====== VELOCIDAD DE MUESTREO ======
unsigned int samplingInterval = 50; // Intervalo entre lecturas (ms)

// Prototipos
void showMenu();

void setup() {
  // Configurar pines y comunicación
  pinMode(ledEnable, OUTPUT);
  digitalWrite(ledEnable, LOW);
  Serial.begin(9600);
  delay(2000);  // Esperar a que el PC abra el puerto serie

  // --- Calibración de setpoint ---
  Serial.println("Calibrando setpoint...");
  const int Ncal = 50;
  float sumPos = 0;
  for (int k = 0; k < Ncal; k++) {
    // Activar LEDs y leer sensores
    digitalWrite(ledEnable, HIGH);
    delayMicroseconds(100);
    float num = 0, den = 0;
    for (int i = 0; i < 6; i++) {
      int Lval = analogRead(sensorPins[i]);
      // Filtro de histéresis para evitar parpadeo
      if      (Lval > THRESH_HIGH) detected[i] = true;
      else if (Lval < THRESH_LOW ) detected[i] = false;
      num += detected[i] * posMM[i];
      den += detected[i];
    }
    digitalWrite(ledEnable, LOW);
    // Calcular posición media o usar última si no hay detección
    float pos = (den == 0) ? lastRawPosition : (num / den);
    sumPos += pos;
    lastRawPosition = pos;
    delay(50);
  }
  setpoint = sumPos / Ncal;  // Establecer setpoint como media de calibración
  Serial.print("Setpoint calibrado: ");
  Serial.print(setpoint, 2);
  Serial.println(" mm");
  Serial.println("Presiona '1' iniciar, '2' detener, '3' cambiar muestreo, 'm' menú.");
  showMenu();
  lastTime = millis();
}

void loop() {
  // --- Manejo de menú serie ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      running = true; Serial.println("=== LECTURA INICIADA ===");
    }
    if (c == '2') {
      running = false; Serial.println("=== LECTURA DETENIDA ===");
    }
    if (c == '3') {
      Serial.println("Nuevo intervalo de muestreo (ms):");
      while (!Serial.available());
      unsigned int v = Serial.parseInt();
      if (v >= 1) samplingInterval = v;
      Serial.print("Intervalo = ");
      Serial.print(samplingInterval);
      Serial.println(" ms");
    }
    if (c=='m'||c=='M') showMenu();
  }
  if (!running) {
    delay(100);
    return;
  }

  // --- Control de tiempo para muestreo constante ---
  unsigned long now = millis();
  if (now - lastTime < samplingInterval) return;
  float dt = (now - lastTime) / 1000.0;  // dt en segundos
  lastTime = now;

  // --- Lectura y filtrado de sensores QTR ---
  digitalWrite(ledEnable, HIGH);
  delayMicroseconds(100);
  float num = 0, den = 0;
  for (int i = 0; i < 6; i++) {
    int Lval = analogRead(sensorPins[i]);
    if      (Lval > THRESH_HIGH) detected[i] = true;
    else if (Lval < THRESH_LOW ) detected[i] = false;
    num += detected[i] * posMM[i];
    den += detected[i];
  }
  digitalWrite(ledEnable, LOW);

  // --- Conteo de vueltas completas (opcional) ---
  bool extI = detected[0], extD = detected[5];
  static bool flagCruce = false;
  if (extI && extD && !flagCruce) flagCruce = true;
  if (!extI && !extD && flagCruce) {
    flagCruce = false;
    Serial.print("→ Vuelta detectada. Total: ");
    Serial.println(++setpoint);
  }

  // --- Cálculo de error para PID ---
  float position   = (den == 0) ? lastRawPosition : (num / den);
  float error      = position - setpoint;    // error>0 indica desvío lateral
  lastRawPosition  = position;
  float derivative = (error - prevError) / dt;

  // Anti-windup: prever integral antes de acumular
  float predictedI = integral + error * dt;
  float P = Kp * error;
  float D = Kd * derivative;
  float rawCorr = P + Ki * predictedI + D;
  if (rawCorr > PWM_MAX - PWM_BASE || rawCorr < PWM_MIN - PWM_BASE) {
    // No acumular integral si la corrección saturaría PWM
  } else {
    integral = predictedI;
  }
  float correction = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  // --- Aplicación de corrección: calcular PWM para cada rueda ---
  int pwmL = constrain(PWM_BASE + correction, PWM_MIN, PWM_MAX);
  int pwmR = constrain(PWM_BASE - correction, PWM_MIN, PWM_MAX);

  // --- Detección de inicio de curva (desvío significativo) ---
  bool turning = (abs(pwmL - PWM_BASE) > 1 || abs(pwmR - PWM_BASE) > 1);
  if (turning && !inCurve) {
    inCurve    = true;
    curveStart = now;
    curveDist  = 0;
    curveAng   = 0;
  }

  // --- (Simulación) Envío de señal PWM a motores ---
//analogWrite(PIN_PWMA, pwmR);  // Motor A = derecho
//analogWrite(PIN_PWMB, pwmL);  // Motor B = izquierdo

  // --- Modelo cinemático: torque, RPM, velocidad y aceleración ---
  float TmL = (pwmL / 1023.0) * Tmax * gearRatio;    // Torque en rueda izquierda
  float TmR = (pwmR / 1023.0) * Tmax * gearRatio;    // Torque en rueda derecha
  float RPMmL = (pwmL / 1023.0) * motorRPM0;         // RPM motor izquierda
  float RPMmR = (pwmR / 1023.0) * motorRPM0;         // RPM motor derecha
  float RPMrL = RPMmL / gearRatio;                   // RPM rueda izquierda
  float RPMrR = RPMmR / gearRatio;                   // RPM rueda derecha
  float wL = RPMrL * 2.0 * PI / 60.0;                // Vel angular rueda L (rad/s)
  float wR = RPMrR * 2.0 * PI / 60.0;                // Vel angular rueda R (rad/s)
  float vL = wL * wheelR;                            // Vel lineal L (m/s)
  float vR = wR * wheelR;                            // Vel lineal R (m/s)
  float aL = (vL - prev_vL) / dt;                    // Aceleración L (m/s²)
  float aR = (vR - prev_vR) / dt;                    // Aceleración R (m/s²)
  prev_vL = vL;
  prev_vR = vR;
  vCar    = (vL + vR) / 2.0;                         // Velocidad media del carro
  aCar    = (vCar - prev_vCar) / dt;                 // Aceleración del carro
  prev_vCar = vCar;

  // --- Acumulación de curvas: distancia y ángulo girado ---
  if (inCurve) {
    curveDist += vCar * dt;                          // Integral de distancia recorrida
    curveAng  += ((vR - vL) / L) * dt;               // Integral de velocidad angular diferencial
  }

  // --- Finalización de curva: imprimir estadísticas ---
  if (!turning && inCurve) {
    unsigned long dtCurve = now - curveStart;
    Serial.println("--- CURVA TERMINADA ---");
    Serial.print("Tiempo (ms): ");      Serial.println(dtCurve);
    Serial.print("Distancia (m): ");    Serial.println(curveDist, 3);
    Serial.print("Ángulo (°): ");       Serial.println(curveAng * 180.0 / PI, 2);
    inCurve = false;
  }

  // --- Salida de datos por Serial (debug y registro) ---
  Serial.print("Err(mm)=");     Serial.print(error, 2);
  Serial.print("  P=");         Serial.print(P, 2);
  Serial.print("  I=");         Serial.print(Ki * integral, 2);
  Serial.print("  D=");         Serial.print(D, 2);
  Serial.print("  Corr(PWM)="); Serial.print(correction, 2);
  Serial.print("  PWM L/R=");   Serial.print(pwmL); Serial.print("/"); Serial.print(pwmR);
  Serial.print(turning ? " CURVA\n" : " LINEA RECTA\n");
  Serial.print("Tm L/R(Nm)=");  Serial.print(TmL, 3); Serial.print("/"); Serial.print(TmR, 3);
  Serial.print("  v L/R(m/s)=");Serial.print(vL, 3);  Serial.print("/"); Serial.print(vR, 3);
  Serial.print("  a L/R(m/s2)=");Serial.print(aL, 3);  Serial.print("/"); Serial.print(aR, 3);
  Serial.print("  vCar(m/s)=");   Serial.print(vCar, 3);
  Serial.print("  aCar(m/s2)=");  Serial.println(aCar, 3);
  Serial.println("--------------------------------------------------");
}

void showMenu() {
  // Imprime el menú de opciones para control vía serie
  Serial.println("===== MENÚ =====");
  Serial.println("1: Iniciar lectura");
  Serial.println("2: Parar lectura");
  Serial.println("3: Cambiar intervalo de muestreo");
  Serial.println("m: Mostrar menú");
  Serial.println("================");
}
