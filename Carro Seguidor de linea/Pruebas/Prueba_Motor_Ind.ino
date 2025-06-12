/**
 * TEAM PAMBACODE
 *
 * Programa de prueba de control de un motor (canal A) usando el driver TB6612FNG.
 * Proporciona:
 *  - Control manual de velocidad vía PWM (con valor mínimo forzado).
 *  - Configuración de rampas progresivas de velocidad (desde RPM inicial hasta final en un tiempo dado).
 *  - Visualización en tiempo real de parámetros (RPM, torque, velocidad, aceleración, distancia).
 *  - Menú interactivo por consola serie para seleccionar modos de operación.
 *
 * Fecha:     2025/05/29
 * Versión:   3.0.0
 *
 * Autores:
 * - César Arturo       / CesarDAlvin
 * - Sara Crystel       / Sara130401
 * - Ceron Dauzon       / Juryelcd
 */

// === PINES TB6612FNG (Motor A) ===
// AIN1/AIN2: dirección, PWMA: señal PWM, STBY: habilita driver
const int PIN_AIN1  = 5;
const int PIN_AIN2  = 4;
const int PIN_PWMA  = 3;
const int PIN_STBY  = 6;

// Límites de PWM en escala de 0–1023
const int PWM_MAX   = 1023;
const int PWM_MIN   = 200;

// === Parámetros físicos del sistema ===
typedef float f;
const f MOTOR_RADIUS    = 0.02f;   // Radio de la rueda en metros
const int MOTOR_MAX_RPM = 2500;    // Revoluciones por minuto máximas del motor
const f GEAR_RATIO      = 10.0f;   // Relación de transmisión

// === Estados de interfaz de usuario ===
enum UIState { U_MENU, U_MANUAL, U_DISPLAY };
UIState uiState = U_MENU;          // Estado inicial: mostrar menú

// === Estados de operación del motor ===
enum OpState { OP_STOP, OP_MANUAL, OP_RAMP };
OpState opState = OP_STOP;         // Estado inicial: motor detenido

// === Variables globales para control y rampa ===
int currentPWM         = 0;        // Último valor PWM aplicado
int r0 = 0, r1 = 0;                // RPM inicial y final para la rampa
int secs = 0;                      // Duración de la rampa en segundos
unsigned long rampPhaseStart = 0;  // Marca de tiempo del inicio de fase actual
bool inHoldPhase = false;          // Indica si se está en fase de retención (hold)

// === Variables para modo display ===
unsigned long lastDisplayMs = 0;   // Último timestamp de actualización de display
f lastVel = 0.0f;                  // Última velocidad lineal registrada
f totalDist = 0.0f;                // Distancia total recorrida

// === Funciones matemáticas de conversión ===

/**
 * Convierte PWM en torque aproximado (Nm).
 */
f torqueFromPWM(int p) {
  return (p / (f)PWM_MAX) * 0.009807f;
}

/**
 * Convierte PWM en RPM de rueda (considerando relación de engranaje).
 */
f wheelRPM(int p) {
  return (p / (f)PWM_MAX) * MOTOR_MAX_RPM / GEAR_RATIO;
}

/**
 * Convierte RPM a radianes por segundo.
 */
f radPerSec(f rpm) {
  return rpm * 2.0f * PI / 60.0f;
}

/**
 * Convierte velocidad angular (rad/s) a velocidad lineal (m/s).
 */
f linearVel(f omega) {
  return omega * MOTOR_RADIUS;
}

// === Funciones de entrada por consola serie ===

/**
 * Lee una línea completa desde Serial hasta '\n' y elimina espacios/adicionales.
 */
String readLine() {
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

/**
 * Solicita y valida un entero entre minV y maxV.
 * @param prompt Texto a mostrar antes de la entrada.
 * @param minV   Límite inferior aceptable.
 * @param maxV   Límite superior aceptable.
 * @return Valor entero validado.
 */
int readInt(const char* prompt, int minV, int maxV) {
  int v = minV - 1;
  Serial.print(prompt);
  while (v < minV || v > maxV) {
    while (!Serial.available());
    String s = readLine();
    if (s.length() == 0) {
      Serial.print("  → vacío, reingresa: ");
      continue;
    }
    v = s.toInt();
    if (v < minV || v > maxV) {
      Serial.print("  → fuera de rango [");
      Serial.print(minV);
      Serial.print("–");
      Serial.print(maxV);
      Serial.print("], reingresa: ");
    }
  }
  Serial.print("  >> Aceptado: ");
  Serial.println(v);
  return v;
}

// === Interfaz de usuario: menú principal ===

/**
 * Muestra el menú de opciones por Serial.
 */
void printMenu() {
  Serial.println(F("\n=== MENÚ PRINCIPAL ==="));
  Serial.println(F("1: PWM manual (>=200)"));
  Serial.println(F("2: Mostrar datos en tiempo real"));
  Serial.println(F("3: Configurar rampa progresiva"));
  Serial.println(F("0: Detener motor"));
  Serial.println(F("m: Mostrar menú"));
  Serial.print(F("Opción: "));
}

// === Control de PWM ===

/**
 * Aplica PWM al motor forzando un mínimo si p>0 && p<PWM_MIN.
 * Actualiza pin AIN1/AIN2 para dirección y PWMA para velocidad.
 */
void applyPWM(int p) {
  if (p > 0 && p < PWM_MIN) {
    p = PWM_MIN;
  }
  currentPWM = p;
  digitalWrite(PIN_AIN1, p > 0 ? HIGH : LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, map(p, 0, PWM_MAX, 0, 255));
}

// === Lógica de rampa progresiva ===

/**
 * Actualiza un paso de la rampa o de la fase de retención de velocidad.
 * Calcula el PWM objetivo en función del tiempo transcurrido.
 */
void updateRamp() {
  unsigned long now = millis();
  if (!inHoldPhase) {
    // Fase de rampa: interpolación lineal de RPM
    unsigned long tEnd = rampPhaseStart + (unsigned long)secs * 1000UL;
    if (now < tEnd) {
      float frac = float(now - rampPhaseStart) / float(secs * 1000UL);
      int targetRPM = r0 + (r1 - r0) * frac;
      int pwm = round(targetRPM * (f)PWM_MAX / MOTOR_MAX_RPM);
      applyPWM(pwm);
    } else {
      // Cambio a fase de retención
      inHoldPhase = true;
      rampPhaseStart = now;
      Serial.print(F("\n> Manteniendo "));
      Serial.print(r1);
      Serial.println(F(" rpm por 2s"));
    }
  } else {
    // Fase de retención: mantener r1 rpm durante 2 segundos
    if (now < rampPhaseStart + 2000UL) {
      int pwm = round(r1 * (f)PWM_MAX / MOTOR_MAX_RPM);
      applyPWM(pwm);
    } else {
      // Reiniciar ciclo de rampa
      inHoldPhase = false;
      rampPhaseStart = now;
    }
  }
}

// === Visualización de datos sin interrumpir la rampa ===

/**
 * Muestra cada 500 ms parámetros calculados: PWM, RPM, torque, velocidad,
 * aceleración y distancia total, hasta recibir 'q' o 'm' para volver al menú.
 */
void doDisplay() {
  // Comprobar salida de display
  if (Serial.available()) {
    char c = readLine().charAt(0);
    if (c == 'q' || c == 'm') {
      uiState = U_MENU;
      printMenu();
      return;
    }
  }

  unsigned long now = millis();
  if (now - lastDisplayMs >= 500) {
    float dt = (now - lastDisplayMs) / 1000.0f;
    lastDisplayMs = now;

    f rpmv  = wheelRPM(currentPWM);
    f T     = torqueFromPWM(currentPWM);
    f omega = radPerSec(rpmv);
    f vel   = linearVel(omega);
    f acc   = (vel - lastVel) / dt;
    totalDist += vel * dt;
    lastVel = vel;

    // Imprimir todos los parámetros formateados
    Serial.print(F("PWM="));   Serial.print(currentPWM);
    Serial.print(F("  RPM="));  Serial.print(rpmv,1);
    Serial.print(F("  T="));    Serial.print(T,4);  Serial.print(F("Nm"));
    Serial.print(F("  v="));    Serial.print(vel,4); Serial.print(F("m/s"));
    Serial.print(F("  a="));    Serial.print(acc,4); Serial.print(F("m/s2"));
    Serial.print(F("  d="));    Serial.print(totalDist,4); Serial.println(F("m"));
  }
}

// === Setup y Loop principales ===

void setup() {
  // Inicializar Serial y configurar pines de control
  Serial.begin(9600);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // Desactivar standby del driver
  digitalWrite(PIN_STBY, HIGH);

  // Mostrar menú inicial
  printMenu();
}

void loop() {
  // 1) Si hay una rampa activa, actualizarla primero
  if (opState == OP_RAMP) {
    updateRamp();
  }

  // 2) Procesar entrada en el menú principal
  if (uiState == U_MENU && Serial.available()) {
    char c = readLine().charAt(0);
    switch (c) {
      case '1':
        uiState = U_MANUAL;
        Serial.print(F("→ Ingresa PWM (200–1023): "));
        break;
      case '2':
        uiState = U_DISPLAY;
        Serial.println(F("\n--- VISUALIZACIÓN (q o m sale) ---"));
        lastDisplayMs = millis();
        lastVel = 0.0f;
        totalDist = 0.0f;
        break;
      case '3':
        r0 = readInt("RPM inicio (400–2500): ", 400, MOTOR_MAX_RPM);
        r1 = readInt("RPM final  (400–2500): ", 400, MOTOR_MAX_RPM);
        secs = readInt("Duración (s): ", 1, 3600);
        rampPhaseStart = millis();
        inHoldPhase = false;
        opState = OP_RAMP;
        Serial.println(F("\n-- Iniciando rampa (x detiene) --"));
        break;
      case '0':
        opState = OP_STOP;
        applyPWM(0);
        Serial.println(F("→ Motor detenido"));
        break;
      case 'm':
        printMenu();
        break;
      default:
        Serial.println(F("Opción inválida"));
        printMenu();
    }
  }

  // 3) Modo manual: ingresar PWM y volver al menú
  if (uiState == U_MANUAL && Serial.available()) {
    int p = readInt("", PWM_MIN, PWM_MAX);
    opState = OP_MANUAL;
    applyPWM(p);
    uiState = U_MENU;
    printMenu();
  }

  // 4) Modo display: mostrar datos
  if (uiState == U_DISPLAY) {
    doDisplay();
  }

  // 5) Si la rampa está activa, permitir detenerla con 'x'
  if (opState == OP_RAMP && Serial.available()) {
    char c = readLine().charAt(0);
    if (c == 'x') {
      opState = OP_STOP;
      applyPWM(0);
      Serial.println(F("\n--- Rampa detenida por 'x'. Volviendo al menú ---"));
      printMenu();
    }
  }
}

