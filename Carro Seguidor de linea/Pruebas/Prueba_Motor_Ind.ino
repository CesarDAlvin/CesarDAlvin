/**
 * TEAM PAMBACODE
 *
 * Programa de prueba de control de motor (canal A) usando el driver TB6612FNG.
 * Ofrece modos de operación:
 *   - U_MENU: menú principal interactivo
 *   - U_MANUAL: control manual de PWM (>= PWM_MIN)
 *   - U_DISPLAY: visualización en tiempo real de parámetros físicos
 *   - OP_RAMP: rampa progresiva entre dos RPM en un tiempo dado
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
// AIN1/AIN2: control de dirección; PWMA: modulación por ancho de pulso
// STBY: habilita el driver (HIGH = activo)
const int PIN_AIN1 = 5, PIN_AIN2 = 4, PIN_PWMA = 3, PIN_STBY = 6;

// Límites de PWM en escala 0–1023
const int PWM_MAX  = 1023;
const int PWM_MIN  = 200;

// === Parámetros físicos del sistema ===
typedef float f;                // Alias para facilitar cambios de tipo
const f MOTOR_RADIUS    = 0.02; // Radio de rueda en metros
const int MOTOR_MAX_RPM = 2500; // RPM máximas del motor
const f GEAR_RATIO      = 10.0; // Relación de engranajes

// === Estados de interfaz de usuario ===
enum UIState { U_MENU, U_MANUAL, U_DISPLAY };
UIState uiState = U_MENU;       // Estado inicial: menú principal

// === Estados de operación del motor ===
enum OpState { OP_STOP, OP_MANUAL, OP_RAMP };
OpState opState = OP_STOP;      // Estado inicial: motor detenido

// === Variables globales para control y rampa ===
int currentPWM = 0;             // Último PWM aplicado
int r0 = 0, r1 = 0, secs = 0;    // RPM inicio, RPM final y duración de rampa
unsigned long rampPhaseStart = 0; // Timestamp de inicio de fase actual
bool inHoldPhase = false;         // Indica si estamos en fase de retención (hold)

// === Variables para modo display ===
unsigned long lastDisplayMs = 0; // Último timestamp de actualización
f lastVel = 0, totalDist = 0;    // Velocidad y distancia acumulada

// — Matemáticas: funciones de conversión entre PWM, torque, RPM y velocidad —

// Devuelve torque (Nm) proporcional al PWM
f torqueFromPWM(int p) { 
  return (p/(f)PWM_MAX)*0.009807; 
}

// Convierte PWM a RPM de rueda considerando relación de engranaje
f wheelRPM(int p) {    
  return (p/(f)PWM_MAX)*MOTOR_MAX_RPM/GEAR_RATIO; 
}

// Convierte RPM a rad/s
f radPerSec(f rpm) {    
  return rpm*2*PI/60; 
}

// Convierte rad/s a velocidad lineal (m/s)
f linearVel(f o) {      
  return o*MOTOR_RADIUS; 
}

// — Entrada por consola: lee línea hasta '\n' y elimina espacios —  
String readLine() {
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

// — Solicita un entero entre minV y maxV, validándolo —  
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
      Serial.print(minV); Serial.print("–"); Serial.print(maxV);
      Serial.print("], reingresa: ");
    }
  }
  Serial.print("  >> Aceptado: "); Serial.println(v);
  return v;
}

// — Menú principal: muestra opciones disponibles —  
void printMenu() {
  Serial.println(F("\n=== MENÚ PRINCIPAL ==="));
  Serial.println(F("1: PWM manual (>=200)"));
  Serial.println(F("2: Mostrar datos en tiempo real"));
  Serial.println(F("3: Configurar rampa progresiva"));
  Serial.println(F("0: Detener motor"));
  Serial.println(F("m: Mostrar menú"));
  Serial.print(F("Opción: "));
}

// — Aplica PWM al motor, forzando mínimo si 0<p<PWM_MIN —  
void applyPWM(int p) {
  if (p>0 && p<PWM_MIN) p = PWM_MIN;
  currentPWM = p;
  digitalWrite(PIN_AIN1, p>0 ? HIGH : LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, map(p,0,PWM_MAX,0,255));
}

// — Actualiza un paso de la rampa o la fase de retención —  
void updateRamp() {
  unsigned long now = millis();
  if (!inHoldPhase) {
    // Fase de subida: interpolación lineal de RPM
    unsigned long tEnd = rampPhaseStart + (unsigned long)secs * 1000UL;
    if (now < tEnd) {
      float frac = float(now - rampPhaseStart) / float(secs * 1000UL);
      int targetRPM = r0 + (r1 - r0) * frac;
      int pwm = round(targetRPM * (f)PWM_MAX / MOTOR_MAX_RPM);
      applyPWM(pwm);
    } else {
      // Cambiar a fase de retención
      inHoldPhase = true;
      rampPhaseStart = now;
      Serial.print("\n> Manteniendo ");
      Serial.print(r1);
      Serial.println(" rpm por 2s");
    }
  } else {
    // Fase de retención de 2 segundos
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

// — Modo display: muestra parámetros cada 500 ms sin pausar la rampa —  
void doDisplay() {
  // Permitir salir con 'q' o 'm'
  if (Serial.available()) {
    char c = readLine().charAt(0);
    if (c=='q' || c=='m') {
      uiState = U_MENU;
      printMenu();
      return;
    }
  }
  unsigned long now = millis();
  if (now - lastDisplayMs >= 500) {
    float dt = (now - lastDisplayMs) / 1000.0;
    lastDisplayMs = now;

    f rpmv  = wheelRPM(currentPWM);
    f T     = torqueFromPWM(currentPWM);
    f omega = radPerSec(rpmv);
    f vel   = linearVel(omega);
    f acc   = (vel - lastVel) / dt;
    totalDist += vel * dt;
    lastVel = vel;

    Serial.print("PWM=");   Serial.print(currentPWM);
    Serial.print("  RPM=");  Serial.print(rpmv,1);
    Serial.print("  T=");    Serial.print(T,4);  Serial.print("Nm");
    Serial.print("  v=");    Serial.print(vel,4); Serial.print("m/s");
    Serial.print("  a=");    Serial.print(acc,4); Serial.print("m/s2");
    Serial.print("  d=");    Serial.print(totalDist,4); Serial.println("m");
  }
}

void setup() {
  // Inicializar Serial y configurar pines de control
  Serial.begin(9600);
  pinMode(PIN_AIN1,OUTPUT); pinMode(PIN_AIN2,OUTPUT);
  pinMode(PIN_PWMA,OUTPUT); pinMode(PIN_STBY,OUTPUT);
  digitalWrite(PIN_STBY,HIGH); // Sacar de standby
  printMenu();                  // Mostrar menú inicial
}

void loop() {
  // 1) Actualizar rampa si está activa
  if (opState == OP_RAMP) updateRamp();

  // 2) Menú principal: procesar comando si estamos en U_MENU
  if (uiState == U_MENU && Serial.available()) {
    char c = readLine().charAt(0);
    switch(c) {
      case '1':
        uiState = U_MANUAL;
        Serial.print("→ Ingresa PWM (200–1023): ");
        break;
      case '2':
        uiState = U_DISPLAY;
        Serial.println(F("\n--- VISUALIZACIÓN (q o m sale) ---"));
        lastDisplayMs = millis();
        lastVel = 0; totalDist = 0;
        break;
      case '3':
        r0 = readInt("RPM inicio(400–2500): ",400,MOTOR_MAX_RPM);
        r1 = readInt("RPM final (400–2500): ",400,MOTOR_MAX_RPM);
        secs = readInt("Duración (s): ",1,3600);
        rampPhaseStart = millis();
        inHoldPhase = false;
        opState = OP_RAMP;
        Serial.println(F("\n-- Iniciando rampa (x detiene) --"));
        break;
      case '0':
        opState = OP_STOP;
        applyPWM(0);
        Serial.println("→ Motor detenido");
        break;
      case 'm':
        printMenu();
        break;
      default:
        Serial.println("Opción inválida");
        printMenu();
    }
  }

  // 3) Modo manual: leer PWM y volver a menú
  if (uiState == U_MANUAL && Serial.available()) {
    int p = readInt("",PWM_MIN,PWM_MAX);
    opState = OP_MANUAL;
    applyPWM(p);
    uiState = U_MENU;
    printMenu();
  }

  // 4) Modo display: actualizar visualización
  if (uiState == U_DISPLAY) doDisplay();

  // 5) En rampa activa, permitir detener con 'x'
  if (opState == OP_RAMP && Serial.available()) {
    char c = readLine().charAt(0);
    if (c=='x') {
      opState = OP_STOP;
      applyPWM(0);
      Serial.println(F("\n--- Rampa detenida por 'x'. Volviendo al menú ---"));
      printMenu();
    }
  }
}

    }
  }
}

