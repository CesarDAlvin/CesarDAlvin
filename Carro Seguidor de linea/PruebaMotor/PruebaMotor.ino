// === PINES TB6612FNG (Motor A) ===
const int PIN_AIN1 = 5, PIN_AIN2 = 4, PIN_PWMA = 3, PIN_STBY = 6;
const int PWM_MAX  = 1023;
const int PWM_MIN  = 200;

// Parámetros físicos
typedef float f;
const f MOTOR_RADIUS    = 0.02;   // m
const int MOTOR_MAX_RPM = 3000;
const f GEAR_RATIO      = 10.0;

// Estados de interfaz
enum UIState { U_MENU, U_MANUAL, U_DISPLAY };
UIState uiState = U_MENU;

// Estados de operación
enum OpState { OP_STOP, OP_MANUAL, OP_RAMP };
OpState opState = OP_STOP;

// Variables globales
int currentPWM = 0;
int r0=0, r1=0, secs=0;
unsigned long rampPhaseStart = 0;
bool inHoldPhase = false;

// Para display
unsigned long lastDisplayMs = 0;
f lastVel = 0, totalDist = 0;

// — Matemáticas —
f torqueFromPWM(int p) { return (p/(f)PWM_MAX)*0.009807; }
f wheelRPM(int p)     { return (p/(f)PWM_MAX)*MOTOR_MAX_RPM/GEAR_RATIO; }
f radPerSec(f rpm)    { return rpm*2*PI/60; }
f linearVel(f o)      { return o*MOTOR_RADIUS; }

// — Leo línea y trimmeo —
String readLine() {
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

// — Leo int entre min…max —
int readInt(const char* prompt, int minV, int maxV) {
  int v = minV - 1;
  Serial.print(prompt);
  while (v < minV || v > maxV) {
    while (!Serial.available());
    String s = readLine();
    if (s.length() == 0) { Serial.print("  → vacío, reingresa: "); continue; }
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

// — Menú de interfaz —
void printMenu() {
  Serial.println(F("\n=== MENÚ PRINCIPAL ==="));
  Serial.println(F("1: PWM manual (>=200)"));
  Serial.println(F("2: Mostrar datos en tiempo real"));
  Serial.println(F("3: Configurar rampa progresiva"));
  Serial.println(F("0: Detener motor"));
  Serial.println(F("m: Mostrar menú"));
  Serial.print(F("Opción: "));
}

// — Aplico PWM forzando mínimo —
void applyPWM(int p) {
  if (p>0 && p<PWM_MIN) p = PWM_MIN;
  currentPWM = p;
  digitalWrite(PIN_AIN1, p>0 ? HIGH : LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, map(p,0,PWM_MAX,0,255));
}

// — Actualiza un paso de rampa o retención —
void updateRamp() {
  unsigned long now = millis();
  if (!inHoldPhase) {
    unsigned long tEnd = rampPhaseStart + (unsigned long)secs * 1000UL;
    if (now < tEnd) {
      float frac = float(now - rampPhaseStart) / float(secs * 1000UL);
      int targetRPM = r0 + (r1 - r0) * frac;
      int pwm = round(targetRPM * (f)PWM_MAX / MOTOR_MAX_RPM);
      applyPWM(pwm);
    } else {
      inHoldPhase = true;
      rampPhaseStart = now;
      Serial.print("\n> Manteniendo ");
      Serial.print(r1);
      Serial.println(" rpm por 2s");
    }
  } else {
    if (now < rampPhaseStart + 2000UL) {
      int pwm = round(r1 * (f)PWM_MAX / MOTOR_MAX_RPM);
      applyPWM(pwm);
    } else {
      inHoldPhase = false;
      rampPhaseStart = now;
    }
  }
}

// — Visualización sin pausar la rampa —
void doDisplay() {
  // Salir con 'q' o 'm'
  if (Serial.available()) {
    char c = readLine().charAt(0);
    if (c=='q') uiState = U_MENU;
    else if (c=='m') uiState = U_MENU;
    if (uiState==U_MENU) { printMenu(); return; }
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

void setup(){
  Serial.begin(9600);
  pinMode(PIN_AIN1,OUTPUT); pinMode(PIN_AIN2,OUTPUT);
  pinMode(PIN_PWMA,OUTPUT); pinMode(PIN_STBY,OUTPUT);
  digitalWrite(PIN_STBY,HIGH);
  printMenu();
}

void loop(){
  // Primero, siempre actualizo la rampa si está activa
  if (opState == OP_RAMP) updateRamp();

  // Luego proceso la interfaz
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
        r0 = readInt("RPM inicio(0–3000): ",0,MOTOR_MAX_RPM);
        r1 = readInt("RPM final (0–3000): ",0,MOTOR_MAX_RPM);
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

  // Modo manual: lee PWM y vuelve a menú manteniendo PWM aplicado
  if (uiState == U_MANUAL && Serial.available()) {
    int p = readInt("",PWM_MIN,PWM_MAX);
    opState = OP_MANUAL;
    applyPWM(p);
    uiState = U_MENU;
    printMenu();
  }

  // Modo display
  if (uiState == U_DISPLAY) doDisplay();

  // Si estamos en rampa y recibimos 'x', detenemos la rampa
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
