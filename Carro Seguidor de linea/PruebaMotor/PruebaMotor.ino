// === CONFIGURACIÓN DE PINES TB6612FNG (Motor A) ===
const int PIN_AIN1   = 5;
const int PIN_AIN2   = 4;
const int PIN_PWMA   = 3;   // PWM A (0-255)
const int PIN_STBY   = 6;   // Standby

// Parámetros del sistema
typedef float f;
const f MOTOR_RADIUS   = 0.02;    // 2 cm en metros
typedef float f;
const f MOTOR_MASS     = 0.028;   // 28 g en kg
const int PWM_MAX       = 1023;   // Rango PWM Arduino (analogWrite 0-255 -> map desde 0-1023)
const int MOTOR_MAX_RPM = 3000;   // RPM sin carga a 6V
typedef float f;
const f GEAR_RATIO     = 10.0;   // Reducción 10:1

// Variables de estado
int currentPWM   = 0;
bool printing    = true;

// Cálculos auxiliares
f torqueFromPWM(int p) {
  f T_max = 0.009807;                // Nm (0.1 kgf·cm)
  return (p / (f)PWM_MAX) * T_max;
}

f rpmFromPWM(int p) {
  return (p / (f)PWM_MAX) * MOTOR_MAX_RPM;
}

f wheelRPM(int p) {
  return rpmFromPWM(p) / GEAR_RATIO;
}

f radPerSec(f rpm) {
  return rpm * 2.0 * PI / 60.0;
}

f linearVel(f omega) {
  return omega * MOTOR_RADIUS;
}

void setMotor(int p) {
  if (p > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, map(p, 0, PWM_MAX, 0, 255));
  } else if (p < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, map(-p, 0, PWM_MAX, 0, 255));
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, 0);
  }
}

void printMenu() {
  Serial.println("\n=== MENÚ PRINCIPAL ===");
  Serial.println("1: Ver datos en Serial");
  Serial.println("2: Cambiar RPM instantáneo");
  Serial.println("3: Cambio progresivo de RPM");
  Serial.println("m: Mostrar menú");
  Serial.print("Opción: ");
}

void setup() {
  // Inicialización Serial y pines
  Serial.begin(9600);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  printMenu();
}

unsigned long lastPrint = 0;

void loop() {
  // Impresión continua
  if (printing && millis() - lastPrint > 200) {
    f T   = torqueFromPWM(currentPWM);
    f rRPM= wheelRPM(currentPWM);
    f w   = radPerSec(rRPM);
    f v   = linearVel(w);
    Serial.print(currentPWM); Serial.print('\t');
    Serial.print(rRPM);      Serial.print(" rpm\t");
    Serial.print(T, 6);      Serial.print(" Nm\t");
    Serial.print(v, 4);      Serial.println(" m/s");
    lastPrint = millis();
  }

  // Procesar comandos
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': printing = true;    break;
      case '2': {
        Serial.print("RPM deseadas (0-3000): ");
        while (!Serial.available());
        int rpm = Serial.parseInt();
        currentPWM = map(constrain(rpm, 0, MOTOR_MAX_RPM), 0, MOTOR_MAX_RPM, 0, PWM_MAX);
        setMotor(currentPWM);
        Serial.println();
        break;
      }
      case '3': {
        Serial.print("RPM inicio: "); while (!Serial.available()); int r0 = Serial.parseInt(); Serial.println(r0);
        Serial.print("RPM fin:   "); while (!Serial.available()); int r1 = Serial.parseInt(); Serial.println(r1);
        Serial.print("Duración (s): "); while (!Serial.available()); int secs = Serial.parseInt(); Serial.println(secs);
        unsigned long t0 = millis(), t1 = t0 + secs*1000UL;
        while (millis() < t1) {
          float frac = (millis() - t0) / float(secs*1000UL);
          int targetRPM = r0 + frac*(r1 - r0);
          currentPWM = map(targetRPM, 0, MOTOR_MAX_RPM, 0, PWM_MAX);
          setMotor(currentPWM);
          delay(50);
        }
        break;
      }
      case 'm':
        printMenu();
        break;
    }
  }
}