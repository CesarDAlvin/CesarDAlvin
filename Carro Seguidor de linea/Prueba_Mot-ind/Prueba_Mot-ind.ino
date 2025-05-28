// === CONFIGURACIÓN DE PINES TB6612FNG (Motor A) ===
const int PIN_AIN1 = 5, PIN_AIN2 = 4, PIN_PWMA = 3, PIN_STBY = 6;
const int PWM_MAX = 1023;
int currentPWM = 0;

// — Función para leer una línea completa y devolverla “trimmed” —
String readLine() {
  String s = Serial.readStringUntil('\n');  // lee hasta \n
  s.trim();                                 // quita espacios, \r, \n
  return s;
}

// — Muestra el menú —
void printMenu() {
  Serial.println("\n=== MENÚ PRINCIPAL ===");
  Serial.println("1: Establecer PWM y girar motor");
  Serial.println("2: Visualización datos en tiempo real");
  Serial.println("m: Mostrar menú");
  Serial.print("Opción: ");
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  printMenu();
}

void loop() {
  if (!Serial.available()) return;

  // 1) Leo línea completa
  String line = readLine();
  if (line.length() == 0) {
    // si el usuario sólo presionó ENTER, vuelvo a pedir
    Serial.print("Opción: ");
    return;
  }

  // 2) Tomo la primera letra
  char c = line.charAt(0);
  Serial.println(c);    // eco de lo que escribió

  // 3) Ejecuto la opción
  switch (c) {
    case '1': {
      Serial.print("→ PWM (0-1023): ");
      int pwm = -1;
      while (pwm < 0) {
        String p = readLine();
        if (p.length() == 0) {
          Serial.print("   No ingresaste nada. Reingresa: ");
          continue;
        }
        bool allDigits = true;
        for (char d : p) if (!isDigit(d)) { allDigits = false; break; }
        if (!allDigits) {
          Serial.print("   Sólo dígitos. Reingresa: ");
          continue;
        }
        int v = p.toInt();
        if (v < 0 || v > PWM_MAX) {
          Serial.print("   Fuera de rango. Reingresa: ");
          continue;
        }
        pwm = v;
      }
      currentPWM = pwm;
      // Aplica PWM
      digitalWrite(PIN_AIN1, currentPWM>0);
      digitalWrite(PIN_AIN2, LOW);
      analogWrite(PIN_PWMA, map(currentPWM, 0, PWM_MAX, 0, 255));
      Serial.print("   >> PWM aplicado: "); Serial.println(currentPWM);
      break;
    }

    case '2':
      // Aquí iría tu rutina de visualización continua...
      Serial.println("Entrando en visualización (q para salir)...");
      // ...
      break;

    case 'm':
      printMenu();
      break;

    default:
      Serial.println("Opción inválida");
      break;
  }

  // 4) Siempre vuelvo a pedir opción
  Serial.print("\nOpción: ");
}

