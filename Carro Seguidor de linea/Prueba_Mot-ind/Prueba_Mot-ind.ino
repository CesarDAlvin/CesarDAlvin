/**
 * TEAM PAMBACODE
 *
 * Programa de prueba de funcionamiento de un motor (canal A) usando el driver
 * TB6612FNG. Proporciona un menú interactivo por consola serie para:
 *   1) Ajustar el valor PWM (0–1023) y girar el motor en una dirección fija.
 *   2) Entrar en modo de visualización de datos en tiempo real (por implementar).
 *   m) Volver a mostrar el menú.
 *
 * Fecha:     2025/06/01
 * Versión:   2.0.0
 *
 * Autores:
 * - César Arturo       / CesarDAlvin
 * - Sara Crystel       / Sara130401
 * - Ceron Dauzon       / Juryelcd
 */

// === CONFIGURACIÓN DE PINES TB6612FNG (Motor A) ===
// AIN1/AIN2: control de dirección, PWMA: señal PWM, STBY: standby del driver
const int PIN_AIN1  = 5;    // Entrada 1 de control de dirección
const int PIN_AIN2  = 4;    // Entrada 2 de control de dirección
const int PIN_PWMA  = 3;    // Señal PWM para velocidad (0–255 en analogWrite)
const int PIN_STBY  = 6;    // Pin de activación del driver (HIGH = activo)
const int PWM_MAX   = 1023; // Valor máximo de PWM que acepta el menú (se escalara a 0–255)
int currentPWM      = 0;    // Variable global para almacenar el PWM actual

/**
 * Lee una línea completa desde la consola serie hasta '\n' y la recorta.
 * @return Cadena sin espacios ni retorno de carro al inicio o final.
 */
String readLine() {
  String s = Serial.readStringUntil('\n');  // Leer hasta salto de línea
  s.trim();                                 // Eliminar espacios, '\r', '\n'
  return s;
}

/**
 * Imprime en la consola serie el menú principal de opciones.
 */
void printMenu() {
  Serial.println("\n=== MENÚ PRINCIPAL ===");
  Serial.println("1: Establecer PWM y girar motor");
  Serial.println("2: Visualización de datos en tiempo real");
  Serial.println("m: Mostrar este menú");
  Serial.print("Opción: ");
}

void setup() {
  // Inicializa la comunicación serie a 9600 baudios
  Serial.begin(9600);

  // Configura los pines de control como salida
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // Activa el driver sacándolo de standby
  digitalWrite(PIN_STBY, HIGH);

  // Muestra el menú inicial al usuario
  printMenu();
}

void loop() {
  // Si no hay datos en el buffer serie, no hacemos nada
  if (!Serial.available()) return;

  // 1) Leer la línea completa del usuario
  String line = readLine();
  if (line.length() == 0) {
    // Si sólo presionó ENTER, volvemos a pedir la opción
    Serial.print("Opción: ");
    return;
  }

  // 2) Tomar la primera letra como comando
  char c = line.charAt(0);
  Serial.println(c);  // Eco del comando ingresado

  // 3) Ejecutar la opción seleccionada
  switch (c) {
    case '1': {
      // Opción 1: solicitar valor de PWM entre 0 y PWM_MAX
      Serial.print("→ PWM (0–");
      Serial.print(PWM_MAX);
      Serial.print("): ");
      int pwm = -1;

      // Bucle de validación de entrada
      while (pwm < 0) {
        String p = readLine();
        if (p.length() == 0) {
          Serial.print("   No ingresaste nada. Reingresa: ");
          continue;
        }
        // Comprobar que todos los caracteres son dígitos
        bool allDigits = true;
        for (char d : p) {
          if (!isDigit(d)) { allDigits = false; break; }
        }
        if (!allDigits) {
          Serial.print("   Sólo dígitos válidos. Reingresa: ");
          continue;
        }
        // Convertir a entero y revisar rango
        int v = p.toInt();
        if (v < 0 || v > PWM_MAX) {
          Serial.print("   Valor fuera de rango. Reingresa: ");
          continue;
        }
        pwm = v;
      }

      // Guardar y aplicar el PWM ingresado
      currentPWM = pwm;
      // Dirección fija: AIN1 = HIGH (adelante), AIN2 = LOW
      digitalWrite(PIN_AIN1, HIGH);
      digitalWrite(PIN_AIN2, LOW);
      // Escalar 0–PWM_MAX a 0–255 para analogWrite
      int outVal = map(currentPWM, 0, PWM_MAX, 0, 255);
      analogWrite(PIN_PWMA, outVal);

      Serial.print("   >> PWM aplicado: ");
      Serial.println(currentPWM);
      break;
    }

    case '2':
      // Opción 2: modo de visualización continua (pendiente de implementación)
      Serial.println("Entrando en visualización (presiona 'q' para salir)...");
      // TODO: agregar lógica de muestreo y despliegue en tiempo real
      break;

    case 'm':
      // Opción 'm': reimprimir el menú
      printMenu();
      break;

    default:
      // Comando no reconocido
      Serial.println("Opción inválida. Presiona 'm' para mostrar el menú.");
      break;
  }

  // 4) Siempre volver a solicitar opción después de procesar
  Serial.print("\nOpción: ");
}

