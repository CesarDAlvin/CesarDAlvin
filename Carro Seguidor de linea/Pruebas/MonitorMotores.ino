/**
 * TEAM PAMBACODE
 *
 * Programa de prueba de control de motor (canal A) con driver TB6612FNG.
 * Diseñado para funcionar en conjunto con un script Python que envía comandos
 * y recibe datos en formato CSV.
 *
 * Fecha:     2025/05/30
 * Versión:   2.0.0
 *
 * Autores:
 * - César Arturo       / CesarDAlvin
 * - Sara Crystel       / Sara130401
 * - Ceron Dauzon       / Juryelcd
 */

// === PINES TB6612FNG (Motor A) ===
// AIN1/AIN2: control de dirección; PWMA: señal PWM; STBY: habilita el driver
const int PIN_AIN1 = 5, PIN_AIN2 = 4, PIN_PWMA = 3, PIN_STBY = 6;
const int PWM_MAX  = 1023;  // Valor máximo de PWM (escala 0–1023)
const int PWM_MIN  = 200;   // Valor mínimo forzado de PWM para mover el motor

// === Parámetros físicos del sistema ===
typedef float f;
const f MOTOR_RADIUS    = 0.02;   // Radio de la rueda en metros
const int MOTOR_MAX_RPM = 3000;   // RPM máximas del motor
const f GEAR_RATIO      = 10.0;   // Relación de transmisión

// === Estados de operación ===
enum OpState { OP_STOP, OP_MANUAL, OP_DISPLAY, OP_RAMP };
OpState opState = OP_STOP;         // Estado inicial: motor detenido

// === Variables para rampa progresiva ===
int r0 = 0, r1 = 0, secs = 0;              // RPM inicio, RPM fin, duración en s
unsigned long rampStart = 0;               // Timestamp inicio de la rampa
bool rampHold = false;                     // Indica fase de retención después de rampa

// === Variables para modo display ===
unsigned long lastDisp = 0;                // Último timestamp de envío de datos
f lastVel = 0, totalDist = 0;              // Velocidad anterior y distancia acumulada

// — Funciones matemáticas de conversión —

/**  
 * Convierte un valor PWM (0–PWM_MAX) en un torque estimado (Nm).  
 */
f torqueFromPWM(int p) { 
  return (p/(f)PWM_MAX)*0.009807; 
}

/**  
 * Convierte un valor PWM en RPM de rueda, considerando la relación de engranaje.  
 */
f wheelRPM(int p) {     
  return (p/(f)PWM_MAX)*MOTOR_MAX_RPM/GEAR_RATIO; 
}

/**  
 * Convierte RPM a radianes por segundo.  
 */
f radPerSec(f rpm) {    
  return rpm*2*PI/60; 
}

/**  
 * Convierte velocidad angular (rad/s) a lineal (m/s).  
 */
f linearVel(f o) {      
  return o*MOTOR_RADIUS; 
}

// — Función de lectura de comandos desde Python —

/**  
 * Lee una línea completa enviada por el script Python (hasta '\n') y la recorta.  
 * @return Cadena del comando sin espacios ni saltos de línea.  
 */
String readCmd() {
  if (!Serial.available()) return "";
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

// — Aplica PWM al motor con forzado de valor mínimo —

/**  
 * Envía la señal PWM al driver y ajusta la dirección.  
 * Garantiza que si 0 < p < PWM_MIN, se aplique PWM_MIN para arrancar el motor.  
 */
void applyPWM(int p) {
  if (p>0 && p<PWM_MIN) p = PWM_MIN;
  digitalWrite(PIN_AIN1, p>0 ? HIGH : LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, map(abs(p), 0, PWM_MAX, 0, 255));
}

// === Configuración inicial ===

void setup(){
  Serial.begin(9600);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);   // Sacar driver de standby
  applyPWM(0);                    // Asegurar motor detenido
  Serial.println("READY");        // Señal al script Python de que está listo
}

// === Bucle principal ===

void loop(){
  // 1) Procesar comando recibido de Python
  String cmd = readCmd();
  if (cmd.length()) {
    char t = cmd.charAt(0);
    if (t=='M') {
      // Modo manual: "M <PWM>"
      int p = cmd.substring(1).toInt();
      applyPWM(p);
      opState = OP_MANUAL;
      Serial.println("OK_MANUAL");
    }
    else if (t=='D') {
      // Modo display: el Arduino enviará datos periódicos
      opState = OP_DISPLAY;
      lastDisp = millis();
      totalDist = 0; lastVel = 0;
      Serial.println("OK_DISPLAY");
    }
    else if (t=='R') {
      // Modo rampa: comando "R <r0> <r1> <secs>"
      int params[3], idx = 0;
      char *ptr = strtok((char*)cmd.c_str()+1, " ");
      while (ptr && idx<3) {
        params[idx++] = atoi(ptr);
        ptr = strtok(NULL, " ");
      }
      r0 = params[0]; r1 = params[1]; secs = params[2];
      rampStart = millis();
      rampHold = false;
      opState = OP_RAMP;
      Serial.println("OK_RAMP");
    }
    else if (t=='S') {
      // Parar motor
      applyPWM(0);
      opState = OP_STOP;
      Serial.println("OK_STOP");
    }
  }

  unsigned long now = millis();

  // 2) Ejecutar lógica según estado

  // ── Rampa progresiva ──
  if (opState == OP_RAMP) {
    unsigned long span = secs * 1000UL;
    if (!rampHold) {
      if (now - rampStart < span) {
        // Interpolación lineal de RPM entre r0 y r1
        float frac = float(now - rampStart) / span;
        int target = r0 + (r1 - r0) * frac;
        applyPWM(round(target * (f)PWM_MAX / MOTOR_MAX_RPM));
      } else {
        // Cambiar a fase de retención
        rampHold = true;
        rampStart = now;
      }
    } else {
      // Mantener r1 rpm durante 2 segundos
      if (now - rampStart < 2000UL) {
        applyPWM(round(r1 * (f)PWM_MAX / MOTOR_MAX_RPM));
      } else {
        rampHold = false;
        rampStart = now;
      }
    }
  }

  // ── Envío de datos para display ──
  if (opState == OP_DISPLAY && now - lastDisp >= 500) {
    lastDisp = now;
    // Leer PWM real aplicado escalado a 0–PWM_MAX
    int p = map(analogRead(PIN_PWMA), 0, 255, 0, PWM_MAX);
    f rpmv = wheelRPM(p);
    f T    = torqueFromPWM(p);
    f omega= radPerSec(rpmv);
    f vel  = linearVel(omega);
    f acc  = (vel - lastVel) / 0.5;    // dt = 0.5 s
    totalDist += vel * 0.5;
    lastVel = vel;

    // Enviar línea CSV: PWM,RPM,Torque,Velocidad,Aceleración,Distancia
    Serial.print(p);     Serial.print(',');
    Serial.print(rpmv);  Serial.print(',');
    Serial.print(T);     Serial.print(',');
    Serial.print(vel);   Serial.print(',');
    Serial.print(acc);   Serial.print(',');
    Serial.println(totalDist);
  }

  // Los modos OP_MANUAL y OP_STOP no requieren procesamiento adicional aquí
}
