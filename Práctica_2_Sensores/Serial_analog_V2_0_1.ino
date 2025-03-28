/*
TEAM PAMBACODE
ESTE CÓDIGO RECOPILA LA INFORMACION PROPORCIONADA POR DOS SENSORES Y UN POTENCIOMETRO, JUNTO CON UN INDICADOR DE LUZ CON EL
FIN DE IMPLEMENTAR MANERAS DE REGISTRAR LOS DATOS EN UN ARCHIVO CSV EN OTRO CODIGO EN PYTHON.
2025/03/09 - V.2.0.1
TRABAJARON: CESAR ARTURO / CesarDAlvin | SARA CRYSTEL / Sara130401 | CERON DAUZON / Juryelcd
 */ 

//Librerias
#include <DHT.h> //e incluye la libreria para el manejo del sensor DHT, que ofrecen datos de temperatura y humedad

//Definimos los pines  analogicos a utilizar en el Arduino
///Comenzamos con la definicion del pin para el sensor de temperatura y humedad
#define DHTPIN1 A3 //Se define el pin que recibe los datos del DHT, como una constante
#define DHTTYPE DHT11 //Definimos que tipo de sensor de DHT es, en este caso 11
DHT dht1(DHTPIN1, DHTTYPE); //Se crea una instancia para leer los datos del sensor DHT

///Se definen los sensores restantes
const int Lumin = A5; //Sensor HW-486 de luminosidad (Fotoresistencia)
const int Poten = A4; //Potenciometro que manipulara el voltaje de que se le suministre

// Pin del LED que controlaremos con el sensor de luminosidad
const int ledPin = 5;


void setup() {
  Serial.begin(9600); // Inicia comunicación serial a 9600 baudios
  dht1.begin(); //Inicialia al sensor DHT para que empiece a tomar lectura de la temperatura y la humedad

  //Definimos la modalidad de los pines
  pinMode(Lumin, INPUT);
  pinMode(Poten, INPUT);

  // Definimos el pin del LED como salida
  pinMode(ledPin, OUTPUT);
}

void loop() {
  //Leer datos de los sensores
  //Para el sensor DHT
  float temperatura = dht1.readTemperature();
  float humedad = dht1.readHumidity();
  //Para el sensor HW-486
  float luminosidad = analogRead(Lumin); //Tipo de lectura (analogica)
  float luz = map(luminosidad, 0, 1023, 10, 0); //Mapeo de los valores del sensor a un intervalo de 0 a 10
  //Para el potenciometro
  float tension = analogRead(Poten);  //Tipo de lectura (analogica)
  float voltajeSP= map(tension, 0, 1023, 0, 100); //Mapeo de los valores del sensor a un intervalo de 0 a 10
  float voltaje = 5.0/100.0 * voltajeSP;

  // Envio de datos por Serial
  Serial.print("TEMP:"); // Encabezado para identificar el dato
  Serial.println(temperatura); // Envía el valor por serial
  Serial.print("HUM:"); // Encabezado para identificar el dato
  Serial.println(humedad); // Envía el valor por serial
  Serial.print("LUM:"); // Encabezado para identificar el dato
  Serial.println(luz); // Envía el valor por serial
  Serial.print("VOLT:"); // Encabezado para identificar el dato
  Serial.println(voltaje); // Envía el valor por serial


  //Condiciones de encendido del led indicador de luminosidad
   if (luz < 5) {
    digitalWrite(ledPin, HIGH); //Si el valor capturado por el sensor de luminosidad es menor a 5 el led se enciende
    }
  else {
    digitalWrite(ledPin, LOW); //Si no se cumple el primer caso entonces el led se apaga
    }
  delay(500); // Espera 1 segundo entre lecturas
}
