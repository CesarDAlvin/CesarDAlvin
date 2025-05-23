import processing.serial.*;

// =====================================================================
//                          VARIABLES GLOBALES
// =====================================================================

// Puerto serie para comunicación con Arduino
Serial myPort;

// Datos de sensores
float temperature1 = 0, temperature2 = 0;
float humidity1 = 0, humidity2 = 0;
float soilHumidity1 = 0, soilHumidity2 = 0, waterLevel = 0; 
float currentHumidity = 0; 
int fireSensorState = 1; // 1: sin fuego, 0: fuego

// Parámetros generales de interfaz
float time = 0; 
float waveAmplitude = 5;   
float waveFrequency = 0.05; 
int leftWidth;
boolean useSecondSensor = false; 

// Historial de datos para gráficas
ArrayList<Float> tempHistory = new ArrayList<>();
ArrayList<Float> humidityHistory = new ArrayList<>();

// Control del LED
float ledBrightness = 50;   // Brillo inicial en % (0-100)
boolean ledOn = false;       // Estado inicial del LED
int ledGaugeX = 500;        // Posición del gauge LED
int ledGaugeY = 130;
float gaugeDiameter = 140;
float gaugeRadius = gaugeDiameter / 2;

// Parámetros del botón para el relé
int relayButtonX = 650; // Posición X del botón
int relayButtonY = 200; // Posición Y del botón
int relayButtonW = 150; // Ancho del botón
int relayButtonH = 40;  // Alto del botón
boolean relayOn = false; // Estado inicial del relé (apagado)

// Botón para encendido/apagado del LED
int ledButtonX = 650;
int ledButtonY = 100;
int ledButtonW = 150;
int ledButtonH = 40;

// Declarar las imágenes globalmente
PImage fireImage, noFireImage;

// =====================================================================
//                              SETUP
// =====================================================================

void setup() {
  size(1750, 1000, P3D);
  
  leftWidth = int(width * 0.5);

  // Inicializar puerto serie
  String portName = "COM6"; 
  myPort = new Serial(this, portName, 9600); 
  myPort.bufferUntil('\n'); 
  
  // Cargar las imágenes desde la carpeta "data"
  fireImage = loadImage("fireImage.png");     // Asegúrate de tener esta imagen en la carpeta "data"
  noFireImage = loadImage("noFireImage.png"); // Asegúrate de tener esta imagen en la carpeta "data"
  
  if (fireImage == null || noFireImage == null) {
    println("Error: Una o más imágenes no se pudieron cargar. Verifica que estén en la carpeta 'data'.");
    exit(); // Salir del programa si las imágenes no se cargan
  }
  // Enviar estado inicial del LED al Arduino
  sendLedCommand();
  sendRelayCommand(); // Enviar el comando al Arduino
}

// =====================================================================
//                              DRAW
// =====================================================================

void draw() {
  background(#FAFAFA);
  time += 1; // Actualizar tiempo para animaciones (ondas en tanque de agua)
  
  // Dibujar fondo gris a la derecha
  noStroke();
  fill(220);
  rect(leftWidth, 40, width - leftWidth, height - 30);
  
  // Dibujar prisma 3D
  drawPrism3D();
  
  // Dibujar gauge del LED y su botón
  drawLedGauge(ledGaugeX, ledGaugeY, "Brillo LED", ledBrightness, 0, 100, "%", color(255, 200, 0));
  
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(24);
  text(int(ledBrightness) + " %", ledGaugeX, ledGaugeY + 40);
  
  //Dibujo de botones de los leds y reles
  drawButton(relayButtonX, relayButtonY, relayButtonW, relayButtonH, relayOn ? "Apagar Relé" : "Encender Relé", relayOn ? color(200, 0, 0) : color(0, 200, 0));
  drawButton(ledButtonX, ledButtonY, ledButtonW, ledButtonH, ledOn ? "Apagar LED" : "Encender LED", ledOn ? color(200, 0, 0) : color(0, 200, 0));
  
  // Dibujar etiqueta sobre el botón del relé
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(20);
  text("Control de riego", relayButtonX + relayButtonW / 2, relayButtonY - 20);
  
  // Título principal
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(30);
  text("Monitoreo Vegetal", width / 2, 15);

  // Elegir sensor actual (1 o 2)
  float currentTemperature = useSecondSensor ? temperature2 : temperature1;
  currentHumidity = useSecondSensor ? humidity2 : humidity1;

  // Calcular porcentajes de humedad y nivel de agua
  float soilHumidity1Percent = map(soilHumidity1, 675, 314, 0, 100);
  float soilHumidity2Percent = map(soilHumidity2, 680, 317, 0, 100);
  float waterLevelPercent    = map(waterLevel, 0, 850, 0, 100);

  // Dibujar agujas de temperatura, humedad y humedad del suelo
  drawNeedleGauge(100, 130, "Temperatura", currentTemperature, 0, 50, "°C", color(255, 100, 100));
  drawNeedleGauge(300, 130, "Humedad", currentHumidity, 0, 100, "%", color(100, 150, 255));
  drawNeedleGauge(100, 330, "Humedad Suelo 1", soilHumidity1Percent, 0, 100, "%", color(100, 255, 100));
  drawNeedleGauge(300, 330, "Humedad Suelo 2", soilHumidity2Percent, 0, 100, "%", color(200, 100, 255));

  // Indicador de fuego
  drawFireIndicator(300, height - 310, 200, 290, fireSensorState);
  
  // Determinar títulos de gráficas
  String sensorNumber = useSecondSensor ? "2" : "1";
  String tempTitle = "Temperatura vs Tiempo (Sensor " + sensorNumber + ")";
  String humTitle  = "Humedad vs Tiempo (Sensor " + sensorNumber + ")";

  // Dibujar gráficas de temperatura y humedad
  drawLineGraph(100, 500, 300, 150, tempTitle, tempHistory, 50, "°C");
  drawLineGraph(500, 500, 300, 150, humTitle, humidityHistory, 100, "%");

  // Dibujar tanque de agua
  drawWaterTank(100, height - 50, 100, 200, waterLevelPercent);

  // Botón para cambiar sensor
  drawButton(350, 400, 150, 40, "Cambiar Sensor", 20);

  // Etiqueta de sensor actual
  fill(0);
  textAlign(LEFT, CENTER);
  textSize(20);
  text("Conjunto de Sensores Actual: " + sensorNumber, 320, 460);

}

// =====================================================================
//                            EVENTOS Y SERIAL
// =====================================================================

void mousePressed() {
  // Botón cambiar sensor
  if (mouseX > 350 && mouseX < 500 && mouseY > 400 && mouseY < 440) {
    useSecondSensor = !useSecondSensor;
  }
  
  // Ajustar brillo del LED al hacer clic en el gauge
  float dx = mouseX - ledGaugeX;
  float dy = mouseY - ledGaugeY;
  float distCenter = sqrt(dx*dx + dy*dy);
  
  if (distCenter < gaugeDiameter / 2) {
    float angle = atan2(dy, dx); 
    if (angle > 0) angle = 0;
    if (angle < -PI) angle = -PI;
    float newBrightness = map(angle, -PI, 0, 0, 100);
    ledBrightness = constrain(newBrightness, 0, 100);
    if (ledOn) {
      sendLedCommand();
    }
  }

  // Detectar clic en botones
  if (mouseX > relayButtonX && mouseX < relayButtonX + relayButtonW && mouseY > relayButtonY && mouseY < relayButtonY + relayButtonH) {
    relayOn = !relayOn; // Cambiar estado del relé
    sendRelayCommand();
  } else if (mouseX > ledButtonX && mouseX < ledButtonX + ledButtonW && mouseY > ledButtonY && mouseY < ledButtonY + ledButtonH) {
    ledOn = !ledOn; // Cambiar estado del LED
    sendLedCommand();
  }

}

void serialEvent(Serial myPort) {
  String data = myPort.readStringUntil('\n');
  if (data != null) {
    String[] values = data.trim().split(",");
    if (values.length == 8) {
      temperature1    = float(values[0]);
      humidity1       = float(values[1]);
      temperature2    = float(values[2]);
      humidity2       = float(values[3]);
      soilHumidity1   = float(values[4]);
      soilHumidity2   = float(values[5]);
      waterLevel      = float(values[6]); 
      fireSensorState = int(values[7]);

      tempHistory.add(useSecondSensor ? temperature2 : temperature1); 
      if (tempHistory.size() > 50) tempHistory.remove(0);

      humidityHistory.add(useSecondSensor ? humidity2 : humidity1); 
      if (humidityHistory.size() > 50) humidityHistory.remove(0);
    }
  }
}

// =====================================================================
//                         FUNCIONES AUXILIARES
// =====================================================================
// Enviar comando para el LED
void sendLedCommand() {
  int pwmValue = int(map(ledBrightness, 0, 100, 0, 255));
  String command = ledOn ? "ACTIVATE," + pwmValue : "DEACTIVATE,0";
  myPort.write(command + "\n");
  println("Comando enviado al Arduino (LED): " + command);
}

// Enviar comando para el relé
void sendRelayCommand() {
  String command = relayOn ? "RELAY,1" : "RELAY,0";
  myPort.write(command + "\n");
  println("Comando enviado al Arduino (Relé): " + command);
}

void drawPrism3D() {
  // Posicionar el centro del objeto en el rango horizontal deseado
  float desiredCenterX = 1312.5; 
  float desiredCenterY = 100 + height / 2;

  pushMatrix();
  translate(desiredCenterX, desiredCenterY, 0);

  // Rotaciones: perspectiva
  rotateY(radians(10));
  rotateY(millis() * 0.0001); // Rota con el tiempo

  // Dimensiones ajustadas
  float w = 900 * 0.5;
  float h = 100 * 1.5;
  float d = 400 * 0.6;
  float hw = w / 2; 
  float hh = h / 2; 
  float hd = d / 2; 

  float[][] topVertices = {
    {-hw, -hh, -hd}, 
    { hw, -hh, -hd}, 
    { hw, -hh,  hd}, 
    {-hw, -hh,  hd}
  };

  float[][] bottomVertices = {
    {-hw, hh, -hd},  
    { hw, hh, -hd},  
    { hw, hh,  hd},  
    {-hw, hh,  hd}   
  };

  hint(DISABLE_DEPTH_TEST);

  // Dividir y rellenar caras con colores amaderados
  drawDividedFacesWithDarkRectangles(topVertices, bottomVertices, 5);

  // Dibujar asas superiores con líneas rectas en vez de arcos curvos
  stroke(230, 210, 150); 
  noFill();

  float arcHeight = 150 * 1.2;  // Ajustar altura proporcionalmente

  // Asa frontal (usando los vértices frontales topVertices[0] y topVertices[1])
  drawRectHandle(topVertices[0][0], topVertices[0][1], topVertices[0][2],
                 topVertices[1][0], topVertices[1][1], topVertices[1][2],
                 arcHeight);

  // Asa trasera (usando los vértices traseros topVertices[3] y topVertices[2])
  drawRectHandle(topVertices[3][0], topVertices[3][1], topVertices[3][2],
                 topVertices[2][0], topVertices[2][1], topVertices[2][2],
                 arcHeight);

 drawAllSpheres(topVertices, bottomVertices, hw, hh, hd);

  popMatrix();
}

void drawAllSpheres(float[][] topVertices, float[][] bottomVertices, float hw, float hh, float hd) {
  float sphereSize = 30; // Tamaño de las esferas

  // Lista de esferas con coordenadas, etiquetas y estados de sensores
  Sphere[] spheres = {
    new Sphere((topVertices[0][0] + topVertices[1][0]) / 2, topVertices[0][1], (topVertices[0][2] + topVertices[1][2]) / 2, "Sensor de Fuego", fireSensorState != 0),
    new Sphere(topVertices[1][0], topVertices[1][1] - 3 * sphereSize, topVertices[1][2], "Sensor DHT 1", temperature1 > 0),
    new Sphere(bottomVertices[3][0], topVertices[1][1] - 3 * sphereSize, bottomVertices[3][2], "Sensor DHT 2", temperature2 > 0),
    new Sphere(0, topVertices[0][1]- 6 * sphereSize, 0, "Iluminacion led", ledOn),
    new Sphere(-hw / 3, 0, 0, "Sensor de humedad de Suelo 2", soilHumidity1 > 0),
    new Sphere(hw / 3, 0, 0, "Sensor de humedad de Suelo 1", soilHumidity2 > 0),
    new Sphere(-hw, 0, hd / 2, "Sensor del nivel del agua", waterLevel > 0)
  };

  // Dibujar todas las esferas con el color correspondiente
  for (Sphere sphere : spheres) {
    fill(sphere.isActive ? color(0, 200, 0) : color(200, 0, 0)); // Verde si activo, rojo si no
    noStroke();
    sphereAtWithLabel(sphere.x, sphere.y, sphere.z, sphereSize, sphere.label);
  }
}

class Sphere {
  float x, y, z;
  String label;
  boolean isActive;

  Sphere(float x, float y, float z, String label, boolean isActive) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.label = label;
    this.isActive = isActive;
  }
}

void sphereAtWithLabel(float x, float y, float z, float size, String label) {
  // Dibujar la esfera
  pushMatrix();
  translate(x, y, z);
  sphere(size);
  popMatrix();

  // Dibujar la etiqueta
  pushMatrix();
  translate(x, y - size - 10, z); // Mover la etiqueta justo encima de la esfera
  rotateY(radians(180)); // Girar el texto 180 grados para que sea legible de frente
  fill(0); // Color negro para el texto
  textSize(18);
  textAlign(CENTER, CENTER);
  text(label, 0, 0); // Dibujar el texto centrado
  popMatrix();
}


void drawDividedFacesWithDarkRectangles(float[][] topVertices, float[][] bottomVertices, int divisions) {
  for (int i = 0; i < topVertices.length; i++) {
    int next = (i + 1) % topVertices.length;

    float[] topStart = topVertices[i];
    float[] topEnd = topVertices[next];
    float[] bottomStart = bottomVertices[i];
    float[] bottomEnd = bottomVertices[next];

    float segmentHeight = (bottomStart[1] - topStart[1]) / divisions;

    for (int j = 0; j < divisions; j++) {
      float[] currentTopStart = {topStart[0], topStart[1] + j * segmentHeight, topStart[2]};
      float[] currentTopEnd = {topEnd[0], topEnd[1] + j * segmentHeight, topEnd[2]};
      float[] currentBottomStart = {topStart[0], topStart[1] + (j + 1) * segmentHeight, topStart[2]};
      float[] currentBottomEnd = {topEnd[0], topEnd[1] + (j + 1) * segmentHeight, topEnd[2]};

      // Rellenar solo las caras pares (0, 2, 4)
      if (j % 2 == 0) { // Índice par
        fill(210, 180, 140); // Color madera oscuro
      } else {
        noFill(); // Sin relleno para las caras impares
      }

      noStroke(); // Desactivar líneas para las caras
      beginShape();
      vertex(currentTopStart[0], currentTopStart[1], currentTopStart[2]);
      vertex(currentTopEnd[0], currentTopEnd[1], currentTopEnd[2]);
      vertex(currentBottomEnd[0], currentBottomEnd[1], currentBottomEnd[2]);
      vertex(currentBottomStart[0], currentBottomStart[1], currentBottomStart[2]);
      endShape(CLOSE);
    }
  }
}

// Función para dibujar las asas
void drawRectHandle(float x1, float y1, float z1, float x2, float y2, float z2, float height) {
  // Línea vertical desde (x1, y1, z1) hacia arriba
  line(x1, y1, z1, x1, y1 - height, z1); 
  // Línea vertical desde (x2, y2, z2) hacia arriba
  line(x2, y2, z2, x2, y2 - height, z2);
  // Línea horizontal uniendo la parte superior de las dos líneas verticales
  line(x1, y1 - height, z1, x2, y2 - height, z2);
}

void drawNeedleGauge(int x, int y, String label, float value, float min, float max, String unit, color needleColor) {
  fill(255);
  noStroke();
  ellipse(x, y, 140, 140);

  noFill();
  stroke(220);
  strokeWeight(10);
  arc(x, y, 120, 120, -PI, 0);

  float angle = map(value, min, max, -PI, 0); 
  stroke(needleColor);
  arc(x, y, 120, 120, -PI, angle);

  stroke(0); 
  strokeWeight(4);
  line(x, y, x + cos(angle)*50, y + sin(angle)*50);

  fill(0); 
  noStroke();
  ellipse(x, y, 10, 10);

  fill(0);
  textAlign(CENTER, CENTER);
  textSize(24);
  text(label, x, y - 80); 
  textSize(24);
  text(nf(value, 1, 1) + " " + unit, x, y + 20); 
}

void drawFireIndicator(int x, int y, int w, int h, int fireState) {
  // Mostrar la imagen adecuada según el estado de fuego
  if (fireState == 1) {
    image(fireImage, x, y, w, h); // Mostrar imagen de fuego detectado
    fill(0);
    textAlign(CENTER, TOP);
    textSize(30);
    text("Fuego Detectado", x + 1.5* w, y + h / 2); // Etiqueta
  } else {
    image(noFireImage, x, y, w, h); // Mostrar imagen sin fuego
    fill(0);
    textAlign(CENTER, TOP);
    textSize(30);
    text("Sin Fuego", x + 1.5* w, y + h / 2); // Etiqueta
  }
}

void drawLineGraph(int x, int y, int w, int h, String title, ArrayList<Float> history, float maxY, String unit) {
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(24);
  text(title, x + w / 2, y - 20);

  stroke(220);
  strokeWeight(1);
  for (int i = 0; i <= 10; i++) {
    line(x + i * w / 10, y, x + i * w / 10, y + h);
  }
  for (int i = 0; i <= 5; i++) {
    line(x, y + i * h / 5, x + w, y + i * h / 5);
  }

  fill(0);
  textAlign(RIGHT, CENTER);
  for (int i = 0; i <= 5; i++) {
    float labelVal = map(i, 0, 5, 0, maxY);
    text(nf(labelVal, 1, 1) + unit, x - 10, y + h - i * h / 5);
  }

  textAlign(CENTER, CENTER);
  text("Segundos", x + w / 2, y + h + 20);

  strokeWeight(2);
  stroke(unit.equals("°C") ? color(255, 100, 100) : color(100, 150, 255));
  noFill();
  beginShape();
  for (int i = 0; i < history.size(); i++) {
    float posX = map(i, 0, 50, x, x + w);
    float posY = map(history.get(i), 0, maxY, y + h, y);
    vertex(posX, posY);
  }
  endShape();

  for (int i = 0; i < history.size(); i++) {
    float posX = map(i, 0, 50, x, x + w);
    float posY = map(history.get(i), 0, maxY, y + h, y);
    fill(unit.equals("°C") ? color(255, 100, 100) : color(100, 150, 255));
    noStroke();
    ellipse(posX, posY, 5, 5);
  }
}

void drawWaterTank(int x, int y, int w, int h, float percent) {
  float waterHeight = map(percent, 0, 100, 0, h);
  float bottomLeft = x;
  float bottomRight = x + w;

  float topWidth = w * 2;
  float topLeft = (x + w/2) - topWidth/2; 
  float topRight = (x + w/2) + topWidth/2; 
  float topY = y - h;

  float lineWidthAtWater = w + (topWidth - w)*(waterHeight/h); 
  float waterY = y - waterHeight;

  float waterLeft = (x + w/2) - lineWidthAtWater/2;
  float waterRight = (x + w/2) + lineWidthAtWater/2;

  int numPoints = 50;

  noStroke();
  fill(0, 100, 255); 
  beginShape();
  for (int i = 0; i <= numPoints; i++) {
    float t = float(i)/numPoints;
    float X = lerp(waterLeft, waterRight, t);
    float wave = sin(time * 0.1 + X * waveFrequency) * waveAmplitude;
    float Y = waterY + wave;
    vertex(X, Y);
  }
  vertex(bottomRight, y);
  vertex(bottomLeft, y);
  endShape(CLOSE);

  noFill();
  stroke(0);
  strokeWeight(2);
  line(bottomLeft, y, bottomRight, y);       
  line(topLeft, topY, topRight, topY);       
  line(bottomLeft, y, topLeft, topY);        
  line(bottomRight, y, topRight, topY);

  fill(0);
  textAlign(CENTER, TOP);
  textSize(30);
  text(int(percent) + "%", x + w/2, y + 10);
  
  fill(0);
  textAlign(CENTER, TOP);
  textSize(30);
  text("Nivel del Agua", x + w/2, y-h-40);
}

void drawButton(float x, float y, float w, float h, String label, int fontSize) {
  // Dibuja el rectángulo del botón
  fill(100, 150, 250);
  rect(x, y, w, h, 5);
  
  // Configura el tamaño de la fuente
  textSize(fontSize);
  textAlign(CENTER, CENTER);
  
  // Dibuja el texto en el centro del botón
  fill(255);
  text(label, x + w / 2, y + h / 2);
}

//Dibujo de los botones leds y Rele
void drawButton(int x, int y, int w, int h, String label, int buttonColor) {
  fill(buttonColor);
  rect(x, y, w, h, 5);
  fill(255);
  textSize(20);
  textAlign(CENTER, CENTER);
  text(label, x + w / 2, y + h / 2);
}

void drawLedGauge(int x, int y, String label, float value, float min, float max, String unit, color needleColor) {
  fill(255); 
  noStroke();
  ellipse(x, y, gaugeDiameter, gaugeDiameter);

  noFill();
  stroke(220); 
  strokeWeight(10);
  arc(x, y, gaugeDiameter - 20, gaugeDiameter - 20, -PI, 0);

  float angle = map(value, min, max, -PI, 0); 
  stroke(needleColor);
  arc(x, y, gaugeDiameter - 20, gaugeDiameter - 20, -PI, angle);

  stroke(0); 
  strokeWeight(4);
  line(x, y, x + cos(angle) * 50, y + sin(angle) * 50);

  fill(0); 
  noStroke();
  ellipse(x, y, 10, 10);

  fill(0);
  textAlign(CENTER, CENTER);
  textSize(24);
  text(label, x, y - 80); 
}
