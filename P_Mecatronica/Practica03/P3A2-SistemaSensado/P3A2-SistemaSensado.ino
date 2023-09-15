// Definiciones de pines
const int sensorObstaculoPin = 36;  // Pin analógico para el sensor de obstáculos
const int distanciaTriggerPin = 2;   // Pin para el trigger del sensor de distancia HC-SR04
const int distanciaEchoPin = 15;      // Pin para el echo del sensor de distancia HC-SR04
const int ldrPin = 4;       // Pin analógico para el sensor de luminosidad

// Constantes para el sensor de luminosidad (LDR)
int channel = 0;
int freq = 1000;
int resolution = 8;

const int capacitivePin1 = 13;        // Pin digital para el sensor capacitivo 1
const int capacitivePin2 = 12;        // Pin digital para el sensor capacitivo 2
const int capacitivePin3 = 14;        // Pin digital para el sensor capacitivo 3

const int ledVerde1Pin = 23;          // Pin para el primer LED verde
const int ledVerde2Pin = 22;          // Pin para el segundo LED verde
const int ledAmarillo1Pin = 21;       // Pin para el primer LED amarillo
const int ledAmarillo2Pin = 19;       // Pin para el segundo LED amarillo
const int ledRojo1Pin = 18;           // Pin para el primer LED rojo
const int ledRojo2Pin = 5;            // Pin para el segundo LED rojo

// Variables de control de banderas
bool bandera1 = false;
bool bandera2 = false;
bool bandera3 = false;

void setup() {
  // Inicializa los pines como entradas o salidas según corresponda
  pinMode(sensorObstaculoPin, INPUT);
  pinMode(distanciaTriggerPin, OUTPUT);
  pinMode(distanciaEchoPin, INPUT);
  pinMode(ldrPin, INPUT);

  // Declaración de los pines para los LEDs
  pinMode(ledVerde1Pin, OUTPUT);
  pinMode(ledVerde2Pin, OUTPUT);
  pinMode(ledAmarillo1Pin, OUTPUT);
  pinMode(ledAmarillo2Pin, OUTPUT);
  pinMode(ledRojo1Pin, OUTPUT);
  pinMode(ledRojo2Pin, OUTPUT);

  // Inicializa la comunicación serial y funciones de PWM para los LEDs
  ledcSetup(channel, freq, resolution);
  // ledcAttachPin(ledPin, channel); // Este comando está comentado y no se utiliza
  Serial.begin(9600);
}

void loop() {
  // Lee los sensores de obstáculos, distancia y luminosidad
  int valorObstaculo = digitalRead(sensorObstaculoPin);
  int luminosidad = map(analogRead(ldrPin), 0, 1023, 0, 100); // Mapea la lectura del LDR a un porcentaje de luminosidad

  // Leer los sensores capacitivos y actualizar las banderas
  int valorCapacitive1 = touchRead(capacitivePin1);
  int valorCapacitive2 = touchRead(capacitivePin2);
  int valorCapacitive3 = touchRead(capacitivePin3);

  // ...
  // (Aquí se encuentra la lógica para determinar el estado de las banderas)

  // Definición de rutinas

  // Rutina 1: Sensor de obstáculos
  if (bandera1) {
    // Muestra el valor del sensor de obstáculos en la consola
    Serial.println(valorObstaculo);

    if (valorObstaculo == 0) {
      // Obstáculo detectado, enciende todos los LEDs
      digitalWrite(ledVerde1Pin, HIGH);
      digitalWrite(ledVerde2Pin, HIGH);
      digitalWrite(ledAmarillo1Pin, HIGH);
      digitalWrite(ledAmarillo2Pin, HIGH);
      digitalWrite(ledRojo1Pin, HIGH);
      digitalWrite(ledRojo2Pin, HIGH);
      delay(1000 / 5);
      digitalWrite(ledVerde1Pin, LOW);
      digitalWrite(ledVerde2Pin, LOW);
      digitalWrite(ledAmarillo1Pin, LOW);
      digitalWrite(ledAmarillo2Pin, LOW);
      digitalWrite(ledRojo1Pin, LOW);
      digitalWrite(ledRojo2Pin, LOW);
      delay(1000 / 5);

      // Muestra un mensaje en la consola
      Serial.println("Obst: 1 hay un obstáculo");
    } else {
      // No se detecta obstáculo, apaga todos los LEDs
      digitalWrite(ledVerde1Pin, LOW);
      digitalWrite(ledVerde2Pin, LOW);
      digitalWrite(ledAmarillo1Pin, LOW);
      digitalWrite(ledAmarillo2Pin, LOW);
      digitalWrite(ledRojo1Pin, LOW);
      digitalWrite(ledRojo2Pin, LOW);

      // Muestra un mensaje en la consola
      Serial.println("Obst: 0 no hay obstáculo");
    }
    valorObstaculo = digitalRead(sensorObstaculoPin);

    // Apaga todos los LEDs
    digitalWrite(ledVerde1Pin, LOW);
    digitalWrite(ledVerde2Pin, LOW);
    digitalWrite(ledAmarillo1Pin, LOW);
    digitalWrite(ledAmarillo2Pin, LOW);
    digitalWrite(ledRojo1Pin, LOW);
    digitalWrite(ledRojo2Pin, LOW);
  }

  // Rutina 2: Sensor de distancia
  if (bandera2) {
    // Lógica para medir la distancia y controlar los LEDs según la distancia
    // ...

    // Apaga todos los LEDs
    digitalWrite(ledVerde1Pin, LOW);
    digitalWrite(ledVerde2Pin, LOW);
    digitalWrite(ledAmarillo1Pin, LOW);
    digitalWrite(ledAmarillo2Pin, LOW);
    digitalWrite(ledRojo1Pin, LOW);
    digitalWrite(ledRojo2Pin, LOW);
  }

  // Rutina 3: Sensor de luminosidad (LDR)
  if (bandera3) {
    // Lógica para medir la luminosidad y controlar los LEDs según el valor
    // ...

    // Apaga todos los LEDs
    digitalWrite(ledVerde1Pin, LOW);
    digitalWrite(ledVerde2Pin, LOW);
    digitalWrite(ledAmarillo1Pin, LOW);
    digitalWrite(ledAmarillo2Pin, LOW);
    digitalWrite(ledRojo1Pin, LOW);
    digitalWrite(ledRojo2Pin, LOW);
  }
}
