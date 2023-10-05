// Definición de pines
#define EnM 13           // Define el pin 13 como EnM
#define MA 14            // Define el pin 14 como MA
#define MB 12            // Define el pin 12 como MB
#define potenciometro 27 // Define el pin 27 como potenciometro

// Variables para configuración PWM
int channel = 0;        // Define el canal PWM como 0
int freq = 1000;        // Frecuencia del PWM
int resolution = 12;    // Resolución del PWM

void setup() {
  pinMode(EnM, OUTPUT);  // Configura el pin EnM como salida
  pinMode(MA, OUTPUT);   // Configura el pin MA como salida
  pinMode(MB, OUTPUT);   // Configura el pin MB como salida

  // Configuración para el PWM
  ledcSetup(channel, freq, resolution); // Configura el canal PWM
  ledcAttachPin(EnM, channel);          // Asocia el pin EnM al canal PWM
  Serial.begin(115200);                 // Inicializa la comunicación serial a 115200 baudios
}

void loop() {
  float val = analogRead(potenciometro);       // Lee el valor analógico del pin potenciometro y lo almacena en val
  float dirMotor = ((val * 3.3) / 4095);       // Convierte el valor analógico a voltaje y lo almacena en dirMotor
  float dirMotorAumentada = dirMotor * 100;    // Escala el valor de dirMotor a un rango de 0 a 100

  // Comprobación del estado del motor en función del valor del potenciómetro
  if (dirMotor >= 0 && dirMotor < 1.32) {
    int min = map(dirMotorAumentada, 0, 132, 4095, 0); // Mapea el valor a un rango PWM inverso
    ledcWrite(channel, min);            // Establece el valor PWM
    digitalWrite(MA, LOW);              // Configura el pin MA en bajo
    digitalWrite(MB, HIGH);             // Configura el pin MB en alto
    Serial.println("Estado: levogiro");  // Imprime el estado "levogiro"
    delay(500);
    float porcentaje = map(dirMotorAumentada, 0, 132, 100, 0); // Calcula el porcentaje correspondiente
    Serial.println(porcentaje + String(" %")); // Imprime el porcentaje
  } else if (dirMotor >= 1.32 && dirMotor < 1.98) {
    ledcWrite(channel, 0);               // Apaga el PWM
    digitalWrite(MA, HIGH);             // Configura el pin MA en alto
    digitalWrite(MB, LOW);              // Configura el pin MB en bajo
    Serial.println("Estado: detenido");  // Imprime el estado "detenido"
    delay(500);
    Serial.println("0 %");
  } else if (dirMotor >= 1.98) {
    int max = map(dirMotorAumentada, 198, 330, 0, 4095); // Mapea el valor a un rango PWM
    ledcWrite(channel, max);             // Establece el valor PWM
    digitalWrite(MA, HIGH);             // Configura el pin MA en alto
    digitalWrite(MB, LOW);              // Configura el pin MB en bajo
    Serial.println("Estado: Dextrogiro"); // Imprime el estado "Dextrogiro"
    delay(500);
    float porcentaje1 = map(dirMotorAumentada, 198, 330, 0, 100); // Calcula el porcentaje correspondiente
    Serial.println(porcentaje1 + String(" %")); // Imprime el porcentaje
  }
  Serial.println(dirMotor + String(" V")); // Imprime el voltaje actual del potenciómetro
}
