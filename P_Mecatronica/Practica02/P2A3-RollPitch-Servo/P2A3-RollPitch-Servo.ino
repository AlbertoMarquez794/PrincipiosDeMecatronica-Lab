#include <Servo.h>  // Incluye la biblioteca Servo para controlar el servo motor

#define x 14        // Definición del pin analógico conectado al acelerómetro en el eje X
#define y 12        // Definición del pin analógico conectado al acelerómetro en el eje Y
#define z 13        // Definición del pin analógico conectado al acelerómetro en el eje Z
#define servoPin 23 // Definición del pin al que está conectado el servo motor
Servo miServo;       // Creación de un objeto Servo llamado "miServo"

// Variables para almacenar los valores leídos de los acelerómetros en el eje X, Y y Z
float valX = 0;
float valY = 0;
float valZ = 0;

// Variables para calcular la gravedad en los ejes X, Y y Z
float Xg = 0;
float Yg = 0;
float Zg = 0;

// Valores mínimos y máximos de los acelerómetros en cada eje
float x_min = 1547.0;
float x_max = 2391.0;
float y_min = 1558.0;
float y_max = 2391.0;
float z_min = 1583.0;
float z_max = 2417.0;

// Ángulos de roll y pitch en radianes y grados
float alphaR = 0.0;
float betaR = 0.0;
float alphaG = 0.0;
float betaG = 0.0;

void setup() {
  Serial.begin(115200);  // Inicializa la comunicación serial a 115200 baudios
  miServo.attach(servoPin);  // Asocia el objeto Servo con el pin del servo motor
}

void loop() {
  // Lee los valores analógicos de los acelerómetros en los ejes X, Y y Z
  valX = analogRead(x);
  valY = analogRead(y);
  valZ = analogRead(z);

  // Calcula la gravedad normalizada en los ejes X, Y y Z
  Xg = map(valX, x_min, x_max, -100, 100);
  Xg = -Xg / 100;
  Yg = map(valY, y_min, y_max, -100, 100);
  Yg = Yg / 100;
  Zg = map(valZ, z_min, z_max, -100, 100);
  Zg = Zg / 100;

  // Calcula los ángulos de roll y pitch en radianes
  alphaR = -atan(Yg / sqrt(pow(Xg, 2) + pow(Zg, 2)));
  betaR = atan(Xg / sqrt(pow(Yg, 2) + pow(Zg, 2)));

  // Convierte los ángulos de radianes a grados
  alphaG = alphaR * 180 / PI;
  betaG = betaR * 180 / PI;

  // Mapea el ángulo de roll a un valor entre 0 y 180 para controlar el servo motor
  int anguloServoMotor = map(alphaG, -90, 90, 0, 180);
  
  // Mueve el servo motor al ángulo calculado
