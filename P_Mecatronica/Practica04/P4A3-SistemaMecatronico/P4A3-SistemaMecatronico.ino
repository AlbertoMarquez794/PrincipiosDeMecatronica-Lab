#include <ESP32Servo.h>  // Incluye la biblioteca Servo para controlar el servo motor en arduino 2.2.1 IDE
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define x 14        // Definición del pin analógico conectado al acelerómetro en el eje X
#define y 12        // Definición del pin analógico conectado al acelerómetro en el eje Y
#define z 13        // Definición del pin analógico conectado al acelerómetro en el eje Z
#define servoPin 15 // Definición del pin al que está conectado el servo motor
Servo miServo;       // Creación de un objeto Servo llamado "miServo"


// Set the LCD address to 0x27 for a 16 columnas y 2 filas
LiquidCrystal_I2C lcd(0x27, 16, 2);


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
  lcd.init();
  lcd.backlight();


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
    float anguloServoMotor = map(alphaG, -90, 90, 0, 180);
    miServo.write(anguloServoMotor);
    // Mueve el servo motor al ángulo calculado
    lcd.setCursor(0, 0);
    lcd.print("Sist. Mecatronico");
    lcd.setCursor(0, 1);
    lcd.print("Angulo: " + String(anguloServoMotor));
    delay(1000);
}
