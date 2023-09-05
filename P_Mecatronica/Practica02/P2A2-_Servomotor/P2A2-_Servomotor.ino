#include <Servo.h> // Incluye la biblioteca Servo para controlar el servo motor

#define servoPin  22      // Definición del pin al que está conectado el servo motor
#define potenciometro  4   // Definición del pin analógico al que está conectado el potenciómetro
Servo miServo;            // Creación de un objeto Servo llamado "miServo"

void setup() {
    Serial.begin(115200); // Inicializa la comunicación serial a 115200 baudios
    miServo.attach(servoPin); // Asocia el objeto Servo con el pin del servo motor
}

void loop() {
    // Lee el valor analógico del potenciómetro y mapea el valor a un ángulo entre 0 y 180 grados
    int anguloServoMotor = map(analogRead(potenciometro), 0, 4065, 0, 180);

    // Controla el servo motor para moverlo al ángulo calculado
    miServo.write(anguloServoMotor);

    // Imprime el valor leído del potenciómetro y el ángulo del servo motor en la comunicación serial
    Serial.print("ADC =");
    Serial.print(analogRead(potenciometro));
    Serial.print(" bits ----- Theta =");
    Serial.print(anguloServoMotor);
    Serial.println(" ° ");
}
