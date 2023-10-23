#include <Arduino.h>
#include <ESP32Encoder.h>

#define EncA 19
#define EncB 18
#define motorPWM 21
#define motorIn3 23
#define motorIn4 22

ESP32Encoder encoder;
long previousPosition = 0;
int ppr = 100; // Ajusta el valor de ppr según las especificaciones de tu encoder

int A, B;

void setup() {
  Serial.begin(115200);

  pinMode(EncA, INPUT);
  pinMode(EncB, INPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);

  encoder.attachFullQuad(EncA, EncB);

  // Inicializa la dirección y velocidad del motor
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  analogWrite(motorPWM, 255);

  delay(3000); // Gira en sentido dextrógiro durante 3 segundos

  // Detén el motor
  analogWrite(motorPWM, 0);
  delay(2000); // Pausa de 2 segundos

  // Cambia la dirección del motor
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  analogWrite(motorPWM, 255);

  delay(3000); // Gira en sentido levógiro durante 3 segundos

  // Detén el motor
  analogWrite(motorPWM, 0);
  delay(2000); // Pausa de 2 segundos
}

void loop() {
  A = digitalRead(EncA);
  B = digitalRead(EncB);

  long newPosition = encoder.getCount();

  if (newPosition > previousPosition) {
    Serial.print("Levógira --- Cantidad de vueltas: ");
    Serial.println(newPosition / ppr);
  } else if (newPosition < previousPosition) {
    Serial.print("Dextrógira --- Cantidad de vueltas: -");
    Serial.println(abs(newPosition / ppr));
  }

  previousPosition = newPosition;
  delay(1000); // Actualiza cada segundo
}
