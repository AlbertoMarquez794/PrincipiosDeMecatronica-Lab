#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
char dato;
// Definir pines para los LEDs
#define LED1  4
#define LED2  5
#define LED3  21


void setup() {
  Serial.begin(115200);
  SerialBT.begin("MingyarEsp32"); // nombre del dispositivo Bluetooth
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);


}


void loop() {
  if (SerialBT.available()) {
    dato = SerialBT.read();
 
    if (dato == '1'){
      if(digitalRead(LED1)==HIGH)
      {
        digitalWrite(LED1, LOW);
        Serial.println("apagar LED1");
        SerialBT.println("LED1 apagado");
      }else
      {
        digitalWrite(LED1, HIGH);
        Serial.println("Encendido LED1");
        SerialBT.println("LED1 Encendido");
      }
    }
 
    if (dato == '2'){
      if(digitalRead(LED2)==HIGH)
      {
        digitalWrite(LED2, LOW);
        Serial.println("apagar LED2");
        SerialBT.println("LED2 apagado");
      }else
      {
        digitalWrite(LED2, HIGH);
        Serial.println("Encendido LED2");
        SerialBT.println("LED2 Encendido");
      }
    }
    if (dato == '3'){
      if(digitalRead(LED3)==HIGH)
      {
        digitalWrite(LED3, LOW);
        Serial.println("apagar LED3");
        SerialBT.println("LED3 apagado");
      }else
      {
        digitalWrite(LED3, HIGH);
        Serial.println("Encendido LED3");
        SerialBT.println("LED3 Encendido");
      }
    }


    if (dato == '4'){
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      Serial.println("Apagados todos los LEDs");
      SerialBT.println("Todos los LEDs Apagados");
    }
  }
  delay(100);
}
