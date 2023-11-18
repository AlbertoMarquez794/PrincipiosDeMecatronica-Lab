#include "BluetoothSerial.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
BluetoothSerial SerialBT;
char dato;

#define IN1 33 //Declara el giro del motor a la izquierda (2)
#define IN2 25 //Declara el giro del motor a la derecha (2)
#define ENB 14 //Permite la activacion de un motor en el puente H (2)
#define EncA2 16
//Manda pulsos al encoder
#define EncB2 4 //Lee los pulsos del encoder
/*--------------------------------------------------*/

int channel = 0;        // Define una variable llamada "channel" y la inicializa en 0
int freq = 1000;        // Define una variable llamada "freq" y la inicializa en 1000
int resolution = 12;    // Define una variable llamada "resolution" y la inicializa en 12
/*--------------------------------------------------*/
#define IN3 26 //Declara el giro del motor a la izquierda
#define IN4 27 //Declara el giro del motor a la derecha
#define ENA 32 //Permite la activación de un motor en el puente H

#define EncA 2
//Manda pulsos al encoder
#define EncB 15 //Lee los pulsos del encoder



volatile long pulses = 0; //Ponemos los pulsos en cero

void IRAM_ATTR PulsesCounter(){
  if (digitalRead(EncB) == HIGH && digitalRead(EncB2)== HIGH ){     // si B es HIGH, sentido horario
    pulses++ ;        // incrementa PULSES en 
  }
  else {          // si B es LOW, sentido anti horario
    pulses-- ;        // decrementa el PULSES en 1
  }
}

void setup() { //Declara las variables del puente H como outputs y las del encoder como inputs
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);

  pinMode(EncA, INPUT);
  pinMode(EncB, INPUT);
} 

void loop() {
    movimientoAdelante();
    detenido();

}

void detenido(){
    digitalWrite(ENA,LOW); //habilitamos el motor a través del puente HH
    digitalWrite(ENB,LOW); //habilitamos el motor a través del puente H
    Serial.println("Detenido"); //Imprimimos "Dextrógiro"
    delay(2000);
}


void movimientoAdelante(){
     encoder.attachFullQuad(EncA, EncB);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(motorPWM, 125);
    delay(5000)
}


void movimientoReversa(){
    digitalWrite(IN3,HIGH); //encendemos giro a la izquierda
    digitalWrite(IN4,LOW); //apagamos giro a la derecha
    digitalWrite(ENA,HIGH); //habilitamos el motor a través del puente H
    digitalWrite(IN1,HIGH); //encendemos giro a la izquierda
    digitalWrite(IN2,LOW); //apagamos giro a la derecha
    digitalWrite(ENB,HIGH); //habilitamos el motor a través del puente H
}
void movimientoIzquierda(){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENB,HIGH); //habilitamos el motor a través del puente H
    digitalWrite(ENA, LOW); 
    Serial.println("Derecha"); //Imprimimos "Levógiro"
}

void movimientoDerecha(){
    movimientoAdelante();
    delay(100);
    
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(ENB, LOW); //habilitamos el motor a través del puente H
    digitalWrite(ENA, HIGH); 
    Serial.println("Izquierda"); //Imprimimos "Levógiro"
}
