#define IN1 33 //Declara el giro del motor a la izquierda (2)
#define IN2 25 //Declara el giro del motor a la derecha (2)
/*--------------------------------------------------*/
#define IN3 26 //Declara el giro del motor a la izquierda
#define IN4 27 //Declara el giro del motor a la derecha
#define ENA 32 //Permite la activación de un motor en el puente H
#define ENB 14 //Permite la activacion de un motor en el puente H (2)
/*-------------------*/
//LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
char dato;
LiquidCrystal_I2C lcd(0x27, 16, 2);
volatile long pulses = 0; //Ponemos los pulsos en cero

void setup() { //Declara las variables del puente H como outputs y las del encoder como inputs
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT);

  Serial.begin(115200); //Inicializamos el serial en 115200 baudios
  SerialBT.begin("Equipo1ESP");
  lcd.init();
  lcd.backlight();
}

void loop() {
  movimientoAdelante();
  parar();
  movimientoIzquierda();
  parar();
}

void parar(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(ENA,LOW);
    digitalWrite(ENB, LOW);
    lcd.setCursor(0,0);
    lcd.print("Parado");
    delay(2000);
}

void movimientoAdelante(){
    //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, HIGH);
    lcd.setCursor(0,0);
    lcd.print("Adelante");
    // Configurar velocidad del motor A mediante PWM
    analogWrite(ENA, 127);  // Ajusta el valor (0-255) para controlar la velocidad

    // Configurar velocidad del motor B mediante PWM
    analogWrite(ENB, 127);  // Ajusta el valor (0-255) para controlar la velocidad
    delay(3000);
}
void movimientoReversa(){
      //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(ENB, HIGH);
    lcd.setCursor(0,0);
    lcd.print("Reversa");
    // Configurar velocidad del motor A mediante PWM
    analogWrite(ENA, 127);  // Ajusta el valor (0-255) para controlar la velocidad

    // Configurar velocidad del motor B mediante PWM
    analogWrite(ENB, 127);  // Ajusta el valor (0-255) para controlar la velocidad
    delay(3000);
}

void movimientoIzquierda(){
   //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, HIGH);
    lcd.setCursor(0,0);
    lcd.print("Izquierda");
    // Configurar velocidad del motor A mediante PWM
    analogWrite(ENA, 127);  // Ajusta el valor (0-255) para controlar la velocidad

    // Configurar velocidad del motor B mediante PWM
    analogWrite(ENB, 127);  // Ajusta el valor (0-255) para controlar la velocidad
    delay(710);
}

void movimientoDerecha(){
   //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, LOW);
    lcd.setCursor(0,0);
    lcd.print("Derecha");
    // Configurar velocidad del motor A mediante PWM
    analogWrite(ENA, 127);  // Ajusta el valor (0-255) para controlar la velocidad

    // Configurar velocidad del motor B mediante PWM
    analogWrite(ENB, 127);  // Ajusta el valor (0-255) para controlar la velocidad
    delay(710);
}
