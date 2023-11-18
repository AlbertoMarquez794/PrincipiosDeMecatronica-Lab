#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
char dato;

#define IN1 33 //Declara el giro del motor a la izquierda (2)
#define IN2 25 //Declara el giro del motor a la derecha (2)
/*--------------------------------------------------*/
#define IN3 26 //Declara el giro del motor a la izquierda
#define IN4 27 //Declara el giro del motor a la derecha
#define ENA 32 //Permite la activación de un motor en el puente H
#define ENB 14 //Permite la activacion de un motor en el puente H (2)
#define EncA 2
//Manda pulsos al encoder
#define EncB 15 //Lee los pulsos del encoder
#define EncA2 16
//Manda pulsos al encoder
#define EncB2 4 //Lee los pulsos del encoder


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
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  pinMode(EncA,INPUT);
  pinMode(EncB,INPUT);
  pinMode(EncA2, INPUT);
  pinMode(EncB2, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(EncA), PulsesCounter, RISING); //Cada vez que hacemos una interrupción llamamos al PulseCounter
  attachInterrupt(digitalPinToInterrupt(EncA2), PulsesCounter, RISING); //Cada vez que hacemos una interrupción llamamos al PulseCounter
  Serial.begin(115200); //Inicializamos el serial en 115200 baudios
  SerialBT.begin("Equipo1ESP");
}

void loop() {
    movimientoAdelante();
    detenido();
    movimientoReversa();
    detenido();
    movimientoDerecha();
    detenido();
    movimientoIzquierda();
    detenido();
}

void detenido(){
    digitalWrite(ENA,LOW); //habilitamos el motor a través del puente HH
    digitalWrite(ENB,LOW); //habilitamos el motor a través del puente H
    Serial.println("Detenido"); //Imprimimos "Dextrógiro"
    delay(5000);
}


void movimientoAdelante(){
    digitalWrite(IN3,LOW); //apagamos giro a la izquierda
    digitalWrite(IN4,HIGH); //encendemos giro a la derecha
    digitalWrite(ENA,HIGH); //habilitamos el motor a través del puente HH
    digitalWrite(IN1,LOW); //apagamos giro a la izquierda
    digitalWrite(IN2,HIGH); //encendemos giro a la derecha
    digitalWrite(ENB,HIGH); //habilitamos el motor a través del puente H
    Serial.println("Adelante"); //Imprimimos "Dextrógiro"
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
