#define IN1 33 //Declara el giro del motor a la izquierda (2)
#define IN2 25 //Declara el giro del motor a la derecha (2)
#define ENB 14 //Permite la activacion de un motor en el puente H (2)
/*--------------------------------------------------*/
#define IN3 26 //Declara el giro del motor a la izquierda
#define IN4 27 //Declara el giro del motor a la derecha
#define ENA 32 //Permite la activación de un motor en el puente H

/*-----------------------------------*/
//Encoder
#define EncA1 4 // Pin A del encoder 1
#define EncA2 15 // Pin A del encoder 2
#define EncB1 2 // Pin B del encoder 1
#define EncB2 16 // Pin B del encoder 2


/*-------------------*/
//LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
/*-------------------------------*/
//Fotoresistores
#define fotoIzq 34 
#define fotoDer 35 
/*--------------------------------*/
//Infrarrojo
#define infroj3 18 
#define infroj1 5 
#define infroj2 17
#define infroj4 19
/*--------------------------------*/
//Ultrasonico
#define echoPin 12
#define trigPin 13
/*--------------------------------*/

int channel = 0;
int freq = 1000;
int resolution = 8;

int channel2 = 0;
int freq2 = 1000;
int resolution2 = 8;

/*--------------------------------*/
volatile long pulsesDer = 0;
volatile long pulsesIzq = 0;

/*¨*/
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
char dato;
LiquidCrystal_I2C lcd(0x27, 16, 2);
//volatile long pulses = 0; //Ponemos los pulsos en cero

void setup() { //Declara las variables del puente H como outputs y las del encoder como inputs
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT);

  pinMode (infroj1, INPUT); // Configuracion de sensores infrarojos
  pinMode (infroj2, INPUT);
  pinMode (infroj3, INPUT);
  pinMode (infroj4, INPUT);


  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(EncA1, INPUT); // Configuración del encoder
  pinMode(EncB1, INPUT);
  pinMode(EncA2, INPUT); 
  pinMode(EncB2, INPUT);

  //Señal pwm motor izquierdo
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(ENA, channel);
  //Señal pwm motro derecha
  ledcSetup(channel2, freq2, resolution2);
  ledcAttachPin(ENB, channel2);

  


  /*Configuración del fotoresistor*/

  pinMode(fotoIzq, INPUT); 
  pinMode(fotoDer, INPUT); 
  /*Inicialización de los parametros*/
  Serial.begin(115200); //Inicializamos el serial en 115200 baudios
  SerialBT.begin("Equipo1ESP");
  lcd.init();
  lcd.backlight();
}

void loop() {
     attachInterrupt(digitalPinToInterrupt(EncA1), Encoder_izquierdo, RISING);
     attachInterrupt(digitalPinToInterrupt(EncA2), Encoder_derecho, RISING);
     imprimirPulsos();
     fotoresistor();
     obstaculos();
     ultrasonico();
     Serial.println();
     delay(4000);
     
}

void imprimirPulsos(){
    Serial.print("Pulsos_Enc_Izq:"+String(pulsesIzq)+" | Pulsos_Enc_Der:"+String(pulsesDer)+" | ");  
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
    digitalWrite(EncB2, HIGH);
    digitalWrite(EncB1, HIGH);
    lcd.setCursor(0,0);
    lcd.print("Adelante");
    ledcWrite(channel, 125);
    ledcWrite(channel2, 145);
    // Configurar velocidad del motor A mediante PWM
}
void movimientoReversa(int delayM){
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
    //Señal pwm motor izquierdo
    ledcWrite(channel, 125);
    ledcWrite(channel2, 125);
    delay(delayM);
}

void movimientoDerecha(int delayM){
   //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, HIGH);
    lcd.setCursor(0,0);
    lcd.print("Derecha");
    // Configurar velocidad del motor A mediante PWM
    //Señal pwm motor izquierdo
    ledcWrite(channel, 125);
    ledcWrite(channel2, 145);
    delay(delayM);
}

void movimientoIzquierda(int delayM){
   //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, LOW);
    lcd.setCursor(0,0);
    lcd.print("Izquierda");
   //Señal pwm motor izquierdo
    ledcWrite(channel, 125);
    ledcWrite(channel2, 125);
    delay(delayM);
}

void ultrasonico(){
   float duracionUlt, distanciaUlt;
   digitalWrite(trigPin, LOW);
   delayMicroseconds(1000);

   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);

  duracionUlt = pulseIn(echoPin,HIGH);

  distanciaUlt = 0.0343 * duracionUlt * 0.5;

  lcd.setCursor(7,0); 
  lcd.print(" L: " + String(distanciaUlt));
  Serial.print(" Distancia:"+String(distanciaUlt)+" | ");
}

void fotoresistor(){
  int luzIzq = analogRead(fotoIzq);
  int luzDer = analogRead(fotoDer);
  lcd.setCursor(0,1);
  lcd.print("I:" + String(luzIzq) + " | D:" + String(luzDer));
  

  Serial.print("LDR_Izq:"+String(luzIzq)+" | LDR_Der:"+String(luzDer)+" |\n");
}

void obstaculos(){
    int inf3 = digitalRead(infroj3); 
    int inf1 = digitalRead(infroj1);
    int inf2 = digitalRead(infroj2);
    int inf4 = digitalRead(infroj4);
    bool  arrInf[] = {inf1, inf2, inf3, inf4};
    int flagsObstaculos [] = {0,0,0,0};
    //inf[3] es el primer de izquierda a derecha
    //inf[2] es el segundo de izquierda a derecha
    //inf[1] es el tercero de izquierda a derecha
    //inf[0] es el cuarto de izquierda a derecha
    for (int i=0; i < 4; i ++){
     if (!arrInf[i]){
        flagsObstaculos[i] = 1; 
     }
    }

    Serial.print(" Obst: "+String(flagsObstaculos[0])+" " + String(flagsObstaculos[1]) + " " + String(flagsObstaculos[2]) + " " + String(flagsObstaculos[3])+" | ");
    lcd.setCursor(0,0);   // Establece la posición del cursor en la columna 0, fila 0
    lcd.print(String(flagsObstaculos[3]) + String(flagsObstaculos[2]) + String(flagsObstaculos[1]) + String(flagsObstaculos[0]) + "   "); 
  }

   void Encoder_izquierdo(){
    if(digitalRead(EncB1)==HIGH){
        pulsesIzq++;
    } else {
      pulsesIzq--;
    }  
  }
  
  void Encoder_derecho(){
    if(digitalRead(EncB2)==HIGH){
      pulsesDer++;
    } else {
      pulsesDer--;
    }
  }
