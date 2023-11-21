#define IN1 33 //Declara el giro del motor a la izquierda (2)
#define IN2 25 //Declara el giro del motor a la derecha (2)
#define ENB 14 //Permite la activacion de un motor en el puente H (2)
/*--------------------------------------------------*/
#define IN3 26 //Declara el giro del motor a la izquierda
#define IN4 27 //Declara el giro del motor a la derecha
#define ENA 32 //Permite la activación de un motor en el puente H
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
volatile long pulses = 0; //Ponemos los pulsos en cero

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
     movimientoAdelante(5000);
     parar();
     movimientoDerecha(5000);
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

void movimientoAdelante(int delayM){
  
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
    ledcWrite(channel, 125);
    ledcWrite(channel2, 145);
    // Configurar velocidad del motor A mediante PWM
    

    delay(delayM);
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

void ultrasonico(int delayUlt){
   float duracionUlt, distanciaUlt;
   digitalWrite(trigPin, LOW);
   delayMicroseconds(1000);

   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);

  duracionUlt = pulseIn(echoPin,HIGH);

  distanciaUlt = 0.0343 * duracionUlt * 0.5;

  //lcd.setCursor(11,0); 
  //lcd.print("L:"+String(distanciaUltrasonico));
  Serial.println("Distancia:"+String(distanciaUlt)+" | ");
  delay(3000);
}

void fotoresistor(int delayFoto){
  int luzIzq = analogRead(fotoIzq);
  int luzDer = analogRead(fotoDer);
  lcd.setCursor(0,1);
  lcd.print("I:" + String(luzIzq) + " | D:" + String(luzDer));
  

  Serial.println("LDR_Izq:"+String(luzIzq)+" | LDR_Der:"+String(luzDer)+" |");
  delay(delayFoto);
}

void obstaculos(int delayInf){
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

    Serial.println("Obst 1:"+String(flagsObstaculos[0])+" "+ " Obst2: " + String(flagsObstaculos[1]) + " Obs3: " + String(flagsObstaculos[2]) + " Obst4: " +String(flagsObstaculos[3])+" | ");
    lcd.setCursor(0,0);   // Establece la posición del cursor en la columna 0, fila 0
    lcd.print(String(flagsObstaculos[3])+" "+String(flagsObstaculos[2])+ " " + String(flagsObstaculos[1]) + " " + String(flagsObstaculos[0]) + " | ");
    delay(delayInf); 
  }
