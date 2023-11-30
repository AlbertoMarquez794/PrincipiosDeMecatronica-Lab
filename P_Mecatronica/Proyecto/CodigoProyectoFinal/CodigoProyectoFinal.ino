#define IN1 33 //Declara el giro del motor a la izquierda (2)
#define IN2 25 //Declara el giro del motor a la derecha (2)
#define ENB 14 //Permite la activacion de un motor en el puente H (2)
//--------------------------------------------------/
#define IN3 26 //Declara el giro del motor a la izquierda
#define IN4 27 //Declara el giro del motor a la derecha
#define ENA 32 //Permite la activación de un motor en el puente H

//-----------------------------------/
//Encoder
#define EncA1 4 // Pin A del encoder 1
#define EncA2 15 // Pin A del encoder 2
#define EncB1 2 // Pin B del encoder 1
#define EncB2 16 // Pin B del encoder 2


//-------------------/
//LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//-------------------------------/
//Fotoresistores
#define fotoIzq 34 
#define fotoDer 35
//--------------------------------/
//Infrarrojo
#define infroj3 18 
#define infroj1 5 
#define infroj2 17
#define infroj4 19
//--------------------------------/
//Ultrasonico
#define echoPin 12
#define trigPin 13
//--------------------------------/

// Inicialización de variables para la configuración de canales
int channel = 0;
int freq = 1000;
int resolution = 8;

// Variables para controlar la velocidad máxima de los motores
int maxSpeedRight = 140;
int maxSpeedLeft = 120;

// Variable para controlar la detención del robot
bool pararAhora = false;

// Almacenamiento del valor máximo leído por los fotoresistores
int maxFotores=0;

// Inicialización de variables para la configuración de PWM del segundo canal
int channel2 = 0;
int freq2 = 1000;
int resolution2 = 8;

// Contadores de pulsos de los encoders de los motores
volatile long pulsesDer = 0;
volatile long pulsesIzq = 0;

// Variables para almacenar el promedio de los pulsos de los encoders
int mediaDer = 0;
int mediaIzq = 0;

// Variable para controlar la fase de calibración
bool calibrating = true;

// Variables para sensores infrarrojos
int inf3; 
int inf1;
int inf2;
int inf4;

// Contador para la función de detención
int detenerCounter = 0;

// Variable para controlar la dirección del giro
bool girarDerecha = true;

// Inclusión de la librería Bluetooth y LCD
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

  


  //Configuración del fotoresistor/

  pinMode(fotoIzq, INPUT); 
  pinMode(fotoDer, INPUT); 
  //Inicialización de los parametros/
  Serial.begin(115200); //Inicializamos el serial en 115200 baudios
  SerialBT.begin("Equipo1ESP");
  lcd.init();
  lcd.backlight();
}

void loop() {
     // Comprueba si el robot está en fase de calibración
    if (calibrating){
      // Imprime un mensaje de calibración en el monitor serial
      Serial.println("Calibrando...");

      // Llama a la función para calibrar los fotoresistores
      calibracionFotoresistencias(23);
      // Imprime un mensaje al completar la calibración
      Serial.println("Calibrado!");
      // Pausa el programa durante 1500 milisegundos (1.5 segundos)
      delay(1500);

      // Cambia el estado de calibración a falso, terminando la calibración
      calibrating = false;
    }

    // Obtiene la lectura del fotoresistor derecho
    int der = fotoresistor(true);
    // Obtiene la lectura del fotoresistor izquierdo
    int izq = fotoresistor(false);
    // Calcula el promedio de las lecturas de los fotoresistores
    int lecturaFotores  = (der + izq)/2;
    // Obtiene la lectura del sensor ultrasónico
    int ultDist = ultrasonicoRaw();

    // Imprime información de las lecturas en el monitor serial
    Serial.print("Lectura Maxima: ");
    Serial.print(maxFotores);
    Serial.print(" | Lecturas: ");
    Serial.print(der);
    Serial.print(" -- ");
    Serial.print(izq);
    Serial.print(" -- ");
    Serial.print(lecturaFotores);
    Serial.print(" == Distancia: ");
    Serial.print(ultDist);
    // Configura la posición del cursor en la pantalla LCD y muestra la distancia
    lcd.setCursor(0,0);
    lcd.print("D: ");
    lcd.print(ultDist);
    // Continúa mostrando información en la pantalla LCD
    lcd.print(" I: ");
    lcd.print(inf3);
    lcd.print(inf1);
    lcd.print(inf2);
    lcd.print(inf4);
    lcd.setCursor(0,1);
    lcd.print("Ft: ");
    lcd.print(lecturaFotores);
    lcd.print(" | ");
    lcd.print(maxFotores);

    // Condiciones para detener el robot
    if (lecturaFotores >= 3800 && ultDist <= 14){
      pararAhora = true;
    }
    // Si la condición de parar es verdadera, ejecuta la función de parar
    if (pararAhora){
      parar();
    }else{
      // Actualiza el estado de los sensores de obstáculos
      obstaculosUpdate();
      
      // Verifica la presencia de obstáculos en frente
      if (ultDist <=15.0){

        // Ejecuta el movimiento hacia la derecha
        movimientoDerecha();

        // Imprime un mensaje sobre la detección de obstáculos
        Serial.print(" | OBSTACULO ENFRENTE |");
        // Pausa el programa durante 730 milisegundos
        delay(730);
        // Cambia la dirección de giro
        girarDerecha = !girarDerecha;
      }

      // Actualiza el valor máximo de la lectura del fotoresistor
      if (lecturaFotores > maxFotores){
        maxFotores = lecturaFotores;
      }

      // Condición para moverse hacia adelante o girar si hay un obstáculo
      if (lecturaFotores>= maxFotores*0.98){
        Serial.print(" | ADELANTE | ");
        if (ultDist <=15.0){

          movimientoDerecha();

          Serial.print(" | OBSTACULO ENFRENTE |");
          girarDerecha = !girarDerecha;
        }else{
           movimientoAdelante();
        }
       

        delay(1000);
      }
      else{
        Serial.print(" | MOVIMIENTO DERECHA | ");
        movimientoDerecha();
      }

      // Pausa breve del programa
      delay(100);
    }
    // Imprime una línea nueva en el monitor serial y limpia la pantalla LCD
    Serial.println();
    lcd.clear(); 
    
      
}


void calibracionFotoresistencias(int lecturas){
  // Variables para almacenar las lecturas acumuladas de los fotoresistores derecho e izquierdo
  int lecturasDer = 0;
  int lecturasIzq = 0;
  // Variable para almacenar el promedio de las lecturas actuales
  int promLecturas=0;

  // Bucle que se ejecuta un número determinado de veces para realizar la calibración
  for (int i = 0; i < lecturas; i++){
    // Lee el valor del fotoresistor derecho (true indica derecho)
    lecturasDer = fotoresistor(true);
    // Lee el valor del fotoresistor izquierdo (false indica izquierdo)
    lecturasIzq = fotoresistor(false);
    // Calcula el promedio de las lecturas actuales de ambos fotoresistores
    promLecturas = (lecturasDer + lecturasIzq) / 2;

    // Si el promedio actual es mayor que el máximo registrado, actualiza el máximo
    if (promLecturas >= maxFotores){
      maxFotores = promLecturas;
    }
    // Ejecuta un movimiento hacia la derecha (parte de la calibración)
    movimientoDerecha();
    // Pausa el programa por 100 milisegundos
    delay(100);
  }

  // Calcula y almacena el promedio de todas las lecturas para cada fotoresistor
  mediaDer = lecturasDer / lecturas;
  mediaIzq = lecturasIzq / lecturas;
}


void parar(){
  // Detiene todos los motores
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(ENA,LOW);
    digitalWrite(ENB, LOW);

}

void seguirDerecha(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, HIGH);
}

void seguirIzquierda(){
   //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, LOW);
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
    
    // Configurar velocidad del motor A mediante PWM
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
    //Señal pwm motor izquierdo
    ledcWrite(channel, 125);
    ledcWrite(channel2, 125);
    //delay(delayM);
}

void movimientoDerecha(){
   //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, HIGH);
    // Configurar velocidad del motor A mediante PWM
    //Señal pwm motor izquierdo
    ledcWrite(channel, 125);
    ledcWrite(channel2, 145);
}

void movimientoIzquierda(){
   //Motor derecho asociados EL PIN 16 y 4
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, HIGH);
    //Motor izquierdo asociados EL PIN 17, 5  
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(ENB, LOW);
}


float ultrasonicoRaw(){
// Declaración de variables para almacenar la duración de la señal y la distancia calculada
   float duracionUlt, distanciaUlt;

   // Establece el pin del trigger del sensor ultrasónico en bajo
   digitalWrite(trigPin, LOW);
   // Espera 1000 microsegundos para estabilizar la señal
   delayMicroseconds(1000);

   // Envía un pulso alto de 10 microsegundos al sensor ultrasónico
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);

   // Mide el tiempo que tarda en llegar el eco, es decir, la duración de la señal de eco
   duracionUlt = pulseIn(echoPin, HIGH);

   // Calcula la distancia en base al tiempo que tardó el eco en volver y la velocidad del sonido
   // La fórmula es: distancia = velocidad del sonido (343 metros/segundo) * tiempo / 2
   // Aquí, 0.0343 cm/microsegundo es la velocidad del sonido y se divide por 2 porque es un viaje de ida y vuelta
   return 0.0343 * duracionUlt * 0.5;

}

void printFotoresistor(){
  int luzIzq = analogRead(fotoIzq);
  int luzDer = analogRead(fotoDer);
  lcd.setCursor(0,1);
  lcd.print("I:" + String(luzIzq) + " | D:" + String(luzDer));
  

  Serial.print("LDR_Izq:"+String(luzIzq)+" | LDR_Der:"+String(luzDer)+" |\n");
}

int fotoresistor(bool derecho){

  // Regresar la lectura de las fotoresistencias a partir de un booleano

  if (derecho){
    return analogRead(fotoDer);
  }

  return analogRead(fotoIzq);
}

void obstaculosUpdate(){

  // Actualizar las variables globales que miden los infrarojos, ajustando para que un 0 sea un obstáculo no detectado
  inf3 = digitalRead(infroj3)==0; 
  inf1 = digitalRead(infroj1)==0;
  inf2 = digitalRead(infroj2)==0;
  inf4 = digitalRead(infroj4)==0;

}
