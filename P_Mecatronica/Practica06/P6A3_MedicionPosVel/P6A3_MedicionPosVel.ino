#include <Arduino.h>
#define IN1 23
#define IN2 22
#define ENA 21
#define ENC_A 18
#define ENC_B 19
#define PPR 480 // Pulsos por revolución resultado de dividir 1920/4 que es el total de cuentas por revolción en los estados que cambia
                    // , a nosotros solo nos interesa 1 de 4 que es el rising
volatile int pulsos = 0;
volatile unsigned muestreoActualInterrupcionR = 0;        // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor derecho
volatile unsigned muestreoAnteriorInterrupcionR = 0;
volatile unsigned deltaMuestreoInterrupcionR = 0;
int N = 64;//ranuras del encode lo encontramos como resolucion del encoder 
double frecuenciaR = 0;                                  // frecuencia de interrupciòn llanta R
double Wr = 0;   

void REncoder() // funciòn de interrupciòn del enconder cada que da un rising en A
{               
  pulsos++;                                                                     
  deltaMuestreoInterrupcionR = muestreoActualInterrupcionR -  muestreoAnteriorInterrupcionR;     // diferencia tiempos de interruciones de ticks del motor     
  muestreoAnteriorInterrupcionR = muestreoActualInterrupcionR;                                   // se actualiza el tiempo de interrupciòn anterior
  frecuenciaR = (1000)/(double) deltaMuestreoInterrupcionR;                                       // frecuencia de interrupciòn 
  Wr = ((2*3.141516)/N)*frecuenciaR;                                                              // frecuencia angular Rad/s                                                           
}   

void setup() {
  Serial.begin(115200);//inicia serial 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), REncoder, RISING);//declaracion de interrrupcion cada que lee el rising del sensor de A del encoder 

}

void loop() {
  // Ajuste de la velocidad del motor en distintos porcentajes
  int velocidades[] = {26, 77, 128, 255, 0}; // aproximaciones de 10%, 30%, 50%, y 100% de 255 (PWM)
  for (int i = 0; i < 5; i++)
  {
  analogWrite(ENA, velocidades[i]);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);//solo se manda un high en in2 porque ya sabemos que va a girar en un sentido
    
    muestreoActualInterrupcionR = millis();//le pasamos el tiempo actual en milis para que sea el comparador 
    while (millis() - muestreoActualInterrupcionR < 3000)//ciclo para que repita la rutina cada 3 segundos, ya que milis lo da en milisegundos a diferencia de micros 
    {
      if (millis() - muestreoAnteriorInterrupcionR >= 100) //condicion para que imprima y vuelva a leer datos cada 100 milisegundos 
      {
        REncoder();//funcion de read encoder 
        mostrarDatos();//mando a mostrar datos
        muestreoActualInterrupcionR=muestreoAnteriorInterrupcionR;//actualizo el tiempo actual con el anterior
      }
    }
    pulsos=0;//reinicio los pulsos para que no se vayan acumulando con cada variacion del pwm
  }
}
void mostrarDatos() {
  float angulo = (float(pulsos) / PPR) * 360.0;  // Cálculo del ángulo
  angulo = fmod(angulo, 360.0);  // Para mantener el ángulo entre 0 y 359 grados
  float w_rps = float(pulsos) / (PPR * 0.1); // Cálculo de velocidad en rps (cada 100 ms = 0.1 s)
  float w_rpm = w_rps * 60.0;    // Cálculo de velocidad en rpm
  float w_rad = Wr;  // Cálculo de velocidad en rad/s

  Serial.print("pulsos: ");
  Serial.print(pulsos);
  Serial.print(" --- angulo: ");
  Serial.print(angulo);
  Serial.print("°");
  Serial.print(" --- W: ");
  Serial.print(w_rps);
  Serial.print(" [rps]");
  Serial.print(" --- ");
  Serial.print(w_rpm);
  Serial.print(" [rpm]");
  Serial.print(" --- ");
  Serial.print(w_rad);
  Serial.println(" [rad/s]");
}
