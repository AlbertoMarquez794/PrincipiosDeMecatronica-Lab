#define LEDRS1 23
#define LEDAS1 22
#define LEDVS1 1
#define LEDRS2 4
#define LEDAS2 2
#define LEDVS2 15

void setup() {
  //Para todo este bloque asignamos que nuestros leds van a ser salidad. 
  pinMode (LEDRS1, OUTPUT); 
  pinMode (LEDAS1, OUTPUT);
  pinMode (LEDVS1, OUTPUT);
  pinMode (LEDRS2, OUTPUT);
  pinMode (LEDAS2, OUTPUT);
  pinMode (LEDVS2, OUTPUT);

}

void loop() {
   //Prendemos el led rojo de nuestro semaforo 1 y prendemos el led verde del semaforo 2
   digitalWrite(LEDRS1, HIGH);
   digitalWrite(LEDVS2, HIGH);
   delay(5000); //Tiempo de espera del semaforo 2
   digitalWrite(LEDRS1, LOW); //Apagamos los leds que ya no utilizaremos
   digitalWrite(LEDVS2, LOW); 
   //Cambio de luces para el semaforo 2, y luces rojas para el semaforo 1
   digitalWrite(LEDRS1, HIGH);
   digitalWrite(LEDAS2, HIGH);
   delay(1000);
   digitalWrite(LEDRS1, LOW);//Apagamos los leds que ya no utilizaremos
   digitalWrite(LEDAS2, LOW);
   //Cambio de luces para el semaforo 2 en verde, y luces rojas para el semaforo 1
   digitalWrite(LEDVS1, HIGH);
   digitalWrite(LEDRS2, HIGH);
   delay(5000);
   digitalWrite(LEDVS1, LOW);//Apagamos los leds que ya no utilizaremos
   digitalWrite(LEDRS2, LOW);
   //Cambio de luces para el semaforo 1, y luces rojas para el semaforo 2
   digitalWrite(LEDAS1, HIGH);
   digitalWrite(LEDRS2, HIGH);
   delay(1000);   
   digitalWrite(LEDAS1, LOW);//Apagamos los leds que ya no utilizaremos
   digitalWrite(LEDRS2, LOW);
   
   }
