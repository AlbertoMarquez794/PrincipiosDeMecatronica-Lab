#define LED 5
void setup() {
  pinMode(LED,OUTPUT);
}

void loop() {
  digitalWrite(LED,HIGH);//pin prende con high, es un 1 
  delay(1000); //en milisegundos 
  digitalWrite(LED,LOW); //pin apaga con low, es un 0
  delay(5000); //milisgundos para el apagado

}
 
