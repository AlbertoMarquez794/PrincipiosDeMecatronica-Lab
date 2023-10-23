#define EncA 19
#define EncB 18


#define IN1 23  
#define IN2 22
#define ENA 21  


boolean dextrogiro=false;
volatile long pulses = 0;


void IRAM_ATTR PulsesCounter(){
  if (digitalRead(EncB) == HIGH){     // si B es HIGH, sentido horario
    pulses++  ;        // incrementa PULSES en 1
    dextrogiro =true;
  }
  else {          // si B es LOW, sentido anti horario
    pulses-- ;        // decrementa el PULSES en 1
    dextrogiro =false;
  }
}


void setup()
{
  pinMode(EncA, INPUT);
  pinMode(EncB, INPUT);
   pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
 
  attachInterrupt(digitalPinToInterrupt(EncA), PulsesCounter, RISING);
  Serial.begin(115200);
}


void loop() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);  // Velocidad máxima
  delay(3000);  // 3 segundos
  printValues();


 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(2000);  // 2 segundos
  Serial.print("detenido...");//


  // Gira levógiro
 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);  // Velocidad máximA
  delay(3000);  // 3 segundos
  printValues();


  // Detener
 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(2000);  // 2 segundos
  Serial.println("detenido...");//


}


void printValues(){
 
  if(dextrogiro)
  {
    Serial.print("Dextrogiro----");//antihorario
  }else
  {
     Serial.print("Levorigo----");//horario
  }
  Serial.print("Cantidad de vueltas: ");
  Serial.println((float)pulses/480);
  delay(10);
}
