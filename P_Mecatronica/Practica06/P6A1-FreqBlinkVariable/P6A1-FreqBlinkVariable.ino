#define Rojo 17
#define Amarillo 16
#define Verde 4
#define Button1 23
#define Button2 22


int factor = 1;
int tiempo = 20;
int buttonstate;


void IRAM_ATTR ISR1()
{
  Serial.println("Botón 1 presionado");
  factor++;
}
void IRAM_ATTR ISR2()
{
  Serial.println("Botón 2 presionado");
  factor--;
}
void setup() {
  pinMode(Rojo, OUTPUT);
  pinMode(Amarillo, OUTPUT);
  pinMode(Verde, OUTPUT);
  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);
  attachInterrupt(digitalPinToInterrupt(Button1), ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(Button2), ISR2, RISING);
  Serial.begin(115200);
}


void loop() {
  digitalWrite(Rojo, LOW);
  digitalWrite(Verde, HIGH);
  delay(factor*tiempo);
  digitalWrite(Verde, LOW);
  digitalWrite(Amarillo, HIGH);
  delay(factor*tiempo);
  digitalWrite(Amarillo, LOW);
  digitalWrite(Rojo, HIGH);
  delay(factor*tiempo);


  Serial.println(factor*tiempo);
}
