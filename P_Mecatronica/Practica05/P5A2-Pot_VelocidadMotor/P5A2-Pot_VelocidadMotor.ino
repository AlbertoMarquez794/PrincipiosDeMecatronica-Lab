#define EnM 13
#define MA 14
#define MB 12
#define potenciometro  27


int channel=0;
int freq=1000;
int resolution=12;


void setup() {
pinMode(EnM,OUTPUT);
pinMode(MA,OUTPUT);
pinMode(MB,OUTPUT);
//para el pwm
ledcSetup(channel,freq,resolution);
ledcAttachPin(EnM,channel);
Serial.begin(115200);
}


void loop()
 {
  float val = analogRead(potenciometro);
  float dirMotor=((val * 3.3) / 4095);
  //Serial.println(val+String(" bits"));
  float dirMotorAumentada=dirMotor*100;
  if(dirMotor>=0 && dirMotor<1.32)
  {
    int min=map(dirMotorAumentada,0,132,4095,0);
    // Serial.println(min + String(" min")); imprime en bits lo que se le manda al motor
    ledcWrite(channel,min);
    digitalWrite(MA,LOW);
    digitalWrite(MB,HIGH);
    Serial.println("Estado: levogiro");
    delay(500);
    float porcentaje=map(dirMotorAumentada,0,132,100,0);
    Serial.println(porcentaje+ String(" %"));
  }else if(dirMotor>=1.32 && dirMotor<1.98)
  {
    ledcWrite(channel,0000);
    digitalWrite(MA,HIGH);
    digitalWrite(MB,LOW);
    Serial.println("Estado: detenido");
    delay(500);
    Serial.println("0 %");
  }else if(dirMotor>=1.98)
  {  
    int max=map(dirMotorAumentada,198,330,0,4095);
   // Serial.println(max+ String(" max")); imprime en bits lo que se le manda al motor
    ledcWrite(channel,max);
    digitalWrite(MA,HIGH);
    digitalWrite(MB,LOW);
    Serial.println("Estado: Dextrogiro");
    delay(500);
    float porcentaje1=map(dirMotorAumentada,198,330,0,100);
    Serial.println(porcentaje1+ String(" %"));
  }
  Serial.println(dirMotor+ String(" V"));
}
