#define EnM 13
#define MA 14
#define MB 12
#define potenciometro  27




int channel=0;
int freq=1000;
int resolution=12;


void setup() {
  // put your setup code here, to run once:
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
  Serial.println(val+String(" bits"));
  Serial.println(dirMotor+ String(" V"));


  if(dirMotor>=0 && dirMotor<1.32)
  {
    ledcWrite(channel,3000);
    digitalWrite(MA,LOW);
    digitalWrite(MB,HIGH);
    Serial.println("levogiro");
    delay(500);
  }else if(dirMotor>=1.32 && dirMotor<1.98)
  {
    ledcWrite(channel,0000);
    digitalWrite(MA,HIGH);
    digitalWrite(MB,LOW);
    Serial.println("detenido");
    delay(500);
  }else if(dirMotor>=1.98)
  {  
    ledcWrite(channel,3000);
    digitalWrite(MA,HIGH);
    digitalWrite(MB,LOW);
    Serial.println("Dextrogiro");
    delay(500);
  }
