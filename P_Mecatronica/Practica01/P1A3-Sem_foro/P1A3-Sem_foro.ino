#define LEDRS1 23
#define LEDAS1 22
#define LEDVS1 1
#define LEDRS2 4
#define LEDAS2 2
#define LEDVS2 15

void setup() {
  pinMode (LEDRS1, OUTPUT);
  pinMode (LEDAS1, OUTPUT);
  pinMode (LEDVS1, OUTPUT);
  pinMode (LEDRS2, OUTPUT);
  pinMode (LEDAS2, OUTPUT);
  pinMode (LEDVS2, OUTPUT);

}

void loop() {
   digitalWrite(LEDRS1, HIGH);
   digitalWrite(LEDVS2, HIGH);
   delay(5000);
   digitalWrite(LEDRS1, LOW);
   digitalWrite(LEDVS2, LOW);

   digitalWrite(LEDRS1, HIGH);
   digitalWrite(LEDAS2, HIGH);
   delay(1000);
   digitalWrite(LEDRS1, LOW);
   digitalWrite(LEDAS2, LOW);
   
   digitalWrite(LEDVS1, HIGH);
   digitalWrite(LEDRS2, HIGH);
   delay(5000);
   digitalWrite(LEDVS1, LOW);
   digitalWrite(LEDRS2, LOW);
   
   digitalWrite(LEDAS1, HIGH);
   digitalWrite(LEDRS2, HIGH);
   delay(1000);   
   digitalWrite(LEDAS1, LOW);
   digitalWrite(LEDRS2, LOW);
   
   }
