#define EnM 13           // Define el pin 13 como EnM
#define MA 14            // Define el pin 14 como MA
#define MB 12            // Define el pin 12 como MB
#define potenciometro 27 // Define el pin 27 como potenciometro

int channel = 0;        // Define una variable llamada "channel" y la inicializa en 0
int freq = 1000;        // Define una variable llamada "freq" y la inicializa en 1000
int resolution = 12;    // Define una variable llamada "resolution" y la inicializa en 12

void setup() {
  pinMode(EnM, OUTPUT);  // Configura el pin EnM como salida
  pinMode(MA, OUTPUT);   // Configura el pin MA como salida
  pinMode(MB, OUTPUT);   // Configura el pin MB como salida

  // Configuración para el PWM
  ledcSetup(channel, freq, resolution); // Configura el canal PWM
  ledcAttachPin(EnM, channel);          // Asocia el pin EnM al canal PWM
  Serial.begin(115200);                 // Inicializa la comunicación serial a 115200 baudios
}

void loop() {
  float val = analogRead(potenciometro);    // Lee el valor analógico del pin potenciometro y lo almacena en val
  float dirMotor = ((val * 3.3) / 4095);     // Convierte el valor analógico a voltaje y lo almacena en dirMotor
  Serial.println(val + String(" bits"));     // Imprime el valor leído del potenciómetro en bits
  Serial.println(dirMotor + String(" V"));   // Imprime el valor convertido a voltaje

  if (dirMotor >= 0 && dirMotor < 1.32) {
    ledcWrite(channel, 3000);               // Establece el valor PWM en 3000
    digitalWrite(MA, LOW);                 // Establece el pin MA en bajo
    digitalWrite(MB, HIGH);                // Establece el pin MB en alto
    Serial.println("levogiro");             // Imprime "levogiro"
    delay(500);                             // Espera 500 milisegundos
  } else if (dirMotor >= 1.32 && dirMotor < 1.98) {
    ledcWrite(channel, 0);                  // Apaga el PWM
    digitalWrite(MA, HIGH);                // Establece el pin MA en alto
    digitalWrite(MB, LOW);                 // Establece el pin MB en bajo
    Serial.println("detenido");             // Imprime "detenido"
    delay(500);                             // Espera 500 milisegundos
  } else if (dirMotor >= 1.98) {
    ledcWrite(channel, 3000);               // Establece el valor PWM en 3000
    digitalWrite(MA, HIGH);                // Establece el pin MA en alto
    digitalWrite(MB, LOW);                 // Establece el pin MB en bajo
    Serial.println("Dextrogiro");           // Imprime "Dextrogiro"
    delay(500);                             // Espera 500 milisegundos
  }
}
