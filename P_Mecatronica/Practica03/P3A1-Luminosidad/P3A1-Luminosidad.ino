#define ldrPin 15  // Pin analógico donde está conectado el LDR
#define ledPin 4   // Pin digital donde está conectado el LED
int channel =0;     // Canal para el control PWM del LED
int freq=1000;      // Frecuencia del PWM
int resolution=8;   // Resolución del PWM

void setup() {
  // Configura el LED para usar PWM
  ledcSetup(channel, freq, resolution);
  // Asocia el pin digital al canal PWM
  ledcAttachPin(ledPin, channel);
  // Inicializa la comunicación serial para la depuración (opcional)
  Serial.begin(9600);
}

void loop() 
{
  int ldrValue = analogRead(ldrPin); // Lee el valor analógico del LDR
  int brightness = map(ldrValue, 1000, 0, 0, 255); // Mapea el valor leído al rango de brillo del LED
  ledcWrite(channel, brightness); // Configura el brillo del LED
  
  // Imprime el valor del LDR en el monitor serial (opcional)
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  
  delay(200); // Pequeña pausa para estabilizar la lectura
}
#define ldrPin 15  // Pin analógico donde está conectado el LDR
#define ledPin 4   // Pin digital donde está conectado el LED
int channel =0;     // Canal para el control PWM del LED
int freq=1000;      // Frecuencia del PWM
int resolution=8;   // Resolución del PWM

void setup() {
  // Configura el LED para usar PWM
  ledcSetup(channel, freq, resolution);
  // Asocia el pin digital al canal PWM
  ledcAttachPin(ledPin, channel);
  // Inicializa la comunicación serial para la depuración (opcional)
  Serial.begin(9600);
}

void loop() 
{
  int ldrValue = analogRead(ldrPin); // Lee el valor analógico del LDR
  int brightness = map(ldrValue, 1000, 0, 0, 255); // Mapea el valor leído al rango de brillo del LED
  ledcWrite(channel, brightness); // Configura el brillo del LED
  
  // Imprime el valor del LDR en el monitor serial (opcional)
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  
  delay(200); // Pequeña pausa para estabilizar la lectura
}
