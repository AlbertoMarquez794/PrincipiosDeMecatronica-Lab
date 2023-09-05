#define Pot 4  // Definición del pin analógico conectado al potenciómetro
#define LED 15  // Definición del pin digital conectado al LED
int val;  // Variable para almacenar el valor leído desde el potenciómetro

void setup() {
   Serial.begin(115200); // Inicialización de la comunicación serial a 115200 baudios
   pinMode(LED, OUTPUT); // Configuración del pin LED como salida
}

void loop() {
  val = analogRead(Pot); // Lectura del valor analógico desde el potenciómetro
  Serial.print("ADC=");
  Serial.println(val); // Impresión del valor leído en la comunicación serial
  Serial.print("----Voltaje:");
  Serial.println(((val * 3.3) / 4095)); // Cálculo y impresión del voltaje correspondiente al valor leído
  delay(100); // Espera de 100 milisegundos

  if (((val * 3.3) / 4095) >= 2) {  // Comprueba si el voltaje es mayor o igual a 2V
    digitalWrite(LED, HIGH); // Enciende el LED
  } else {
    digitalWrite(LED, LOW); // Apaga el LED
  }
}
