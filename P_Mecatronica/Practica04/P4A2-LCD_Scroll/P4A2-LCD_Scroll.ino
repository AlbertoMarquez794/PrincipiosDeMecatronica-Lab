#include <Wire.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);


char nombre1[] = "Mingyar";
char nombre2[] = "Brayan ";


void setup() {
  lcd.init();
  lcd.backlight(); //lcd.nobacklight();
}
 
void loop()
{
  // Desplazar el primer nombre desde la derecha hacia la izquierda
  for (int i=16; i >= -7;i--) {  // 16 caracteres en la pantalla
    lcd.clear();
    lcd.setCursor(i, 0);
    lcd.print(nombre1);
    lcd.setCursor(i, 1);
    lcd.print(nombre2);
    delay(150);
  }
  delay(1000);
  // Borrar la pantalla y repetir el ciclo
  lcd.clear();
}
