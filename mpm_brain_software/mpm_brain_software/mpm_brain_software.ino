#include <LiquidCrystal_I2C.h>

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x3F, lcdColumns, lcdRows);  

void setup(){
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
}

void loop(){
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("Verbindung wird");
  lcd.setCursor(0, 1);
  lcd.print("hergestellt.");
  delay(150);
    lcd.setCursor(0, 0);
  lcd.print("Verbindung wird");
  lcd.setCursor(0, 1);
  lcd.print("hergestellt..");
  delay(150);
    lcd.setCursor(0, 0);
  lcd.print("Verbindung wird");
  lcd.setCursor(0, 1);
  lcd.print("hergestellt...");
  delay(150);
  lcd.setCursor(0, 0);
  lcd.print("Verbindung wird");
  lcd.setCursor(0, 1);
  lcd.print("hergestellt.");
  delay(150);
    lcd.setCursor(0, 0);
  lcd.print("Verbindung wird");
  lcd.setCursor(0, 1);
  lcd.print("hergestellt..");
  delay(150);
    lcd.setCursor(0, 0);
  lcd.print("Verbindung wird");
  lcd.setCursor(0, 1);
  lcd.print("hergestellt...");
   delay(2000);
  lcd.clear();
  // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Smart Power Measurement!");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("Modus wählen:");
  delay(2000);
  lcd.clear(); 
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print("M1: Wirkleistungsverbrauch");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("3.2 kWh");
  delay(2000);
  lcd.clear();
}
