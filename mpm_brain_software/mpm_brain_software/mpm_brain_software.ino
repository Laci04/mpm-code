#include <Wire.h>
#include <LiquidCrystal_I2C.h> 

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x3F, 16, 2);

String auswahl[6];
int pos;
int diff;
int threshold = 250;

//Statemachine
enum State {STATE_IDLE,STATE_COUNTING};
State currentState = STATE_IDLE;

//Joystick Pins
const int VRxPin = 39;    // Joystick X-Achse Pin
const int VRyPin = 0;    // Joystick Y-Achse Pin
const int SWPin = 36; 


int sel = 0;
int i = 0;

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.noBlink();
  Serial.begin(9600);
}

void loop() {

  int VRxVal = analogRead(VRxPin);   // Joystick X-Achse auslesen
  int VRyVal = analogRead(VRyPin);   // Joystick Y-Achse auslesen
  int SWVal = digitalRead(SWPin);    // Joystick Schalter auslesen


  delay(100); 
  
  auswahl[0] = "Scheinleistung"; // Erstes Wort wird hinzugefügt
  auswahl[1] = "Wirkleistung"; // Zweites Wort wird hinzugefügt
  auswahl[2] = "Blindleistung"; // Drittes Wort wird hinzugefügt
  auswahl[3] = "Stromverbrauch";
  auswahl[4] = "akt. Spannung";
   auswahl[5] = "Phase U&A";
  
 


     while(sel == 0){

      diff = abs(analogRead(VRxPin) - 2502); 
      
     if(2502 < analogRead(VRxPin)&& diff > threshold ){
      i++;
      lcd.setCursor(1,1);
      lcd.print("              ");
      Serial.println(i);
      if(i == 6){ i = 0;}
      
      }
     else if(2502 > analogRead(VRxPin) && diff > threshold ){
      i--;
      lcd.setCursor(1,1);
      lcd.print("              ");
      Serial.println(i);
      if(i == -1){ i = 2;}
      }
      
     VRxVal = analogRead(VRxPin); 
     lcd.setCursor(4, 0); 
     lcd.print("Auswahl:");
        
     lcd.setCursor(0, 1);
     lcd.print("<");
    
     lcd.setCursor(15, 1);
     lcd.print(">");

     
     pos = (16 - auswahl[i].length() ) / 2;
     lcd.setCursor(pos,1);
     lcd. print(auswahl[i]);

      Serial.println(analogRead(SWPin));

      if(analogRead(SWPin) == 0){sel = 1; lcd.clear();}
      
      }

      
     delay(100);
     
     lcd.setCursor(0,0);
     lcd. print(auswahl[i]);
     
     if(analogRead(SWPin) == 0){sel = 0; lcd.clear();}

      


   

//  delay(300);
 
}
