#include <Wire.h>
#include <LiquidCrystal_I2C.h> 
#include <ESP32Encoder.h>


ESP32Encoder encoder;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x3F, 16, 2);



String auswahl[6];
int pos;
int diff;
int threshold = 250;


#define CLK 36 // CLK ENCODER
#define DT 39 // DT ENCODER
#define BT 34 // Button

int sel = 0;
int i = 0;

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.noBlink();
  
  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);
  
  Serial.begin ( 115200 );
}

void loop() {

   
  
  delay(100); 
  
  auswahl[0] = "Scheinleistung"; // Erstes Wort wird hinzugefügt
  auswahl[1] = "Wirkleistung"; // Zweites Wort wird hinzugefügt
  auswahl[2] = "Blindleistung"; // Drittes Wort wird hinzugefügt
  auswahl[3] = "Stromverbrauch"; //
  auswahl[4] = "akt. Spannung"; // 
   auswahl[5] = "Phase U&A"; // 
  
 
    

     while(sel == 0){

      long oldPos = encoder.getCount();
      delay(100);
      
     if(encoder.getCount() < oldPos ){
      i++;
      lcd.setCursor(1,1);
      lcd.print("              ");
      Serial.println(i);
      if(i == 6){ i = 0;}
      }
     else if(oldPos < encoder.getCount()){
      i--;
      lcd.setCursor(1,1);
      lcd.print("              ");
      Serial.println(i);
      if(i == -1){ i = 2;}
      }
      
     lcd.setCursor(4, 0); 
     lcd.print("Auswahl:");
        
     lcd.setCursor(0, 1);
     lcd.print("<");
    
     lcd.setCursor(15, 1);
     lcd.print(">");

     
     pos = (16 - auswahl[i].length() ) / 2;
     lcd.setCursor(pos,1);
     lcd. print(auswahl[i]);

      if(analogRead(BT) == 0){sel = 1; lcd.clear();}
      
      }

      
     delay(100);
     
     lcd.setCursor(0,0);
     lcd. print(auswahl[i]);

     while(sel == 1) {

       if(analogRead(BT) == 0){sel = 0; lcd.clear();}

       switch(i) {
          case 0:
            lcd.setCursor(0, 1);
            lcd.print("... W");
            break;
          case 1:
            lcd.setCursor(0, 1);
            lcd.print("... W");
            break;
          case 2:
            lcd.setCursor(0, 1);
            lcd.print("... W");
            break;         
          case 3:
            lcd.setCursor(0, 1);
            lcd.print("... A");
            break;
          case 4:
            lcd.setCursor(0, 1);
            lcd.print("... V");
            break;
          case 5:
            lcd.setCursor(0, 1);
            lcd.print("... °");
            break;
            
          default:
            // führe Aktionen aus, wenn sensorValue keinen der obigen Werte hat
            break;
        }
       
      }
    

      
    

   

//  delay(300);
 
}
