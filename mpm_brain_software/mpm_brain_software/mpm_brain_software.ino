#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ENCODER_A_PIN 18
#define ENCODER_B_PIN 5
#define BUTTON_PIN 34
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4

volatile int encoderPosition = 0;
volatile bool encoderAPrevState = false;
volatile bool encoderBPrevState = false;
volatile bool buttonState = false;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

void ICACHE_RAM_ATTR encoderAISR() {
  bool state = digitalRead(ENCODER_A_PIN);
  if (state != encoderAPrevState) {
    encoderAPrevState = state;
    if (state && !encoderBPrevState) {
      encoderPosition++;
    } else if (!state && encoderBPrevState) {
      encoderPosition--;
    }
  }
}

void ICACHE_RAM_ATTR encoderBISR() {
  bool state = digitalRead(ENCODER_B_PIN);
  if (state != encoderBPrevState) {
    encoderBPrevState = state;
    if (state && !encoderAPrevState) {
      encoderPosition--;
    } else if (!state && encoderAPrevState) {
      encoderPosition++;
    }
  }
}

void ICACHE_RAM_ATTR buttonISR() {
  buttonState = !digitalRead(BUTTON_PIN);
}

void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderBISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);
}

void loop() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Encoder Position:");
  lcd.setCursor(0,1);
  lcd.print(encoderPosition);
  lcd.setCursor(10,1);
  lcd.print(buttonState ? "Active" : "Inactive");
  delay(1000);
}
