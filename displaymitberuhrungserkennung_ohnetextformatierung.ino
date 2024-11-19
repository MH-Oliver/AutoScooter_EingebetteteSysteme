// Test for minimum program size.

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);


// LED an Pin 7, Taster an Pin 6
int beruhrungskabel = 13; 
int leben = 100;  
long letzteberuhrung = 0;

//------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  Wire.setClock(400000L);




#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

oled.setFont(Callibri15);
display.setTextSize(3);
display.setCursor(64, 32); // funktioniert noch nicht
display.setTextColor(SSD1306_WHITE); // Textfarbe einstellen

oled.print(leben);

  // der Taster wird als INPUT (Eingang) deklariert
  pinMode(beruhrungskabel, INPUT);  
  Serial.begin(115200);
}




//------------------------------------------------------------------------------
void loop() {

  if (digitalRead(beruhrungskabel) == 1 && millis() - letzteberuhrung > 3000) {
    letzteberuhrung = millis();
    //leben abziehen
    leben -= 10;
    oled.clear(); 
    oled.print(leben);
  } 
  
}






















