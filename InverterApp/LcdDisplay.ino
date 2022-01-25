#include <Wire.h>
#include <LiquidCrystal_I2C.h> 
#include <TM1637TinyDisplay.h>
#include "gpioHandler.h"


//Created new object that connected to the Arduino board.
LiquidCrystal_I2C lcd(0x27,20,4);     
//The byte arrays were created using the website https://maxpromer.github.io/LCD-Character-Creator/
byte fullBox[] = 
{
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte halfBoxBottom[] =
{
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111  
};
byte halfBoxTop[] =
{
  B11111,
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000  
};

byte topLeftAChar[] = {
  B00001,
  B00011,
  B00111,
  B01111,
  B01111,
  B11111,
  B11111,
  B11111
};
byte sideLeftAChar[] = {
  B00001,
  B00011,
  B00111,
  B01111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte sideLeftAInnerChar[] = {
  B11111,
  B11111,
  B11110,
  B11110,
  B11100,
  B11000,
  B10000,
  B10000
};
byte sideLeftInnerCharConnector[] = {
  B11111,
  B11111,
  B11110,
  B11100,
  B11111,
  B11111,
  B11111,
  B11111
};
byte topRightSCurve[] = {
  B10000,
  B11000,
  B11100,
  B11110,
  B11111,
  B11111,
  B11111,
  B11111
};
byte topLeftSCurve[] = {
  B00001,
  B00011,
  B00111,
  B01111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte bottomLeftSCurve[] = {
  B00000,
  B00000,
  B11100,
  B11110,
  B11110,
  B01111,
  B01111,
  B00111
};

void setup()

{       
  GPIOHandler gpioMan;
  gpioMan.Init();
   
  //Initialized screen and backlight.
  lcd.init();
  lcd.backlight();
  
  //Characters used to create the logo
  lcd.begin(20, 4);
  lcd.createChar(0, fullBox);
  lcd.createChar(1, halfBoxBottom);
  lcd.createChar(2, halfBoxTop);
  lcd.createChar(3, topLeftAChar);
  lcd.createChar(4, sideLeftAChar);
  lcd.createChar(5, sideLeftAInnerChar);
  lcd.createChar(6, sideLeftInnerCharConnector);
  lcd.createChar(7, topRightSCurve);
  //Creates Letter "E"
  int y1 = 1;
  for(int x = 13; x<18; x++)
  {
    lcd.setCursor(x,0);
    lcd.write(0);  
    lcd.setCursor(x-3,0);
    lcd.write(0);
    lcd.setCursor(13,y1);
    lcd.write(0);
    lcd.setCursor(x,3);
    lcd.write(0);
    lcd.setCursor(12,y1);
    lcd.write(0);
    y1++;
  }


  //Creates Letter A
   for(int x = 14; x<18; x++)
  {
    lcd.setCursor(x,1);
    lcd.write(1);
    lcd.setCursor(x,2);
    lcd.write(2);  
  }

  lcd.setCursor(9,0);
  lcd.write(3);
  lcd.setCursor(8,1);
  lcd.write(4);
  lcd.setCursor(7,2);
  lcd.write(4);
  lcd.setCursor(6,3);
  lcd.write(0);
  lcd.setCursor(9,1);
  lcd.write(5);
  lcd.setCursor(7,3);
  lcd.write(5);
  lcd.setCursor(8,2);
  lcd.write(0);

  for(int x = 9; x<12; x++)
  {
    lcd.setCursor(x,2);
    lcd.write(0);  
  }

  //Creates Letter S
  for(int x  = 2; x<6; x++)
  {
      lcd.setCursor(x,0);
      lcd.write(0);
  }
  lcd.setCursor(6,0);
  lcd.write(7);
  lcd.setCursor(6,1);
  lcd.write(2);
  lcd.setCursor(1,0);
  lcd.write(3);
  lcd.setCursor(1,1);
  lcd.write(0);
  lcd.setCursor(1,2);
  lcd.write(2);

  for(int x = 2; x<6; x++)
  {
    lcd.setCursor(x,2);
    lcd.write(2);
  }
  lcd.setCursor(6,2);
  lcd.write(7);
  lcd.setCursor(1,3);
  lcd.write(6);

  for(int x = 2; x<6;x++)
  {
    lcd.setCursor(x,3);
    lcd.write(1);  
  }
  lcd.display();
  delay(3000);
  lcd.noDisplay();
}


void loop()
{
  
}