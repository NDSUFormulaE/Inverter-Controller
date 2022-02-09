#include "gpioHandler.h"
#include "TM1637TinyDisplay.h"
#include "lcd.h"
uint16_t speed = 0;


bool GPIOHandler::Init()
{
    // Configure all of our GPIOs
    #ifdef DISPLAYS_ENABLED
        // ALL OF OUR CODE
        TM1637TinyDisplay speedDisplay(22, 24), batteryDisplay(26,28), motorTempDisplay(30,32), coolantTempDisplay(34,36);
        LiquidCrystal_I2C lcd(0x27,20,4);
        speedDisplay.setBrightness(7);
        batteryDisplay.setBrightness(7);
        coolantTempDisplay.setBrightness(7);
        motorTempDisplay.setBrightness(7);
        speedDisplay.showString("NDSU SAE");
        speedDisplay.showNumber(00.00);
        batteryDisplay.showNumber(000.0);
        coolantTempDisplay.showNumber(000.0);
        motorTempDisplay.showNumber(000.0);

    #else
        return true;
    #endif

unsigned long last_clear = 0;

bool GPIOHandler::LcdInit()
{
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
}
uint16_t GPIOHandler::GetPedalSpeed()
{
    int potent_read = analogRead(POT_GPIO);
    return map(potent_read, 0, 1023, SPEED_OFFSET + SPEED_MIN_RPM, SPEED_OFFSET + SPEED_MAX_RPM);;
}

uint8_t GPIOHandler::GetClearPin()
{
    if(analogRead(CLEAR_FAULT_GPIO) > 900 && ((millis() - last_clear) > CLEAR_INTERVAL_MILLIS))
    {
        last_clear = millis();
        return 1;
    }
    return 0;
}
void GPIOHandler::LCDDisplaySAE()
{
    lcd.display();
    delay(3000);
    lcd.noDisplay();
}