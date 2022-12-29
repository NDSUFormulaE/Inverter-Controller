#include "gpioHandler.h"
#include "../TM1637/TM1637Display.h"
uint16_t speed = 0;


bool GPIOHandler::Init()
{
    // Configure all of our GPIOs
    #ifdef DISPLAYS_ENABLED
        // ALL OF OUR CODE
        TM1637TinyDisplay speedDisplay(SPD_CLK, SPD_DATA), batteryDisplay(BATT_CLK,BATT_DATA), motorTempDisplay(TEMP_CLK,TEMP_DATA), coolantTempDisplay(COOL_CLK,COOL_DATA);
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
}
unsigned long last_clear = 0;

uint16_t GPIOHandler::GetPedalSpeed()
{
    int potent_read = analogRead(POT_GPIO);
    return map(potent_read, 0, 1023, SPEED_OFFSET + SPEED_MIN_RPM, SPEED_OFFSET + SPEED_MAX_RPM);;
}

uint16_t GPIOHandler::GetClearPin()
{
    uint16_t analogReadVal = analogRead(CLEAR_FAULT_GPIO);
    if(analogReadVal > 500 && ((millis() - last_clear) > CLEAR_INTERVAL_MILLIS))
    {
        last_clear = millis();
        return analogReadVal;
    }
    return 0;
}