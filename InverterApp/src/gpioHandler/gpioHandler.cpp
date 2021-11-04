#include "gpioHandler.h"
#include "TM1637TinyDisplay.h"
uint16_t speed = 0;


bool GPIOHandler::Init()
{
    // Configure all of our GPIOs
    #ifdef DISPLAYS_ENABLED
        // ALL OF OUR CODE
        TM1637TinyDisplay speedDisplay(22, 24), batteryDisplay(26,28), motorTempDisplay(30,32), coolantTempDisplay(34,36);
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

uint16_t GPIOHandler::GetPedalSpeed()
{
    speed++;
    return speed;
}