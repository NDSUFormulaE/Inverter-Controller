#include "gpioHandler.h"

unsigned long last_clear = 0;

bool GPIOHandler::Init(void)
{
    return true;
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