#include "gpioHandler.h"

bool GPIOHandler::Init()
{
    // Configure all of our GPIOs
    return true;
}

uint16_t GPIOHandler::GetPedalSpeed()
{
    int potent_read = analogRead(POT_GPIO);
    return map(potent_read, 0, 1023, SPEED_OFFSET + SPEED_MIN_RPM, SPEED_OFFSET + SPEED_MAX_RPM);;
}