#include "gpioHandler.h"

uint16_t speed = 0;

bool GPIOHandler::Init()
{
    // Configure all of our GPIOs
}

uint16_t GPIOHandler::GetPedalSpeed()
{
    speed++;
    return speed;
}