#include <stdlib.h>
#include <TimeLib.h>
#include <Arduino.h>


class GPIOHandler
{
    public:
        uint16_t GetPedalSpeed();
        bool Init();
    private:
        uint16_t speed;
};