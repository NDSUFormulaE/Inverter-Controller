#include <stdlib.h>
#include <TimeLib.h>
#include <Arduino.h>


#define DISPLAYS_ENABLED

#define SPD_DISPLAY_CLK 0

class GPIOHandler
{
    public:
        uint16_t GetPedalSpeed();
        bool Init();
    private:
        uint16_t speed;
};