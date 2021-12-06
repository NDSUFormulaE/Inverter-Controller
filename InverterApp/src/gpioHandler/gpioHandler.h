#include <stdlib.h>
#include <TimeLib.h>
#include <Arduino.h>

#define POT_GPIO           2
#define SPEED_OFFSET   32000
#define SPEED_MIN_RPM      0
#define SPEED_MAX_RPM    200

class GPIOHandler
{
    public:
        uint16_t GetPedalSpeed();
        bool Init();
    private:
        uint16_t speed;
};