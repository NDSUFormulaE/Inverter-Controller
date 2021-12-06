#include <stdlib.h>
#include <TimeLib.h>
#include <Arduino.h>
#include "../TaskScheduler/TaskScheduler.h"

#define POT_GPIO           2
#define CLEAR_FAULT_GPIO   A5

#define SPEED_OFFSET   32000
#define SPEED_MIN_RPM      0
#define SPEED_MAX_RPM    200

#define CLEAR_INTERVAL_MILLIS 5000

class GPIOHandler
{
    public:
        bool Init(void);
        uint16_t GetPedalSpeed();
        uint8_t GetClearPin();
    private:
        uint16_t speed;
};