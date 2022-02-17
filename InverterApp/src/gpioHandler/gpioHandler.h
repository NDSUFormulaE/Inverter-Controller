#include <stdlib.h>
#include "../Time/TimeLib.h"
#include <Arduino.h>
#include "../TaskScheduler/TaskScheduler.h"

#define PWM_PINLOW 5
#define PWM_PINHIGH 6

struct PWM_RETURN {
    uint8_t duty_cycle_low;
    uint8_t duty_cycle_high;
    uint8_t freq_low;
    uint8_t freq_high;
};

class GPIOHandler
{
    public:
        bool Init(void);
        PWM_RETURN GetPWM();
    private:
};