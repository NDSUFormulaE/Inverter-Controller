#include <stdlib.h>
#include "../Time/TimeLib.h"
#include <Arduino.h>
#include "../TaskScheduler/TaskScheduler.h"

#define POT_GPIO           2
#define CLEAR_FAULT_GPIO   A5

#define SPEED_OFFSET   32000
#define SPEED_MIN_RPM      0
#define SPEED_MAX_RPM      750

// 0% -> 0.7%
#define TORQUE_OFFSET      32000
#define TORQUE_PERC_MIN    0
#define TORQUE_PERC_MAX    180

#define CLEAR_INTERVAL_MILLIS 5000

//If displays connected, uncomment next line.
#define DISPLAYS_ENABLED

// Speed Display Pins
#define SPD_CLK 22
#define SPD_DATA 24

// Battery Voltage Pins
#define BATT_CLK 26
#define BATT_DATA 28

// Motor Temperature Pins
#define TEMP_CLK 30
#define TEMP_DATA 32

// Coolant Temperature Pins
#define COOL_CLK 34
#define COOL_DATA 36

class GPIOHandler
{
    public:
        bool Init(void);
        uint16_t GetPedalSpeed();
        uint16_t GetClearPin();
        uint16_t GetPedalTorque();
        void UpdateDisplays();
    private:
        uint16_t speed;
        bool LcdInit();
        void LCDDisplaySAE();
};