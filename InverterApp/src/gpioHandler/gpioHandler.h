#ifndef GPIO_HANDLER_H
#define GPIO_HANDLER_H

#include <stdlib.h>
#include "../Time/TimeLib.h"
#include <Arduino.h>
#include "../FreeRTOS/src/Arduino_FreeRTOS.h"
#include "../FreeRTOS/src/semphr.h"  // Add this for SemaphoreHandle_t
// USE_APPS defined in powerMode.h
#include "../TaskScheduler/TaskScheduler.h"
#include "powerMode.h"

#define POT_GPIO           A1
#define LEFT_APPS_GPIO     A2
#define RIGHT_APPS_GPIO    A3

#define CLEAR_FAULT_GPIO   A5

//minimum voltage 0.206V
#define LEFT_APPS_MIN_ADC 63
//Maximum voltage 1.157V
#define LEFT_APPS_MAX_ADC 358

//minimum voltage 0.492V
#define RIGHT_APPS_MIN_ADC 152
//Maximum voltage 1.496V
#define RIGHT_APPS_MAX_ADC 494

#define SPEED_OFFSET   16000
#define SPEED_MIN_RPM      0
#define SPEED_MAX_RPM      600

#define MIN_SPEED_VAL (SPEED_OFFSET + SPEED_MIN_RPM) * 2
#define MAX_SPEED_VAL (SPEED_OFFSET + SPEED_MAX_RPM) * 2

// 0% -> 0.7%
#define TORQUE_OFFSET      16000
#define TORQUE_PERC_MIN    0
#define TORQUE_PERC_MAX    5

#define MIN_TORQUE_VAL (TORQUE_OFFSET + TORQUE_PERC_MIN) * 2
#define MAX_TORQUE_VAL (TORQUE_OFFSET + TORQUE_PERC_MAX) * 2

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
    private:
        uint16_t speed;
        char displayBuffer[4][20];  // Current display content
        char newBuffer[4][20];      // New content to be displayed
        bool needsUpdate;           // Flag to track if update is needed
        SemaphoreHandle_t lcdMutex; // Protect LCD access
        bool LcdInit();
        void LCDDisplaySAE();
        void LcdUpdate();
    public:
        bool Init(void);
        uint16_t GetPedalSpeed();
        uint16_t GetClearPin();
        uint16_t GetPedalTorque();
        void UpdateLCDs();
        void UpdateSevenSegments();
        void UpdateDisplayText(const char* text, uint8_t row, uint8_t col);
};

#endif /* GPIO_HANDLER_H */