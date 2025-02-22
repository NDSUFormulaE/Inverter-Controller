#ifndef GPIO_HANDLER_H
#define GPIO_HANDLER_H

#include <stdlib.h>
#include <Arduino.h>
#include "..//FreeRTOS/src/Arduino_FreeRTOS.h"
#include "../TaskScheduler/TaskScheduler.h"
#include "../AppConfig.h"
class GPIOHandler
{
    public:
        bool Init(void);
        uint16_t GetPedalSpeed();
        uint16_t GetClearPin();
        uint16_t GetPedalTorque();
        void UpdateLCDs();
        void UpdateSevenSegments();
    private:
        uint16_t speed;
        bool LcdInit();
        void LCDDisplaySAE();
        void LcdUpdate();
};
#endif /* GPIO_HANDLER_H */