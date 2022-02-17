#include "src/FreeRTOS/src/Arduino_FreeRTOS.h"
#include <stdlib.h>
#include <inttypes.h>
#include <SPI.h>

//// CAN library and PD400 definitions
#include "src/ARD1939/CAN_SPEC/PGN.h"
#include "src/ARD1939/CAN_SPEC/StateTransition.h"
#include "src/ARD1939/CAN_SPEC/MotorControlUnitState.h"

//// Subsystem imports
#include "src/gpioHandler/gpioHandler.h"
// #include "src/TaskScheduler/TaskScheduler.h" // imported in gpioHandler.h Arduino IDE has a stroke if also defined here

//// Definitions
// Reset function
void(* resetFunc) (void) = 0; //declare reset function @ address 0
#define InverterSA 0xA2 // Shouldn't be hard coded
uint8_t LastCommandedInverterState = MCU_STDBY;
bool InitialState = true;
bool InverterPowerOffState = false;
bool InverterNormalOpState = false;

// Common TaskScheduler/GPIOHandler pointer struct
struct ManagerPointers
{
    TaskScheduler* TaskPoint;
    GPIOHandler* GPIOPoint;
};

// Task Defines
void TaskInverterStateMachineControl(void * pvParameters);
void TaskCANLoop(void * pvParameters);
void TaskClearFaults(void * pvParameters);

// Managers
TaskScheduler taskMan;
GPIOHandler gpioMan;
PWM_RETURN pwm_val;

// Imports
extern struct CANVariables InverterState;

//// Functions
void setup()
{
    if ((taskMan.Init() == false) || (gpioMan.Init() == false))
    {
        delay(500);
        resetFunc(); // If CAN Controller doesnt init correctly, wait 500ms then try again.
    }

    xTaskCreate(
        TaskCANLoop,
        "CANLoop",
        256,
        NULL,
        8,
        NULL
    );

    xTaskCreate(
        TaskReadPWMs,
        "ReadPWMs",
        256,
        NULL,
        7,
        NULL
        );
}

void loop()
{
    // Empty loop task means that we won't execute anything when all tasks
    // are blocked. In our application that shouldn't ever happen.
}


void TaskReadPWMs(void * pvParameters)
{
    for (;;)
    {
        pwm_val = gpioMan.GetPWM();
        taskMan.UpdateMsgByte(PWM_MESSAGE_INDEX, pwm_val.freq_low, 1);
        taskMan.UpdateMsgByte(PWM_MESSAGE_INDEX, pwm_val.duty_cycle_low, 2);
        taskMan.UpdateMsgByte(PWM_MESSAGE_INDEX, pwm_val.freq_high, 3);
        taskMan.UpdateMsgByte(PWM_MESSAGE_INDEX, pwm_val.duty_cycle_high, 4);
        vTaskDelay(1);
    }
}


void TaskCANLoop(void * pvParameters)
{
    for (;;)
    {
        taskMan.RunLoop();
        vTaskDelay(2);
    }
}
