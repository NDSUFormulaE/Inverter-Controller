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
#include "src/TaskScheduler/TaskScheduler.h"

//// Definitions
// Reset function
void(* resetFunc) (void) = 0; //declare reset function @ address 0
uint8_t LastCommandedInverterState = MCU_STDBY;
bool InitialState = true;
bool InverterPowerOffState = false;
bool InverterNormalOpState = false;

// Task Defines
void TaskPD400InverterStateMachineControl(void * pvParameters);
void TaskCANLoop(void * pvParameters);
void TaskClearFaults(void * pvParameters);

// Managers
TaskScheduler taskMan;
GPIOHandler gpioMan;

// Imports
extern struct CANVariables InverterState;

//// Functions
void setup()
{
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
#endif
    if ((taskMan.Init() != 0) || (gpioMan.Init() == false))
    {
#ifndef DEBUG_SERIAL
        Serial.begin(115200);
#endif
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.println("Resetting");
        resetFunc(); // If CAN Controller doesnt init correctly, wait 500ms then try again.
    }

    xTaskCreate(
        TaskClearFaults,
        "ClearFaults",
        128,
        NULL,
        6,
        NULL
    );

#ifdef SEVEN_SEGMENT_DISPLAYS_ENABLED
    xTaskCreate(
        TaskUpdateSevenSegments,
        "UpdateSevenSegments",
        128,
        NULL,
        4,
        NULL
    );
#endif

#ifdef LCD_DISPLAY_ENABLED
    xTaskCreate(
        TaskUpdateLCDs,
        "UpdateLCDs",
        512,
        NULL,
        4,  // Lower priority than CAN task (4) to prevent LCD from blocking CAN
        NULL
    );
#endif

    xTaskCreate(
        TaskCANLoop,
        "CANLoop",
        256,
        NULL,
        4,
        NULL
    );

    xTaskCreate(
        TaskPD400InverterStateMachineControl,
        "InverterStateMachineControl",
        128,
        NULL,
        6,
        NULL
    );

    LcdPrintStatus("Stack Initialized");
}

void loop()
{
    // Empty loop task means that we won't execute anything when all tasks
    // are blocked. In our application that shouldn't ever happen.
}

void TaskClearFaults(void * pvParameters)
{
    (void) pvParameters;
    for (;;)
    {
        uint16_t ClearPinVal = gpioMan.GetClearPin();
        #ifdef AUTO_CLEAR_CAN_FAULTS 
        if (ClearPinVal || InverterState.MCU_State == MCU_FAULT_CLASSA || InverterState.MCU_State == MCU_FAULT_CLASSB)
        #else
        if (ClearPinVal)
        #endif
        {
            taskMan.ClearInverterFaults();
        }
        // Try to make these delays powers of 2.
        vTaskDelay(pdMS_TO_TICKS(2048));
    }
}

void TaskUpdateSevenSegments(void * pvParameters)
{
    (void) pvParameters;
    for (;;)
    {
        gpioMan.UpdateSevenSegments();
        // Try to make these delays powers of 2.
        vTaskDelay(pdMS_TO_TICKS(256));
    }
}

void TaskUpdateLCDs(void * pvParameters)
{
    (void) pvParameters;
    for (;;)
    {
        gpioMan.UpdateLCDs();
        // Try to make these delays powers of 2.
        vTaskDelay(pdMS_TO_TICKS(3072));
    }
}

void TaskCANLoop(void * pvParameters)
{
    (void) pvParameters;
    for (;;)
    {   
        #ifndef USE_APPS
            taskMan.UpdateCommandedPower(gpioMan.GetPedalSpeed(), INVERTER_CMD_MESSAGE_INDEX);
        #else
            taskMan.UpdateCommandedPower(gpioMan.GetPedalTorque(), INVERTER_CMD_MESSAGE_INDEX);
        #endif
        taskMan.RunLoop();
        // Try to make these delays powers of 2.
        vTaskDelay(CAN_CONTROL_LOOP_INTERVAL_TICKS);
    }
}

void TaskPD400InverterStateMachineControl(void * pvParameters)
{
    (void) pvParameters;

    for (;;)
    {
        if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED)
        {
            if (InverterState.MCU_State != MCU_FAIL_SAFE || InverterState.MCU_State != MCU_CNTRL_PWR_DOWN)
            {
                InverterPowerOffState = false;
            }
            if(InverterState.MCU_State != MCU_NORM_OPS)
            {
                InverterNormalOpState = false;
            }
            switch(InverterState.MCU_State)
            {
                case MCU_STDBY:
                    taskMan.ChangeState(STDBY_TO_FUNCTIONAL_DIAG, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_FUNCTIONAL_DIAG;      
                    break;
                case MCU_FUNCTIONAL_DIAG:
                    taskMan.ChangeState(STDBY_TO_IGNIT_READY, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_IGNIT_READY;      
                    break;
                case MCU_IGNIT_READY:
                    LastCommandedInverterState = MCU_PWR_READY;  
                    break;
                case MCU_PWR_READY:
                    taskMan.ChangeState(PWR_READY_TO_DRIVE_READY, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_DRIVE_READY;  
                    break;
                case MCU_PWR_DIAG:

                    break;
                case MCU_DRIVE_READY:
                    taskMan.ChangeState(DRIVE_READY_TO_NORM_OPS, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_NORM_OPS; 
                    break;
                case MCU_NORM_OPS:
                    InverterNormalOpState = true;
                    break;

                case MCU_CNTRL_PWR_DOWN:
                    LcdPrintStatus("Powering Down...");
                    InverterPowerOffState = true;
                    LastCommandedInverterState = MCU_STDBY;
                    break;
                case MCU_FAIL_SAFE:
                    LcdPrintStatus("FAIL SAFE MODE");
                    InverterPowerOffState = true;
                    LastCommandedInverterState = MCU_STDBY;
                    break;
                case MCU_FAULT_CLASSA:
                    LastCommandedInverterState = MCU_STDBY;
                    break;
                case MCU_FAULT_CLASSB:
                    LastCommandedInverterState = MCU_STDBY;
                    break;
                case MCU_ADV_DIAG_CLASSA:

                    break;
                case MCU_DISCHARGE_DIAG:
                
                    break;
                case MCU_ADV_DIAG_CLASSB:

                    break;
                default:
                    break;
            }
            if(InitialState)
            {
                InitialState = false;
            }
        }
        // Try to make these delays powers of 2.
        vTaskDelay(pdMS_TO_TICKS(256));
    }
}