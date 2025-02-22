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
void TaskInverterStateMachineControl(void * pvParameters);
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
    Serial.begin(115200);
    if ((taskMan.Init() != 0) || (gpioMan.Init() == false))
    {
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

#ifdef DISPLAYS_ENABLED
    xTaskCreate(
        TaskUpdateSevenSegments,
        "UpdateSevenSegments",
        128,
        NULL,
        7,
        NULL
    );

    xTaskCreate(
        TaskUpdateLCDs,
        "UpdateLCDs",
        128,
        NULL,
        6,
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
        TaskInverterStateMachineControl,
        "InverterStateMachineControl",
        128,
        NULL,
        5,
        NULL
    );

    Serial.println("Application Stack Initialized");
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
        vTaskDelay(pdMS_TO_TICKS(512));
    }
}

void TaskUpdateLCDs(void * pvParameters)
{
    (void) pvParameters;
    for (;;)
    {
        gpioMan.UpdateLCDs();
        // Try to make these delays powers of 2.
        vTaskDelay(pdMS_TO_TICKS(768));
    }
}

void TaskCANLoop(void * pvParameters)
{
    (void) pvParameters;
    for (;;)
    {
        // taskMan.UpdateSpeed(gpioMan.GetPedalSpeed(), INVERTER_CMD_MESSAGE_INDEX);
        // taskMan.UpdateSpeed(gpioMan.GetPedalTorque(), INVERTER_CMD_MESSAGE_INDEX);
        taskMan.RunLoop();
        // Try to make these delays powers of 2.
        vTaskDelay(CAN_CONTROL_LOOP_INTERVAL_TICKS);
    }
}

void TaskInverterStateMachineControl(void * pvParameters)
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
                    if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
                    {
                        Serial.println("Inverter in Standby State");
                    }
                    
                    taskMan.ChangeState(STDBY_TO_FUNCTIONAL_DIAG, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_FUNCTIONAL_DIAG;      
                    break;
                case MCU_FUNCTIONAL_DIAG:
                    if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
                    {
                        Serial.println("Inverter in Functional Diagnostics State");
                        Serial.println("Inverter will automatically transition to Ignition Ready");
                    }
                    taskMan.ChangeState(STDBY_TO_IGNIT_READY, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_IGNIT_READY;      
                    break;
                case MCU_IGNIT_READY:
                    if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
                    {
                        Serial.println("Inverter in Ignition Ready State");
                        Serial.println("Inverter will automatically transition to Power Ready");
                        Serial.println("Connect HVDC Bus");
                    }
                    LastCommandedInverterState = MCU_PWR_READY;  
                    break;
                case MCU_PWR_READY:
                    if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
                    {
                        Serial.println("Inverter in Power Ready State");
                        Serial.println("Commanding Inverter to Drive Ready");
                    }
                    taskMan.ChangeState(PWR_READY_TO_DRIVE_READY, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_DRIVE_READY;  
                    break;
                case MCU_PWR_DIAG:

                    break;
                case MCU_DRIVE_READY:
                    if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
                    {
                        Serial.println("Inverter in Drive Ready State");
                        Serial.println("Commanding Inverter to Normal Operation");
                    }
                    taskMan.ChangeState(DRIVE_READY_TO_NORM_OPS, INVERTER_CMD_MESSAGE_INDEX);
                    LastCommandedInverterState = MCU_NORM_OPS; 
                    break;
                case MCU_NORM_OPS:
                    if(InitialState || (LastCommandedInverterState == InverterState.MCU_State && !InverterNormalOpState))
                    {
                        Serial.println("Inverter in Normal Operation State");
                    }
                    InverterNormalOpState = true;
                    break;

                case MCU_CNTRL_PWR_DOWN:
                    InverterPowerOffState = true;
                    Serial.println("Inverter in Controlled Power Down State");
                    Serial.println("Inverter Powering Down");
                    LastCommandedInverterState = MCU_STDBY;
                    break;
                case MCU_FAIL_SAFE:
                    InverterPowerOffState = true;
                    Serial.println("Inverter in Fail Safe State");
                    Serial.println("Inverter Powering Down");
                    LastCommandedInverterState = MCU_STDBY;
                    break;
                case MCU_FAULT_CLASSA:
                    if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
                    {
                        Serial.println("Inverter in Class A Fault State");
                        Serial.println("Commanding Inverter to Standby");
                    }
                    LastCommandedInverterState = MCU_STDBY;
                    break;
                case MCU_FAULT_CLASSB:
                    if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
                    {
                        Serial.println("Inverter in Class B Fault State");
                        Serial.println("Commanding Inverter to Standby");
                    }
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