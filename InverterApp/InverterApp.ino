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
bool InverterPowerOffState = false;  // Flag indicating inverter shutdown
bool InverterNormalOpState = false;  // Flag indicating normal operation achieved

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

// Imports
extern struct CANVariables InverterState;

//// Functions
void setup()
{
    // Initialize GPIO first since we need it for display
    if (gpioMan.Init() == false)
    {
        delay(500);
        resetFunc();  // If GPIO init fails, just reset immediately
    }

    // Now we can use the display for task manager init
    if (taskMan.Init() != 0)
    {
        gpioMan.UpdateLCD("Error", "Task Init Failed", "Resetting...", "");
        delay(1000);
        resetFunc();
    }

    xTaskCreate(
        TaskClearFaults,
        "ClearFaults",
        192,
        NULL,
        4,
        NULL
    );

    xTaskCreate(
        TaskUpdateSevenSegments,
        "UpdateSevenSegments",
        192,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        TaskUpdateLCDs,
        "UpdateLCDs",
        256,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        TaskCANLoop,
        "CANLoop",
        384,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        TaskInverterStateMachineControl,
        "InverterStateMachineControl",
        256,
        NULL,
        6,
        NULL
    );

    gpioMan.UpdateLCD("Status", "Initialized", "Ready", "");
}

void loop()
{
    // Empty loop task means that we won't execute anything when all tasks
    // are blocked. In our application that shouldn't ever happen.
}

void TaskClearFaults(void * pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    
    for (;;)
    {
        uint16_t ClearPinVal = gpioMan.GetClearPin(); 
        if (ClearPinVal || InverterState.MCU_State == MCU_FAULT_CLASSA || InverterState.MCU_State == MCU_FAULT_CLASSB)
        {
            taskMan.ClearInverterFaults();
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskUpdateSevenSegments(void * pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    
    for (;;)
    {
        gpioMan.UpdateSevenSegments();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskUpdateLCDs(void * pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms update interval
    
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        gpioMan.UpdateLCDs();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Consistent timing
    }
}

void TaskCANLoop(void * pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    
    for (;;)
    {
        #ifndef USE_APPS
        taskMan.UpdateSpeed(gpioMan.GetPedalSpeed(), INVERTER_CMD_MESSAGE_INDEX);
        #else
        taskMan.UpdateSpeed(gpioMan.GetPedalTorque(), INVERTER_CMD_MESSAGE_INDEX);
        #endif
        taskMan.RunLoop();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// State machine transition structure
struct StateTransition {
    uint8_t currentState;
    uint8_t nextState;
    uint8_t transitionCommand;
    const char* stateMessage;
};

// Define state transitions table
const StateTransition stateTransitions[] = {
    {MCU_STDBY, MCU_FUNCTIONAL_DIAG, STDBY_TO_FUNCTIONAL_DIAG, "Transitioning to Functional Diagnostics"},
    {MCU_FUNCTIONAL_DIAG, MCU_IGNIT_READY, STDBY_TO_IGNIT_READY, "Transitioning to Ignition Ready"},
    {MCU_IGNIT_READY, MCU_PWR_READY, NO_CHANGE, "Waiting for Power Ready"}, // Auto transition
    {MCU_PWR_READY, MCU_DRIVE_READY, PWR_READY_TO_DRIVE_READY, "Transitioning to Drive Ready"},
    {MCU_DRIVE_READY, MCU_NORM_OPS, DRIVE_READY_TO_NORM_OPS, "Transitioning to Normal Operation"}
};

void TaskInverterStateMachineControl(void * pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms interval
    static uint8_t initRetries = 0;
    const uint8_t MAX_INIT_RETRIES = 3;
    static unsigned long lastCANStatusTime = 0;
    const unsigned long CAN_STATUS_INTERVAL = 5000; // Print CAN status every 5 seconds
    
    for (;;)
    {
        if (InitialState)
        {
            if (InverterState.CAN_Bus_Status != ADDRESSCLAIM_FINISHED)
            {
                gpioMan.UpdateState("Initializing...");
                gpioMan.UpdateFault("Waiting for CAN");
                gpioMan.UpdateInfo("Bus Status:", "Not Connected");
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            uint8_t currentState = InverterState.MCU_State;
            
            // If not in standby, try to get there
            if (currentState != MCU_STDBY && currentState != MCU_PWR_UP)
            {
                if (currentState == MCU_FAULT_CLASSA)
                {
                    gpioMan.DisplayFaultState(currentState,
                                                "Class A Fault",
                                                "Attempting Clear");
                    taskMan.ChangeState(FAULT_CLASSA_TO_STDBY, INVERTER_CMD_MESSAGE_INDEX);
                }
                else if (currentState == MCU_FAULT_CLASSB)
                {
                    gpioMan.DisplayFaultState(currentState,
                                                "Class B Fault",
                                                "To Power Ready");
                    taskMan.ChangeState(FAULT_CLASSB_TO_PWR_READY, INVERTER_CMD_MESSAGE_INDEX);
                }
                else
                {
                    gpioMan.DisplayFaultState(currentState,
                                                "Error",
                                                "Cannot -> Standby");
                }
                
                initRetries++;
                if (initRetries >= MAX_INIT_RETRIES)
                {
                    gpioMan.UpdateState("ERROR");
                    gpioMan.UpdateFault("Max Retries Hit");
                    gpioMan.UpdateInfo("Manual Reset", "Required");
                    while(1) { vTaskDelay(portMAX_DELAY); }
                }
                
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            
            // If we get here, we're in standby and can start normal initialization
            gpioMan.UpdateState("Standby");
            gpioMan.UpdateFault("No Faults");
            gpioMan.UpdateInfo("Starting Init", "Sequence...");
            taskMan.ChangeState(STDBY_TO_FUNCTIONAL_DIAG, INVERTER_CMD_MESSAGE_INDEX);
            InitialState = false;
            LastCommandedInverterState = MCU_FUNCTIONAL_DIAG;
            initRetries = 0;
        }
        else if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED)
        {
            uint8_t currentState = InverterState.MCU_State;
            lastCANStatusTime = millis();
            
            // Handle fault states
            if (currentState == MCU_FAULT_CLASSA || currentState == MCU_FAULT_CLASSB || 
                currentState == MCU_FAIL_SAFE)
            {
                handleFaultState(currentState);
            }
            // Detect power down state
            else if (currentState == MCU_CNTRL_PWR_DOWN && LastCommandedInverterState != MCU_STDBY)
            {
                gpioMan.UpdateState("Power Down");
                gpioMan.UpdateFault("No Faults");
                gpioMan.UpdateInfo("Ignition Line", "Low");
                LastCommandedInverterState = MCU_STDBY;
            }
            // Handle normal state transitions
            else if (!InverterPowerOffState)
            {
                executeStateTransition(currentState);
            }
        }
        else if (millis() - lastCANStatusTime >= CAN_STATUS_INTERVAL)
        {
            const char* statusStr = gpioMan.getCANStatusName(InverterState.CAN_Bus_Status);
            gpioMan.UpdateState("CAN Bus Status");
            gpioMan.UpdateFault(statusStr);
            gpioMan.UpdateInfo("Address Claim", "In Progress", false);  // Don't persist CAN status info
            lastCANStatusTime = millis();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void executeStateTransition(uint8_t currentState) {
    static unsigned long lastStateChangeTime = 0;
    const unsigned long STATE_CHANGE_DELAY = 500; // 500ms minimum between state changes
    
    // Don't process transitions too quickly
    if (millis() - lastStateChangeTime < STATE_CHANGE_DELAY) {
        return;
    }
    
    // Find matching transition
    for (const StateTransition& transition : stateTransitions) {
        if (transition.currentState == currentState) {
            gpioMan.DisplayStateTransition(currentState, transition.stateMessage);
            
            // Execute transition if command exists
            if (transition.transitionCommand != NO_CHANGE) {
                taskMan.ChangeState(transition.transitionCommand, INVERTER_CMD_MESSAGE_INDEX);
                LastCommandedInverterState = transition.nextState;
            }
            
            lastStateChangeTime = millis();
            break;
        }
    }
}

void handleFaultState(uint8_t faultState) {
    if (faultState == MCU_FAULT_CLASSA) {
        gpioMan.DisplayFaultState(faultState, "Class A Fault", "Attempting Clear");
        taskMan.ChangeState(FAULT_CLASSA_TO_STDBY, INVERTER_CMD_MESSAGE_INDEX);
    } else if (faultState == MCU_FAULT_CLASSB) {
        gpioMan.DisplayFaultState(faultState, "Class B Fault", "To Power Ready");
        taskMan.ChangeState(FAULT_CLASSB_TO_PWR_READY, INVERTER_CMD_MESSAGE_INDEX);
    } else if (faultState == MCU_FAIL_SAFE) {
        gpioMan.DisplayFaultState(faultState, "Fail Safe", "System Halted");
        while(1) { vTaskDelay(portMAX_DELAY); }  // Stay in fail safe
    }
}