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
    Serial.begin(115200);
    if ((taskMan.Init() != 0) || (gpioMan.Init() == false))
    {
        delay(500);
        Serial.println("Resetting");
        resetFunc(); // If CAN Controller doesnt init correctly, wait 500ms then try again.
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

    Serial.println("Initialized");
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

// Helper function to find and execute state transition
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
            // Log state change
            Serial.print("Current State: 0x");
            Serial.print(currentState, HEX);
            Serial.print(" - ");
            Serial.println(transition.stateMessage);
            
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
    if (LastCommandedInverterState != MCU_STDBY) {
        Serial.print("Fault detected: 0x");
        Serial.println(faultState, HEX);
        Serial.println("Transitioning to Standby");
        LastCommandedInverterState = MCU_STDBY;
    }
}

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
                Serial.println("Waiting for CAN bus initialization...");
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            uint8_t currentState = InverterState.MCU_State;
            
            // If not in standby, try to get there
            if (currentState != MCU_STDBY && currentState != MCU_PWR_UP)
            {
                Serial.print("Error: Device not in standby state. Current state: 0x");
                Serial.println(currentState, HEX);
                
                // If in a fault state, try to transition to standby
                if (currentState == MCU_FAULT_CLASSA)
                {
                    Serial.println("Attempting to clear Class A fault and return to standby...");
                    taskMan.ChangeState(FAULT_CLASSA_TO_STDBY, INVERTER_CMD_MESSAGE_INDEX);
                }
                else if (currentState == MCU_FAULT_CLASSB)
                {
                    Serial.println("Attempting to transition from Class B fault to Power Ready...");
                    taskMan.ChangeState(FAULT_CLASSB_TO_PWR_READY, INVERTER_CMD_MESSAGE_INDEX);
                }
                else
                {
                    Serial.println("Unable to automatically transition to standby.");
                }
                
                initRetries++;
                if (initRetries >= MAX_INIT_RETRIES)
                {
                    Serial.println("ERROR: Maximum initialization retries reached. Manual intervention required.");
                    while(1) { vTaskDelay(portMAX_DELAY); } // Stop trying to initialize
                }
                
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            
            // If we get here, we're in standby and can start normal initialization
            Serial.println("Device in standby state, beginning initialization sequence...");
            taskMan.ChangeState(STDBY_TO_FUNCTIONAL_DIAG, INVERTER_CMD_MESSAGE_INDEX);
            InitialState = false;
            LastCommandedInverterState = MCU_FUNCTIONAL_DIAG;
            initRetries = 0;
        }
        else if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED)
        {
            uint8_t currentState = InverterState.MCU_State;
            lastCANStatusTime = millis(); // Reset timer when CAN is working
            
            // Handle fault states
            if (currentState == MCU_FAULT_CLASSA || currentState == MCU_FAULT_CLASSB || 
                currentState == MCU_FAIL_SAFE)
            {
                handleFaultState(currentState);
            }
            // Detect power down state
            else if (currentState == MCU_CNTRL_PWR_DOWN && LastCommandedInverterState != MCU_STDBY)
            {
                Serial.println("Inverter powering down - ignition line low");
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
            // Periodically report when CAN address claim isn't finished
            Serial.print("Warning: CAN bus not ready. Status: 0x");
            Serial.println(InverterState.CAN_Bus_Status, HEX);
            lastCANStatusTime = millis();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}