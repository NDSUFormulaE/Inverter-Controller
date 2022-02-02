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

// Managers
TaskScheduler taskMan;
GPIOHandler gpioMan;

// Imports
extern struct CANVariables InverterState;

//// Functions
void setup()
{
    Serial.begin(115200);
    if ((taskMan.Init() == false) || (gpioMan.Init() == false))
    {
        delay(500);
        resetFunc(); // If CAN Controller doesnt init correctly, wait 500ms then try again.
    }
    Serial.println("Initialized");
}

void loop()
{
    taskMan.UpdateSpeed(gpioMan.GetPedalSpeed(), INVERTER_CMD_MESSAGE_INDEX);
    taskMan.RunLoop();
    if (gpioMan.GetClearPin())
    {
        taskMan.ClearInverterFaults();
    }
    InverterStateMachineControl();
}

void InverterStateMachineControl()
{
  if (InverterState.MCU_State != MCU_FAIL_SAFE || InverterState.MCU_State != MCU_CNTRL_PWR_DOWN)
  {
      InverterPowerOffState = false;
  }
  if(InverterState.MCU_State != MCU_NORM_OPS)
  {
      InverterNormalOpState = false;
  }
  if(!InverterPowerOffState)
  {
    switch(InverterState.MCU_State)
    {
        case MCU_STDBY:
            if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
            {
                Serial.println("Inverter in Standby State");
                Serial.println("Commanding Inverter to Ignition Ready");
            }
            
            taskMan.ChangeState(STDBY_TO_IGNIT_READY, INVERTER_CMD_MESSAGE_INDEX);
            LastCommandedInverterState = MCU_IGNIT_READY;      
            break;
        case MCU_FUNCTIONAL_DIAG:
            if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
            {
                Serial.println("Inverter in Functional Diagnostics State");
                Serial.println("Inverter will automatically transition to Ignition Ready");
            }
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
            taskMan.ClearInverterFaults();
            LastCommandedInverterState = MCU_STDBY;
            break;
        case MCU_FAULT_CLASSB:
            if(InitialState || LastCommandedInverterState == InverterState.MCU_State)
            {
                Serial.println("Inverter in Class B Fault State");
                Serial.println("Commanding Inverter to Standby");
            }
            taskMan.ClearInverterFaults();
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
  }
  if(InitialState)
  {
      InitialState = false;
  }
}