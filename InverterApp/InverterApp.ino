
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
    Serial.println("Attempting to move Inverter to Ignition Ready");
    while(InverterState.MCU_State != MCU_IGNIT_READY)
    {
        if(InverterState.MCU_State == MCU_FAULT_CLASSA || InverterState.MCU_State == MCU_FAULT_CLASSA)
        {
          taskMan.ClearInverterFaults();
          Serial.println("In Fault State");
        }
        else
        {
          taskMan.ChangeState(STDBY_TO_IGNIT_READY, INVERTER_CMD_MESSAGE_INDEX);      
        }

        loop();
    }
    Serial.println("Inverter State Ignition Ready");
    Serial.println("Connect HVDC");
    while(InverterState.MCU_State != MCU_PWR_READY)
    {
      loop();
    }
    Serial.println("Inverter State Power Ready");
    while(InverterState.MCU_State != MCU_ADV_DIAG_CLASSB)
    {
        taskMan.ChangeState(PWR_READY_TO_ADV_DIAG_CLASSB, INVERTER_CMD_MESSAGE_INDEX);
        loop();
    }
    Serial.println("Inverter State Adv Diag Class B");
    //taskMan.ChangeState(DIAG_AD_B_MOTOR_POS_SENS_CALIB, INVERTER_CMD_MESSAGE_INDEX);
    
}

void loop()
{
    taskMan.UpdateSpeed(gpioMan.GetPedalSpeed(), INVERTER_CMD_MESSAGE_INDEX);
    taskMan.RunLoop();
    if (gpioMan.GetClearPin())
    {
        taskMan.ClearInverterFaults();
    }
}
