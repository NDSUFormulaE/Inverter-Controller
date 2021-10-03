#include <stdlib.h>
#include <inttypes.h>
#include <SPI.h>

//// CAN library and PD400 definitions
#include "src/ARD1939/CAN_SPEC/PGN.h"

//// Subsystem imports
#include "src/TaskScheduler/TaskScheduler.h"
#include "src/gpioHandler/gpioHandler.h"

//// Definitions
// Reset function
void(* resetFunc) (void) = 0; //declare reset function @ address 0
#define InverterSA 0xA2 // Shouldn't be hard coded

// Managers
TaskScheduler taskMan;
GPIOHandler gpioMan;

// Imports
extern struct CANVariables InverterState;
// Variable Definitions
int SpeedCANMsgIndex;
uint16_t CurrentPedalSpeed;

// Default Message bytearrays
uint8_t DefaultSpeedArray[] = {0xF4, 0x1B, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x1F};

//// Functions
void setup()
{
    if ((taskMan.Init() == false) || (gpioMan.Init() == false))
    {
        delay(500);
        resetFunc(); // If CAN Controller doesnt init correctly, wait 500ms then try again.
    }
    SpeedCANMsgIndex = taskMan.AddCANTask(0x18, COMMAND2_SPEED, taskMan.GetSourceAddress(), 0xA2, 8, 15, DefaultSpeedArray);
}

void loop()
{
    CurrentPedalSpeed = gpioMan.GetPedalSpeed();
    taskMan.UpdateMsgByte(SpeedCANMsgIndex, CurrentPedalSpeed % 0xFF ,2);
    taskMan.UpdateMsgByte(SpeedCANMsgIndex, CurrentPedalSpeed >> 8, 3);
    taskMan.RunLoop();
}
