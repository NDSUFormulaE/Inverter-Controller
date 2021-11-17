#include <stdlib.h>
#include <inttypes.h>
#include <SPI.h>

//// CAN library and PD400 definitions
#include "src/ARD1939/CAN_SPEC/PGN.h"
#include "src/ARD1939/CAN_SPEC/StateTransition.h"

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

//// Functions
void setup()
{
    Serial.begin(115200);
    if ((taskMan.Init() == false) || (gpioMan.Init() == false))
    {
        delay(500);
        resetFunc(); // If CAN Controller doesnt init correctly, wait 500ms then try again.
    }
    Serial.print("Initialized\n");
}

void loop()
{
    int tempSpeed = analogRead(2);
    tempSpeed = map(tempSpeed, 0, 1023, 32000, 32200);
    taskMan.UpdateSpeed(tempSpeed, INVERTER_CMD_MESSAGE_INDEX);
    Serial.println(InverterState.Abs_Machine_Speed);
    taskMan.RunLoop();
}
