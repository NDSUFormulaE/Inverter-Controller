#include <stdlib.h>
#include <inttypes.h>
#include <SPI.h>

// CAN library and PD400 definitions

#include "src/ARD1939/CAN_SPEC/PGN.h"

// Task Scheduler
#include "src/TaskScheduler/TaskScheduler.h"

// Definitions

TaskScheduler taskMan;
extern struct CANVariables InverterState;
void(* resetFunc) (void) = 0; //declare reset function @ address 0

// TODO: Add tasks for the periodic messages
uint8_t heartbeat[] = {1,2,3,4,5,6,7,8};
uint8_t heartbeat2[] = {8,7,6,5,4,3,2,1};

void setup()
{
    int TaskSchedulerInit = taskMan.Init();
    if (TaskSchedulerInit != 0)
    {
        delay(500);
        resetFunc(); // If CAN Controller doesnt init correctly, wait 500ms then try again.
    }

    //Initiate Serial communication.
    Serial.begin(MONITOR_BAUD_RATE);
    Serial.print("Serial Open\n\r");

    taskMan.AddCANTask(18, COMMAND2_SPEED, 0xF1, 0xFF, 8, 20000, heartbeat);
    //taskMan.AddCANTask(18, COMMAND2_SPEED, 0xF1, 0xFF, 8,100, heartbeat2);
}// end setup

void loop()
{
    taskMan.SendMessages();
}// end loop
