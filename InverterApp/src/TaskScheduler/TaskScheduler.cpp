#include "TaskScheduler.h"

InitializedCANTask CANTasks[MAX_TASKS];
ARD1939 j1939;

//// Main Tasks
int TaskScheduler::Init()
{
    // Initialize the J1939 protocol including CAN settings
    if (j1939.Init(SYSTEM_TIME) != 0))
    {
        return 1;
    }    
    // Set the preferred address and address range
    j1939.SetPreferredAddress(SA_PREFERRED);
    j1939.SetAddressRange(ADDRESSRANGEBOTTOM, ADDRESSRANGETOP);
 
   // Set the NAME
   j1939.SetNAME(NAME_IDENTITY_NUMBER,
                NAME_MANUFACTURER_CODE,
                NAME_FUNCTION_INSTANCE,
                NAME_ECU_INSTANCE,
                NAME_FUNCTION,
                NAME_VEHICLE_SYSTEM,
                NAME_VEHICLE_SYSTEM_INSTANCE,
                NAME_INDUSTRY_GROUP,
                NAME_ARBITRARY_ADDRESS_CAPABLE);  
    return 0;
}

void TaskScheduler::SendMessages()
{
    for (int i = 0; i < MAX_TASKS; i++)
    {
        if (CANTasks[i].initialized == true)
        {
            InitializedCANTask currentTask = CANTasks[i];
            long time = millis();
            Serial.print(time);
            Serial.print("\n\r");
            Serial.print(currentTask.lastRunTime);
            Serial.print("\n\r");
            if((time - currentTask.lastRunTime) >= currentTask.task.interval)
            {
                j1939.Transmit(currentTask.task.priority,
                               currentTask.task.PGN,
                               currentTask.task.srcAddr,
                               currentTask.task.destAddr,
                               &currentTask.task.msg[0],
                               currentTask.task.msgLen);
                currentTask.lastRunTime = time;
            }
        }
    }
}

//// Add/Remove Tasks
int TaskScheduler::AddCANTask(uint8_t priority, long PGN, uint8_t srcAddr, uint8_t destAddr, int msgLen, unsigned long interval, uint8_t msg[J1939_MSGLEN])
{
    int firstFree = FirstFreeInArray();
    if (firstFree != -1)
    {
        CANTasks[firstFree].task.priority = priority;
        CANTasks[firstFree].task.PGN = PGN;
        CANTasks[firstFree].task.srcAddr = srcAddr;
        CANTasks[firstFree].task.destAddr = destAddr;
        CANTasks[firstFree].task.msgLen = msgLen;
        CANTasks[firstFree].task.interval = interval;
        for (int i = 0; i < J1939_MSGLEN; i++)
        {
            CANTasks[firstFree].task.msg[i] = msg[i];
        }
        CANTasks[firstFree].initialized = true;
    }
    return firstFree;
}


//// Message Manipulation
MsgReturn TaskScheduler::GetMsg(int taskIndex)
{
    struct MsgReturn toReturn = {CANTasks[taskIndex].task.msgLen, &CANTasks[taskIndex].task.msg[0]};
    return toReturn;
}

void TaskScheduler::UpdateMsg(int taskIndex, int * msg, int msgLen)
{
    for (int i = 0; i < msgLen; i++)
    {
        CANTasks[taskIndex].task.msg[i] = msg[i];
    }
}

//// Private Functions
int TaskScheduler::FirstFreeInArray()
{
    for (int i = 0; i < MAX_TASKS; i++)
    {
        if (CANTasks[i].initialized == false)
        {
            return i;
        }
    }
    return -1;
}
