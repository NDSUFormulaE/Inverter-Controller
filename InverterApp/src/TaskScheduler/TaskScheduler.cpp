#include "TaskScheduler.h"

InitializedCANTask CANTasks[MAX_TASKS];
ARD1939 j1939;

//// Main Tasks
int TaskScheduler::Init()
{
    // Initialize the J1939 protocol including CAN settings
    if (j1939.Init(SYSTEM_TIME) != 0)
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
    long time;
    for (int i = 0; i < MAX_TASKS; i++)
    {
        time = millis();
        if (CANTasks[i].initialized == true && ((time - CANTasks[i].lastRunTime) >= CANTasks[i].task.interval))
        {
            j1939.Transmit(CANTasks[i].task.priority,
                            CANTasks[i].task.PGN,
                            CANTasks[i].task.srcAddr,
                            CANTasks[i].task.destAddr,
                            &CANTasks[i].task.msg[0],
                            CANTasks[i].task.msgLen);
            CANTasks[i].lastRunTime = millis();
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
        for (int i = 0; i < msgLen; i++)
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
