#include "TaskScheduler.h"

InitializedCANTask CANTasks[MAX_TASKS];
ARD1939 j1939;

int TaskScheduler::Init()
{
    /**
    * Initialize the j1939 subsystem
    *
    * Parameters:
    *    none
    * Returns:
    *    none
    **/
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

void TaskScheduler::RunLoop()
{
    /**
    * Periodic Tasks for all subsystems
    *
    * Parameters:
    *     none
    * Returns:
    *     none
    **/
    TaskScheduler::SendMessages();
    // Caused issues with bytes to send over CAN. Test with the Due.
    TaskScheduler::RecieveMessages();
}

void TaskScheduler::SendMessages()
{
    /**
    * Iterates over the entire CANTasks array and sends any messages which have been initialized.
    *
    * Parameters:
    *     none
    * Returns:
    *     none
    **/
    long time;
    for (int i = 0; i < MAX_TASKS; i++)
    {
        time = millis();
        if (CANTasks[i].initialized == true && ((CANTasks[i].lastRunTime == 0) || (time - CANTasks[i].lastRunTime) >= CANTasks[i].task.interval))
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

void TaskScheduler::RecieveMessages()
{
    /**
    * Reads messages from the CAN bus and interprets them.
    * Saves their values in InverterState struct. 
    *   
    * Parameters:
    *     none
    * Returns:
    *     none
    **/
    // J1939 Variables
    uint8_t MsgId;
    uint8_t DestAddr;
    uint8_t SrcAddr;
    uint8_t Priority;
    uint8_t J1939Status;
    int MsgLen;
    long PGN;
    uint8_t Msg[J1939_MSGLEN];

    J1939Status = j1939.Operate(&MsgId, &PGN, &Msg[0], &MsgLen, &DestAddr, &SrcAddr, &Priority);
    if (J1939Status == NORMALDATATRAFFIC)
    {
        j1939.CANInterpret(&PGN, &Msg[0], &MsgLen, &DestAddr, &SrcAddr, &Priority);
    }
}

int TaskScheduler::AddCANTask(uint8_t priority, long PGN, uint8_t srcAddr, uint8_t destAddr, int msgLen, unsigned long interval, uint8_t msg[J1939_MSGLEN])
{
    /**
    * Configure a new CANTask in the array.
    *
    * Parameters:
    *     priority   (uint8_t): Priority of the message to be sent
    *     PGN           (long): Message PGN
    *     srcAddr    (uint8_t): Address of the device sending the message
    *     destAddr   (uint8_t): Address of the device recieving the message
    *     msgLen         (int): Length of the message 
    *     interval     (ulong): Number of milliseconds between sending the message
    *     msg      (uint8_t[]): Message to be sent
    * Returns:
    *     firstFree      (int): Index of the newly added CANTask
    **/
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
        CANTasks[firstFree].lastRunTime = 0;
    }
    return firstFree;
}

void RemoveCANTask(int taskIndex)
{
    /**
    * De-initializes a CANTask in the array.
    * 
    * Parameters:
    *     taskIndex   (int): index of the task in CANTasks
    * Returns:
    *     none
    **/
    CANTasks[taskIndex].initialized = false;
    CANTasks[taskIndex].lastRunTime = 0;
}

MsgReturn TaskScheduler::GetMsg(int taskIndex)
{
    /**
    * Gets the msg of the specified task.
    *
    * Parameters:
    *     taskIndex  (int): index of the task in CANTasks
    * Returns:
    *     toReturn (struct MsgReturn): returns object with .length and .message properties.
    **/
    struct MsgReturn toReturn = {CANTasks[taskIndex].task.msgLen, &CANTasks[taskIndex].task.msg[0]};
    return toReturn;
}

void TaskScheduler::UpdateMsg(int taskIndex, int * msg, int msgLen)
{
    /**
    * Updates a specified message in the CANTasks array.
    * 
    * Parameters:
    *     taskIndex     (int): index of the task in CANTasks
    *     msg         (int *): pointer to the msg array
    *     msgLen        (int): length of msg 
    * Returns:
    *     none
    **/
    for (int i = 0; i < msgLen; i++)
    {
        CANTasks[taskIndex].task.msg[i] = msg[i];
    }
}

void TaskScheduler::UpdateMsgByte(int taskIndex, int byte, int indexOfByte)
{
    /**
    * Updates a byte in a specified message in the CANTasks array.
    * 
    * Parameters:
    *     taskIndex   (int): index of the task in CANTasks
    *     byte        (int): data to be stored
    *     indexOfByte (int): index of the message byte 
    * Returns:
    *     none
    **/
        CANTasks[taskIndex].task.msg[indexOfByte] = byte;
}

int TaskScheduler::FirstFreeInArray()
{
    /**
    * Iterates over the CANTasks array until it finds the first open spot in the array.
    * 
    * Parameters:
    *     none
    * Returns:
    *     i (int): index of the first free object in array
    **/
    for (int i = 0; i < MAX_TASKS; i++)
    {
        if (CANTasks[i].initialized == false)
        {
            return i;
        }
    }
    return -1;
}
