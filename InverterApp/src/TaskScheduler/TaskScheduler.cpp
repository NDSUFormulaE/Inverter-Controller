#include "TaskScheduler.h"
#include "../ARD1939/CAN_SPEC/StateTransition.h"
#include "../ARD1939/CAN_SPEC/MotorControlUnitState.h"


InitializedCANTask CANTasks[MAX_TASKS];
ARD1939 j1939;

//// Main Tasks
bool TaskScheduler::Init()
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
    int j1939_init = j1939.Init(SYSTEM_TIME);
    if (j1939_init != 0)
    {
        return j1939_init;
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
    return true;
}

uint8_t TaskScheduler::GetSourceAddress()
{
    return j1939.GetSourceAddress();
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

bool TaskScheduler::ChangeState(int stateTransition, int speedMessageIndex)
{

 /**
    * Transition through the inverter state machine.
    * State commands can be found in StateTransition.h
    * Motor Contorl Unit State Definitions can be found in MotorControlUnitState.h
    *
    * Parameters:
    *    stateTransition             (int): State transition command from CAN Spec 2.3.3.
    *    speedMessageIndex           (int): 6th byte of the speed mode in CAN Spec 2.3.1.2.
    * Returns:
    *    False if InverterState.MCU_State != start. True otherwise.
    **/

    int start;
    int end;
    
    extern struct CANVariables InverterState;

    switch(stateTransition)
    {
        case STDBY_TO_FUNCTIONAL_DIAG: start = MCU_STDBY;
        break;

        case PWR_READY_TO_PWR_DIAG: start = MCU_PWR_READY;
        break;

        case DRIVE_READY_TO_NORM_OPS: start = MCU_DRIVE_READY;
        break;

        case NORM_OPS_TO_DISCHARGE_DIAG: start = MCU_NORM_OPS;
        break;

        case FAULT_CLASSA_TO_STDBY: start = MCU_FAULT_CLASSA;
        break;

        case IGNIT_READY_TO_ADV_DIAG_CLASSA: start = MCU_IGNIT_READY;
        break;

        case FAULT_CLASSA_TO_ADV_DIAG_CLASSA: start = MCU_FAULT_CLASSA;
        break;

        case FAULT_CLASSB_TO_PWR_READY: start = MCU_FAULT_CLASSB;
        break;

        case NORM_OPS_TO_DRIVE_READY: start = MCU_NORM_OPS;
        break;

        case PWR_READY_TO_ADV_DIAG_CLASSB: start = MCU_PWR_READY;
        break;

        case FAULT_CLASSB_TO_ADV_DIAG_CLASSB: start = MCU_FAULT_CLASSB;
        break;

        case DRIVE_READY_TO_ADV_DIAG_CLASSB: start = MCU_DRIVE_READY;
        break;

        case FAULT_CLASSB_TO_FAIL_SAFE: start = MCU_FAULT_CLASSB;
        break;

        case FAULT_CLASS_B_ADV_DIAG_CLASSA_TO_FAIL_SAFE: start = MCU_ADV_DIAG_CLASSA;
        break;

        case PWR_READY_TO_DRIVE_READY: start = MCU_PWR_READY;
        break;

        case STDBY_TO_ADV_DIAG_CLASSA: start = MCU_STDBY;
        break;

        case STDBY_TO_IGNIT_READY: start = MCU_STDBY;
        break;

        case ADV_DIAG_CLASSB_TO_PWR_READY: start = MCU_ADV_DIAG_CLASSB;
        break;

        case ADV_DIAG_CLASSA_TO_STDBY: start = MCU_ADV_DIAG_CLASSA;
        break;

        case FAULT_CLASSB_TO_STDBY: start = MCU_FAULT_CLASSB;
        break;

        case IGNIT_READY_TO_STDBY: start = MCU_IGNIT_READY;
        break;

        case PWR_READY_TO_STDBY: start = MCU_PWR_READY;
        break;

        case DRIVE_READY_TO_STDBY: start = MCU_DRIVE_READY;
        break;

        case NORM_OPS_TO_STDBY: start = MCU_NORM_OPS;
        break;

        case NORM_OPS_TO_PWR_READY: start = MCU_NORM_OPS;
        break;

        case DRIVE_READY_TO_PWR_READY: start = MCU_DRIVE_READY;
        break;
        }

        if(InverterState.MCU_State != start)
        {
            return -1;
        }

        uint8_t array[J1939_MSGLEN];

        for (int i = 0; i < J1939_MSGLEN; i++)
        {
            array[i] = CANTasks[speedMessageIndex].task.msg[i];
        }

        array[6] = stateTransition;

       j1939.Transmit(CANTasks[speedMessageIndex].task.priority, 
                            CANTasks[speedMessageIndex].task.PGN,
                            CANTasks[speedMessageIndex].task.srcAddr,
                            CANTasks[speedMessageIndex].task.destAddr,
                            &array[0], 
                            J1939_MSGLEN);
}

void TaskScheduler::UpdateSpeed(int currentPedalSpeed, int speedMessageIndex)
{
    /**
    * Updates current speed of the car.
    *
    * Parameters:
    *    currentPedalSpeed      (uint16_t): Speed of pedal (RPM).
    *    speedMessageIndex           (int): 6th byte of the speed mode in CAN Spec 2.3.1.2.
    * Returns:
    *    none
    **/

   if(currentPedalSpeed >1023)
   {
       currentPedalSpeed = 1023;
   }

    uint16_t newSpeed = 0;
    newSpeed = (currentPedalSpeed *.34100684)+32000;

    UpdateMsgByte(speedMessageIndex, newSpeed % 0xFF ,2);
    UpdateMsgByte(speedMessageIndex, newSpeed >> 8, 3);
}

//// Private Functions
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
