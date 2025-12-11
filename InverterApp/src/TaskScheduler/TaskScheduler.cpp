#include "TaskScheduler.h"
#include "../ARD1939/CAN_SPEC/StateTransition.h"
#include "../ARD1939/CAN_SPEC/MotorControlUnitState.h"

InitializedCANTask CANTasks[MAX_CAN_TASKS];
ARD1939 j1939;
extern struct CANVariables InverterState;

//// Main Tasks
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
    int j1939_init = j1939.Init(CAN_CONTROL_LOOP_INTERVAL_MS);
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

    uint8_t DefaultSpeedArray[] = {0xF4, 0x1B, 0x00, 0x7D, 0xFF, 0xFF, 0x00, 0x1F};
    uint8_t DefaultTorqueArray[] = {0xF4, 0x18, 0x00, 0x7D, 0xFF, 0xFF, 0x00, 0x1F};
    uint8_t DefaultAccumulatorArray[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    
    
    #ifdef INVERTER_CONTROLLER_MODE
    #ifndef USE_APPS
    TaskScheduler::SetupCANTask(0x04, COMMAND2_SPEED, 0xA2, 8, INVERTER_CMD_INVERVAL_TICKS, DefaultSpeedArray, INVERTER_CMD_MESSAGE_INDEX);
    #else
    TaskScheduler::SetupCANTask(0x04, COMMAND2_SPEED, 0xA2, 8, INVERTER_CMD_INVERVAL_TICKS, DefaultTorqueArray, INVERTER_CMD_MESSAGE_INDEX);
    #endif
    #endif

    #ifdef ACCUMULATOR_CONTROLLER_MODE
    #ifndef USE_APPS
    TaskScheduler::SetupCANTask(0x04, COMMAND2_SPEED, 0xA2, 8, ACCUMULATOR_CMD_INVERVAL_TICKS, DefaultAccumulatorArray, ACCUMULATOR_CMD_MESSAGE_INDEX);
    
    #else
    TaskScheduler::SetupCANTask(0x04, COMMAND2_SPEED, 0xA2, 8, ACCUMULATOR_CMD_INVERVAL_TICKS, DefaultAccumulatorArray, ACCUMULATOR_CMD_MESSAGE_INDEX);
    
    #endif
    #endif

    
    return 0;
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
    TaskScheduler::RecieveMessages();
    TaskScheduler::SendMessages();
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
    TickType_t current_ticks;
    for (int i = 0; i < MAX_CAN_TASKS; i++)
    {
        current_ticks = xTaskGetTickCount();
        if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED && CANTasks[i].initialized == true && ((CANTasks[i].lastRunTime == 0) || (current_ticks - CANTasks[i].lastRunTime) >= CANTasks[i].task.interval))
        {
            j1939.Transmit(CANTasks[i].task.priority,
                           CANTasks[i].task.PGN,
                           TaskScheduler::GetSourceAddress(),
                           CANTasks[i].task.destAddr,
                           &CANTasks[i].task.msg[0],
                           CANTasks[i].task.msgLen);
            CANTasks[i].lastRunTime = current_ticks;
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
    char sString[80];
    // DEBUG_INIT();

    InverterState.CAN_Bus_Status = j1939.Operate(&MsgId, &PGN, &Msg[0], &MsgLen, &DestAddr, &SrcAddr, &Priority);
    if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FAILED)
    {
        Serial.println("Address Claim Failed, resetting CAN stack.");
        TaskScheduler::Init();
    }
    else if (TaskScheduler::GetSourceAddress() == 0xFE && InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED)
    {
        Serial.println("Claimed Null Address, resetting CAN stack.");
        TaskScheduler::Init();
    }
    // J1939_MSG_APP means normal Data Packet && J1939_MSG_PROTOCOL means Transport Protocol Announcement.
    if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED && (MsgId == J1939_MSG_APP || MsgId == J1939_MSG_PROTOCOL))
    {
        // DEBUG_PRINTHEX("Source Address: ", SrcAddr);
        // DEBUG_PRINTHEX("Destination Address: ", DestAddr);
        // DEBUG_PRINTHEX("Priority: ", Priority);
        // DEBUG_PRINTHEX("PGN: ", PGN);
        // DEBUG_PRINTARRAYHEX("Msg: ", Msg, MsgLen);
        j1939.CANInterpret(&PGN, &Msg[0], &MsgLen, &DestAddr, &SrcAddr, &Priority);
    }
}

int TaskScheduler::AddCANTask(uint8_t priority, long PGN, uint8_t destAddr, int msgLen, TickType_t interval_ticks, uint8_t msg[J1939_MSGLEN])
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
    int firstFree = FirstFreeInCANTasks();
    if (firstFree != -1)
    {
        TaskScheduler::SetupCANTask(priority, PGN, destAddr, msgLen, interval_ticks, msg, firstFree);
    }
    return firstFree;
}

void TaskScheduler::SetupCANTask(uint8_t priority, long PGN, uint8_t destAddr, int msgLen, TickType_t interval_ticks, uint8_t msg[J1939_MSGLEN], int index)
{
    /**
     * Configure a new CANTask in the array with a given index.
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
     *     none
     **/
    CANTasks[index].task.priority = priority;
    CANTasks[index].task.PGN = PGN;
    CANTasks[index].task.destAddr = destAddr;
    CANTasks[index].task.msgLen = msgLen;
    CANTasks[index].task.interval = interval_ticks;
    for (int i = 0; i < msgLen; i++)
    {
        CANTasks[index].task.msg[i] = msg[i];
    }
    CANTasks[index].initialized = true;
    CANTasks[index].lastRunTime = 0;
}

// void TaskScheduler::EnableDriveMessage(void)
// {
//     /**
//      * Enables the Drive Message at INVERTER_CMD_MESSAGE_INDEX
//      *
//      * Parameters:
//      *     none
//      * Returns:
//      *     none
//      **/
//     CANTasks[INVERTER_CMD_MESSAGE_INDEX].initialized = true;
// }

// void TaskScheduler::DisableDriveMessage(void)
// {
//     /**
//      * Disables the Drive Message at INVERTER_CMD_MESSAGE_INDEX
//      *
//      * Parameters:
//      *     none
//      * Returns:
//      *     none
//      **/
//     CANTasks[INVERTER_CMD_MESSAGE_INDEX].initialized = false;
// }

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

void TaskScheduler::UpdateMsg(int taskIndex, int *msg, int msgLen)
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


void TaskScheduler::UpdateAccumulatorArray(uint16_t accumulatorValue, int accumulatorIndex, int byteIndex)
{
    /**
     * Updates current speed/requested torque of the car.
     *
     * Parameters:
     *    currentCommandedPower      (uint16_t): Commanded power prescaled for either the speed or torque message.
     *    commandedPowerIndex           (int): Index of the commanded power message in CANTasks array.
     * Returns:
     *    none
     **/
    uint8_t top_byte = (uint8_t)(accumulatorValue % 0x100);
    uint8_t bottom_byte = (uint8_t)(accumulatorValue >> 8);
    UpdateMsgByte(accumulatorIndex, top_byte, byteIndex); //2
    UpdateMsgByte(accumulatorIndex, bottom_byte, byteIndex + 1); //3
}

// bool TaskScheduler::ChangeState(int stateTransition, int speedMessageIndex)
// {

//     /**
//      * Transition through the inverter state machine.
//      * State commands can be found in StateTransition.h
//      * Motor Contorl Unit State Definitions can be found in MotorControlUnitState.h
//      *
//      * Parameters:
//      *    stateTransition             (int): State transition command from CAN Spec 2.3.3.
//      *    speedMessageIndex           (int): 6th byte of the speed mode in CAN Spec 2.3.1.2.
//      * Returns:
//      *    False if InverterState.MCU_State isn't in the commanded start state. True otherwise.
//      **/

//     int start;
//     int end;

//     extern struct CANVariables InverterState;

//     switch (stateTransition)
//     {
//     case STDBY_TO_FUNCTIONAL_DIAG:
//         start = MCU_STDBY;
//         break;

//     case PWR_READY_TO_PWR_DIAG:
//         start = MCU_PWR_READY;
//         break;

//     case DRIVE_READY_TO_NORM_OPS:
//         start = MCU_DRIVE_READY;
//         break;

//     case NORM_OPS_TO_DISCHARGE_DIAG:
//         start = MCU_NORM_OPS;
//         break;

//     case FAULT_CLASSA_TO_STDBY:
//         start = MCU_FAULT_CLASSA;
//         break;

//     case IGNIT_READY_TO_ADV_DIAG_CLASSA:
//         start = MCU_IGNIT_READY;
//         break;

//     case FAULT_CLASSA_TO_ADV_DIAG_CLASSA:
//         start = MCU_FAULT_CLASSA;
//         break;

//     case FAULT_CLASSB_TO_PWR_READY:
//         start = MCU_FAULT_CLASSB;
//         break;

//     case NORM_OPS_TO_DRIVE_READY:
//         start = MCU_NORM_OPS;
//         break;

//     case PWR_READY_TO_ADV_DIAG_CLASSB:
//         start = MCU_PWR_READY;
//         break;

//     case FAULT_CLASSB_TO_ADV_DIAG_CLASSB:
//         start = MCU_FAULT_CLASSB;
//         break;

//     case DRIVE_READY_TO_ADV_DIAG_CLASSB:
//         start = MCU_DRIVE_READY;
//         break;

//     case FAULT_CLASSB_TO_FAIL_SAFE:
//         start = MCU_FAULT_CLASSB;
//         break;

//     case FAULT_CLASS_B_ADV_DIAG_CLASSA_TO_FAIL_SAFE:
//         start = MCU_ADV_DIAG_CLASSA;
//         break;

//     case PWR_READY_TO_DRIVE_READY:
//         start = MCU_PWR_READY;
//         break;

//     case STDBY_TO_ADV_DIAG_CLASSA:
//         start = MCU_STDBY;
//         break;

//     case STDBY_TO_IGNIT_READY:
//         start = MCU_STDBY;
//         break;

//     case ADV_DIAG_CLASSB_TO_PWR_READY:
//         start = MCU_ADV_DIAG_CLASSB;
//         break;

//     case ADV_DIAG_CLASSA_TO_STDBY:
//         start = MCU_ADV_DIAG_CLASSA;
//         break;

//     case FAULT_CLASSB_TO_STDBY:
//         start = MCU_FAULT_CLASSB;
//         break;

//     case IGNIT_READY_TO_STDBY:
//         start = MCU_IGNIT_READY;
//         break;

//     case PWR_READY_TO_STDBY:
//         start = MCU_PWR_READY;
//         break;

//     case DRIVE_READY_TO_STDBY:
//         start = MCU_DRIVE_READY;
//         break;

//     case NORM_OPS_TO_STDBY:
//         start = MCU_NORM_OPS;
//         break;

//     case NORM_OPS_TO_PWR_READY:
//         start = MCU_NORM_OPS;
//         break;

//     case DRIVE_READY_TO_PWR_READY:
//         start = MCU_DRIVE_READY;
//         break;
//     }

//     if (InverterState.MCU_State != start)
//     {
//         return false;
//     }

//     uint8_t array[J1939_MSGLEN];

//     for (int i = 0; i < J1939_MSGLEN; i++)
//     {
//         array[i] = CANTasks[speedMessageIndex].task.msg[i];
//     }

//     array[6] = stateTransition;

//     if (InverterState.CAN_Bus_Status == ADDRESSCLAIM_FINISHED)
//     {
//     j1939.Transmit(CANTasks[speedMessageIndex].task.priority,
//                    CANTasks[speedMessageIndex].task.PGN,
//                    TaskScheduler::GetSourceAddress(),
//                    CANTasks[speedMessageIndex].task.destAddr,
//                    &array[0],
//                    J1939_MSGLEN);
//     }
//     return true;
// }

void TaskScheduler::UpdateCommandedPower(uint16_t currentCommandedPower, int commandedPowerIndex)
{
    /**
     * Updates current speed/requested torque of the car.
     *
     * Parameters:
     *    currentCommandedPower      (uint16_t): Commanded power prescaled for either the speed or torque message.
     *    commandedPowerIndex           (int): Index of the commanded power message in CANTasks array.
     * Returns:
     *    none
     **/
    uint8_t top_byte = (uint8_t)(currentCommandedPower % 0x100);
    uint8_t bottom_byte = (uint8_t)(currentCommandedPower >> 8);
    UpdateMsgByte(commandedPowerIndex, top_byte, 2);
    UpdateMsgByte(commandedPowerIndex, bottom_byte, 3);
}

// void TaskScheduler::ClearInverterFaults(void)
// {
//     /**
//      * Clears Fault Table and sends DM3 && DM11 Messages.
//      * DM3: Clear of Previously Active Diagnostic Trouble Codes
//      * DM11: Clear of Active Diagnostic Trouble Codes
//      *
//      * Parameters:
//      *    none
//      * Returns:
//      *    none
//      **/
//     j1939.ClearFaults();
//     if (InverterState.MCU_State == MCU_FAULT_CLASSA)
//     {
//         Serial.println("State is MCU Class A");
//         TaskScheduler::ChangeState(FAULT_CLASSA_TO_STDBY, INVERTER_CMD_MESSAGE_INDEX);
//     }
//     else if (InverterState.MCU_State == MCU_FAULT_CLASSB)
//     {
//         Serial.println("State is MCU Class B");
//         TaskScheduler::ChangeState(FAULT_CLASSB_TO_STDBY, INVERTER_CMD_MESSAGE_INDEX);
//     }
// }

//// Private Functions
int TaskScheduler::FirstFreeInCANTasks()
{
    /**
     * Iterates over the CANTasks array until it finds the first open spot in the array.
     * Will not return the index of the Control Message (0)
     *
     * Parameters:
     *     none
     * Returns:
     *     i (int): index of the first free object in array
     **/
    for (int i = 0; i < MAX_CAN_TASKS; i++)
    {
        if (CANTasks[i].initialized == false)
        {
            return i;
        }
    }
    return -1;
}