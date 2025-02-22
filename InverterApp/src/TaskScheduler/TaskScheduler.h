#include <stdlib.h>
#include <Arduino.h>
#include "../FreeRTOS/src/Arduino_FreeRTOS.h"

#include "../ARD1939/ARD1939.h"
#include "../ARD1939/CAN_SPEC/PGN.h"

// Increase this if we actually need more than 1 task.
enum {MAX_TASKS = 1};

#define INVERTER_CMD_MESSAGE_INDEX 0
#define INVERTER_CMD_INTERVAL_MS 15
#define INVERTER_CMD_INVERVAL_TICKS pdMS_TO_TICKS(INVERTER_CMD_INTERVAL_MS)
#define CAN_CONTROL_LOOP_INTERVAL_MS 8
#define CAN_CONTROL_LOOP_INTERVAL_TICKS pdMS_TO_TICKS(CAN_CONTROL_LOOP_INTERVAL_MS)

#if INVERTER_CMD_INTERVAL_MS < CAN_CONTROL_LOOP_INTERVAL_MS
    #error "INVERTER_CMD_INTERVAL_MS must be greater than CAN_CONTROL_LOOP_INTERVAL_MS"
#endif

// Enable this value to have TaskClearFaults clear the faults when the
// inverter goes into either of the Fault Class states.
// #define AUTO_CLEAR_CAN_FAULTS

struct CANTask
{
    uint8_t priority;
    long PGN;
    uint8_t destAddr;
    int msgLen;
    TickType_t interval;
    uint8_t msg[J1939_MSGLEN];
};

struct InitializedCANTask
{
    CANTask task;
    bool initialized = false;
    TickType_t lastRunTime = 0;
};

struct MsgReturn
{
    int length;
    uint8_t * message[J1939_MSGLEN];
};

class TaskScheduler
{
    public:
        int Init();
        int AddCANTask(uint8_t priority, long PGN, uint8_t destAddr, int msgLen, TickType_t interval_ticks, uint8_t msg[J1939_MSGLEN]);
        void RemoveCANTask(int taskIndex);
        MsgReturn GetMsg(int taskIndex);
        void UpdateMsg(int taskIndex, int msg[], int msgLen);
        void UpdateMsgByte(int taskIndex, int byte, int msgIndex);
        void RunLoop();
        uint8_t GetSourceAddress();
        bool ChangeState(int StateTransition, int speedMessageIndex);
        void UpdateSpeed(uint16_t CurrentPedalSpeed, int speedMessageIndex);
        void EnableDriveMessage(void);
        void DisableDriveMessage(void);
        void ClearInverterFaults(void);
    private:
        void SendMessages();
        void RecieveMessages();
        int FirstFreeInCANTasks();
        void SetupCANTask(uint8_t priority, long PGN, uint8_t destAddr, int msgLen, TickType_t interval_ticks, uint8_t msg[J1939_MSGLEN], int index);
        InitializedCANTask CANTasks[MAX_TASKS];
        ARD1939 j1939;
};