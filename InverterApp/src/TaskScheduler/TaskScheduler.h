#include <stdlib.h>
#include <TimeLib.h>
#include <Arduino.h>

#include "../ARD1939/ARD1939.h"
#include "../ARD1939/CAN_SPEC/PGN.h"

enum {MAX_TASKS = 5};
enum {MAX_FAULTS = 20};
#define INVERTER_CMD_MESSAGE_INDEX 0

struct CANTask
{
    uint8_t priority;
    long PGN;
    uint8_t srcAddr;
    uint8_t destAddr;
    int msgLen;
    unsigned long interval;
    uint8_t msg[J1939_MSGLEN];
};

struct InitializedCANTask
{
    CANTask task;
    bool initialized = false;
    unsigned long lastRunTime = 0;
};

struct MsgReturn
{
    int length;
    uint8_t * message[J1939_MSGLEN];
};

struct FaultEntry
{
    uint32_t SPN;
    uint8_t FMI;
    uint8_t OC;
    uint8_t CM;
    boolean active = true;
};

class TaskScheduler
{
    public:
        bool Init();
        int AddCANTask(uint8_t priority, long PGN, uint8_t srcAddr, uint8_t destAddr, int msgLen, unsigned long interval, uint8_t msg[J1939_MSGLEN]);
        void RemoveCANTask(int taskIndex);
        MsgReturn GetMsg(int taskIndex);
        void UpdateMsg(int taskIndex, int msg[], int msgLen);
        void UpdateMsgByte(int taskIndex, int byte, int msgIndex);
        void RunLoop();
        uint8_t GetSourceAddress();
        bool ChangeState(int StateTransition, int speedMessageIndex);
        void UpdateSpeed(uint16_t CurrentPedalSpeed, int speedMessageIndex);
        int UpdateAddFault(uint32_t SPN, uint8_t FMI, uint8_t Occurance, uint8_t CM);
    private:
        void SendMessages();
        void RecieveMessages();
        int FirstFreeInCANTasks();
        bool isFaultTableClear();
        int isFaultInArray(uint32_t SPN, uint8_t FMI);
        int FirstFreeInFaultArray();
        int AddNewFault(uint32_t SPN, uint8_t FMI, uint8_t Occurance, uint8_t CM);
        InitializedCANTask CANTasks[MAX_TASKS];
        FaultEntry FaultTable[MAX_FAULTS];
        ARD1939 j1939;
};