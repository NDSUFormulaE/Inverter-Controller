#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H
#include <stdlib.h>
#include <Arduino.h>
#include "../FreeRTOS/src/Arduino_FreeRTOS.h"

#include "../ARD1939/ARD1939.h"
#include "../ARD1939/CAN_SPEC/PGN.h"
#include "../AppConfig.h"

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
        InitializedCANTask CANTasks[MAX_CAN_TASKS];
        ARD1939 j1939;
};
#endif /* TASK_SCHEDULER_H */