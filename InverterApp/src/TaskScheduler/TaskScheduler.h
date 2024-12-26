#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <stdlib.h>
#include "../Time/TimeLib.h"
#include <Arduino.h>

#include "../ARD1939/ARD1939.h"
#include "../ARD1939/CAN_SPEC/PGN.h"

// We currntly support 3 tasks, and only use one. If needed we can add more.
enum {MAX_TASKS = 3};
#define INVERTER_CMD_MESSAGE_INDEX 0

// Default CAN message configuration
#define DEFAULT_INVERTER_CMD_PRIORITY 0x04
#define DEFAULT_INVERTER_CMD_PGN COMMAND2_SPEED
#define DEFAULT_INVERTER_CMD_DEST_ADDR 0xA2
#define DEFAULT_INVERTER_CMD_MSG_LEN 8
#define DEFAULT_INVERTER_CMD_INTERVAL 15

struct CANTask
{
    uint8_t priority;
    long PGN;
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

class TaskScheduler
{
    private:
        void SendMessages();
        void RecieveMessages();
        int FirstFreeInCANTasks();
        void SetupCANTask(uint8_t priority, long PGN, uint8_t destAddr, 
                         int msgLen, unsigned long interval, uint8_t msg[J1939_MSGLEN], int index);
        void SetupDefaultCANTasks();  // New private function to setup default CAN tasks
        InitializedCANTask CANTasks[MAX_TASKS];
        ARD1939 j1939;

    public:
        int Init();
        int AddCANTask(uint8_t priority, long PGN, uint8_t destAddr, 
                      int msgLen, unsigned long interval, uint8_t msg[J1939_MSGLEN]);
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
};

#endif /* TASK_SCHEDULER_H */