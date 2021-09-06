#include <stdlib.h>
#include <TimeLib.h>
#include <Arduino.h>

#include "../ARD1939/ARD1939.h"

// "#defines"
enum {MAX_TASKS = 20};

// Task Structure Def
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

class TaskScheduler
{
    public:
        int Init();
        int AddCANTask(uint8_t priority, long PGN, uint8_t srcAddr, uint8_t destAddr, int msgLen, unsigned long interval, uint8_t msg[J1939_MSGLEN]);
        void RemoveCANTask(int taskIndex);
        MsgReturn GetMsg(int taskIndex);
        void UpdateMsg(int taskIndex, int msg[], int msgLen);
        void UpdateMsgByte(int taskIndex, int byte, int msgIndex);
        void RunLoop();
    private:
        void SendMessages();
        void RecieveMessages();
        int FirstFreeInArray();
        InitializedCANTask CANTasks[MAX_TASKS];
        ARD1939 j1939;
};