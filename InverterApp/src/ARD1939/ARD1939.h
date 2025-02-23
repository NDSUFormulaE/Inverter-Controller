#include "CAN_SPEC/CANVariables.h"

#ifndef ARD1939_H
#define ARD1939_H

// Arduino Definitions
#define MONITOR_BAUD_RATE                      115200
#define CS_PIN                                 10     // Use pin 10 for Seeed Studio CAN Shield up to version 1.0
                                                     // Use pin 9 for Seeed Studio CAN Shield version 1.1 and higher

// System Settings
#define SYSTEM_TIME                             1    // Milliseconds

// Program Version
// -----------------------------------------------
// 0 - ARD1939-Uno
// 1 - ARD1939-Uno/TP
// 2 - ARD1939-Mega
#define ARD1939VERSION                          1

// J1939 Settings
#if ARD1939VERSION == 0
  #define TRANSPORT_PROTOCOL                    0
  #define J1939_MSGLEN                          8
  #define MSGFILTERS                            10
#endif

#if ARD1939VERSION == 1
  #define TRANSPORT_PROTOCOL                    1
  #define J1939_MSGLEN                          8
  #define MSGFILTERS                            10
#endif

#if ARD1939VERSION == 2
  #define TRANSPORT_PROTOCOL                    0
  #define J1939_MSGLEN                          8
  #define MSGFILTERS                            0
#endif

#define SA_PREFERRED                      	0x03
#define ADDRESSRANGEBOTTOM                	0x04
// Set this lower than the inverter's address, since we wont ever
// have enough devices on the bus to need this whole range.
#define ADDRESSRANGETOP                   	0xA1

#define GLOBALADDRESS                    	255
#define NULLADDRESS                      	254

// NAME Fields Default 
#define NAME_IDENTITY_NUMBER              	0x000001
#define NAME_MANUFACTURER_CODE            	0xFFF
#define NAME_FUNCTION_INSTANCE            	0
#define NAME_ECU_INSTANCE                 	0x00
#define NAME_FUNCTION                     	0x02
#define NAME_RESERVED                     	0
#define NAME_VEHICLE_SYSTEM               	0x0
#define NAME_VEHICLE_SYSTEM_INSTANCE      	0
#define NAME_INDUSTRY_GROUP               	0x00
#define NAME_ARBITRARY_ADDRESS_CAPABLE    	0x01

// Return Codes
#define ADDRESSCLAIM_INIT                   0
#define ADDRESSCLAIM_INPROGRESS           	1
#define ADDRESSCLAIM_FINISHED             	2
#define NORMALDATATRAFFIC                 	2
#define ADDRESSCLAIM_FAILED               	3

#define J1939_MSG_NONE                   	  0
#define J1939_MSG_PROTOCOL               	  1
#define J1939_MSG_NETWORKDATA               2
#define J1939_MSG_APP                   	  3

// Compiler Settings
#define OK                                  0
#define ERR                                 1

// Debugger Settings
#define DEBUG                               1

#if DEBUG == 1

  #define DEBUG_INIT() char sDebug[128];
  #define DEBUG_PRINTHEX(T, v) Serial.print(T); sprintf(sDebug, "%x\n\r", v); Serial.print(sDebug);
  #define DEBUG_PRINTDEC(T, v) Serial.print(T); sprintf(sDebug, "%d\n\r", v); Serial.print(sDebug);
  #define DEBUG_PRINTARRAYHEX(T, a, l) Serial.print(T); if(l == 0) Serial.print("Empty.\n\r"); else {for(int x=0; x<l; x++){sprintf(sDebug, "%x ", a[x]); Serial.print(sDebug);} Serial.print("\n\r");}
  #define DEBUG_PRINTARRAYDEC(T, a, l) Serial.print(T); if(l == 0) Serial.print("Empty.\n\r"); else {for(int x=0; x<l; x++){sprintf(sDebug, "%d ", a[x]); Serial.print(sDebug);} Serial.print("\n\r");}
  #define DEBUG_HALT() while(Serial.available() == 0); Serial.setTimeout(1); Serial.readBytes(sDebug, 1);

#endif

struct v35
{
  int v36;
  bool v21;
  bool v37;
};

struct FaultEntry
{
    unsigned long SPN = 0;
    uint8_t FMI = 0;
    uint8_t OC = 0;
    uint8_t active = 0;
};

struct FaultString
{
  char* FaultTable;
  uint32_t length;
};

enum {MAX_FAULTS = 25};
enum {TP_BUFFER_LENGTH = 1785};

class ARD1939
{
  public: 
    // Initialization
    uint8_t Init(int nSystemTime);
    void SetPreferredAddress(uint8_t nAddr);
    void SetAddressRange(uint8_t nAddrBottom, uint8_t nAddrTop);
    void SetNAME(long lIdentityNumber, int nManufacturerCode, uint8_t nFunctionInstance, uint8_t nECUInstance, 
                           uint8_t nFunction, uint8_t nVehicleSystem, uint8_t nVehicleSystemInstance, uint8_t nIndustryGroup, uint8_t nArbitraryAddressCapable);
    uint8_t SetMessageFilter(long lPGN);
  
    // Read/Write - Check Status
    uint8_t Operate(uint8_t* nMsgId, long* lPGN, uint8_t* pMsg, int* nMsgLen, uint8_t* nDestAddr, uint8_t* nSrcAddr, uint8_t* nPriority);
    uint8_t Transmit(uint8_t nPriority, long lPGN, uint8_t nSourceAddress, uint8_t nDestAddress, uint8_t* pData, int nDataLen);
  
    // Other Application Functions
    void Terminate(void);
    uint8_t GetSourceAddress(void);
    void DeleteMessageFilter(long lPGN);
    void CANInterpret(long* CAN_PGN, uint8_t* CAN_Message, int* CAN_MessageLen, uint8_t* CAN_DestAddr, uint8_t* CAN_SrcAddr, uint8_t* CAN_Priority);
    bool CheckValidState(void);
    int UpdateAddFault(unsigned long SPN, uint8_t FMI, uint8_t Occurance);
    void ClearFaults(void);
  private:
    uint8_t f01(uint8_t, uint8_t*);
    bool f02(void);
    uint8_t f03(uint8_t*, uint8_t*);
    uint8_t f04(long*, uint8_t*, int*, uint8_t*, uint8_t*, uint8_t*);
    void f05(void);
    void f06(struct v35*);
    bool f07(long*, uint8_t*);
    bool f08(long);
    bool f09(long);
    bool isFaultTableClear();
    int isFaultInArray(unsigned long SPN, uint8_t FMI);
    int FirstFreeInFaultArray();
    int AddNewFault(unsigned long SPN, uint8_t FMI, uint8_t Occurance);
    bool TPMessageRecived(uint8_t num_packets);
    void DecodeTransportProtocol();
    void ClearFaultTable(void);
    FaultEntry FaultTable[MAX_FAULTS];
    

#if TRANSPORT_PROTOCOL == 1
    uint8_t f10(uint8_t, long, uint8_t, uint8_t, uint8_t*, int);
    void f11(uint8_t);
    void f12(uint8_t);
    uint8_t f13(long, uint8_t*, int, uint8_t, uint8_t, uint8_t);
#endif
  
}; // end class ARD1939

#endif
