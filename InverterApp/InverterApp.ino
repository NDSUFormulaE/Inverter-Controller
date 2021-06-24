#include <stdlib.h>
#include <inttypes.h>
#include <SPI.h>

// CAN library and PD400 definitions
#include "src/ARD1939/ARD1939.h"
#include "src/ARD1939/CAN_SPEC/PGN.h"

// Task Scheduler
#include "src/TaskScheduler/src/TaskScheduler.h"

// Definitions
ARD1939 j1939;
Scheduler taskMan;

extern struct CANVariables InverterState;
int nCounter = 0;
void(* resetFunc) (void) = 0; //declare reset function @ address 0

// TODO: Add tasks for the periodic messages

// ------------------------------------------------------------------------
//  Setup routine runs on power-up or reset
// ------------------------------------------------------------------------
void setup() 
{
  // Set the serial interface baud rate
  Serial.begin(MONITOR_BAUD_RATE);
  
  // Initialize the J1939 protocol including CAN settings
  if(j1939.Init(SYSTEM_TIME) == 0)
    Serial.print("CAN Controller Init OK.\n\r");
  else{
    Serial.print("CAN Controller Init Failed.\n\r");
    delay(500);
    resetFunc(); // If CAN Controller doesnt init correctly, wait 100ms then try again.
  }    
 // Set the preferred address and address range
 j1939.SetPreferredAddress(SA_PREFERRED);
 j1939.SetAddressRange(ADDRESSRANGEBOTTOM, ADDRESSRANGETOP);
 
 // Set the message filter
 //j1939.SetMessageFilter(59999);
 
 //j1939.SetMessageFilter(65242);
 //j1939.SetMessageFilter(65259);
 //j1939.SetMessageFilter(65267);
 
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

// Initialize the TaskScheduler
taskMan.init();

}// end setup

// ------------------------------------------------------------------------
// Main Loop - Arduino Entry Point
// ------------------------------------------------------------------------
void loop()
{
  // J1939 Variables
  byte nMsgId;
  byte nDestAddr;
  byte nSrcAddr;
  byte nPriority;
  byte nJ1939Status;
  
  int nMsgLen;
  
  long lPGN;
  char sString[80];
  byte pMsg[J1939_MSGLEN];
  
  // Variables for proof of concept tests
  byte msgFakeNAME[] = {0xde, 0xad, 0xbe, 0xef, 0, 0, 0, 0x69};
  
  // Establish the timer base in units of milliseconds
  delay(SYSTEM_TIME);
  
  // Call the J1939 protocol stack
  nJ1939Status = j1939.Operate(&nMsgId, &lPGN, &pMsg[0], &nMsgLen, &nDestAddr, &nSrcAddr, &nPriority);

  // Check for reception of PGNs for our ECU/CA
  switch(nJ1939Status)
  {
    case ADDRESSCLAIM_INPROGRESS:     
      break;
    
    case ADDRESSCLAIM_FAILED:
    // TODO: This needs to be way cleaner. We don't want to restart the unit while
    // we are sending important commands as the PD400 will shut down.
      Serial.print("AddressClaim Failed\n\r");
      resetFunc();
      delay(1000);
      break;
    
    case NORMALDATATRAFFIC:
      j1939.CANInterpret(&nMsgId, &lPGN, &pMsg[0], &nMsgLen, &nDestAddr, &nSrcAddr, &nPriority);
      //Log incoming messages to Serial. For Testing.
      // if(nMsgLen != 0){
      //   sprintf(sString, "PGN: 0x%X Src: 0x%X Dest: 0x%X ", (int)lPGN, nSrcAddr, nDestAddr);
      //   Serial.print(sString);
      //   Serial.print("Data: ");
      //   for(int nIndex = 0; nIndex < nMsgLen; nIndex++)
      //   {          
      //     sprintf(sString, "0x%X ", InverterState.Last_Message[nIndex]);
      //     Serial.print(sString);
      //   }
      //   Serial.print("\n\r");
      // }
      // nMsgId = J1939_MSG_NONE;
      break;
      

    
  }// end switch(nJ1939Status)  
}// end loop
