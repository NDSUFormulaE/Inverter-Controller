// ------------------------------------------------------------------------
// J1939 CAN Connection
// ------------------------------------------------------------------------
#include <inttypes.h>
#include "mcp_can.h"
#include "ARD1939.h"
#include "CAN_SPEC/PGN.h"

MCP_CAN CAN0(CS_PIN);                                      // Set CS to pin 9/10

// ------------------------------------------------------------------------
// CAN message ring buffer setup
// ------------------------------------------------------------------------
#define CANMSGBUFFERSIZE 10
struct CANMsg
{
  long lID;
  unsigned char pData[8];
  int nDataLen;
};
CANMsg CANMsgBuffer[CANMSGBUFFERSIZE];
int nWritePointer;
int nReadPointer;

// ------------------------------------------------------------------------
// Initialize the CAN controller
// ------------------------------------------------------------------------
byte canInit(void)
{
  // Default settings
  nReadPointer = 0;
  nWritePointer = 0;
  
  // Initialize the CAN controller
  if(CAN0.begin(CAN_1000KBPS) == 0)
    return 0;
  else return 1;

}// end canInitialize

// ------------------------------------------------------------------------
// Check CAN controller for error
// ------------------------------------------------------------------------
byte canCheckError(void)
{
  if(CAN0.checkError() == 0)
    return 0;
  else return 1;
  
}// end canCheckError

// ------------------------------------------------------------------------
// Transmit CAN message
// ------------------------------------------------------------------------
byte canTransmit(long lID, unsigned char* pData, int nDataLen)
{
  
  if(CAN0.sendMsgBuf(lID, CAN_EXTID, nDataLen, pData) == 0)
    return 0;
  else
    return 1;
  
}// end canTransmit

// ------------------------------------------------------------------------
// Receive CAN message
// ------------------------------------------------------------------------
byte canReceive(long* lID, unsigned char* pData, int* nDataLen)
{
  // Declarations
  byte nCnt;

  // In case there is a message, put it into the buffer
  while(CAN0.checkReceive() == CAN_MSGAVAIL)
  {
    // Read the message buffer
    CAN0.readMsgBuf(&nCnt, &CANMsgBuffer[nWritePointer].pData[0]);
    CANMsgBuffer[nWritePointer].nDataLen = (int)nCnt;
    CANMsgBuffer[nWritePointer].lID = CAN0.getCanId();    
    
    if(++nWritePointer == CANMSGBUFFERSIZE)
      nWritePointer = 0;
    
  }

  // Check ring buffer for a message
  if(nReadPointer != nWritePointer)
  {
    // Read the next message buffer entry
    *nDataLen = CANMsgBuffer[nReadPointer].nDataLen;
    *lID = CANMsgBuffer[nReadPointer].lID;

    for(int nIdx = 0; nIdx < *nDataLen; nIdx++)
      pData[nIdx] = CANMsgBuffer[nReadPointer].pData[nIdx];

    if(++nReadPointer == CANMSGBUFFERSIZE)
      nReadPointer = 0;
    
    return 0;
  }
  else return 1;

}// end canReceive

void CANInterpret(byte* CAN_messageID, long* CAN_PGN, byte* CAN_Message, uint32_t* CAN_MessageLen, byte* CAN_DestAddr, byte* CAN_SrcAddr, byte* CAN_Priority, byte* StorageArray, uint32_t* StorageArrayLen){
  
  byte[CAN_MessageLen] message;
  for(int i = 0; i < CAN_MessageLen; i++)
    message[i] = CAN_MessageLen[i];

  // TO ADD: pull stored inverter address from memory table and ensure that we run this switch statement when from the inverter
  switch(CAN_PGN){
    case ADDRESS_CLAIM_RESPONSE:

      // pull info into name table
      // US9
      break;
    
    // Shared PGN for STATUS1_RELTORQUE_SPEED, STATUS2_STATE_VOLTAGE, PROGNOSTIC1_RMS_CURRENT, PROGNOSTIC2_DIAGNOSTIC,
    // PROGNOSTIC3_DIAGNOSTIC & PROGNOSTIC5_POSITION
    case STATUS1_RELTORQUE_SPEED:

      // STATUS1_RELTORQUE_SPEED
      if(message[0] == 0x79 && message[1] = 0xFF){
        float Avg_Torque_Percent = ((message[2] + (message[3] << 8)) * 0.00390625) - 125.0;
        // Check if these machine speeds are different
        float Rel_Machine_Speed = ((message[4] + (message[5] << 8)) * 0.5) - 16000.0;        // Units: RPM
      }
      // STATUS2_STATE_VOLTAGE
      else if(message[0] == 0x77 && message[1] = 0xFF){
        uint32_t MCU_State = message[2];
        float DC_Bus_Voltage = (message[3] + (message[4] << 8)) * 0.03125;                   // Units: Volts
        uint32_t Derate_Owner = message[5];
        uint32_t Diag_Function = message[6] + ((message[7] >> 3) << 8);
        uint32_t Diag_Status = message[7] % 8; // pulls the last 3 bits from the byte
      }
      // PROGNOSTIC1_RMS_CURRENT
      else if(message[0] == 0x7A){
        float RMS_Current_Phase_A = (message[1] + (message[2] << 8)) * 0.0625;               // Units: Amps
        float RMS_Current_Phase_B = (message[3] + (message[4] << 8)) * 0.0625;               // Units: Amps
        float RMS_Current_Phase_C = (message[5] + (message[6] << 8)) * 0.0625;               // Units: Amps
        uint32_t Brake_Resistor_RMS_Current = message[7];                                    // Units: Amps
      }
      // PROGNOSTIC2_DIAGNOSTIC
      else if(message[0] == 0xF7){
        float Brake_Resistance = (message[1] + (message[2] << 8)) * 0.5;                     // Units: milliOhm
        float DC_Link_Capacitance = (message[3] + (message[4] << 8)) * 0.5;                  // Units: microFarad
        float Motor_BEMF = (message[5] + (message[6] << 8)) * 0.000030517578125;             // Units: Volts/RPM
        uint32_t EMI_Capacitance = message[7] * 32;                                          // Units: nanoFarad
      }
      // PROGNOSTIC3_DIAGNOSTIC
      else if(message[0] == 0xF8){
        float Machine_Speed_200ms_Avg = ((message[1] + (message[2] << 8)) * 0.5) - 16000.0;  // Units: RPM
        float Mach_Torq_Percent_200ms_Avg = ((message[3] + (message[4] << 8)) * 0.00390625) - 16000.0; 
      // PROGNOSTIC5_POSITION
      else if(message[0] == 0x81){
        float Stored_Pos_Offset = (message[2] + (message[3] << 8)) * 0.0078125;              // Units: Elec. Degrees
        float Calculated_Pos_Offset = (message[4] + (message[5] << 8)) * 0.0078125;          // Units: Elec. Degrees
      }
      break;

    // Shared PGN for STATUS3_ABSTORQUE_SPEED, DC_LINK_PWR_STATUS && VOLTAGE_RMS1
    case STATUS3_ABSTORQUE_SPEED:

      // STATUS3_ABSTORQUE_SPEED
      if(message[0] == 0x00 && message[0] == 0x51){
        float Avg_Abs_Torque = ((message[2] + (message[3] << 8)) * 0.1) - 3200.0;            // Units: Nm
        // Check if these machine speeds are different
        float Abs_Machine_Speed = ((message[4] + (message[5] << 8)) * 0.5) - 16000.0;        // Units: RPM
      }
      // DC_LINK_PWR_STATUS
      else if(message[0] == 0x00 && message[0] == 0x56){
        float Actual_Power = ((message[2] + (message[3] << 8)) * 0.001) - 32.0;
        float Max_Power_Generating = (message[4] + (message[5] << 8)) * 0.001;
        float Max_Power_Motoring = (message[6] + (message[7] << 8)) * 0.001;
      }
      // VOLTAGE_RMS1
      else if(message[0] == 0x00 && message[0] == 0x54){
        float RMS_Voltage_Phase_A = (message[2] + (message[3] << 8)) * 0.03125;               // Units: Amps
        float RMS_Voltage_Phase_B = (message[4] + (message[5] << 8)) * 0.03125;               // Units: Amps
        float RMS_Voltage_Phase_C = (message[6] + (message[7] << 8)) * 0.03125;               // Units: Amps
      }

      break;

    // Shared PGN for STATUS4_TORQUE_PWRSTAGE_OVRLD, INVERTER_TEMP1_IGBT, AC_SUPPLY_STATUS & DC_LINK_PWR_CURRENT_STATUS
    case STATUS4_TORQUE_PWRSTAGE_OVRLD:

      // STATUS4_TORQUE_PWRSTAGE_OVRLD
      if(message[0] == 0x32 && message[0] == 0xFF){
      float Neg_Torque_Available = ((message[2] + (message[3] << 8)) * 0.1) - 3200.0;        // Units: Nm
      float Pos_Torque_Available = ((message[2] + (message[3] << 8)) * 0.1) - 3200.0;        // Units: Nm
      // Power Stage Status Values
      // 0 = Outputs Off
      // 1 = Normal Switching
      // 2 = High Side Three Phase Short
      // 3 = Low Side Three Phase Short
      uint32_t Power_Stage_Status = message[6];
      float Overload_Percent = message[7] * 0.5;
      }
      // INVERTER_TEMP1_IGBT
      else if(message[0] == 0x90){
        int32_t IGBT1_Temp = message[1] - 40;                                                // Units: Degrees Celcius
        int32_t IGBT2_Temp = message[2] - 40;                                                // Units: Degrees Celcius
        int32_t IGBT3_Temp = message[3] - 40;                                                // Units: Degrees Celcius
        int32_t IGBT4_Temp = message[4] - 40;                                                // Units: Degrees Celcius
        int32_t IGBT5_Temp = message[5] - 40;                                                // Units: Degrees Celcius
        int32_t IGBT6_Temp = message[6] - 40;                                                // Units: Degrees Celcius
        int32_t Brake_Chopper_IGBT_Temp = message[7] - 40;                                   // Units: Degrees Celcius
      }
      // AC_SUPPLY_STATUS
      else if(message[0] == 0x31 && message[0] == 0xFF){
        uint32_t AC_Voltage_Output = message[2] + (message[3] << 8);                         // Units: Vrms
        uint32_t AC_Frequency = message[4] + (message[5] << 8);                              // Units: Hz
        uint32_t AC_Voltage_Desired = message[6] + (message[7] << 8);                        // Units: Vrms
      }
      // DC_LINK_PWR_CURRENT_STATUS
      else if(message[0] == 0x36 && message[0] == 0xFF){
        float Actual_Current = ((message[2] + (message[3] << 8)) * 0.001) - 32.0;
        float Max_Current_Generating = (message[4] + (message[5] << 8)) * 0.001;
        float Max_Current_Motoring = (message[6] + (message[7] << 8)) * 0.001;
      }

      break;

    case INVERTER_TEMP2_MACHINE:
      if(message[0] == 0xE4){
        int32_t Motor_Temp_1 = message[1] - 40;                                              // Units: degrees Celcius
        int32_t Motor_Temp_2 = message[2] - 40;                                              // Units: degrees Celcius
        int32_t Motor_Temp_3 = message[3] - 40;                                              // Units: degrees Celcius

        int32_t Brake_Resistor_Temp = message[5] - 40;                                       // Units: degrees Celcius
        int32_t Control_Board_Temp = message[6] - 40;                                        // Units: degrees Celcius
        int32_t Inverter_Coolant_Temp = message[7] - 40;                                     // Units: degrees Celcius
      }
      break;

    case DM1:

      uint32_t Protect_Lamp_Status = message[0] >> 6;
      uint32_t Amber_Warning_Lamp_Status = (message[0] >> 4) % 4;
      uint32_t Red_Stop_Lamp_Status = (message[0] >> 2) % 4;
      uint32_t Multi_Indicator_Lamp_Status = message[0] % 4;

      uint32_t Flash_Protect_Lamp_Status = message[1] >> 6;
      uint32_t Flash_Amber_Warning_Lamp_Status = (message[1] >> 4) % 4;
      uint32_t Flash_Red_Stop_Lamp_Status = (message[1] >> 2) % 4;
      uint32_t Flash_Multi_Indicator_Lamp_Status = message[1] % 4;

      uint32_t SPN = message[2] + (message[3] << 8) + ((message[4] >> 5) << 16);

      uint32_t FMI = message[4] % 8;

      uint32_t Occurrence_Count = message[5] % 2;
      break;
  }
}
