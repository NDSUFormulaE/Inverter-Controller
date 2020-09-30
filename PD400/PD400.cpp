#include "Arduino.h"
#include "PD400.h"
#include <SPI.h>
#include <mcp2515.h>

#define pd400Address 0xFFFC
PD400::PD400(int pin):mcp(pin) {
  _pin = pin;
}



void PD400::Begin(){
  SPI.begin();
  mcp.reset();
  mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp.setNormalMode();
}


struct can_frame PD400::recieve() {
  if (mcp.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print("    ");
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print("     ");

    for (int i = 0; i < canMsg.can_dlc; i++)  { // print the data

      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");

    }
    return canMsg;
    Serial.println();
  }
}




void PD400::setSpeed(short rpm){
  struct can_frame canMsg1;

  if(rpm>16000){
    rpm=16000;
  }
  if(rpm<-16000){
    rpm=-160000;
  }
  rpm = rpm+16000;

  union cnv{
    short _rpm;
    byte b[2];

  }data;

  data._rpm=rpm;


  canMsg1.can_id  = pd400Address;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xF4;
  canMsg1.data[1] = 0x1B;
  canMsg1.data[2] = data.b[1];
  canMsg1.data[3] = data.b[0];
  canMsg1.data[4] = 0xFF;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0x17;
  canMsg1.data[7] = 0x0F;

  mcp.sendMessage(&canMsg1);




}


















