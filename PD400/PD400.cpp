#include "Arduino.h"
#include "PD400.h"
#include <SPI.h>
#include "mcp2515.h"

PD400::PD400(int pin)
{
  pinMode(pin, OUTPUT);
  _pin = pin;
}


void PD400::Begin(){
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}


// String[] PD400::recieve() {
//   if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

//     Serial.print(canMsg.can_id, HEX); // print ID
//     Serial.print("    ");
//     Serial.print(canMsg.can_dlc, HEX); // print DLC
//     Serial.print("     ");

//     for (int i = 0; i < canMsg.can_dlc; i++)  { // print the data

//       Serial.print(canMsg.data[i], HEX);
//       Serial.print(" ");

//     }

//     Serial.println();
//   }
//   return receiveString;
// }


void PD400::dot()
{
  digitalWrite(_pin, HIGH);
  delay(250);
  digitalWrite(_pin, LOW);
  delay(250);  
}

void PD400::dash()
{
  digitalWrite(_pin, HIGH);
  delay(1000);
  digitalWrite(_pin, LOW);
  delay(250);
}
