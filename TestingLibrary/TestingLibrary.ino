#include "PD400.h"
#include <FlexCAN_T4.h>
CAN_message_t msg;
PD400 pd400(10);
int milli = 0;
void setup() {
  pd400.Begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  msg = pd400.read();
  if(millis()>milli+1000){
    milli = millis();
    pd400.setSpeed(14000);
//    pd400.canSniff(msg);
  }
  
  
}
