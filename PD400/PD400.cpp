//PD400.cpp
#include "Arduino.h"
#include "PD400.h"
#include "FlexCAN_T4.h"

#define pd400Address 0xFFFC



PD400::PD400(int pin)

{
  
  _pin = pin;
}


void PD400::Begin(){
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(FIFO, canSniff);
  Can0.mailboxStatus();
}


CAN_message_t PD400::read(){
  Can0.events();
  
  
  // Can0.read(frame);

  // if ( Can0.read(frame) ){
    
  //   Serial.println("hey im returning" + frame.id);
  //   canSniff(frame);
  // } 
  // return frame;
}




void PD400::setSpeed(int rpm){

  if(rpm>16000){
    rpm=16000;
  }
  if(rpm<-16000){
    rpm=-160000;
  }
  rpm = rpm+16000;

 byte rpm1 = rpm;
 byte rpm2 = rpm>>8;



CAN_message_t line;

line.id = pd400Address;
line.len = 8;
line.buf[0]=0xf4;
line.buf[1]=0x1b;
line.buf[2]=rpm2;
line.buf[3]=rpm1;
line.buf[4]=0xff;
line.buf[5]=0xff;
line.buf[6]=0x01;
line.buf[7]=0x0f;

Can0.write(line);

}


 static void PD400::canSniff(const CAN_message_t &msg ) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();

   // frame = msg;

}













