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
  
  // CAN_message_t msg;
  // Can0.read(msg)

  // if ( Can0.read(msg) ){
    
  //   Serial.println("hey im returning" + msg.id);
  //   canSniff(msg);
  //   return msg;
  // } 

}




void PD400::setSpeed(short rpm){

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
  Serial.println(data._rpm);
  Serial.println(data.b[0]);
CAN_message_t msg;

msg.id = pd400Address;
msg.len = 8;
msg.buf[0]=0xf4;
msg.buf[1]=0x1b;
msg.buf[2]=data.b[1];
msg.buf[3]=data.b[0];
msg.buf[4]=0xff;
msg.buf[5]=0xff;
msg.buf[6]=0x01;
msg.buf[7]=0x0f;

Can0.write(msg);

}




static void PD400::canSniff(const CAN_message_t &msg) { // global callback
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}













