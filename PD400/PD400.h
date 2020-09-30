#ifndef PD400_h
#define PD400_h
#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>


class PD400
{
  public:
    PD400(int pin);
    void Begin();
    struct can_frame recieve();
    void setSpeed(short rpm);
    MCP2515 mcp;
  private:
    struct can_frame canMsg;
    int _pin;
    
    

};

#endif
