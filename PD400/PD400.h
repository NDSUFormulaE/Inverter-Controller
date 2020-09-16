#ifndef PD400_h
#define PD400_h
#include <SPI.h>
#include "mcp2515.h"
#include "Arduino.h"

class PD400
{
  public:
    PD400(int pin);
    // String[] recieve();
    void Begin();
    void dot();
    void dash();
  private:
    int _pin;
    MCP2515 mcp2515();

    // String[] recieveString;
    struct can_frame canMsg;

};

#endif
