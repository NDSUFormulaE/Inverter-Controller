//PD400.h
#ifndef PD400_h
#define PD400_h
#include "Arduino.h"
#include "FlexCAN_T4.h"


class PD400
{
  public:
    PD400(int pin);
    void Begin();
    void setSpeed(int rpm);
    CAN_message_t read();
    static void canSniff(const CAN_message_t &msg );
    static CAN_message_t frame;
  private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
    
    int _pin;
    
    

};

#endif
