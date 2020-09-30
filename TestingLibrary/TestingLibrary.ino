#include "PD400.h"
#include "mcp2515.h"
PD400 pd400(10);
struct can_frame canMsg;
void setup() {
  pd400.Begin();
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  canMsg = pd400.recieve();
  // put your main code here, to run repeatedly:
  pd400.setSpeed(400);
}
