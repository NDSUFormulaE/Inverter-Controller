#include "PD400.h"
#include "mcp2515.h"
PD400 pd400(10);
void setup() {
  pd400.Begin();
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  pd400.recieve();
  // put your main code here, to run repeatedly:

}
